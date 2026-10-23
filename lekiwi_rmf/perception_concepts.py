"""Learned concepts: a new category from taught views, matched by exemplar without retraining the detector.

The open-vocabulary detector proposes regions for the concept's prompt phrases; the perception provider
embeds each proposal with the concept's encoder and keeps it only when it is closer to the taught
exemplars than the concept's calibrated match profile allows. The concept's revision covers exactly the
document the provider checks, so an observation's query revision pins the exemplars that admitted it.
"""

import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import time

import cv2
import numpy as np

from lekiwi_rmf.perception_query import ConceptRef, normalize_label

MAX_EXEMPLARS = 16
MAX_PROMPTS = 4
MIN_CROP_PIXELS = 8
STORE = Path.home() / '.ros/lekiwi/perception/concepts'


def concept_revision(document):
    body = {key: value for key, value in document.items() if key != 'revision'}
    return hashlib.sha256(json.dumps(body, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def _vectors(values, name, dimensions, minimum):
    if not isinstance(values, list) or not minimum <= len(values) <= MAX_EXEMPLARS:
        raise ValueError(f'{name} needs {minimum}..{MAX_EXEMPLARS} vectors')
    for vector in values:
        if (not isinstance(vector, list) or len(vector) != dimensions
                or not all(type(value) is float and math.isfinite(value) for value in vector)
                or abs(math.sqrt(sum(value * value for value in vector)) - 1) > 1e-3):
            raise ValueError(f'{name} must be L2-normalized {dimensions}-dimension vectors')


def validate_concept(document):
    """The provider-facing concept; raises unless it is canonical and its revision is its own."""
    keys = {'name', 'revision', 'prompts', 'encoder', 'exemplars', 'negatives', 'match'}
    if not isinstance(document, dict) or set(document) != keys:
        raise ValueError('concept requires exactly ' + ', '.join(sorted(keys)))
    name, prompts, encoder, match = document['name'], document['prompts'], document['encoder'], document['match']
    if not isinstance(name, str) or normalize_label(name) != name:
        raise ValueError('concept name must be a canonical label')
    if (not isinstance(prompts, list) or not 1 <= len(prompts) <= MAX_PROMPTS or len(set(prompts)) != len(prompts)
            or any(not isinstance(p, str) or normalize_label(p) != p or p == name for p in prompts)):
        raise ValueError(f'concept needs 1..{MAX_PROMPTS} distinct canonical prompts other than its name')
    if (not isinstance(encoder, dict) or set(encoder) != {'model', 'model_sha256'} or not isinstance(encoder['model'], str)
            or not isinstance(encoder['model_sha256'], str) or len(encoder['model_sha256']) != 64):
        raise ValueError('concept encoder requires model and model_sha256')
    exemplars = document['exemplars']
    dimensions = len(exemplars[0]) if isinstance(exemplars, list) and exemplars and isinstance(exemplars[0], list) else 0
    if not 1 <= dimensions <= 4096:
        raise ValueError('concept needs at least one exemplar vector')
    _vectors(exemplars, 'exemplars', dimensions, 1)
    _vectors(document['negatives'], 'negatives', dimensions, 0)
    if (not isinstance(match, dict) or set(match) != {'min_similarity', 'min_margin'}
            or type(match['min_similarity']) is not float or not -1 <= match['min_similarity'] <= 1
            or type(match['min_margin']) is not float or not 0 <= match['min_margin'] <= 2):
        raise ValueError('concept match needs min_similarity in [-1,1] and min_margin in [0,2]')
    if document['revision'] != concept_revision(document):
        raise ValueError('concept revision does not match its content')
    return document


def reference(document):
    return ConceptRef(document['name'], document['revision'], tuple(document['prompts']))


def load_concept(revision, store=None):
    """A stored concept by exact revision; a file that does not prove its revision is refused."""
    if not isinstance(revision, str) or len(revision) != 64 or any(c not in '0123456789abcdef' for c in revision):
        raise ValueError('concept revisions are SHA-256 hex')
    path = Path(store or STORE) / f'{revision}.json'
    if not path.is_file() or path.stat().st_size > 4 * 1024**2:
        raise ValueError(f'concept {revision[:12]} is not in the concept store')
    record = json.loads(path.read_text())
    concept = validate_concept(record.get('concept') if isinstance(record, dict) else None)
    if concept['revision'] != revision:
        raise ValueError('stored concept has another revision')
    return concept


def similarity(vector, references):
    return max((float(np.dot(vector, other)) for other in references), default=-1.0)


def crop(image_bytes, region):
    """Exact region crop, encoded losslessly; the provider crops proposals the same way."""
    pixels = cv2.imdecode(np.frombuffer(image_bytes, np.uint8), cv2.IMREAD_COLOR)
    if pixels is None:
        raise ValueError('taught view image cannot be decoded')
    x, y, w, h = region
    left, top = max(0, int(math.floor(x))), max(0, int(math.floor(y)))
    right, bottom = min(pixels.shape[1], int(math.ceil(x + w))), min(pixels.shape[0], int(math.ceil(y + h)))
    if right - left < MIN_CROP_PIXELS or bottom - top < MIN_CROP_PIXELS:
        raise ValueError(f'taught region is smaller than {MIN_CROP_PIXELS} pixels')
    ok, encoded = cv2.imencode('.png', pixels[top:bottom, left:right])
    if not ok:
        raise ValueError('taught crop cannot be encoded')
    return encoded.tobytes()


def calibrate(positives, negatives, exemplars, references):
    """Smallest similarity that admits no held-out negative, keeping as many held-out positives as possible.

    A positive is admitted when it is at least `min_similarity` from the exemplars and closer to them than to
    any taught negative. Returns (match, evidence); match is None when no threshold admits any positive
    without also admitting a negative, which means the views cannot teach this concept by exemplar.
    """
    def scores(vectors):
        return [(similarity(v, exemplars), similarity(v, references)) for v in vectors]
    held_positive, held_negative = scores(positives), scores(negatives)
    margin = 0.0
    candidates = sorted({s for s, n in held_positive if s - n >= margin}, reverse=True)
    best = None
    for threshold in candidates:
        if any(s >= threshold and s - n >= margin for s, n in held_negative):
            break
        best = threshold
    evidence = {'heldout_positives': len(positives), 'heldout_negatives': len(negatives),
                'positive_similarities': [round(s, 6) for s, _ in held_positive],
                'negative_similarities': [round(s, 6) for s, _ in held_negative]}
    if best is None:
        return None, {**evidence, 'admitted_positives': 0, 'false_admissions': 0}
    admitted = sum(s >= best and s - n >= margin for s, n in held_positive)
    return {'min_similarity': float(best), 'min_margin': margin}, {
        **evidence, 'admitted_positives': admitted, 'false_admissions': 0, 'recall': admitted / len(positives)}


def learn_concept(manifest, embed, *, store=None, now=None):
    """Teach a concept from gallery views; returns the stored record or raises with what is missing.

    `manifest` = {name, prompts, encoder, views: [{image (bytes), artifact_sha256, region [x,y,w,h], polarity
    'positive'|'negative', split 'teach'|'heldout', source}]}. `embed(model, crops)` returns the provider's
    /v1/embed document. Views are byte-exact artifacts: each image must hash to its declared digest.
    """
    name = normalize_label(manifest['name'])
    prompts = list(dict.fromkeys(normalize_label(prompt) for prompt in manifest['prompts']))
    views = manifest['views']
    if not isinstance(views, list) or not 1 <= len(views) <= 4 * MAX_EXEMPLARS:
        raise ValueError(f'teaching needs 1..{4 * MAX_EXEMPLARS} views')
    crops, provenance = [], []
    for view in views:
        if view.get('polarity') not in ('positive', 'negative') or view.get('split') not in ('teach', 'heldout'):
            raise ValueError('each view needs polarity positive|negative and split teach|heldout')
        if hashlib.sha256(view['image']).hexdigest() != view['artifact_sha256']:
            raise ValueError('a taught view does not match its artifact digest; it is not the captured evidence')
        region = view['region']
        if (not isinstance(region, list) or len(region) != 4 or not all(type(v) in (int, float) for v in region)
                or region[2] <= 0 or region[3] <= 0):
            raise ValueError('each view needs a region [x, y, width, height]')
        crops.append(crop(view['image'], region))
        provenance.append({key: view[key] for key in ('artifact_sha256', 'region', 'polarity', 'split')}
                          | {'source': str(view.get('source', ''))[:256],
                             'crop_sha256': hashlib.sha256(crops[-1]).hexdigest()})
    encoder, vectors, model_sha256 = manifest['encoder'], [], None
    for start in range(0, len(crops), 8):
        response = embed(encoder, crops[start:start + 8])
        if (response.get('model') != encoder or response.get('modality') != 'image' or response.get('normalized') is not True
                or not isinstance(response.get('embeddings'), list) or len(response['embeddings']) != len(crops[start:start + 8])):
            raise ValueError('the encoder answered outside its embed contract')
        identity = response.get('model_sha256')
        if vectors and identity != model_sha256:
            raise ValueError('the encoder changed identity while embedding the taught views')
        model_sha256 = identity
        vectors.extend([float(value) for value in vector] for vector in response['embeddings'])
    groups = {(v['polarity'], v['split']): [] for v in provenance}
    for item, vector in zip(provenance, vectors):
        groups[(item['polarity'], item['split'])].append(vector)
    exemplars, references = groups.get(('positive', 'teach'), []), groups.get(('negative', 'teach'), [])
    held_positive, held_negative = groups.get(('positive', 'heldout'), []), groups.get(('negative', 'heldout'), [])
    if not exemplars or not held_positive or not held_negative:
        raise ValueError('teaching needs taught positives and held-out positives and negatives; '
                         'show more views of the object and of similar things that are not it')
    if len(exemplars) > MAX_EXEMPLARS or len(references) > MAX_EXEMPLARS:
        raise ValueError(f'at most {MAX_EXEMPLARS} taught positives and negatives')
    match, evidence = calibrate(held_positive, held_negative, exemplars, references)
    if match is None:
        raise ValueError('no match threshold separates held-out positives from negatives; the concept needs '
                         'a reviewed dataset and a trained detector rather than exemplar matching')
    concept = {'name': name, 'prompts': prompts, 'encoder': {'model': encoder, 'model_sha256': model_sha256},
               'exemplars': exemplars, 'negatives': references, 'match': match}
    concept['revision'] = concept_revision(concept)
    validate_concept(concept)
    record = {'schema': 1, 'concept': concept, 'views': provenance, 'evaluation': evidence,
              'learned_at_unix': time.time() if now is None else now,
              'method': 'few-shot exemplar matching over open-vocabulary proposals; detector weights unchanged'}
    directory = Path(store or STORE)
    directory.mkdir(parents=True, exist_ok=True)
    target = directory / f'{concept["revision"]}.json'
    with tempfile.NamedTemporaryFile('w', dir=directory, delete=False) as stream:
        json.dump(record, stream, indent=1)
    os.replace(stream.name, target)
    return record
