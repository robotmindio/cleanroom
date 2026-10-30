"""The active open-vocabulary query: bounded, normalized labels with a content revision.

A query is not configuration. It is requested at runtime, travels with every frame it produced and
forms part of that inference's operating point, so the same capture under another query is another
inference and replay can rebuild the exact prompt from the observation alone. Labels may declare the
whole they are a part of, and learned concepts add exemplar-matched phrases without retraining.
"""

from dataclasses import dataclass
import hashlib
import json
import unicodedata

MAX_LABELS = 16
MAX_LABEL_CHARS = 48
# Grounding DINO truncates its text at 256 tokens; a BERT word piece spans at least one character,
# so a prompt of at most 254 characters plus [CLS]/[SEP] can never be truncated.
MAX_PROMPT_CHARS = 254


def normalize_label(text):
    """Case- and whitespace-insensitive spelling, matching the search and Fiber label comparison."""
    if not isinstance(text, str):
        raise ValueError('query labels must be strings')
    if len(text) > 4 * MAX_LABEL_CHARS:
        raise ValueError(f'query labels need 1..{MAX_LABEL_CHARS} characters')
    label = ' '.join(unicodedata.normalize('NFKC', text).lower().split())
    if not 1 <= len(label) <= MAX_LABEL_CHARS:
        raise ValueError(f'query labels need 1..{MAX_LABEL_CHARS} characters')
    # '.' separates Grounding DINO phrases; other punctuation and controls would change the tokenization.
    if any(not (char.isalnum() or char in " -'") for char in label):
        raise ValueError('query labels may contain only letters, digits, spaces, hyphens and apostrophes')
    return label


@dataclass(frozen=True)
class ConceptRef:
    """A learned concept as a query names it; the full exemplar document lives in the concept store."""
    name: str
    revision: str
    prompts: tuple


@dataclass(frozen=True)
class Query:
    labels: tuple
    # (part, whole) pairs: a part detection is assigned only to a detection of its declared whole.
    parts: tuple = ()
    concepts: tuple = ()

    @property
    def phrases(self):
        return self.labels + tuple(prompt for concept in self.concepts for prompt in concept.prompts)

    @property
    def prompt(self):
        return '. '.join(self.phrases) + '.'

    @property
    def reportable(self):
        """Labels a detection under this query can carry: requested labels and matched concept names."""
        return self.labels + tuple(concept.name for concept in self.concepts)

    @property
    def revision(self):
        document = {'schema': 1, 'labels': list(self.labels)}
        # Plain label queries keep their established revision; parts and concepts extend it.
        if self.parts:
            document['parts'] = [list(pair) for pair in self.parts]
        if self.concepts:
            document['concepts'] = [[concept.name, concept.revision, list(concept.prompts)] for concept in self.concepts]
        canonical = json.dumps(document, separators=(',', ':'), ensure_ascii=False)
        return hashlib.sha256(canonical.encode()).hexdigest()

    def document(self):
        document = {'revision': self.revision, 'labels': list(self.labels), 'prompt': self.prompt}
        if self.parts:
            document['parts'] = [{'label': part, 'whole': whole} for part, whole in self.parts]
        if self.concepts:
            document['concepts'] = [{'name': concept.name, 'revision': concept.revision, 'prompts': list(concept.prompts)}
                                    for concept in self.concepts]
        return document


def _hex(value):
    return isinstance(value, str) and len(value) == 64 and all(char in '0123456789abcdef' for char in value)


def make_query(labels, part_of=None, concepts=()):
    """Normalize, de-duplicate in request order and bound an untrusted label list.

    `part_of` is parallel to `labels`: '' or the whole that label is a part of. `concepts` are
    ConceptRef values of learned concepts the recognizer matches by exemplar.
    """
    if not isinstance(labels, (list, tuple)) or len(labels) > MAX_LABELS:
        raise ValueError(f'a query accepts at most {MAX_LABELS} plain labels')
    part_of = list(part_of or [])
    if part_of and len(part_of) != len(labels):
        raise ValueError('part_of must name one whole (or nothing) for every label')
    normalized = [normalize_label(label) for label in labels]
    wholes = {}
    for label, whole in zip(normalized, part_of):
        if whole:
            whole = normalize_label(whole)
            if whole == label or whole not in normalized:
                raise ValueError(f'part {label!r} needs another label of this query as its whole')
            if wholes.setdefault(label, whole) != whole:
                raise ValueError(f'part {label!r} declares two wholes')
    if set(wholes) & set(wholes.values()):
        raise ValueError('a whole cannot itself be a part')
    concepts = tuple(concepts)
    if len(concepts) > 4 or len({concept.revision for concept in concepts}) != len(concepts):
        raise ValueError('a query takes at most 4 distinct concepts')
    for concept in concepts:
        if not isinstance(concept, ConceptRef) or not _hex(concept.revision) or not 1 <= len(concept.prompts) <= 4:
            raise ValueError('invalid concept reference')
        if normalize_label(concept.name) != concept.name or any(normalize_label(p) != p for p in concept.prompts):
            raise ValueError('concept names and prompts must be canonical labels')
    unique = tuple(dict.fromkeys(normalized))
    if not unique and not concepts:
        raise ValueError('a query needs labels or learned concepts')
    query = Query(unique, tuple((label, wholes[label]) for label in unique if label in wholes), concepts)
    names = query.phrases + tuple(concept.name for concept in concepts)
    if len(set(names)) != len(names):
        raise ValueError('labels, concept names and concept prompts must not repeat one another')
    # The provider refuses a prompt of more phrases than this, so a query it cannot run is refused here.
    if len(query.phrases) > MAX_LABELS:
        raise ValueError(f'labels and concept prompts together exceed {MAX_LABELS} phrases')
    if len(query.prompt) > MAX_PROMPT_CHARS:
        raise ValueError(f'query prompt exceeds {MAX_PROMPT_CHARS} characters')
    return query


def recorded_query(document):
    """A query as a frame carried it; anything not already canonical was not produced by this detector."""
    if isinstance(document, (list, tuple)):
        document = {'labels': list(document)}
    if not isinstance(document, dict) or set(document) - {'revision', 'labels', 'prompt', 'parts', 'concepts'}:
        raise ValueError('frame query is malformed')
    labels = document.get('labels')
    parts = {item.get('label'): item.get('whole') for item in document.get('parts', []) if isinstance(item, dict)}
    concepts = [ConceptRef(item['name'], item['revision'], tuple(item['prompts']))
                for item in document.get('concepts', []) if isinstance(item, dict)
                and set(item) == {'name', 'revision', 'prompts'} and isinstance(item['prompts'], list)]
    if len(concepts) != len(document.get('concepts', [])) or len(parts) != len(document.get('parts', [])):
        raise ValueError('frame query is malformed')
    if not isinstance(labels, list):
        raise ValueError('frame query labels are not canonical')
    query = make_query(labels, [parts.get(label, '') for label in labels] if parts else None, concepts)
    if list(query.labels) != labels or dict(query.parts) != parts:
        raise ValueError('frame query labels are not canonical')
    for key in ('revision', 'prompt'):
        if key in document and document[key] != getattr(query, key):
            raise ValueError(f'frame query {key} does not match its labels')
    return query


def limits():
    return {'max_labels': MAX_LABELS, 'max_label_chars': MAX_LABEL_CHARS, 'max_prompt_chars': MAX_PROMPT_CHARS,
            'max_concepts': 4, 'max_concept_prompts': 4}
