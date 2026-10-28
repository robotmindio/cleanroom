"""Pinned local and remote recognizers with one source-pixel prediction contract."""

import base64
import hashlib
import importlib.metadata
import json
import math
import os
import sys
import time
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

import cv2
import numpy as np
import yaml

from lekiwi_rmf.object_cameras import config_directory
from lekiwi_rmf.perception_concepts import load_concept
from lekiwi_rmf.perception_profiles import default_profile
from lekiwi_rmf.perception_query import make_query
from lekiwi_rmf.opencv_yolox import YoloX

# Native vocabulary of the pinned OpenCV Zoo YOLOX checkpoint, indexed by output class.
COCO_LABELS = (
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
    'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
    'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
    'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
    'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
    'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
    'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard',
    'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
    'scissors', 'teddy bear', 'hair drier', 'toothbrush')


TORCH_BACKENDS = ('rtdetr_v2', 'grounding_dino')
REMOTE_BACKENDS = ('remote_grounding_dino',)
# Backends that localize the active query instead of a fixed native catalogue.
OPEN_BACKENDS = ('grounding_dino', 'remote_grounding_dino')
# Backends whose provider matches learned concepts by exemplar embedding.
CONCEPT_BACKENDS = ('remote_grounding_dino',)
MAX_RESPONSE_BYTES = 1024 * 1024
# Registry governance, not operating point: promotion changes these without changing what the model computes,
# so they stay out of the identity every observation and evaluation report records.
GOVERNANCE_KEYS = ('status', 'replaces', 'training')
STATUSES = ('experimental', 'qualified')


class ProviderUnavailable(RuntimeError):
    """A remote provider could not serve this frame under its admitted operating point."""


def model_spec(name):
    configured = yaml.safe_load((config_directory() / 'recognition_models.yaml').read_text())
    if name not in configured:
        raise ValueError(f'unknown pinned recognizer: {name}')
    return configured[name]


def provider_endpoint(name):
    """Where a remote provider listens. Deliberately outside every identity: the provider proves
    its operating point in each response, so moving it cannot silently change observations."""
    configured = yaml.safe_load((config_directory() / 'perception_providers.yaml').read_text())
    endpoint = configured.get(name) if isinstance(configured, dict) else None
    if not isinstance(endpoint, dict) or set(endpoint) != {'url', 'timeout_sec'}:
        raise ValueError(f'unknown perception provider: {name}')
    url, timeout = endpoint['url'], endpoint['timeout_sec']
    parts = urlsplit(url) if isinstance(url, str) else None
    if (parts is None or parts.scheme not in ('http', 'https') or not parts.hostname or parts.path not in ('', '/')
            or parts.query or parts.fragment or parts.username or parts.password):
        raise ValueError(f'perception provider {name} needs an http(s) origin URL')
    if type(timeout) not in (int, float) or not 0 < timeout <= 10:
        raise ValueError(f'perception provider {name} timeout must be within (0,10] seconds')
    return url.rstrip('/'), float(timeout)


def open_vocabulary(name):
    return model_spec(name)['backend'] in OPEN_BACKENDS


class Rows(list):
    """Detections as [x, y, w, h, score, label, extras] rows; `run` holds the provider's run reference."""
    run = None


def overlap(left, right):
    x, y = max(left[0], right[0]), max(left[1], right[1])
    width = max(0., min(left[0] + left[2], right[0] + right[2]) - x)
    height = max(0., min(left[1] + left[3], right[1] + right[3]) - y)
    area = width * height
    union = left[2] * left[3] + right[2] * right[3] - area
    return area / union if union > 0 else 0.


def suppress(rows, scope, iou):
    """Greedy NMS within each suppression group, keeping every suppressed candidate as evidence.

    `scope` 'label' groups rows by label, so overlapping objects of different classes survive; 'prompt'
    makes every query phrase one group, so synonyms proposing one object collapse into the strongest
    phrase, which records the others as alternatives. `iou` 1 disables suppression.
    """
    rows = [list(row[:6]) + [dict(row[6]) if len(row) > 6 else {}] for row in rows]
    if iou >= 1:
        return rows
    kept = []
    for row in sorted(rows, key=lambda item: -item[4]):
        for survivor in kept:
            if (scope == 'prompt' or survivor[5] == row[5]) and overlap(survivor, row) > iou:
                if row[5] != survivor[5]:
                    alternatives = survivor[6].setdefault('alternatives', [])
                    if len(alternatives) < 4:
                        alternatives.append({'label': row[5], 'score': float(row[4])})
                break
        else:
            kept.append(row)
    order = {id(row): index for index, row in enumerate(rows)}
    return sorted(kept, key=lambda row: order[id(row)])


class RemoteProvider:
    """Stateless HTTP client for the RobotMind perception provider's operations."""

    def __init__(self, url, timeout, opener=urlopen):
        self.url, self.timeout, self._open = url, timeout, opener

    def infer(self, body):
        return self.post('/v1/infer', body)

    def post(self, path, body, timeout=None):
        payload = json.dumps(body, separators=(',', ':')).encode()
        request = Request(self.url + path, data=payload, method='POST',
                          headers={'Content-Type': 'application/json', 'Accept': 'application/json'})
        try:
            with self._open(request, timeout=timeout or self.timeout) as response:
                data = response.read(MAX_RESPONSE_BYTES + 1)
        except HTTPError as error:
            code = ''
            try:
                code = json.loads(error.read(65536)).get('error', {}).get('code', '')
            except (ValueError, AttributeError, OSError):
                pass
            raise ProviderUnavailable(f'perception provider returned HTTP {error.code} {code}'.rstrip()) from error
        except (URLError, OSError, ValueError) as error:
            raise ProviderUnavailable(f'perception provider unreachable: {getattr(error, "reason", error)}') from error
        if len(data) > MAX_RESPONSE_BYTES:
            raise ProviderUnavailable('perception provider response exceeds 1 MiB')
        try:
            document = json.loads(data)
        except ValueError as error:
            raise ProviderUnavailable('perception provider returned invalid JSON') from error
        if not isinstance(document, dict):
            raise ProviderUnavailable('perception provider returned a non-object response')
        return document


def provider_timings(document):
    """The provider's own decode/queue/inference/total split in seconds, or None when absent or inconsistent.

    Only latency accounting uses it; a response is admitted or refused on its operating point alone.
    """
    timings = document.get('timings_ms') if isinstance(document, dict) else None
    names = ('decode', 'queue', 'inference', 'total')
    if not isinstance(timings, dict) or any(
            type(timings.get(name)) not in (int, float) or not math.isfinite(timings[name]) or timings[name] < 0
            for name in names):
        return None
    if timings['decode'] + timings['queue'] + timings['inference'] > timings['total'] + 1:
        return None
    return {name: timings[name] / 1000 for name in names}


def verify_model(directory, spec):
    directory = Path(directory).expanduser()
    for name, expected in spec['files'].items():
        file = directory / name
        if file.stat().st_size != expected['size']:
            raise ValueError(f'model artifact has wrong size: {name}')
        with file.open('rb') as stream:
            if hashlib.file_digest(stream, 'sha256').hexdigest() != expected['sha256']:
                raise ValueError(f'model artifact has wrong hash: {name}')
    return directory


def recognition_identity(name):
    """The recognizer's model identity: artifacts, runtime and input normalization, but no thresholds.

    Thresholds, suppression, resolution and budget are the proposal profile, whose revision every
    frame records beside this identity (lekiwi_rmf/perception_profiles.py).
    """
    spec = model_spec(name)
    runtime = {'opencv': cv2.__version__, 'numpy': np.__version__}
    if spec['backend'] in TORCH_BACKENDS:
        runtime.update({package: importlib.metadata.version(package) for package in ['torch', 'torchvision', 'transformers', 'Pillow', 'safetensors', 'tokenizers']})
    # An open recognizer's labels are the per-frame query, recorded beside this identity in each observation;
    # the startup default is not an operating point of the model.
    artifacts = {key: value for key, value in spec.items() if key != 'default_query' and key not in GOVERNANCE_KEYS}
    # Every recognizer consumes the frame's stored evidence artifact (a deterministic JPEG), so a stored
    # observation replays byte-exact; see object_observations.capture_artifact.
    document = {'model': name, 'artifacts': artifacts, 'postprocessing': 'profile', 'preprocessing_revision': 2,
                'input': 'evidence_jpeg_q80', 'labels': 'query' if spec['backend'] in OPEN_BACKENDS else 'native', 'runtime': runtime,
                'device': 'cpu', 'threads': 2, 'interop_threads': 1}
    if spec['backend'] in REMOTE_BACKENDS:
        # The artifacts pin the provider's own operating-point hash (weights, Torch/CUDA, GPU), which every
        # response must echo; the robot sends the evidence artifact bytes unchanged. The endpoint is not identity.
        document.update(device='remote', threads=None, interop_threads=None)
    identity = hashlib.sha256(json.dumps(document, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    return identity, document


def reject_parameter_changes(node, names):
    """Startup values form the admitted operating point; a silent no-op change would misreport it."""
    from rcl_interfaces.msg import SetParametersResult

    def reject(parameters):
        changed = sorted({parameter.name for parameter in parameters} & set(names))
        return SetParametersResult(successful=not changed, reason=(
            f'{", ".join(changed)} fixed at startup; change the tracked configuration and restart' if changed else ''))
    node.add_on_set_parameters_callback(reject)


class Recognizer:
    def __init__(self, name='yolox', directory=None, allow_experimental=False):
        self.name = name
        self.spec = model_spec(name)
        self.backend = self.spec['backend']
        # A trained recognizer serves only after promotion (scripts/register-recognizer.py); before that,
        # only the offline evaluator and the trainer's load check may run it.
        self.status = self.spec.get('status', 'qualified')
        if self.status not in STATUSES:
            raise ValueError(f'{name}: status must be one of {STATUSES}')
        if self.status == 'experimental' and not allow_experimental:
            raise ValueError(f'{name} is an experimental recognizer; promote it with a passing evaluation first')
        self.sha256, self.identity = recognition_identity(name)
        self.default_profile = default_profile(self.backend)
        # Open recognizers report only the phrases of the query they ran; closed ones their native catalogue.
        self.open_vocabulary = self.backend in OPEN_BACKENDS
        self.supports_concepts = self.backend in CONCEPT_BACKENDS
        self.default_query = make_query(self.spec['default_query']) if self.open_vocabulary else None
        self.labels = None
        # Remote only: (round trip, provider timings) of the last answer, for the latency budget.
        self.last_remote = None
        cv2.setNumThreads(2)
        if self.backend in REMOTE_BACKENDS:
            self.directory = None
            self.model = RemoteProvider(*provider_endpoint(self.spec['provider']))
            return
        self.directory = verify_model(directory or Path.home() / '.ros/lekiwi/models' / name, self.spec)
        if self.backend == 'yolox':
            self.model = YoloX(str(self.directory / next(iter(self.spec['files']))))
            self.labels = list(COCO_LABELS)
        else:
            import torch
            from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
            from transformers import RTDetrImageProcessor, RTDetrV2ForObjectDetection
            torch.set_num_threads(2)
            if torch.get_num_interop_threads() != 1:
                torch.set_num_interop_threads(1)
            self.torch = torch
            if self.backend == 'rtdetr_v2':
                self.processor = RTDetrImageProcessor.from_pretrained(
                    self.directory, local_files_only=True,
                    size={'height': self.spec['input_size'], 'width': self.spec['input_size']})
                self.model = RTDetrV2ForObjectDetection.from_pretrained(
                    self.directory, local_files_only=True, use_safetensors=True).eval().to('cpu')
                self.labels = [self.model.config.id2label[index] for index in sorted(self.model.config.id2label)]
            else:
                self.processor = AutoProcessor.from_pretrained(self.directory, local_files_only=True)
                self.model = AutoModelForZeroShotObjectDetection.from_pretrained(
                    self.directory, local_files_only=True, use_safetensors=True).eval().to('cpu')

    def predict(self, image, query=None, profile=None, artifact=None):
        """Detections of `image` (the decoded evidence `artifact`) under `profile`.

        Open recognizers localize exactly `query`; closed ones cannot express a query and refuse one.
        A remote recognizer sends `artifact` itself, so the provider sees the stored evidence bytes.
        """
        if self.open_vocabulary != (query is not None):
            raise ValueError('an open-vocabulary recognizer needs a query' if self.open_vocabulary else
                             'a closed-vocabulary recognizer cannot answer a query')
        profile = profile or self.default_profile
        if profile.backend != self.backend:
            raise ValueError(f'profile for {profile.backend} cannot run {self.backend}')
        if query is not None and query.concepts and not self.supports_concepts:
            raise ValueError(f'{self.name} cannot match learned concepts; it has no exemplar encoder')
        values = profile.parameters
        height, width = image.shape[:2]
        if self.backend in REMOTE_BACKENDS:
            if artifact is None:
                raise ValueError('a remote recognizer needs the evidence artifact')
            return self._remote(artifact, (width, height), query, profile)
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        if self.backend == 'yolox':
            scale = min(640 / width, 640 / height)
            resized = cv2.resize(rgb, (max(1, int(width * scale)), max(1, int(height * scale))))
            padded = np.full((640, 640, 3), 114.0, dtype=np.float32)
            padded[:resized.shape[0], :resized.shape[1]] = resized
            # The graph's own class-aware NMS is the suppression; no second pass is added after it.
            self.model.confThreshold, self.model.nmsThreshold = values['score_threshold'], values['nms_iou']
            predictions = self.model.infer(padded)
            # max_detections is applied by object_detector.detect, which every caller shares.
            return Rows([[x / scale, y / scale, w / scale, h / scale, score, COCO_LABELS[int(label)], {}]
                         for x, y, w, h, score, label in predictions])
        from PIL import Image
        pixels = Image.fromarray(rgb)
        if self.backend == 'rtdetr_v2':
            inputs = self.processor(images=pixels, return_tensors='pt')
        else:
            inputs = self.processor(images=pixels, text=query.prompt, return_tensors='pt',
                                    size={'shortest_edge': values['shortest_edge'], 'longest_edge': 1333})
        with self.torch.inference_mode():
            outputs = self.model(**inputs)
        if self.backend == 'rtdetr_v2':
            result = self.processor.post_process_object_detection(
                outputs, target_sizes=[(height, width)], threshold=values['score_threshold'])[0]
            labels = [self.model.config.id2label[int(label)] for label in result['labels']]
            scope = 'label'
        else:
            result = self.processor.post_process_grounded_object_detection(
                outputs, inputs.input_ids, threshold=values['box_threshold'],
                text_threshold=values['text_threshold'], target_sizes=[(height, width)])[0]
            labels, scope = result['text_labels'], values['nms_scope']
        rows = []
        for label, box, score in zip(labels, result['boxes'], result['scores']):
            x0, y0, x1, y1 = box.tolist()
            rows.append([x0, y0, x1 - x0, y1 - y0, float(score), str(label), {}])
        return Rows(suppress(rows, scope, values['nms_iou']))

    def concept_documents(self, query):
        # A stored concept is immutable under its revision, so a validated load is cached by it.
        cache = self.__dict__.setdefault('_concepts', {})
        for concept in query.concepts:
            if concept.revision not in cache:
                cache[concept.revision] = load_concept(concept.revision)
        return [cache[concept.revision] for concept in query.concepts]

    def _remote(self, data, size, query, profile):
        """The evidence artifact out; a response counts only if it proves the admitted operating point and query."""
        parameters = profile.parameters
        labels = list(query.labels)
        expected = {'labels': labels, 'prompt': query.prompt,
                    'box_threshold': parameters['box_threshold'], 'text_threshold': parameters['text_threshold']}
        body = {'model': self.spec['provider_model'], 'image': base64.b64encode(data).decode('ascii'),
                'labels': labels, 'profile': parameters, 'deadline_ms': int(self.model.timeout * 1000)}
        concepts = self.concept_documents(query)
        if concepts:
            body['concepts'] = concepts
        self.last_remote = None
        sent = time.monotonic()
        response = self.model.infer(body)
        self.last_remote = time.monotonic() - sent, provider_timings(response)
        source, echoed = response.get('image'), response.get('profile')
        if (response.get('schema') != 1 or response.get('model') != self.spec['provider_model']
                or response.get('model_sha256') != self.spec['provider_model_sha256']
                or response.get('state', 'succeeded') != 'succeeded'
                or response.get('effective') != expected or not isinstance(source, dict)
                or not isinstance(echoed, dict) or echoed.get('parameters') != parameters
                or source.get('sha256') != hashlib.sha256(data).hexdigest()
                or (source.get('width'), source.get('height')) != size):
            raise ProviderUnavailable('perception provider answered outside the admitted operating point')
        detections = response.get('detections')
        if not isinstance(detections, list) or len(detections) > parameters['max_detections']:
            raise ProviderUnavailable('perception provider returned malformed detections')
        reportable, revisions = set(query.reportable), {concept['revision'] for concept in concepts}
        rows = Rows()
        for item in detections:
            box = item.get('box') if isinstance(item, dict) else None
            if (not isinstance(box, list) or len(box) != 4
                    or not all(type(value) in (int, float) and math.isfinite(value) for value in box)
                    or type(item.get('score')) not in (int, float) or not isinstance(item.get('label'), str)):
                raise ProviderUnavailable('perception provider returned malformed detections')
            extras = {}
            alternatives = item.get('alternatives', [])
            if (not isinstance(alternatives, list) or len(alternatives) > 4 or any(
                    not isinstance(other, dict) or not isinstance(other.get('label'), str)
                    or type(other.get('score')) not in (int, float) for other in alternatives)):
                raise ProviderUnavailable('perception provider returned malformed alternatives')
            if alternatives:
                extras['alternatives'] = [{'label': other['label'], 'score': float(other['score'])} for other in alternatives]
            concept = item.get('concept')
            if concept is not None:
                if (not isinstance(concept, dict) or concept.get('revision') not in revisions
                        or concept.get('name') != item['label'] or item['label'] not in reportable
                        or any(type(concept.get(key)) not in (int, float) for key in ('similarity', 'negative_similarity'))):
                    raise ProviderUnavailable('perception provider returned a concept match outside the query')
                extras['concept'] = {key: concept[key] for key in ('name', 'revision', 'similarity', 'negative_similarity')}
            x0, y0, x1, y1 = box
            rows.append([x0, y0, x1 - x0, y1 - y0, float(item['score']), item['label'], extras])
        run = {key: response.get(key) for key in ('run_id', 'inference_id')}
        proposals = response.get('proposals')
        if all(isinstance(value, str) and 1 <= len(value) <= 64 for value in run.values()):
            rows.run = {**run, 'proposals': proposals if isinstance(proposals, dict) else None}
        return rows


def selected_models(settings):
    """The primary recognizer and, when configured, the local recognizer that serves while it is unavailable."""
    return [name for name in (settings['model_name'], settings.get('fallback_model_name', '')) if name]


def needs_private_runtime(*names):
    """Torch recognizers need the private CPU environment; YOLOX and remote providers run in ROS's Python."""
    return any(model_spec(name)['backend'] in TORCH_BACKENDS for name in names)


def model_runtime(*names):
    return (Path.home() / '.ros/lekiwi/perception-venv/bin/python') if needs_private_runtime(*names) else Path(sys.executable)


def use_model_runtime(*names):
    python = model_runtime(*names)
    if needs_private_runtime(*names) and Path(sys.prefix) != python.parent.parent:
        if not python.exists():
            raise ValueError('install the selected model with scripts/install-object-model.py first')
        os.execve(python, [str(python), str(Path(sys.argv[0]).resolve()), *sys.argv[1:]],
                  {**os.environ, 'PYTHONNOUSERSITE': '1'})


def main():
    from lekiwi_rmf.object_cameras import camera_bindings
    settings = yaml.safe_load((config_directory() / 'object_perception.yaml').read_text())['/**']['ros__parameters']
    use_model_runtime(*selected_models(settings))
    identity, operating = recognition_identity(settings['model_name'])
    deployment = yaml.safe_load((config_directory() / 'object_perception_deployment.yaml').read_text())
    described = {'model_sha256': identity, 'operating_point': operating,
                 'profile': default_profile(model_spec(settings['model_name'])['backend']).document(),
                 'vocabulary': 'open' if open_vocabulary(settings['model_name']) else 'closed'}
    if open_vocabulary(settings['model_name']):
        described['default_query'] = make_query(model_spec(settings['model_name'])['default_query']).document()
    if settings.get('fallback_model_name'):
        # Frames served by the fallback carry its own identity; an admitting profile lists both.
        fallback, fallback_operating = recognition_identity(settings['fallback_model_name'])
        described.update(fallback_model_sha256=fallback, fallback_operating_point=fallback_operating,
                         fallback_profile=default_profile(model_spec(settings['fallback_model_name'])['backend']).document())
    described['cameras'] = {name: value['sha256'] for name, value in camera_bindings(deployment['cameras']).items()}
    print(json.dumps(described, indent=2))


if __name__ == '__main__':
    main()
