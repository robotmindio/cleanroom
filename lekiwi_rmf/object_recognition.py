"""Pinned local and remote recognizers with one source-pixel prediction contract."""

import base64
import hashlib
import importlib.metadata
import json
import math
import os
import sys
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

import cv2
import numpy as np
import yaml

from lekiwi_rmf.object_cameras import config_directory
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
# Grounding DINO's phrase-to-token threshold; local and remote providers must agree on it.
TEXT_THRESHOLD = .25
MAX_RESPONSE_BYTES = 1024 * 1024


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


def suppress(rows, backend, confidence):
    # Synonymous prompt phrases can propose the same object twice, so prompts share one NMS group;
    # closed-vocabulary classes are suppressed per class so overlapping different objects survive.
    groups = [''] * len(rows) if backend in OPEN_BACKENDS else [row[5] for row in rows]
    kept = []
    for group in dict.fromkeys(groups):
        indices = [i for i, value in enumerate(groups) if value == group]
        selected = cv2.dnn.NMSBoxes([rows[i][:4] for i in indices], [rows[i][4] for i in indices], confidence, .5)
        kept.extend(indices[int(index)] for index in np.asarray(selected).reshape(-1))
    return [rows[i] for i in sorted(kept)]


class RemoteProvider:
    """Stateless HTTP client for the RobotMind perception provider's `infer` operation."""

    def __init__(self, url, timeout, opener=urlopen):
        self.url, self.timeout, self._open = url, timeout, opener

    def infer(self, body):
        payload = json.dumps(body, separators=(',', ':')).encode()
        request = Request(self.url + '/v1/infer', data=payload, method='POST',
                          headers={'Content-Type': 'application/json', 'Accept': 'application/json'})
        try:
            with self._open(request, timeout=self.timeout) as response:
                data = response.read(MAX_RESPONSE_BYTES + 1)
        except HTTPError as error:
            raise ProviderUnavailable(f'perception provider returned HTTP {error.code}') from error
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


def recognition_identity(name, confidence=.5):
    if not 0 < confidence <= 1:
        raise ValueError('confidence must be within (0,1]')
    spec = model_spec(name)
    runtime = {'opencv': cv2.__version__, 'numpy': np.__version__}
    if spec['backend'] in TORCH_BACKENDS:
        runtime.update({package: importlib.metadata.version(package) for package in ['torch', 'torchvision', 'transformers', 'Pillow', 'safetensors', 'tokenizers']})
    # An open recognizer's labels are the per-frame query, recorded beside this identity in each observation;
    # the startup default is not an operating point of the model.
    artifacts = {key: value for key, value in spec.items() if key != 'default_query'}
    document = {'model': name, 'artifacts': artifacts, 'confidence_threshold': confidence, 'preprocessing_revision': 1,
                'labels': 'query' if spec['backend'] in OPEN_BACKENDS else 'native', 'runtime': runtime,
                'device': 'cpu', 'threads': 2, 'interop_threads': 1}
    if spec['backend'] in REMOTE_BACKENDS:
        # The artifacts pin the provider's own operating-point hash (weights, Torch/CUDA, GPU), which every
        # response must echo; the local runtime only encodes lossless PNG. The endpoint is not identity.
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
    def __init__(self, name='yolox', directory=None, confidence=.5):
        self.spec = model_spec(name)
        self.sha256, self.identity = recognition_identity(name, confidence)
        self.confidence = confidence
        self.open = self.spec['backend'] in OPEN_BACKENDS
        # Open recognizers report only the phrases of the query they ran; closed ones their native catalogue.
        self.default_query = make_query(self.spec['default_query']) if self.open else None
        self.labels = None
        cv2.setNumThreads(2)
        if self.spec['backend'] in REMOTE_BACKENDS:
            self.directory = None
            self.model = RemoteProvider(*provider_endpoint(self.spec['provider']))
            return
        self.directory = verify_model(directory or Path.home() / '.ros/lekiwi/models' / name, self.spec)
        if self.spec['backend'] == 'yolox':
            self.model = YoloX(str(self.directory / next(iter(self.spec['files']))), confThreshold=confidence)
            self.labels = list(COCO_LABELS)
        else:
            import torch
            from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
            from transformers import RTDetrImageProcessor, RTDetrV2ForObjectDetection
            torch.set_num_threads(2)
            if torch.get_num_interop_threads() != 1:
                torch.set_num_interop_threads(1)
            self.torch = torch
            if self.spec['backend'] == 'rtdetr_v2':
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

    def predict(self, image, query=None):
        """Open recognizers localize exactly `query`; closed ones cannot express a query and refuse one."""
        if self.open != (query is not None):
            raise ValueError('an open-vocabulary recognizer needs a query' if self.open else
                             'a closed-vocabulary recognizer cannot answer a query')
        height, width = image.shape[:2]
        if self.spec['backend'] in REMOTE_BACKENDS:
            return suppress(self._remote(image, query), self.spec['backend'], self.confidence)
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        if self.spec['backend'] == 'yolox':
            scale = min(640 / width, 640 / height)
            resized = cv2.resize(rgb, (max(1, int(width * scale)), max(1, int(height * scale))))
            padded = np.full((640, 640, 3), 114.0, dtype=np.float32)
            padded[:resized.shape[0], :resized.shape[1]] = resized
            predictions = self.model.infer(padded)
            return [[x / scale, y / scale, w / scale, h / scale, score, COCO_LABELS[int(label)]]
                    for x, y, w, h, score, label in predictions]
        from PIL import Image
        pixels = Image.fromarray(rgb)
        if self.spec['backend'] == 'rtdetr_v2':
            inputs = self.processor(images=pixels, return_tensors='pt')
        else:
            inputs = self.processor(images=pixels, text=query.prompt, return_tensors='pt')
        with self.torch.inference_mode():
            outputs = self.model(**inputs)
        if self.spec['backend'] == 'rtdetr_v2':
            result = self.processor.post_process_object_detection(
                outputs, target_sizes=[(height, width)], threshold=self.confidence)[0]
            labels = [self.model.config.id2label[int(label)] for label in result['labels']]
        else:
            result = self.processor.post_process_grounded_object_detection(
                outputs, inputs.input_ids, box_threshold=self.confidence, text_threshold=TEXT_THRESHOLD,
                target_sizes=[(height, width)])[0]
            labels = result['text_labels']
        rows = []
        for label, box, score in zip(labels, result['boxes'], result['scores']):
            x0, y0, x1, y1 = box.tolist()
            rows.append([x0, y0, x1 - x0, y1 - y0, float(score), str(label)])
        return suppress(rows, self.spec['backend'], self.confidence)

    def _remote(self, image, query):
        """Lossless pixels out; a response counts only if it proves the admitted operating point and query."""
        height, width = image.shape[:2]
        encoded, png = cv2.imencode('.png', image)
        if not encoded:
            raise ValueError('could not encode the capture for the perception provider')
        data = png.tobytes()
        labels = list(query.labels)
        expected = {'labels': labels, 'prompt': query.prompt,
                    'box_threshold': self.confidence, 'text_threshold': TEXT_THRESHOLD}
        response = self.model.infer({
            'model': self.spec['provider_model'], 'image': base64.b64encode(data).decode('ascii'),
            'labels': labels, 'box_threshold': self.confidence, 'text_threshold': TEXT_THRESHOLD,
            'deadline_ms': int(self.model.timeout * 1000)})
        source = response.get('image')
        if (response.get('schema') != 1 or response.get('model') != self.spec['provider_model']
                or response.get('model_sha256') != self.spec['provider_model_sha256']
                or response.get('effective') != expected or not isinstance(source, dict)
                or source.get('sha256') != hashlib.sha256(data).hexdigest()
                or (source.get('width'), source.get('height')) != (width, height)):
            raise ProviderUnavailable('perception provider answered outside the admitted operating point')
        detections = response.get('detections')
        if not isinstance(detections, list) or len(detections) > 900:
            raise ProviderUnavailable('perception provider returned malformed detections')
        rows = []
        for item in detections:
            box = item.get('box') if isinstance(item, dict) else None
            if (not isinstance(box, list) or len(box) != 4
                    or not all(type(value) in (int, float) and math.isfinite(value) for value in box)
                    or type(item.get('score')) not in (int, float) or not isinstance(item.get('label'), str)):
                raise ProviderUnavailable('perception provider returned malformed detections')
            x0, y0, x1, y1 = box
            rows.append([x0, y0, x1 - x0, y1 - y0, float(item['score']), item['label']])
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
    identity, operating = recognition_identity(settings['model_name'], settings['confidence_threshold'])
    deployment = yaml.safe_load((config_directory() / 'object_perception_deployment.yaml').read_text())
    described = {'model_sha256': identity, 'operating_point': operating,
                 'vocabulary': 'open' if open_vocabulary(settings['model_name']) else 'closed'}
    if open_vocabulary(settings['model_name']):
        described['default_query'] = make_query(model_spec(settings['model_name'])['default_query']).document()
    if settings.get('fallback_model_name'):
        # Frames served by the fallback carry its own identity; an admitting profile lists both.
        fallback, fallback_operating = recognition_identity(settings['fallback_model_name'], settings['confidence_threshold'])
        described.update(fallback_model_sha256=fallback, fallback_operating_point=fallback_operating)
    described['cameras'] = {name: value['sha256'] for name, value in camera_bindings(deployment['cameras']).items()}
    print(json.dumps(described, indent=2))


if __name__ == '__main__':
    main()
