"""Pinned local recognizers with one source-pixel prediction contract."""

import hashlib
import importlib.metadata
import json
import os
import sys
import re
from pathlib import Path

import cv2
import numpy as np
import yaml

from lekiwi_rmf.object_cameras import config_directory
from lekiwi_rmf.opencv_yolox import YoloX


def model_spec(name):
    configured = yaml.safe_load((config_directory() / 'recognition_models.yaml').read_text())
    if name not in configured:
        raise ValueError(f'unknown pinned recognizer: {name}')
    return configured[name]


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
    if spec['backend'] != 'yolox':
        runtime.update({package: importlib.metadata.version(package) for package in ['torch', 'torchvision', 'transformers', 'Pillow', 'safetensors', 'tokenizers']})
    document = {'model': name, 'artifacts': spec, 'confidence_threshold': confidence,
                'preprocessing_revision': 1, 'runtime': runtime, 'device': 'cpu', 'threads': 2, 'interop_threads': 1}
    identity = hashlib.sha256(json.dumps(document, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
    return identity, document


class Recognizer:
    def __init__(self, name='yolox', directory=None, confidence=.5):
        self.spec = model_spec(name)
        self.sha256, self.identity = recognition_identity(name, confidence)
        self.directory = verify_model(directory or Path.home() / '.ros/lekiwi/models' / name, self.spec)
        self.confidence = confidence
        cv2.setNumThreads(2)
        if self.spec['backend'] == 'yolox':
            self.model = YoloX(str(self.directory / next(iter(self.spec['files']))), confThreshold=confidence)
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
            else:
                self.processor = AutoProcessor.from_pretrained(self.directory, local_files_only=True)
                self.model = AutoModelForZeroShotObjectDetection.from_pretrained(
                    self.directory, local_files_only=True, use_safetensors=True).eval().to('cpu')

    def predict(self, image):
        height, width = image.shape[:2]
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        if self.spec['backend'] == 'yolox':
            scale = min(640 / width, 640 / height)
            resized = cv2.resize(rgb, (max(1, int(width * scale)), max(1, int(height * scale))))
            padded = np.full((640, 640, 3), 114.0, dtype=np.float32)
            padded[:resized.shape[0], :resized.shape[1]] = resized
            predictions = self.model.infer(padded)
            return [[x / scale, y / scale, w / scale, h / scale, score, 41]
                    for x, y, w, h, score, label in predictions if int(label) == 41]
        from PIL import Image
        pixels = Image.fromarray(rgb)
        if self.spec['backend'] == 'rtdetr_v2':
            inputs = self.processor(images=pixels, return_tensors='pt')
        else:
            inputs = self.processor(images=pixels, text=self.spec['prompt'], return_tensors='pt')
        with self.torch.inference_mode():
            outputs = self.model(**inputs)
        if self.spec['backend'] == 'rtdetr_v2':
            result = self.processor.post_process_object_detection(
                outputs, target_sizes=[(height, width)], threshold=self.confidence)[0]
            labels = [self.model.config.id2label[int(label)] for label in result['labels']]
        else:
            result = self.processor.post_process_grounded_object_detection(
                outputs, inputs.input_ids, box_threshold=self.confidence, text_threshold=.25,
                target_sizes=[(height, width)])[0]
            labels = result['text_labels']
        rows = []
        for label, box, score in zip(labels, result['boxes'], result['scores']):
            if not any(word in re.findall(r'[a-z]+', str(label).lower()) for word in ['cup', 'mug']):
                continue
            x0, y0, x1, y1 = box.tolist()
            rows.append([x0, y0, x1 - x0, y1 - y0, float(score), 41])
        # Synonymous open-vocabulary prompts can propose the same cup twice.
        if rows:
            selected = cv2.dnn.NMSBoxes([r[:4] for r in rows], [r[4] for r in rows], self.confidence, .5)
            rows = [rows[int(index)] for index in np.asarray(selected).reshape(-1)]
        return rows


def model_runtime(name):
    return (Path.home() / '.ros/lekiwi/perception-venv/bin/python') if name != 'yolox' else Path(sys.executable)


def use_model_runtime(name):
    if name != 'yolox' and Path(sys.prefix) != model_runtime(name).parent.parent:
        python = model_runtime(name)
        if not python.exists():
            raise ValueError('install the selected model with scripts/install-object-model.py first')
        os.execve(python, [str(python), str(Path(sys.argv[0]).resolve()), *sys.argv[1:]],
                  {**os.environ, 'PYTHONNOUSERSITE': '1'})


def main():
    from lekiwi_rmf.object_cameras import camera_bindings
    settings = yaml.safe_load((config_directory() / 'object_perception.yaml').read_text())['/**']['ros__parameters']
    use_model_runtime(settings['model_name'])
    identity, operating = recognition_identity(settings['model_name'], settings['confidence_threshold'])
    deployment = yaml.safe_load((config_directory() / 'object_perception_deployment.yaml').read_text())
    print(json.dumps({'model_sha256': identity, 'operating_point': operating,
                      'cameras': {name: value['sha256'] for name, value in camera_bindings(deployment['cameras']).items()}}, indent=2))


if __name__ == '__main__':
    main()
