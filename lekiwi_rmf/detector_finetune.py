"""Fine-tune an RT-DETR v2 detector on a frozen perception dataset; standalone, shipped into the training pod.

Imports only PyTorch, transformers, Pillow and safetensors, never ROS or the rest of this
package, so the same file runs in the cluster image and in the local perception
environment. `python detector_finetune.py JOB.json` reads the job written by
scripts/train-recognizer.py.

The vocabulary keeps every base label at its base class index and appends new labels after
them; the class rows of every head (encoder score head, each decoder layer's class head and
the denoising label embedding) are copied from the base weights for the old indices, so the
base catalogue and decoder survive and only the new rows start untrained. Previously
qualified labels are still measured, never assumed, by the regression suites.

Training is deterministic for a seed: the batch order of each epoch and the flip decisions
come from generators seeded by (seed, epoch). A checkpoint every `checkpoint_every` steps
holds the model, optimizer and generator states and the metrics up to that step; a resumed
run restores exactly that state and rewrites metrics.jsonl from it, so no step is logged twice.
"""

import argparse
import importlib.metadata
import json
import math
import os
from pathlib import Path
import random
import shutil
import time


def load_job(path):
    job = json.loads(Path(path).read_text())
    for key in ('base', 'dataset', 'output', 'checkpoints', 'metrics', 'compute', 'vocabulary', 'base_labels',
                'input_size', 'config', 'device'):
        if key not in job:
            raise ValueError(f'job is missing {key}')
    return job


def examples(dataset, vocabulary, split='development'):
    """(image path, class ids, normalized cx/cy/w/h boxes) of every scene in `split`."""
    import PIL.Image
    index = {label: position for position, label in enumerate(vocabulary)}
    result = []
    for scene in json.loads((Path(dataset) / 'scenes.json').read_text()):
        if scene['split'] != split:
            continue
        path = Path(dataset) / scene['image']
        with PIL.Image.open(path) as image:
            width, height = image.size
        classes, boxes = [], []
        for item in scene['objects']:
            x, y, w, h = item['bbox']
            classes.append(index[item['label']])
            boxes.append([(x + w / 2) / width, (y + h / 2) / height, w / width, h / height])
        result.append((path, classes, boxes))
    if not result:
        raise ValueError(f'the dataset has no {split} scenes')
    return result


def extend_classes(state, base, old, new):
    """Copy the base rows of every class-indexed tensor into the extended model's state.

    A tensor is class-indexed when its first dimension is the class count (score heads) or the
    class count plus one (the denoising embedding, whose last row is the padding label). Old
    classes keep their base rows; the padding row moves to the new end.
    """
    copied = []
    for name, tensor in state.items():
        source = base.get(name)
        if source is None or source.shape == tensor.shape:
            continue
        if source.shape[1:] != tensor.shape[1:]:
            raise ValueError(f'{name}: base {tuple(source.shape)} cannot extend to {tuple(tensor.shape)}')
        if source.shape[0] == old and tensor.shape[0] == new:
            tensor[:old] = source
        elif source.shape[0] == old + 1 and tensor.shape[0] == new + 1:
            tensor[:old] = source[:old]
            tensor[new] = source[old]
        else:
            raise ValueError(f'{name}: unexpected class dimension {tuple(source.shape)} -> {tuple(tensor.shape)}')
        copied.append(name)
    return copied


def build_model(job):
    import torch
    from safetensors.torch import load_file
    from transformers import AutoConfig, RTDetrV2ForObjectDetection
    vocabulary, old = job['vocabulary'], job['base_labels']
    config = AutoConfig.from_pretrained(job['base'], local_files_only=True)
    base_ids = [config.id2label[index] for index in sorted(config.id2label)]
    if len(base_ids) != old or vocabulary[:old] != base_ids:
        raise ValueError('the vocabulary must keep every base label at its base index')
    config.num_labels = len(vocabulary)
    config.id2label = dict(enumerate(vocabulary))
    config.label2id = {label: index for index, label in enumerate(vocabulary)}
    model = RTDetrV2ForObjectDetection.from_pretrained(job['base'], config=config, local_files_only=True,
                                                       use_safetensors=True, ignore_mismatched_sizes=True)
    copied = []
    if len(vocabulary) != old:
        state = model.state_dict()
        with torch.no_grad():
            copied = extend_classes(state, load_file(str(Path(job['base']) / 'model.safetensors')), old,
                                    len(vocabulary))
    return model, copied


def batches(items, size, seed, epoch, flip):
    order = list(range(len(items)))
    generator = random.Random(f'{seed}:{epoch}')
    generator.shuffle(order)
    for start in range(0, len(order), size):
        yield [(items[index], flip and generator.random() < .5) for index in order[start:start + size]]


def prepare(batch, processor, device):
    import PIL.Image
    import torch
    images, labels = [], []
    for (path, classes, boxes), flipped in batch:
        with PIL.Image.open(path) as source:
            image = source.convert('RGB')
        if flipped:
            image = image.transpose(PIL.Image.Transpose.FLIP_LEFT_RIGHT)
            boxes = [[1 - cx, cy, w, h] for cx, cy, w, h in boxes]
        images.append(image)
        labels.append({'class_labels': torch.tensor(classes, dtype=torch.long, device=device),
                       'boxes': torch.tensor(boxes, dtype=torch.float32, device=device).reshape(-1, 4)})
    pixels = processor(images=images, return_tensors='pt')['pixel_values'].to(device)
    return pixels, labels


def save_checkpoint(directory, step, model, optimizer, metrics):
    import torch
    target = Path(directory) / f'step-{step:07d}'
    partial = target.with_name(target.name + '.partial')
    shutil.rmtree(partial, ignore_errors=True)
    partial.mkdir(parents=True)
    torch.save({'step': step, 'model': model.state_dict(), 'optimizer': optimizer.state_dict(),
                'torch_rng': torch.get_rng_state(),
                'cuda_rng': torch.cuda.get_rng_state_all() if torch.cuda.is_available() else None}, partial / 'state.pt')
    (partial / 'metrics.jsonl').write_text(''.join(json.dumps(line, sort_keys=True) + '\n' for line in metrics))
    shutil.rmtree(target, ignore_errors=True)
    os.replace(partial, target)
    (Path(directory) / 'latest.tmp').write_text(target.name)
    os.replace(Path(directory) / 'latest.tmp', Path(directory) / 'latest')
    for older in Path(directory).glob('step-*'):
        if older.name != target.name:
            shutil.rmtree(older, ignore_errors=True)


def latest_checkpoint(directory):
    pointer = Path(directory) / 'latest' if directory else None
    if pointer is None or not pointer.is_file():
        return None
    return Path(directory) / pointer.read_text().strip()


def train(job):
    import torch
    import transformers
    from transformers import RTDetrImageProcessor
    config, device = job['config'], job['device']
    torch.manual_seed(config['seed'])
    model, copied = build_model(job)
    model.to(device).train()
    # The runtime's own processor at the runtime's input size (lekiwi_rmf/object_recognition.py).
    processor = RTDetrImageProcessor.from_pretrained(job['base'], local_files_only=True,
                                                     size={'height': job['input_size'], 'width': job['input_size']})
    backbone = [p for name, p in model.named_parameters() if 'backbone' in name]
    rest = [p for name, p in model.named_parameters() if 'backbone' not in name]
    optimizer = torch.optim.AdamW([{'params': rest, 'lr': config['learning_rate']},
                                   {'params': backbone, 'lr': config['backbone_learning_rate']}],
                                  weight_decay=config['weight_decay'])
    items = examples(job['dataset'], job['vocabulary'])
    step, metrics = 0, []
    resume = latest_checkpoint(job.get('resume_from')) or latest_checkpoint(job['checkpoints'])
    if resume is not None:
        state = torch.load(resume / 'state.pt', map_location=device, weights_only=False)
        model.load_state_dict(state['model'])
        optimizer.load_state_dict(state['optimizer'])
        torch.set_rng_state(state['torch_rng'])
        if state['cuda_rng'] is not None and torch.cuda.is_available():
            torch.cuda.set_rng_state_all(state['cuda_rng'])
        step = state['step']
        metrics = [json.loads(line) for line in (resume / 'metrics.jsonl').read_text().splitlines() if line]
        print(f'resumed at step {step} from {resume}', flush=True)
    Path(job['checkpoints']).mkdir(parents=True, exist_ok=True)
    log = Path(job['metrics'])
    log.write_text(''.join(json.dumps(line, sort_keys=True) + '\n' for line in metrics))
    per_epoch = math.ceil(len(items) / config['batch_size'])
    started = time.monotonic()
    budget = job.get('budget_sec')
    with log.open('a') as stream:
        while step < config['steps']:
            epoch, offset = divmod(step, per_epoch)
            for batch in list(batches(items, config['batch_size'], config['seed'], epoch, config['horizontal_flip']))[offset:]:
                if step >= config['steps']:
                    break
                pixels, labels = prepare(batch, processor, device)
                outputs = model(pixel_values=pixels, labels=labels)
                loss = outputs.loss
                if not torch.isfinite(loss):
                    raise RuntimeError(f'non-finite loss at step {step + 1}')
                optimizer.zero_grad(set_to_none=True)
                loss.backward()
                torch.nn.utils.clip_grad_norm_(model.parameters(), 0.1)
                optimizer.step()
                step += 1
                line = {'step': step, 'epoch': epoch, 'loss': round(float(loss), 6),
                        **{f'loss_{key}': round(float(value), 6) for key, value in (outputs.loss_dict or {}).items()}}
                metrics.append(line)
                stream.write(json.dumps(line, sort_keys=True) + '\n')
                stream.flush()
                if step % config['checkpoint_every'] == 0 or step == config['steps']:
                    save_checkpoint(job['checkpoints'], step, model, optimizer, metrics)
                if budget is not None and time.monotonic() - started > budget:
                    save_checkpoint(job['checkpoints'], step, model, optimizer, metrics)
                    raise TimeoutError(f'training budget of {budget:.0f} s exhausted at step {step}')
    model.eval().to('cpu')
    output = Path(job['output'])
    shutil.rmtree(output, ignore_errors=True)
    model.save_pretrained(output, safe_serialization=True)
    # The processor is the base's, byte for byte: preprocessing is part of the artifact identity.
    shutil.copy(Path(job['base']) / 'preprocessor_config.json', output / 'preprocessor_config.json')
    compute = {'elapsed_sec': round(time.monotonic() - started, 1), 'steps': step, 'torch': torch.__version__,
               'transformers': transformers.__version__, 'pillow': importlib.metadata.version('Pillow'),
               'scipy': importlib.metadata.version('scipy'), 'cuda': torch.version.cuda, 'device_name': 'cpu',
               'extended_tensors': copied, 'training_scenes': len(items)}
    if device == 'cuda':
        compute.update(device_name=torch.cuda.get_device_name(0),
                       capability='sm_%d%d' % torch.cuda.get_device_capability(0),
                       peak_gpu_memory_mib=round(torch.cuda.max_memory_allocated() / 2**20))
    Path(job['compute']).write_text(json.dumps(compute, sort_keys=True))
    return compute


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('job')
    job = load_job(parser.parse_args().job)
    try:
        train(job)
    except TimeoutError as error:
        print(error, flush=True)
        raise SystemExit(13)


if __name__ == '__main__':
    main()
