#!/usr/bin/env python3
"""Learn a new category from taught gallery views by exemplar embedding; the detector is not retrained.

The manifest lists byte-exact views (for example Knowledge appearance-gallery region observations and
their source artifacts):
  {"name": "lata", "prompts": ["a soda can"], "encoder": "siglip2-base-patch16-224",
   "views": [{"image": "a.jpg", "artifact_sha256": "...", "region": [x, y, w, h],
              "polarity": "positive", "split": "teach", "source": "knowledge:<region observation IRI>"}]}
Held-out positives and negatives calibrate the match; the learned concept is written to the concept store
and named by its revision in /robot/perception/set_query (concept_revisions).
"""

import argparse
import base64
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lekiwi_rmf.object_recognition import RemoteProvider, provider_endpoint
from lekiwi_rmf.perception_concepts import STORE, learn_concept


def load_manifest(path):
    path = Path(path)
    if path.stat().st_size > 1024 * 1024:
        raise ValueError('manifest exceeds 1 MiB')
    manifest = json.loads(path.read_text())
    if not isinstance(manifest, dict) or set(manifest) != {'name', 'prompts', 'encoder', 'views'}:
        raise ValueError('manifest requires name, prompts, encoder and views')
    for view in manifest['views']:
        image = Path(view.pop('image'))
        image = image if image.is_absolute() else path.parent / image
        if image.stat().st_size > 8 * 1024**2:
            raise ValueError('a taught view exceeds 8 MiB')
        view['image'] = image.read_bytes()
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest')
    parser.add_argument('--provider', default='robotmind_gpu')
    parser.add_argument('--store', default=str(STORE))
    args = parser.parse_args()
    url, timeout = provider_endpoint(args.provider)
    provider = RemoteProvider(url, timeout)

    def embed(model, crops):
        return provider.post('/v1/embed', {'model': model, 'images': [base64.b64encode(crop).decode('ascii') for crop in crops],
                                           'deadline_ms': 10000}, timeout=15)
    record = learn_concept(load_manifest(args.manifest), embed, store=args.store)
    print(json.dumps({'revision': record['concept']['revision'], 'name': record['concept']['name'],
                      'match': record['concept']['match'], 'evaluation': record['evaluation']}))


if __name__ == '__main__':
    main()
