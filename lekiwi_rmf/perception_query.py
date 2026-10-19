"""The active open-vocabulary query: bounded, normalized labels with a content revision.

A query is not configuration. It is requested at runtime, travels with every frame it produced and
forms part of that inference's operating point, so the same capture under another query is another
inference and replay can rebuild the exact prompt from the observation alone.
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
class Query:
    labels: tuple

    @property
    def prompt(self):
        return '. '.join(self.labels) + '.'

    @property
    def revision(self):
        canonical = json.dumps({'schema': 1, 'labels': list(self.labels)}, separators=(',', ':'), ensure_ascii=False)
        return hashlib.sha256(canonical.encode()).hexdigest()

    def document(self):
        return {'revision': self.revision, 'labels': list(self.labels), 'prompt': self.prompt}


def make_query(labels):
    """Normalize, de-duplicate in request order and bound an untrusted label list."""
    if not isinstance(labels, (list, tuple)) or not 1 <= len(labels) <= MAX_LABELS:
        raise ValueError(f'a query needs 1..{MAX_LABELS} labels')
    query = Query(tuple(dict.fromkeys(normalize_label(label) for label in labels)))
    if len(query.prompt) > MAX_PROMPT_CHARS:
        raise ValueError(f'query prompt exceeds {MAX_PROMPT_CHARS} characters')
    return query


def recorded_query(labels):
    """A query as a frame carried it; anything not already canonical was not produced by this detector."""
    query = make_query(labels)
    if list(query.labels) != list(labels):
        raise ValueError('frame query labels are not canonical')
    return query


def limits():
    return {'max_labels': MAX_LABELS, 'max_label_chars': MAX_LABEL_CHARS, 'max_prompt_chars': MAX_PROMPT_CHARS}
