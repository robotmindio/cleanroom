"""Per-stage latency budgets: bounded samples, p50/p95/p99 and the age gate in front of control.

Every operation that turns sensor data into robot action records how long each
stage took. Stages are a closed vocabulary and operations are the keys of the
tracked config/latency_budgets.yaml, so diagnostic names and keys stay bounded;
task, goal and entity IDs belong in journals and logs, never in a status name.

The age gate is the operation's own tracked freshness limit (for example the
detector's ``max_source_age_sec``). A result older than that limit may be kept as
history by its owner, but `fresh()` is False and the caller must not let it feed
control.
"""

from collections import deque
import json
import math
from pathlib import Path
import re
import threading

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
import yaml

from lekiwi_rmf.object_cameras import config_directory

STAGES = ('sensor_age', 'queue', 'inference', 'network', 'planning', 'execution')
QUANTILES = (('p50', .50), ('p95', .95), ('p99', .99))
OPERATION = re.compile(r'^[a-z][a-z_]{0,31}$')
# Samples per stage: enough for a stable p99 at a few hertz, small enough to sort every second.
WINDOW = 512
MAX_BUDGET_SEC = 3600.


def load_budgets(path=None):
    """The tracked p95 budget of every stage each operation measures; one bad entry rejects the file."""
    path = Path(path) if path else config_directory() / 'latency_budgets.yaml'
    document = yaml.safe_load(path.read_text())
    if not isinstance(document, dict) or set(document) != {'operations'} or not isinstance(document['operations'], dict):
        raise ValueError('latency budgets must be a mapping with only `operations`')
    budgets = {}
    for operation, stages in document['operations'].items():
        if not isinstance(operation, str) or not OPERATION.match(operation):
            raise ValueError(f'invalid operation name {operation!r}')
        if not isinstance(stages, dict) or not stages or set(stages) - set(STAGES):
            raise ValueError(f'{operation}: stages must be a non-empty subset of {STAGES}')
        for stage, seconds in stages.items():
            if type(seconds) not in (int, float) or not math.isfinite(seconds) or not 0 < seconds <= MAX_BUDGET_SEC:
                raise ValueError(f'{operation}.{stage}: budget must be in (0, {MAX_BUDGET_SEC}] seconds')
        budgets[operation] = {stage: float(stages[stage]) for stage in STAGES if stage in stages}
    return budgets


def quantile(ordered, fraction):
    """Nearest-rank quantile of an ascending sequence; None without samples."""
    return ordered[max(0, math.ceil(fraction * len(ordered)) - 1)] if ordered else None


class LatencyBudget:
    """Thread-safe stage samples for one operation and the count of answers too late for control."""

    def __init__(self, operation, max_age_sec=None, budgets=None, window=WINDOW):
        budgets = load_budgets() if budgets is None else budgets
        if operation not in budgets:
            raise ValueError(f'{operation} has no tracked latency budget')
        if max_age_sec is not None and (type(max_age_sec) not in (int, float) or not math.isfinite(max_age_sec)
                                        or max_age_sec <= 0):
            raise ValueError('max_age_sec must be positive and finite')
        self.operation, self.max_age_sec, self.budgets = operation, max_age_sec, budgets[operation]
        self._lock = threading.Lock()
        self._samples = {stage: deque(maxlen=window) for stage in self.budgets}
        self._recent = deque(maxlen=window)
        self._counts = {'fresh': 0, 'late': 0, 'invalid': 0}

    def record(self, stage, seconds):
        if stage not in self._samples:
            raise ValueError(f'{self.operation} does not measure {stage}')
        with self._lock:
            if type(seconds) not in (int, float) or not math.isfinite(seconds) or seconds < 0:
                # A negative age is a clock fault, not a fast stage; it must not lower the percentiles.
                self._counts['invalid'] += 1
                return
            self._samples[stage].append(float(seconds))

    def fresh(self, age_sec):
        """Whether a result this old may feed control; every answer is counted either way."""
        late = (not isinstance(age_sec, (int, float)) or not math.isfinite(age_sec) or age_sec < 0
                or (self.max_age_sec is not None and age_sec > self.max_age_sec))
        with self._lock:
            self._counts['late' if late else 'fresh'] += 1
            self._recent.append(late)
        return not late

    def summary(self):
        with self._lock:
            samples = {stage: sorted(values) for stage, values in self._samples.items()}
            counts, recent = dict(self._counts), list(self._recent)
        stages = {}
        for stage, ordered in samples.items():
            stages[stage] = {'count': len(ordered), 'budget_p95_sec': self.budgets[stage],
                             **{f'{name}_sec': quantile(ordered, fraction) for name, fraction in QUANTILES}}
        over = [stage for stage, value in stages.items()
                if value['p95_sec'] is not None and value['p95_sec'] > value['budget_p95_sec']]
        return {'operation': self.operation, 'max_age_sec': self.max_age_sec, **counts,
                'late_in_window': sum(recent), 'window': len(recent), 'stages': stages, 'over_budget': over}

    def status(self):
        """One bounded DiagnosticStatus: `lekiwi/budget/<operation>` with `<stage>_<quantile>_sec` keys."""
        summary = self.summary()
        late = summary['late_in_window']
        reasons = [f'{stage} p95 above its budget' for stage in summary['over_budget']]
        if late:
            reasons.append(f'{late} of the last {summary["window"]} answers exceeded the control age limit')
        status = DiagnosticStatus(name=f'lekiwi/budget/{self.operation}', hardware_id='lekiwi',
                                  level=DiagnosticStatus.WARN if reasons else DiagnosticStatus.OK,
                                  message='; '.join(reasons) or 'within budget')
        values = {'max_age_sec': summary['max_age_sec'], 'fresh': summary['fresh'], 'late': summary['late'],
                  'late_in_window': late, 'invalid': summary['invalid'], 'over_budget': summary['over_budget']}
        for stage, value in summary['stages'].items():
            values[f'{stage}_count'] = value['count']
            values[f'{stage}_budget_p95_sec'] = value['budget_p95_sec']
            for name, _ in QUANTILES:
                values[f'{stage}_{name}_sec'] = value[f'{name}_sec']
        status.values = [KeyValue(key=key, value=json.dumps(value)) for key, value in values.items()]
        return status
