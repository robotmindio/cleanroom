"""Per-stage latency budgets: tracked configuration, percentiles, the control age gate and bounded diagnostics."""

import json

from diagnostic_msgs.msg import DiagnosticStatus
import pytest
import yaml

from lekiwi_rmf.latency_budget import QUANTILES, LatencyBudget, load_budgets, quantile
from lekiwi_rmf.object_cameras import config_directory


def tracked(name, node):
    return yaml.safe_load((config_directory() / name).read_text())[node]['ros__parameters']


def values(status):
    return {item.key: json.loads(item.value) for item in status.values}


def test_tracked_budgets_cover_every_instrumented_operation_within_their_age_gates():
    budgets = load_budgets()
    assert set(budgets) == {'perception', 'skill_control', 'navigation', 'task'}
    # A frame at budget in every stage is still fresh for the detector's own gate.
    assert sum(budgets['perception'].values()) <= tracked('object_perception.yaml', 'object_detector')['max_source_age_sec']
    # The observation plus the policy's chunk must fit the skill runner's default 0.5 s control age.
    assert budgets['skill_control']['sensor_age'] + budgets['skill_control']['inference'] <= .5
    explorer = yaml.safe_load((config_directory() / 'exploration.yaml').read_text())['robot_explorer']['ros__parameters']
    assert budgets['navigation']['execution'] <= explorer['navigation_timeout_sec']
    assert budgets['navigation']['planning'] <= explorer['service_timeout_sec']


@pytest.mark.parametrize('document', [
    {}, {'operations': [], 'extra': 1}, {'operations': {'Bad': {'queue': 1}}}, {'operations': {'x': {}}},
    {'operations': {'x': {'thinking': 1}}}, {'operations': {'x': {'queue': 0}}}, {'operations': {'x': {'queue': 3601}}},
    {'operations': {'x': {'queue': '1'}}}, {'operations': {'x': {'queue': float('nan')}}}])
def test_malformed_budgets_reject_the_whole_file(tmp_path, document):
    path = tmp_path / 'budgets.yaml'
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ValueError):
        load_budgets(path)


def test_quantiles_are_nearest_rank():
    ordered = [float(value) for value in range(1, 101)]
    assert [quantile(ordered, fraction) for _, fraction in QUANTILES] == [50., 95., 99.]
    assert quantile([3.], .99) == 3. and quantile([], .5) is None


def test_percentiles_warn_only_above_the_stage_budget():
    budget = LatencyBudget('demo', budgets={'demo': {'queue': 1., 'inference': 2.}})
    for value in range(100):
        budget.record('queue', value / 100)       # p95 0.94 s, within 1 s
        budget.record('inference', value / 10)    # p95 9.4 s, above 2 s
    summary = budget.summary()
    assert summary['stages']['queue']['p95_sec'] == pytest.approx(.94)
    assert summary['stages']['inference']['p99_sec'] == pytest.approx(9.8)
    assert summary['over_budget'] == ['inference']
    status = budget.status()
    assert status.level == DiagnosticStatus.WARN and 'inference p95' in status.message


def test_late_answers_are_counted_and_refused_for_control():
    budget = LatencyBudget('demo', max_age_sec=.5, budgets={'demo': {'sensor_age': 1.}})
    assert budget.fresh(.2) and budget.fresh(.5)
    assert not budget.fresh(.6) and not budget.fresh(-.1) and not budget.fresh(float('nan'))
    summary = budget.summary()
    assert (summary['fresh'], summary['late'], summary['late_in_window']) == (2, 3, 3)
    assert budget.status().level == DiagnosticStatus.WARN
    ungated = LatencyBudget('demo', budgets={'demo': {'sensor_age': 1.}})
    assert ungated.fresh(1e6) and ungated.status().level == DiagnosticStatus.OK


def test_clock_faults_do_not_lower_percentiles_and_unknown_stages_are_refused():
    budget = LatencyBudget('demo', budgets={'demo': {'sensor_age': 1.}})
    budget.record('sensor_age', .4)
    for bad in (-1., float('inf'), float('nan'), None):
        budget.record('sensor_age', bad)
    summary = budget.summary()
    assert summary['invalid'] == 4 and summary['stages']['sensor_age']['p50_sec'] == .4
    with pytest.raises(ValueError):
        budget.record('network', .1)
    with pytest.raises(ValueError):
        LatencyBudget('unknown', budgets={'demo': {'queue': 1.}})
    with pytest.raises(ValueError):
        LatencyBudget('demo', max_age_sec=0, budgets={'demo': {'queue': 1.}})


def test_samples_and_diagnostic_keys_stay_bounded():
    budget = LatencyBudget('perception', 2.)
    for value in range(5000):
        budget.record('queue', value)
        budget.fresh(value % 3)
    summary = budget.summary()
    assert summary['stages']['queue']['count'] == 512 and summary['window'] == 512
    status = budget.status()
    assert status.name == 'lekiwi/budget/perception' and status.hardware_id == 'lekiwi'
    counters = {'max_age_sec', 'fresh', 'late', 'late_in_window', 'invalid', 'over_budget'}
    # Keys depend only on the tracked stages, never on what was measured.
    assert set(values(status)) - counters == {f'{stage}_{suffix}' for stage in load_budgets()['perception']
                          for suffix in ('count', 'budget_p95_sec', 'p50_sec', 'p95_sec', 'p99_sec')}
    assert values(status)['max_age_sec'] == 2.
