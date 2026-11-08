#!/usr/bin/env python3
"""Measure live perception and injected provider loss without publishing into the robot's domain.

Source scripts/setup.bash first. Real front/wrist images are relayed unchanged
from --source-domain into --test-domain. Only this detector sees the injected
ProviderUnavailable; the real provider, running stack and observation store
are unchanged. This measures failover, not a network/server outage.
"""

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from lekiwi_rmf.object_recognition import ProviderUnavailable, use_model_runtime


class ProviderLoss:
    def __init__(self, provider):
        self.provider = provider
        self.unavailable = threading.Event()

    def infer(self, body):
        if self.unavailable.is_set():
            raise ProviderUnavailable('qualification: injected provider loss')
        return self.provider.infer(body)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-domain', type=int, default=0)
    parser.add_argument('--test-domain', type=int, default=222)
    parser.add_argument('--phase-sec', type=float, default=60.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if (not 0 <= args.source_domain <= 232 or not 101 <= args.test_domain <= 230
            or args.source_domain == args.test_domain or not math.isfinite(args.phase_sec)
            or not 30 <= args.phase_sec <= 300):
        parser.error('use distinct domains, test domain 101..230, and phase duration 30..300 seconds')
    import yaml
    from lekiwi_rmf.object_cameras import camera_bindings, config_directory
    settings = yaml.safe_load((config_directory() / 'object_perception.yaml').read_text())
    parameters = settings['/**']['ros__parameters'] | settings['object_detector']['ros__parameters']
    use_model_runtime(parameters['model_name'], parameters['fallback_model_name'])

    import rclpy
    from rclpy.context import Context
    from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from lekiwi_rmf.latency_budget import LatencyBudget
    from lekiwi_rmf.object_detector import ObjectDetector

    source_context, test_context = Context(), Context()
    rclpy.init(context=source_context, domain_id=args.source_domain)
    rclpy.init(context=test_context, domain_id=args.test_domain)
    source = Node('perception_latency_source', context=source_context)
    relay = Node('perception_latency_relay', context=test_context)
    detector = None
    source_executor = SingleThreadedExecutor(context=source_context)
    executor = MultiThreadedExecutor(num_threads=4, context=test_context)
    executor.add_node(relay)
    report = {'source_domain': args.source_domain, 'test_domain': args.test_domain,
              'injection': 'process-local ProviderUnavailable; no server or network interruption',
              'settings': parameters, 'phases': {}, 'passed': False}
    worker = None
    phase_lock = threading.Lock()
    phase = 'warmup'
    budgets = defaultdict(lambda: LatencyBudget('perception', parameters['max_source_age_sec'], window=4096))

    class PhaseBudget(LatencyBudget):
        def record(self, stage, seconds):
            super().record(stage, seconds)
            with phase_lock:
                budgets[phase].record(stage, seconds)

        def fresh(self, age_sec):
            admitted = super().fresh(age_sec)
            with phase_lock:
                budgets[phase].fresh(age_sec)
            return admitted

    try:
        # Allow discovery before creating any input or inference publisher.
        until = time.monotonic() + 2
        while time.monotonic() < until:
            executor.spin_once(timeout_sec=.1)
        if relay.get_node_names() != ['perception_latency_relay']:
            raise RuntimeError('test domain already contains nodes; choose an unused isolated domain')
        bindings = camera_bindings(['front', 'wrist'])
        for binding in bindings.values():
            publisher = relay.create_publisher(Image, binding['image_topic'], qos_profile_sensor_data)
            source.create_subscription(Image, binding['image_topic'], publisher.publish, qos_profile_sensor_data)
        detector = ObjectDetector(context=test_context, parameter_overrides=[
            Parameter(key, value=value) for key, value in (parameters | {'camera_ids': list(bindings)}).items()])
        detector._budget = PhaseBudget('perception', parameters['max_source_age_sec'])
        loss = ProviderLoss(detector._detector.model)
        detector._detector.model = loss
        report['models'] = {role: recognizer.sha256 for role, recognizer in detector._recognizers.items()}
        executor.add_node(detector)
        source_executor.add_node(source)
        worker = threading.Thread(target=executor.spin, daemon=True)
        worker.start()

        def spin_until(deadline, ready=None):
            while time.monotonic() < deadline:
                source_executor.spin_once(timeout_sec=.1)
                if not worker.is_alive():
                    raise RuntimeError('isolated detector executor stopped unexpectedly')
                if ready and ready():
                    return True
            return False

        def stats():
            with detector._frame_lock:
                return {camera: dict(values) for camera, values in detector._stats.items()}

        if not spin_until(time.monotonic() + 90, lambda: all(row['published'] > 0 for row in stats().values())):
            raise RuntimeError('both live cameras did not publish fresh detections during warmup')
        for name in ('normal', 'provider_loss', 'recovery'):
            loss.unavailable.set() if name == 'provider_loss' else loss.unavailable.clear()
            # Keep in-flight replies and retry backoff out of the measured phase.
            with phase_lock:
                phase = 'transition'
            spin_until(time.monotonic() + 12)
            before = stats()
            with phase_lock:
                phase = name
            started = time.monotonic()
            spin_until(started + args.phase_sec)
            with phase_lock:
                phase = 'transition'
                measured = budgets[name].summary()
            after = stats()
            counts = {camera: {key: after[camera][key] - before[camera][key]
                               for key in ('received', 'published', 'rejected', 'fallback_published')}
                      for camera in bindings}
            fresh_outputs = all(row['published'] > 0 for row in counts.values())
            role_ok = all((row['fallback_published'] == row['published'] if name == 'provider_loss'
                           else row['published'] > row['fallback_published']) for row in counts.values())
            report['phases'][name] = {'elapsed_sec': time.monotonic() - started, 'counts': counts,
                                     'latency': measured, 'passed': fresh_outputs and role_ok}
            print(json.dumps({'phase': name, 'counts': counts, 'passed': fresh_outputs and role_ok}), flush=True)
        report['passed'] = all(row['passed'] for row in report['phases'].values())
    finally:
        executor.shutdown(timeout_sec=15)
        if worker:
            worker.join(timeout=15)
        source_executor.shutdown()
        if detector:
            detector.destroy_node()
        relay.destroy_node()
        source.destroy_node()
        rclpy.try_shutdown(context=test_context)
        rclpy.try_shutdown(context=source_context)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'passed': report['passed'], 'output': str(args.output)}))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
