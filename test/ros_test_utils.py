import signal
import subprocess
import time

import rclpy


def spin_until(node, predicate, timeout: float = 5.0, period: float = 0.05) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=period)
        if predicate():
            return True
    return False


def release_graph(*, executor, thread=None, finished=None, waitables=(), nodes=(), context=None, processes=()):
    """Tear a test graph down completely, even after a failure, then report what failed.

    One test process runs every suite on one ROS domain. A step that raises
    part-way would leave action servers, subprocesses or an executor thread
    answering the next test's goals, so every step runs and the first error is
    raised only at the end.
    """
    errors = []

    def attempt(step):
        try:
            step()
        except Exception as error:  # noqa: BLE001 - keep releasing; the first error is re-raised below
            errors.append(error)

    for process in processes:
        def stop(process=process):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)  # Lets ROS processes shut down their participants.
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
        attempt(stop)
    if finished is not None:
        attempt(lambda: finished.done() or finished.set_result(True))

    def shutdown():
        if not executor.shutdown(timeout_sec=5):
            raise AssertionError('executor did not shut down')
    attempt(shutdown)
    if thread is not None:
        def join():
            thread.join(timeout=5)
            if thread.is_alive():
                raise AssertionError('executor thread did not stop')
        attempt(join)
    for waitable in waitables:
        attempt(waitable.destroy)
    for node in nodes:
        attempt(node.destroy_node)
    if context is not None:
        attempt(lambda: context.try_shutdown())
    if errors:
        raise errors[0]
