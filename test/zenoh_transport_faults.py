"""Native mTLS transport probe on two isolated DDS domains; no robot endpoints."""

import asyncio
from contextlib import ExitStack
import json
import logging
import os
from pathlib import Path
import socket
import subprocess
import threading
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import CompressedImage, LaserScan, PointCloud2, PointField


class DelayedLink:
    """Forward encrypted bytes; hold only device-to-compute traffic on demand."""

    def __init__(self, target):
        self.target = target
        self.hold = False
        self.connections = 0
        self.writers = set()
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.loop.run_forever, daemon=True)
        self.thread.start()
        self.server = asyncio.run_coroutine_threadsafe(self.start(), self.loop).result(5)
        self.port = self.server.sockets[0].getsockname()[1]

    async def start(self):
        return await asyncio.start_server(self.forward, '127.0.0.1', 0)

    async def forward(self, reader, writer):
        try:
            remote_reader, remote_writer = await asyncio.open_connection('127.0.0.1', self.target)
        except ConnectionError as error:
            logging.info('Transport probe upstream unavailable: %s', error)
            writer.close()
            return
        self.connections += 1
        self.writers.update((writer, remote_writer))

        async def pipe(source, destination, delay):
            while data := await source.read(65536):
                while delay and self.hold:
                    await asyncio.sleep(.005)
                destination.write(data)
                await destination.drain()

        tasks = [asyncio.create_task(pipe(reader, remote_writer, False)),
                 asyncio.create_task(pipe(remote_reader, writer, True))]
        try:
            await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
        finally:
            for task in tasks:
                task.cancel()
            for result in await asyncio.gather(*tasks, return_exceptions=True):
                if isinstance(result, Exception):
                    logging.warning('Transport probe proxy closed: %s', result)
            writer.close()
            remote_writer.close()
            self.writers.difference_update((writer, remote_writer))

    def disconnect(self):
        async def reset():
            for writer in list(self.writers):
                writer.close()
        asyncio.run_coroutine_threadsafe(reset(), self.loop).result(5)

    def close(self):
        async def stop():
            self.server.close()
            await self.server.wait_closed()
            tasks = [t for t in asyncio.all_tasks() if t is not asyncio.current_task()]
            for task in tasks:
                task.cancel()
            for result in await asyncio.gather(*tasks, return_exceptions=True):
                if isinstance(result, Exception):
                    logging.warning('Transport probe proxy closed: %s', result)
        asyncio.run_coroutine_threadsafe(stop(), self.loop).result(5)
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(5)
        self.loop.close()


def probe(root, output, binary):
    """Exercise the repository configs with bulk delay and a complete short outage."""
    root, output = Path(root), Path(output)
    with ExitStack() as cleanup:
        with socket.socket() as reservation:
            reservation.bind(('127.0.0.1', 0))
            port = reservation.getsockname()[1]
        links = [DelayedLink(port) for _ in range(3)]
        for link in links:
            cleanup.callback(link.close)
        processes = []
        for role, domain in [('compute_scan', 223), ('compute_depth', 223), ('compute_images', 223), ('device', 222)]:
            config = (root / 'config' / f'zenoh_{role}.json5').read_text()
            config = config.replace('/etc/lekiwi/zenoh-tls', str(output / 'pki'))
            config = config.replace('tls/0.0.0.0:7447', f'tls/127.0.0.1:{port}')
            path = output / f'{role}.json5'
            path.write_text(config)
            command = [binary, '-c', str(path), '-d', str(domain)]
            if role.startswith('compute_'):
                index = ('compute_scan', 'compute_depth', 'compute_images').index(role)
                command += ['-e', f'tls/127.0.0.1:{links[index].port}?prio={(("0-1", "2-5", "6-7")[index])}']
            if role == 'device':
                time.sleep(.15)
                assert all(p.poll() is None for p in processes), 'client exited while device was unavailable'
            log = cleanup.enter_context((output / f'{role}.log').open('w'))
            process = subprocess.Popen(command, stdout=log, stderr=log,
                                       env={**os.environ, 'CYCLONEDDS_URI': f'file://{root}/config/cyclonedds.xml'})
            processes.append(process)

            def stop(process=process):
                process.terminate()
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
            cleanup.callback(stop)

        nodes, executors = [], []
        for domain in (222, 223):
            context = Context()
            rclpy.init(context=context, domain_id=domain)
            cleanup.callback(context.shutdown)
            node = Node(f'transport_probe_{domain}', context=context)
            cleanup.callback(node.destroy_node)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            cleanup.callback(executor.shutdown)
            nodes.append(node)
            executors.append(executor)

        topics = [('scan', LaserScan, '/pi/lidar/scan'),
                  ('depth', PointCloud2, '/camera/depth/points'),
                  ('image', CompressedImage, '/pi/camera/front/image_raw/compressed')]
        publishers = {key: nodes[0].create_publisher(kind, topic, QoSProfile(depth=1))
                      for key, kind, topic in topics}
        received = {key: [] for key, _, _ in topics}
        sent = set()
        phase = 'warmup'
        report = {'passed': False, 'samples': received}

        def save_report():
            report.update(phase=phase, connections=[link.connections for link in links])
            (output / 'transport-proof.json').write_text(json.dumps(report, indent=2) + '\n')
        cleanup.callback(save_report)

        def receive(key, message):
            stamp = message.header.stamp.sec * 10**9 + message.header.stamp.nanosec
            assert stamp in sent, 'transport changed the acquisition timestamp'
            received[key].append((phase, time.monotonic(), time.time() - stamp / 1e9))

        for key, kind, topic in topics:
            nodes[1].create_subscription(kind, topic, lambda m, key=key: receive(key, m), QoSProfile(depth=1))

        def publish():
            stamp = nodes[0].get_clock().now().to_msg()
            sent.add(stamp.sec * 10**9 + stamp.nanosec)
            for key, kind, _ in topics:
                message = kind()
                message.header.stamp = stamp
                message.header.frame_id = 'isolated_transport_probe'
                if key == 'scan':
                    message.ranges = [1.0] * 360
                elif key == 'depth':
                    message.height, message.width = 1, 4096
                    message.fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                                      for i, n in enumerate(('x', 'y', 'z'))]
                    message.point_step, message.row_step = 12, 4096 * 12
                    message.data = bytes(message.row_step)
                else:
                    message.format = 'transport-test-bytes'
                    message.data = bytes(65536)
                publishers[key].publish(message)

        next_publish = 0

        def spin(duration):
            nonlocal next_publish
            end = time.monotonic() + duration
            while time.monotonic() < end:
                assert all(p.poll() is None for p in processes), 'native bridge exited'
                if time.monotonic() >= next_publish:
                    publish()
                    next_publish = time.monotonic() + .1
                for executor in executors:
                    executor.spin_once(timeout_sec=.002)

        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and not all(received.values()):
            spin(.1)
        assert all(received.values()), 'sensor discovery did not complete'
        assert all(link.connections == 1 for link in links), 'priority links were not established independently'
        assert all(nodes[1].count_publishers(topic) == 1 for _, _, topic in topics), 'duplicate imported sensor writers'
        phase = 'camera_delayed'
        links[2].hold = True
        spin(.8)
        for key in ('scan', 'depth'):
            samples = [age for p, _, age in received[key] if p == phase]
            assert len(samples) >= 4 and max(samples) < .3, (key, samples)
        assert time.monotonic() - received['image'][-1][1] >= .5
        links[2].hold = False
        spin(.5)
        phase = 'depth_delayed'
        links[1].hold = True
        spin(.8)
        samples = [age for p, _, age in received['scan'] if p == phase]
        assert len(samples) >= 4 and max(samples) < .3, samples
        assert time.monotonic() - received['depth'][-1][1] >= .5
        links[1].hold = False
        spin(.5)
        phase = 'all_delayed'
        for link in links:
            link.hold = True
        spin(.8)
        assert all(time.monotonic() - received[key][-1][1] >= .5 for key in received)
        resumed_at = time.monotonic()
        phase = 'recovery'
        for link in links:
            link.hold = False
        while time.monotonic() - resumed_at < 2 and not all(
                any(p == phase and age < .3 for p, _, age in values) for values in received.values()):
            spin(.02)
        recovery = {key: min((at - resumed_at for p, at, age in values if p == phase and age < .3), default=None)
                    for key, values in received.items()}
        phase = 'reconnect'
        reconnect_at = time.monotonic()
        for link in links:
            link.disconnect()
        while time.monotonic() - reconnect_at < 5 and not (
                all(link.connections >= 2 for link in links)
                and all(any(p == phase and age < .3 for p, _, age in values) for values in received.values())
                and all(nodes[1].count_publishers(topic) == 1 for _, _, topic in topics)):
            spin(.02)
        assert all(link.connections >= 2 for link in links), 'closed priority links did not reconnect'
        assert all(any(p == phase and age < .3 for p, _, age in values) for values in received.values())
        version = subprocess.check_output([binary, '--version'], text=True).strip().splitlines()[-1]
        assert all(nodes[1].count_publishers(topic) == 1 for _, _, topic in topics), 'duplicate writers after reconnect'
        report.update({'bridge_version': version, 'started_without_device': True, 'camera_delay_sec': .8, 'depth_delay_sec': .8, 'all_links_delay_sec': .8,
                  'reconnect_sec': time.monotonic() - reconnect_at,
                  'recovery_sec': recovery})
        assert all(value is not None and value < 2 for value in recovery.values()), recovery
        report['passed'] = True
        return report


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--binary', required=True)
    args = parser.parse_args()
    probe(args.root, args.output, args.binary)
