#!/usr/bin/env python3
"""Explicit live qualification host: USR1 diagnostic error, USR2 duplicate frame.

Only the transmitted observation changes. Servo reads, commands, authenticated
transport and the local watchdog run unchanged. Injection expires after 8 s.
Never used by the production service; start with robot-host.sh's test option.
"""
import importlib.util
import json
from pathlib import Path
import signal
import sys
import time


class FaultSocket:
    def __init__(self, socket, clock=time.monotonic):
        self.socket, self.clock = socket, clock
        self.mode, self.until, self.last = None, 0, None

    def inject(self, mode):
        if mode not in ('diagnostic', 'duplicate'):
            raise ValueError('unsupported telemetry fault')
        if mode == 'duplicate' and self.last is None:
            raise RuntimeError('cannot replay before a successful observation')
        self.mode, self.until = mode, self.clock()+8
        print('qualification telemetry fault:', mode, flush=True)

    def __getattr__(self, name):
        return getattr(self.socket, name)

    def send_multipart(self, frames, **kwargs):
        if self.clock() >= self.until:
            self.mode = None
        outgoing = frames
        if self.mode == 'duplicate':
            outgoing = self.last
        elif self.mode == 'diagnostic':
            payload = json.loads(frames[0])
            for status in payload['_lekiwi_motor_health']['statuses'].values():
                status.update(level=2, message='explicit qualification fault injection')
            outgoing = [json.dumps(payload).encode(), *frames[1:]]
        self.socket.send_multipart(outgoing, **kwargs)
        if self.mode is None:
            self.last = [bytes(frame) for frame in frames]


def main():
    spec = importlib.util.spec_from_file_location('qualification_torque_host',
        Path(__file__).with_name('torque-host.py'))
    host = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = host
    spec.loader.exec_module(host)
    original = host.HostLoop.__init__

    def initialize(loop, *args, **kwargs):
        original(loop, *args, **kwargs)
        socket = FaultSocket(loop.host.zmq_observation_socket)
        loop.host.zmq_observation_socket = socket
        signal.signal(signal.SIGUSR1, lambda *_: socket.inject('diagnostic'))
        signal.signal(signal.SIGUSR2, lambda *_: socket.inject('duplicate'))

    host.HostLoop.__init__ = initialize
    host.main()


if __name__ == '__main__':
    main()
