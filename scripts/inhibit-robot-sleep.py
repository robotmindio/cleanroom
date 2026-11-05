#!/usr/bin/python3
"""Keep logind's sleep-inhibitor descriptor across exec, preserving the launcher PID."""

import os
import sys

import dbus


def main(argv=None):
    command = sys.argv[1:] if argv is None else argv
    if not command:
        raise ValueError('a command is required')
    manager = dbus.SystemBus().get_object('org.freedesktop.login1', '/org/freedesktop/login1')
    lease = manager.Inhibit('sleep', 'lekiwi', 'Robot stack is running', 'block',
                            dbus_interface='org.freedesktop.login1.Manager')
    fd = lease.take()
    try:
        os.set_inheritable(fd, True)
        os.execvp(command[0], command)
    finally:
        os.close(fd)


if __name__ == '__main__':
    try:
        main()
    except (dbus.DBusException, OSError, ValueError) as error:
        print(f'sleep inhibitor: {error}', file=sys.stderr)
        sys.exit(1)
