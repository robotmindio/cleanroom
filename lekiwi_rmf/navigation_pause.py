"""Recoverable navigation inputs; independent of ROS transport."""


class NavigationPause:
    def __init__(self, timeout=1.0, settle=1.0):
        self.timeout = timeout
        self.settle = settle
        self.inputs = {}
        self.clear_after = 0.0
        self.stop_name = ''

    def permission(self, allowed, now):
        self.inputs['base_motion_permitted'] = (allowed, now)

    def collision(self, stopped, name, now):
        self.inputs['collision monitor'] = (not stopped, now)
        if stopped:
            self.clear_after = now + self.settle
            self.stop_name = name

    def reason(self, now):
        for name, (allowed, seen) in self.inputs.items():
            if now - seen >= self.timeout:
                return f'stale {name}'
            if not allowed:
                return f'collision monitor stop: {self.stop_name}' if name == 'collision monitor' else f'denied {name}'
        if len(self.inputs) != 2:
            return 'navigation safety inputs unavailable'
        if now < self.clear_after:
            return f'collision monitor settling: {self.stop_name}'
        return ''
