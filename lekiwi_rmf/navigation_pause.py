"""Recoverable navigation inputs; independent of ROS transport."""

# Stable codes for why managed navigation waits; the text beside them is for people.
CODES = ('safety_input_stale', 'base_motion_denied', 'collision_stop', 'safety_inputs_unavailable',
         'collision_settling')


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

    def cause(self, now):
        """Why navigation must wait, as (code, text); None when it may move."""
        for name, (allowed, seen) in self.inputs.items():
            if now - seen >= self.timeout:
                return 'safety_input_stale', f'stale {name}'
            if not allowed:
                return (('collision_stop', f'collision monitor stop: {self.stop_name}') if name == 'collision monitor'
                        else ('base_motion_denied', f'denied {name}'))
        if len(self.inputs) != 2:
            return 'safety_inputs_unavailable', 'navigation safety inputs unavailable'
        if now < self.clear_after:
            return 'collision_settling', f'collision monitor settling: {self.stop_name}'
        return None
