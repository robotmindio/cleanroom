"""Safety recovery requires fresh permission and settled collision monitoring."""

from lekiwi_rmf.navigation_pause import CODES, NavigationPause


def test_missing_denied_stale_and_settling_inputs():
    pause = NavigationPause()
    seen = [pause.cause(1)]
    assert seen[-1] == ('safety_inputs_unavailable', 'navigation safety inputs unavailable')
    pause.permission(True, 1)
    pause.collision(False, '', 1)
    assert pause.cause(1) is None
    pause.collision(True, 'StopZone', 1.1)
    seen.append(pause.cause(1.1))
    assert seen[-1] == ('collision_stop', 'collision monitor stop: StopZone')
    pause.collision(False, '', 1.2)
    seen.append(pause.cause(1.2))
    assert seen[-1] == ('collision_settling', 'collision monitor settling: StopZone')
    pause.permission(True, 2.2)
    pause.collision(False, '', 2.2)
    assert pause.cause(2.2) is None
    seen.append(pause.cause(3.2))
    assert seen[-1] == ('safety_input_stale', 'stale base_motion_permitted')
    pause.permission(False, 3.3)
    seen.append(pause.cause(3.3))
    assert seen[-1] == ('base_motion_denied', 'denied base_motion_permitted')
    assert {code for code, _ in seen} == set(CODES)
