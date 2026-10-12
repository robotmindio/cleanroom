"""Safety recovery requires fresh permission and settled collision monitoring."""

from lekiwi_rmf.navigation_pause import NavigationPause


def test_missing_denied_stale_and_settling_inputs():
    pause = NavigationPause()
    assert pause.reason(1)
    pause.permission(True, 1)
    pause.collision(False, '', 1)
    assert not pause.reason(1)
    pause.collision(True, 'StopZone', 1.1)
    assert 'StopZone' in pause.reason(1.1)
    pause.collision(False, '', 1.2)
    assert 'settling' in pause.reason(1.2)
    pause.permission(True, 2.2)
    pause.collision(False, '', 2.2)
    assert not pause.reason(2.2)
    assert 'stale' in pause.reason(3.2)
    pause.permission(False, 3.3)
    assert 'denied' in pause.reason(3.3)
