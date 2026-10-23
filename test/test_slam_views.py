from lekiwi_rmf.slam_views import ViewSet


def views(result):
    return [message for _, message in result]


def test_both_cameras_publish_once_together_and_a_silent_one_is_left_out():
    views_set = ViewSet(2, max_interval=0.35, stale_after=1.5)
    # A camera that has not delivered yet does not hold the first one back.
    assert views(views_set.offer(0, 9.6, 99.6, "astra@9.6")) == ["astra@9.6"]
    # The fast front camera waits for the Astra frame it pairs with.
    assert views_set.offer(1, 10.0, 100.0, "front@10.0") == []
    assert views(views_set.offer(0, 10.1, 100.1, "astra@10.1")) == ["astra@10.1", "front@10.0"]
    # Frames already published are not repeated by the next arrival.
    assert views_set.offer(1, 10.2, 100.2, "front@10.2") == []
    # The front camera wedges: once it is silent for stale_after, the Astra carries on alone.
    assert views_set.offer(0, 10.6, 100.6, "astra@10.6") == []
    assert views(views_set.offer(0, 11.8, 101.8, "astra@11.8")) == ["astra@11.8"]
    # It comes back and the pair resumes.
    assert views_set.offer(1, 12.0, 102.0, "front@12.0") == []
    assert views(views_set.offer(0, 12.1, 102.1, "astra@12.1")) == ["astra@12.1", "front@12.0"]


def test_a_lone_front_camera_still_feeds_slam():
    views_set = ViewSet(2, max_interval=0.35, stale_after=1.5)
    assert views(views_set.offer(1, 5.0, 50.0, "front@5.0")) == ["front@5.0"]
    assert views(views_set.offer(1, 5.1, 50.1, "front@5.1")) == ["front@5.1"]


def test_frames_too_far_apart_are_not_joined():
    views_set = ViewSet(2, max_interval=0.35, stale_after=1.5)
    views_set.offer(0, 9.6, 99.6, "astra@9.6")
    assert views_set.offer(1, 10.0, 100.0, "front@10.0") == []
    assert views_set.offer(0, 10.5, 100.5, "astra@10.5") == []


def test_an_arrival_joins_the_closest_queued_frame_not_only_the_latest():
    views_set = ViewSet(2, max_interval=0.35, stale_after=1.5)
    views_set.offer(0, 9.0, 99.0, "astra@9.0")
    views_set.offer(1, 9.95, 99.95, "front@9.95")
    assert views_set.offer(1, 10.4, 100.4, "front@10.4") == []
    # The late Astra frame pairs with the queued front frame closest to it.
    assert views(views_set.offer(0, 10.0, 100.5, "astra@10.0")) == ["astra@10.0", "front@9.95"]
    # The next Astra frame cannot reuse that front frame, only a newer one.
    assert views(views_set.offer(0, 10.5, 100.6, "astra@10.5")) == ["astra@10.5", "front@10.4"]
