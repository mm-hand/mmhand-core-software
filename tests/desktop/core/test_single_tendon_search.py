from desktop.core.single_tendon_search import SearchFeedback, SingleTendonSearch


def build_search(direction=1, abs_range=(-20000, 20000)):
    targets, revoked = [], []
    machine = SingleTendonSearch(started_ms=0, direction=direction,
        abs_range=abs_range, publish=lambda target: targets.append(target) or True,
        revoke=lambda: revoked.append(True))
    return machine, targets, revoked


def sample(now, encoder, servo, load=250, **kwargs):
    kwargs.setdefault("permission_valid", True)
    return SearchFeedback(now, now, now, encoder, servo, load, **kwargs)


def test_single_search_preserves_stall_settle_and_release_sequence():
    machine, targets, revoked = build_search()
    machine.observe(sample(0, 3000, 0))
    machine.observe(sample(200, 3000, 80))
    assert machine.current_state == machine.settling
    machine.observe(sample(400, 3000, 80))
    assert machine.current_state == machine.settling
    machine.observe(sample(600, 3000, 80))
    assert machine.current_state == machine.releasing
    machine.observe(sample(800, 2000, 0))
    machine.observe(sample(1000, 2000, -80))
    assert machine.result is None
    machine.observe(sample(1200, 2000, -160))
    assert machine.current_state == machine.completed
    assert (machine.result.tight_abs, machine.result.released_abs) == (80, -160)
    assert revoked == [True]
    before = list(targets)
    machine.observe(sample(1400, 2000, -240))
    assert targets == before


def test_missing_feedback_timeout_is_driven_by_clock_without_samples():
    machine, targets, revoked = build_search()
    machine.tick(1199)
    assert not machine.current_state.final
    machine.tick(1200)
    assert machine.current_state == machine.failed
    assert targets == [] and revoked == [True]


def test_cancel_revokes_source_and_late_feedback_cannot_restart():
    machine, targets, revoked = build_search()
    machine.observe(sample(0, 3000, 0))
    machine.cancel()
    machine.observe(sample(200, 3000, 80))
    assert targets == [] and revoked == [True]
    assert machine.current_state == machine.cancelled


def test_permission_loss_and_stale_sample_reject_before_publishing():
    for feedback in (sample(20, 3000, 0, permission_valid=False),
                     SearchFeedback(301, 0, 301, 3000, 0, 250, permission_valid=True)):
        machine, targets, revoked = build_search()
        machine.observe(sample(0, 3000, 0))
        machine.observe(feedback)
        assert machine.current_state == machine.failed
        assert targets == [] and revoked == [True]


def test_reverse_motor_direction_is_preserved_and_range_is_enforced():
    machine, targets, _ = build_search(direction=-1, abs_range=(-10, 10))
    machine.observe(sample(0, 3000, 0))
    machine.observe(sample(20, 3010, -8, load=0))
    assert targets == [-8]
    machine.observe(sample(40, 3020, -8, load=0))
    assert machine.current_state == machine.failed
    assert targets == [-8]


def test_load_alone_does_not_find_tight_endpoint():
    machine, _, _ = build_search()
    machine.observe(sample(0, 3000, 0))
    machine.observe(sample(200, 3000, 0))
    assert machine.current_state == machine.tightening


def test_no_follow_travel_cannot_finish_release():
    machine, _, _ = build_search()
    for now, servo in ((0, 0), (200, 80), (400, 80), (600, 80),
                       (800, 0), (1000, -80), (1200, -160)):
        machine.observe(sample(now, 3000, servo))
    assert machine.current_state == machine.releasing
    assert machine.result is None


def test_publisher_exception_revokes_motion_source():
    machine, targets, revoked = build_search()
    def broken_publish(target):
        raise OSError("transport unavailable")
    machine.publish = broken_publish
    machine.observe(sample(0, 3000, 0))
    machine.observe(sample(20, 3010, 0, load=0))
    assert machine.current_state == machine.failed
    assert "transport unavailable" in machine.failure_reason
    assert revoked == [True]


def test_feedback_without_explicit_permission_cannot_start_search():
    machine, targets, revoked = build_search()
    machine.observe(SearchFeedback(0, 0, 0, 3000, 0, 250))
    assert machine.current_state == machine.failed
    assert targets == [] and revoked == [True]
