from desktop.core.comm_layer import LowerComputerComm


def test_latest_live_joint_target_stays_before_pending_feedback_start():
    comm = LowerComputerComm("")
    comm.tx_queue.put_nowait("run_feedback")

    command = ("angle_live", [0.0] * 21)
    comm._enqueue_latest_high_rate(command)

    assert list(comm.tx_queue.queue) == [command, "run_feedback"]


def test_joint_target_and_feedback_start_are_one_queue_item():
    comm = LowerComputerComm("")
    comm._protocol_compatible = True
    comm.is_connected = lambda: True
    command = ("joint_target_run", [12.0] * 21)

    assert comm.send_command(command) is True
    assert list(comm.tx_queue.queue) == [command]


def test_latest_paired_joint_command_replaces_stale_target_and_run_request():
    comm = LowerComputerComm("")
    comm.tx_queue.put_nowait(("angle_live", [1.0] * 21))
    comm.tx_queue.put_nowait("run_feedback")
    command = ("joint_target_run", [2.0] * 21)

    assert comm._enqueue_latest_high_rate(command) is True
    assert list(comm.tx_queue.queue) == [command]


def test_latest_masked_joint_command_replaces_stale_full_hand_command():
    comm = LowerComputerComm("")
    comm.tx_queue.put_nowait(("joint_target_run", [1.0] * 21))
    command = ("joint_target_run_masked", [2.0] * 21, 1 << 3)

    assert comm._enqueue_latest_high_rate(command) is True
    assert list(comm.tx_queue.queue) == [command]


def test_masked_joint_command_encodes_mask_before_21_floats():
    comm = LowerComputerComm("")
    encoded = comm._encode_command(
        ("joint_target_run_masked", [2.0] * 21, 1 << 3)
    )

    # FE, LEN, CB, mask(08 00 00 00), 21 little-endian floats, FF;
    # paired RUN_FEEDBACK frame follows immediately.
    assert encoded[0] == 0xFE
    assert encoded[2] == 0xCB
    assert encoded[3:7] == bytes([8, 0, 0, 0])
    assert encoded[91] == 0xFF
    assert encoded[92:] == bytes([0xFE, 0x02, 0xD8, 0xFF])


def test_invalidating_joint_commands_also_removes_feedback_request():
    comm = LowerComputerComm("")
    comm.tx_queue.put_nowait(("joint_target_run", [2.0] * 21))
    comm.tx_queue.put_nowait("run_feedback")
    comm.tx_queue.put_nowait(("state_sync", 1))

    assert comm.invalidate_pending_joint_commands() == 2
    assert list(comm.tx_queue.queue) == [("state_sync", 1)]


def test_target_stays_replaceable_while_waiting_for_next_motion_slot(monkeypatch):
    comm = LowerComputerComm("")
    now = [10.0]
    monkeypatch.setattr("desktop.core.comm_layer.time.monotonic", lambda: now[0])
    comm._last_motion_tx = now[0]
    comm._enqueue_latest_high_rate(("angle_live", [1.0] * 21))
    latest = ("angle_live", [2.0] * 21)

    def next_slot(delay):
        comm.tx_queue.queue.clear()
        comm.tx_queue.queue.append(latest)
        now[0] += delay + 0.000001

    monkeypatch.setattr(comm.tx_queue.not_empty, "wait", next_slot)
    assert comm._get_paced_tx_command() == latest
    assert now[0] >= 10.01


def test_stop_is_not_delayed_by_motion_rate_limit():
    comm = LowerComputerComm("")
    comm._last_motion_tx = float("inf")
    comm.tx_queue.put_nowait("stop")
    assert comm._get_paced_tx_command() == "stop"
