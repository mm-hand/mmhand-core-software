from desktop.core.comm_layer import LowerComputerComm


def test_latest_live_joint_target_stays_before_pending_feedback_start():
    comm = LowerComputerComm("")
    comm.tx_queue.put_nowait("run_feedback")

    command = ("angle_live", [0.0] * 21)
    comm._enqueue_latest_high_rate(command)

    assert list(comm.tx_queue.queue) == [command, "run_feedback"]


def test_joint_target_and_feedback_start_are_one_queue_item():
    comm = LowerComputerComm("")
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


def test_live_target_preserves_pending_atomic_feedback_start():
    comm = LowerComputerComm("")
    start = ("joint_target_run", [1.0] * 21)
    comm.tx_queue.put_nowait(start)
    latest = ("angle_live", [2.0] * 21)

    assert comm._enqueue_latest_high_rate(latest) is True
    assert list(comm.tx_queue.queue) == [start, latest]


def test_new_live_target_replaces_only_live_frame_after_pending_start():
    comm = LowerComputerComm("")
    start = ("joint_target_run", [1.0] * 21)
    stale = ("angle_live", [2.0] * 21)
    latest = ("angle_live", [3.0] * 21)
    comm.tx_queue.put_nowait(start)
    comm.tx_queue.put_nowait(stale)

    assert comm._enqueue_latest_high_rate(latest) is True
    assert list(comm.tx_queue.queue) == [start, latest]


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

    # FE, LEN, EA, mask(00 00 00 08), 21 little-endian floats, FF;
    # paired RUN_FEEDBACK frame follows immediately.
    assert encoded[0] == 0xFE
    assert encoded[2] == 0xEA
    assert encoded[3:7] == bytes([0, 0, 0, 8])
    assert encoded[91] == 0xFF
    assert encoded[92:] == bytes([0xFE, 0x02, 0xD8, 0xFF])


def test_invalidating_joint_commands_also_removes_feedback_request():
    comm = LowerComputerComm("")
    comm.tx_queue.put_nowait(("joint_target_run", [2.0] * 21))
    comm.tx_queue.put_nowait("run_feedback")
    comm.tx_queue.put_nowait("query_manual_encoder_reference")

    assert comm.invalidate_pending_joint_commands() == 2
    assert list(comm.tx_queue.queue) == ["query_manual_encoder_reference"]
