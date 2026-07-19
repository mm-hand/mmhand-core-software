from desktop.core.comm_layer import LowerComputerComm


def test_latest_live_joint_target_stays_before_pending_feedback_start():
    comm = LowerComputerComm("")
    comm.tx_queue.put_nowait("run_feedback")

    command = ("angle_live", [0.0] * 21)
    comm._enqueue_latest_high_rate(command)

    assert list(comm.tx_queue.queue) == [command, "run_feedback"]
