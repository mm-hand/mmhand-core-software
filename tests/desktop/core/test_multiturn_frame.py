"""掉电后多圈对账：只平移标定包的伺服 ABS 字段。"""

import pytest

from desktop.core.multiturn_frame import shift_snapshot, turn_offset


def _snapshot():
    return {
        'multiturn_anchor_valid_bitmap': '0x7ffffff',
        'multiturn_anchor_logical_abs': [0] * 27,
        'motor_abs_limit_valid_bitmap': '0x7ffffff',
        'motor_abs_limit_min': [0] * 27,
        'motor_abs_limit_max': [0] * 27,
        'joint_calib_results': [None] * 21,
    }


@pytest.mark.parametrize('anchor,current,sign,offset', [
    (5402, 200, 1, 1), (5402, 2600, 1, 0),
    (3172, 1042, 1, 0), (2409, 205, 1, 0),
    (18109, 1725, 1, 4), (-488, 3607, 1, -1),
    (-5402, -200, -1, -1), (-5402, -2600, -1, 0),
    (0, 0, 1, 0), (0, 4095, 1, -1), (0, 1, -1, 0),
])
def test_turn_offset_follows_release_direction(anchor, current, sign, offset):
    assert turn_offset(anchor, current, sign) == offset


def test_turn_offset_rejects_unknown_direction():
    with pytest.raises(ValueError):
        turn_offset(0, 0, 0)


def test_shift_moves_anchors_limits_and_lut_but_not_encoder_raw():
    snapshot = _snapshot()
    snapshot['multiturn_anchor_logical_abs'][11] = 18109
    snapshot['motor_abs_limit_min'][11] = 18000
    snapshot['joint_calib_results'][9] = {
        'tendonLutMotor': [11, 255],
        'startServoAbs': 18000,
        'tightServoAbs': 18109,
        'zeroEncoder': 8115,
        'tendonLutMotorAbs': [[18000, 18109], [0, 0]],
    }

    shifted = shift_snapshot(snapshot, {11: 4})

    assert shifted['multiturn_anchor_logical_abs'][11] == 18109 - 4 * 4096
    assert shifted['motor_abs_limit_min'][11] == 18000 - 4 * 4096
    result = shifted['joint_calib_results'][9]
    assert result['startServoAbs'] == 18000 - 4 * 4096
    assert result['tightServoAbs'] == 18109 - 4 * 4096
    assert result['tendonLutMotorAbs'][0] == [18000 - 4 * 4096, 18109 - 4 * 4096]
    assert result['zeroEncoder'] == 8115  # 编码器 raw 不动
    assert snapshot['multiturn_anchor_logical_abs'][11] == 18109  # 原快照不变


def test_shift_refuses_values_outside_the_servo_command_range():
    snapshot = _snapshot()
    snapshot['multiturn_anchor_logical_abs'][0] = -30000
    with pytest.raises(ValueError, match='M00'):
        shift_snapshot(snapshot, {0: 1})


def test_dual_motor_columns_shift_independently_and_scalar_only_once():
    snapshot = _snapshot()
    snapshot['joint_calib_results'][0] = {
        'tendonLutMotor': [0, 1], 'startServoAbs': 8192,
        'tendonLutMotorAbs': [[8192, 8292], [-4096, -3996]],
    }
    result = shift_snapshot(snapshot, {0: 2, 1: -1})['joint_calib_results'][0]
    assert result['startServoAbs'] == 0
    assert result['tendonLutMotorAbs'] == [[0, 100], [0, 100]]
