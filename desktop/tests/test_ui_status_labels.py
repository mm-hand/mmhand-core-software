"""状态栏英文全称与多对象摘要回归测试。"""

from desktop.ui.ui_app import HandGUI


def test_status_mask_summary_keeps_all_objects_in_detail() -> None:
    """状态栏只能容纳两个对象时，详情字符串仍不得遗漏任何关节。"""
    short, full = HandGUI._format_index_mask("J", (1 << 2) | (1 << 7) | (1 << 14))

    assert short == "J02,J07 +1"
    assert full == "J02,J07,J14"


def test_status_labels_use_full_english_words() -> None:
    """避免重新引入 OL、CALIB、TURN 等非自解释缩写。"""
    assert HandGUI._calibration_status_name("SUCCESS") == "Success"
    assert HandGUI._calibration_status_name("NO_CALIB") == "Not Calibrated"
    assert HandGUI._format_delay_status(25.0) == ("Status Age: 25ms", "ok")
