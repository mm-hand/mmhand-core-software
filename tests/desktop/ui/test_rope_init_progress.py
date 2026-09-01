from desktop.ui.ui_app import HandGUI


def test_manual_events_are_not_misclassified_as_calibration():
    assert HandGUI.event_type_from_message(
        "Manual saved and verified; automatic calibration is required"
    ) == "manual_reference_write"
    assert HandGUI.event_type_from_message(
        "Manual endpoints cleared; Flash=empty"
    ) == "manual_reference_clear"
    assert HandGUI.event_type_from_message(
        "Manual endpoints read back"
    ) == "manual_reference_refresh"
