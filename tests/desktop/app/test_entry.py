from desktop.app.entry import build_parser


def test_cli_args_parse_ros2_auto_start() -> None:
    args = build_parser().parse_args(
        ["cli", "--port", "COM7", "--ros2-input", "raw_ik", "--ros2-output", "joint_cmd", "--auto-start", "--no-plot"]
    )

    assert args.command == "cli"
    assert args.port == "COM7"
    assert args.ros2 is True
    assert args.ros2_input == "raw_ik"
    assert args.ros2_output == "joint_cmd"
    assert args.auto_start is True
    assert args.no_plot is True


def test_ui_args_parse_mode_and_ros2() -> None:
    args = build_parser().parse_args(["ui", "--mode", "replay", "--ros2"])

    assert args.command == "ui"
    assert args.mode == "replay"
    assert args.ros2 is True


def test_ros2_is_enabled_by_default_and_can_be_disabled() -> None:
    default_args = build_parser().parse_args(["ui", "--mode", "monitor"])
    disabled_args = build_parser().parse_args(["ui", "--no-ros2"])

    assert default_args.ros2 is True
    assert disabled_args.ros2 is False


def test_ros2_fixed_topic_modes_have_concise_defaults() -> None:
    args = build_parser().parse_args(["cli", "-p", "COM7"])

    assert args.ros2_input == "joint_target"
    assert args.ros2_output == "all"
    assert not hasattr(args, "ros2_joint_target_topic")
    assert not hasattr(args, "ros2_raw_ik_topic")
    assert not hasattr(args, "ros2_joint_state_topic")
    assert not hasattr(args, "ros2_raw_ik_space")
