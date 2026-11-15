"""Motor badges show offline motors and servo-reported faults; voltage 0x01 is ignored."""
from nicegui import Client, ui

from desktop.ui.web.common import motor_badge


def badges(client):
    return [element.text for element in client.elements.values() if isinstance(element, ui.badge)]


def test_motor_badge_only_appears_for_offline_motors():
    client = Client(ui.page('/test-motor-badge'))
    with client:
        refresh = motor_badge()
        refresh({'motors': [dict(motor=4, joint=1, online=False),
                            dict(motor=5, joint=1, online=True)]}, joint=1)
        assert badges(client) == ['M04 LOSS']
        # Unknown (stale or disconnected) is not a loss; one lost motor of a pair still shows.
        refresh({'motors': [dict(motor=4, joint=1, online=False),
                            dict(motor=5, joint=1, online=None)]}, joint=1)
        assert badges(client) == ['M04 LOSS']
        refresh({'motors': [dict(motor=4, joint=1, online=True),
                            dict(motor=5, joint=1, online=True)]}, joint=1)
        assert not badges(client)
    client.delete()


def test_motor_badge_labels_fault_bits_and_ignores_voltage():
    client = Client(ui.page('/test-motor-badge-fault'))
    with client:
        refresh = motor_badge()
        refresh({'motors': [dict(motor=11, joint=8, online=True, fault=0x04)]}, joint=8)
        assert badges(client) == ['M11 OVERTEMP']
        # 0x01 电压异常不显示，也不当作丢失。
        refresh({'motors': [dict(motor=11, joint=8, online=True, fault=0x01)]}, joint=8)
        assert not badges(client)
        # 多个故障位合并成一个徽标。
        refresh({'motors': [dict(motor=11, joint=8, online=True, fault=0x0C)]}, joint=8)
        assert badges(client) == ['M11 OVERTEMP/OVERCURRENT']
        # 离线优先显示 LOSS。
        refresh({'motors': [dict(motor=11, joint=8, online=False, fault=0x04)]}, joint=8)
        assert badges(client) == ['M11 LOSS']
        # 故障清除后徽标消失。
        refresh({'motors': [dict(motor=11, joint=8, online=True, fault=0)]}, joint=8)
        assert not badges(client)
    client.delete()


def test_motor_badge_filters_by_motor_for_direct_control_rows():
    client = Client(ui.page('/test-motor-badge-motor'))
    with client:
        refresh = motor_badge()
        refresh({'motors': [dict(motor=4, joint=1, online=False),
                            dict(motor=5, joint=1, online=False)]}, motor=5)
        assert badges(client) == ['M05 LOSS']
    client.delete()
