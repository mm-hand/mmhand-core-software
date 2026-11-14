"""Motor ID planning, maintenance frames and independent S3 data clocks."""
import itertools
import struct
import threading
import time
import pytest
from desktop.core.id_protocol import (
    MaintenanceChannel, command_frame, plan_motor_ids,
    PROBE, RENAME,
)
from desktop.core.comm_layer import TactileSerialComm
from desktop.core import protocol


def simulate(rows, moves):
    occupied = {(r['bus'], r['id']) for r in rows}
    for bus, old, new in moves:
        assert (bus, old) in occupied
        assert (bus, new) not in occupied
        assert new < 254 and old < 254
        occupied.remove((bus, old)); occupied.add((bus, new))
    return occupied


def test_batch_moves_directly_to_free_ids():
    rows = [dict(bus=0, id=i) for i in range(5)]
    targets = {(0,i): i+5 for i in range(5)}
    moves = plan_motor_ids(rows, targets)
    assert moves == [(0, i, i + 5) for i in range(5)]
    assert simulate(rows,moves) == {(0,i) for i in range(5,10)}


def test_all_permutations_are_collision_free():
    rows = [dict(bus=0,id=i) for i in range(5)]
    for permutation in itertools.permutations(range(5)):
        targets = {(0,i): permutation[i] for i in range(5)}
        assert simulate(rows,plan_motor_ids(rows,targets)) == {(0,i) for i in permutation}


def test_cross_bus_duplicate_old_ids_can_be_resolved():
    rows = [dict(bus=0,id=1),dict(bus=1,id=1)]
    assert simulate(rows,plan_motor_ids(rows,{(0,1):0,(1,1):1})) == {(0,0),(1,1)}


def test_existing_253_is_moved_before_it_is_used_to_break_a_cycle():
    rows = [dict(bus=0, id=identifier) for identifier in (1, 2, 3, 253)]
    moves = plan_motor_ids(rows, {(0, 1): 2, (0, 2): 1, (0, 3): 4, (0, 253): 3})
    assert moves == [(0, 3, 4), (0, 253, 3), (0, 1, 253), (0, 2, 1), (0, 253, 2)]
    assert simulate(rows, moves) == {(0, identifier) for identifier in (1, 2, 3, 4)}


@pytest.mark.parametrize('targets', [{(0,0):254}, {(0,0):253}, {(0,0):-1}, {(0,0):.5}, {(0,0):None}])
def test_invalid_id_never_generates_moves(targets):
    with pytest.raises(ValueError): plan_motor_ids([dict(bus=0,id=0)],targets)


def test_duplicate_final_ids_are_rejected():
    with pytest.raises(ValueError,match='unique'):
        plan_motor_ids([dict(bus=0,id=0),dict(bus=1,id=1)],{(0,0):1,(1,1):1})


def test_channel_accepts_only_matching_token_and_fields():
    channel = MaintenanceChannel()
    def send(frame):
        assert frame[1] == 14 and len(frame)==16 and frame[-1]==255
        channel.feed(frame[3:-1]+bytes((0,1,3,0)))
        return True
    reply = channel.request(send,244,7,PROBE,2,18)
    assert reply['present']==1 and reply['bus']==2
    def wrong(frame):
        channel.feed(frame[3:-1][:8]+bytes((RENAME,2,18,0,0,1,3,0)))
        return True
    with pytest.raises(ValueError,match='match'): channel.request(wrong,244,7,PROBE,2,18)


def test_close_wakes_pending_requests():
    channel=MaintenanceChannel()
    errors=[]
    def waiting():
        try: channel.request(lambda _:True,244,1,PROBE)
        except Exception as exc: errors.append(exc)
    thread=threading.Thread(target=waiting); thread.start()
    channel.close(); thread.join(1)
    assert not thread.is_alive() and isinstance(errors[0],(ConnectionError,InterruptedError))


def test_s3_zero_force_updates_clock_but_removed_maintenance_frames_do_not():
    comm=TactileSerialComm('unused')
    comm._process_tactile_packets([(protocol.PACKET_TYPE_TACTILE,bytes(48))])
    assert comm.connection_diagnostics()['last_valid_rx']>0
    before=comm.connection_diagnostics()['frames']
    comm._process_tactile_packets([(0x30, struct.pack('>IIBBBBBBBB',1,0,0,0,0,0,0,3,1,0)),
                                   (0x31,bytes.fromhex('112233445566'))])
    link=comm.connection_diagnostics()
    assert link['frames']==before and 'device_id' not in link
    comm._process_tactile_packets([(0x30,b'broken')])
    assert comm.connection_diagnostics()['frames']==before


def test_command_payload_is_big_endian():
    frame=command_frame(244,0x12345678,0x90ABCDEF,RENAME,3,20,253)
    assert frame==bytes.fromhex('FE0EF41234567890ABCDEF040314FDFF')
