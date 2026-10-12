/**
 * @file HostAckPublisher.cpp
 * @brief 实现 HostCommTask 私有的 ACK FIFO，保证执行顺序与确认顺序一致。
 */
#include "HostAckPublisher.h"

#include "HostProtocolCodec.h"
#include "../../hal/serial/HostSerialPort.h"
#include "../../config/protocol/HostProtocol.h"
#include "../../system/log/FirmwareLog.h"
#include "../../system/time/Clock.h"

namespace {

struct PendingProtoAck {
    uint8_t cmd;
    uint8_t appliedMode;
    uint8_t status;
};

PendingProtoAck s_pendingProtoAcks[32] = {};
uint8_t s_pendingProtoAckHead = 0;
uint8_t s_pendingProtoAckCount = 0;

bool writeProtoAckPacket(uint8_t cmd, uint8_t appliedMode, uint8_t status)
{
    uint8_t buffer[12];
    size_t index = 0;
    buffer[index++] = PROTOCOL_HEADER;
    buffer[index++] = 0x00;
    buffer[index++] = PACKET_TYPE_PROTO_ACK;
    buffer[index++] = cmd;
    buffer[index++] = appliedMode;
    buffer[index++] = status;
    p4::host::appendU32BigEndian(
        buffer, &index, p4::platform::time::monotonicMillis());
    buffer[index++] = PROTOCOL_TAIL;
    buffer[1] = static_cast<uint8_t>(index - 2);
    return FW_SERIAL_WRITE(buffer, index) == index;
}

bool enqueueProtoAck(uint8_t cmd, uint8_t appliedMode, uint8_t status)
{
    constexpr uint8_t kCapacity =
        sizeof(s_pendingProtoAcks) / sizeof(s_pendingProtoAcks[0]);
    if (s_pendingProtoAckCount >= kCapacity) return false;
    const uint8_t tail = static_cast<uint8_t>(
        (s_pendingProtoAckHead + s_pendingProtoAckCount) % kCapacity);
    s_pendingProtoAcks[tail] = {cmd, appliedMode, status};
    ++s_pendingProtoAckCount;
    return true;
}

} // namespace

namespace p4::host {

bool sendProtoAckPacket(uint8_t cmd, uint8_t appliedMode, uint8_t status)
{
    if (s_pendingProtoAckCount == 0 &&
        writeProtoAckPacket(cmd, appliedMode, status)) return true;
    return enqueueProtoAck(cmd, appliedMode, status);
}

bool retryPendingProtoAckPacket()
{
    if (s_pendingProtoAckCount == 0) return true;
    const PendingProtoAck& ack = s_pendingProtoAcks[s_pendingProtoAckHead];
    if (!writeProtoAckPacket(ack.cmd, ack.appliedMode, ack.status)) {
        return false;
    }
    constexpr uint8_t kCapacity =
        sizeof(s_pendingProtoAcks) / sizeof(s_pendingProtoAcks[0]);
    s_pendingProtoAckHead =
        static_cast<uint8_t>((s_pendingProtoAckHead + 1U) % kCapacity);
    --s_pendingProtoAckCount;
    return true;
}

} // namespace p4::host
