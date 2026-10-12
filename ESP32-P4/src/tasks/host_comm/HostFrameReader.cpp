/**
 * @file HostFrameReader.cpp
 * @brief 实现可重同步的上位机协议增量帧解码。
 */
#include "HostFrameReader.h"

#include <string.h>

#include "HostProtocolCodec.h"

namespace p4::host {

HostFrameReader::HostFrameReader()
    : buffer_{0},
      length_(0),
      lastRejectReason_(ProtocolFrameRejectReason::None),
      rejectSequence_(0),
      consumedRejectSequence_(0)
{}

void HostFrameReader::reset()
{
    length_ = 0;
    lastRejectReason_ = ProtocolFrameRejectReason::None;
    rejectSequence_ = 0;
    consumedRejectSequence_ = 0;
}

void HostFrameReader::noteReject(ProtocolFrameRejectReason reason)
{
    lastRejectReason_ = reason;
    ++rejectSequence_;
}

bool HostFrameReader::takeRejectReason(ProtocolFrameRejectReason* reason)
{
    if (!reason || consumedRejectSequence_ == rejectSequence_) return false;
    *reason = lastRejectReason_;
    consumedRejectSequence_ = rejectSequence_;
    return true;
}

void HostFrameReader::discardPrefix(size_t count)
{
    if (count >= length_) {
        length_ = 0;
        return;
    }
    memmove(buffer_, buffer_ + count, length_ - count);
    length_ -= count;
}

void HostFrameReader::push(uint8_t value)
{
    if (length_ < sizeof(buffer_)) {
        buffer_[length_++] = value;
        return;
    }
    noteReject(ProtocolFrameRejectReason::BufferOverflow);
    memmove(buffer_, buffer_ + 1, sizeof(buffer_) - 1);
    buffer_[sizeof(buffer_) - 1] = value;
}

bool HostFrameReader::pop(uint8_t* outFrame,
                              size_t capacity,
                              size_t* outLength)
{
    if (!outFrame || !outLength) return false;

    while (length_ > 0) {
        if (buffer_[0] != PROTOCOL_HEADER) {
            discardPrefix(1);
            continue;
        }
        if (length_ < 4) return false;

        const uint8_t wireLength = buffer_[1];
        if (wireLength < 2) {
            noteReject(ProtocolFrameRejectReason::InvalidWireLength);
            discardPrefix(1);
            continue;
        }
        const size_t fullFrameLength = (size_t)wireLength + 2;
        if (length_ < fullFrameLength) return false;
        if (buffer_[fullFrameLength - 1] != PROTOCOL_TAIL) {
            noteReject(ProtocolFrameRejectReason::InvalidTail);
            discardPrefix(1);
            continue;
        }

        const uint8_t command = buffer_[2];
        const size_t payloadLength = (size_t)wireLength - 2;
        const size_t expected = p4::host::getCommandPayloadLength(command);
        const bool knownCommand = expected != (size_t)-1 ||
                                  command == CMD_CALIB_LOAD_JOINT;
        const bool validLength = command == CMD_CALIB_LOAD_JOINT
            ? payloadLength == kCalibLoadJointPayloadBytes
            : knownCommand && expected == payloadLength;
        const size_t decodedLength = payloadLength + 1;
        if (!knownCommand) {
            noteReject(ProtocolFrameRejectReason::UnsupportedCommand);
            discardPrefix(fullFrameLength);
            continue;
        }
        if (!validLength) {
            noteReject(ProtocolFrameRejectReason::PayloadLengthMismatch);
            discardPrefix(fullFrameLength);
            continue;
        }
        if (decodedLength > capacity) {
            noteReject(ProtocolFrameRejectReason::OutputBufferTooSmall);
            discardPrefix(fullFrameLength);
            continue;
        }

        memcpy(outFrame, buffer_ + 2, decodedLength);
        *outLength = decodedLength;
        discardPrefix(fullFrameLength);
        return true;
    }
    return false;
}

} // namespace p4::host
