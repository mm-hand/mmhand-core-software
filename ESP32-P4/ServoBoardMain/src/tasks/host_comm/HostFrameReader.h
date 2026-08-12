/**
 * @file HostFrameReader.h
 * @brief 定义上位机串口字节流到完整协议帧的增量解码器。
 * @details
 * - 线程：由 HostCommTask 单线程调用。
 * - 输入输出：逐字节 push，按“CMD + payload”形式 pop 完整帧。
 * - 所有权：内部固定缓冲区属于对象，不持有串口或 SystemRuntimeContext。
 */
#ifndef P4_MODULES_COMMUNICATION_PROTOCOL_FRAME_READER_H
#define P4_MODULES_COMMUNICATION_PROTOCOL_FRAME_READER_H

#include <stddef.h>
#include <stdint.h>

#include "../../modules/protocol/HostProtocol.h"

namespace p4::host {

/** 上位机字节流被拒绝的原因；仅用于 COM-6005 诊断，不参与协议 ACK。 */
enum class ProtocolFrameRejectReason : uint8_t {
    None = 0,
    BufferOverflow = 1,
    InvalidWireLength = 2,
    InvalidTail = 3,
    UnsupportedCommand = 4,
    PayloadLengthMismatch = 5,
    OutputBufferTooSmall = 6,
};

class HostFrameReader {
public:
    HostFrameReader();

    /** 追加一个串口字节；缓冲区满时丢弃最旧字节以继续寻找帧头。 */
    void push(uint8_t value);

    /**
     * 弹出一帧已通过帧头、帧尾和命令长度校验的数据。
     * outFrame[0] 为命令号，后续为 payload；没有完整帧时返回 false。
     */
    bool pop(uint8_t* outFrame, size_t capacity, size_t* outLength);

    void reset();

    /** 读取并消费最近一次拒绝原因；没有新拒绝时返回 false。 */
    bool takeRejectReason(ProtocolFrameRejectReason* reason);

private:
    // 缓冲区尺寸来自生成协议契约；协议常量位于全局作用域，Codec 函数才属于 p4::host。
    uint8_t buffer_[kSerialRxBufferSize];
    size_t length_;
    ProtocolFrameRejectReason lastRejectReason_;
    uint32_t rejectSequence_;
    uint32_t consumedRejectSequence_;

    void discardPrefix(size_t count);
    void noteReject(ProtocolFrameRejectReason reason);
};

} // namespace p4::host

#endif // P4_MODULES_COMMUNICATION_PROTOCOL_FRAME_READER_H
