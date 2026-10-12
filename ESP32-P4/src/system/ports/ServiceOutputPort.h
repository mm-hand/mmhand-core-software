/**
 * @file ServiceOutputPort.h
 * @brief 定义业务服务向状态机发布系统事件的最小输出端口。
 * @details 安全证据必须通过 Safety Facade 提交，禁止在通用服务端口保留旁路。
 */
#ifndef P4_RUNTIME_PORTS_SERVICE_OUTPUT_PORT_H
#define P4_RUNTIME_PORTS_SERVICE_OUTPUT_PORT_H

#include <stdint.h>

struct ServiceOutputPort {
    void* context; ///< 适配器私有对象，业务层不得解释其类型。
    bool (*postEvent)(void* context,
                      uint8_t event,
                      uint32_t token,
                      bool tokenValid);
};

inline bool servicePostEvent(const ServiceOutputPort& port,
                             uint8_t event,
                             uint32_t token = 0,
                             bool tokenValid = false)
{
    return port.postEvent &&
           port.postEvent(port.context, event, token, tokenValid);
}

#endif // P4_RUNTIME_PORTS_SERVICE_OUTPUT_PORT_H
