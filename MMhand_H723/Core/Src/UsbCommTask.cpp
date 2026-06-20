#include "UsbCommTask.h"
#include "main.h"
#include "task.h"
#include "usbd_def.h"
#include "usbd_cdc_if.h"

#include <cstring>

UsbComm usb1;

static bool timeReached(uint32_t nowMs, uint32_t deadlineMs)
{
    return static_cast<int32_t>(nowMs - deadlineMs) >= 0;
}

// 保持 freertos.c 里的 C 任务入口不变，把实际 USB 通信逻辑转交给 usb1 对象。
extern "C" void UsbCommTask_Run(void *parameter)
{
    usb1.run(parameter);
}

// CDC 初始化回调入口，供 usbd_cdc_if.c 在 USB 虚拟串口 ready 时通知功能层。
extern "C" void UsbComm_OnCdcInit(void)
{
    usb1.onInit();
}

// CDC 反初始化回调入口，供 usbd_cdc_if.c 在 USB 虚拟串口关闭时通知功能层。
extern "C" void UsbComm_OnCdcDeInit(void)
{
    usb1.onDeInit();
}

// CDC 控制请求回调入口，用于记录上位机设置的串口参数和控制命令。
extern "C" void UsbComm_OnCdcControl(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
    usb1.onControl(cmd, pbuf, length);
}

// CDC GET_LINE_CODING 响应入口，向主机返回当前虚拟串口参数。
extern "C" void UsbComm_FillLineCoding(uint8_t *pbuf, uint16_t length)
{
    usb1.fillLineCoding(pbuf, length);
}

// CDC 接收回调入口，把 USB OUT 端点收到的原始数据保存到 usb1.RX* 字段。
extern "C" void UsbComm_OnCdcRx(uint8_t *data, uint32_t length)
{
    usb1.onRx(data, length);
}

// CDC 发送完成回调入口，用于清理发送 busy 标志并更新调试计数。
extern "C" void UsbComm_OnCdcTxComplete(uint8_t *data, uint32_t *length, uint8_t epnum)
{
    usb1.onTxComplete(data, length, epnum);
}

// C 侧通用 USB CDC 发送入口，后续协议层可以直接复用。
extern "C" uint8_t UsbComm_Send(const uint8_t *data, uint16_t length)
{
    return usb1.send(data, length);
}

extern "C" uint8_t UsbComm_EnqueueTx(const uint8_t *data, uint16_t length)
{
    return usb1.enqueueTx(data, length);
}

extern "C" void UsbComm_SetStreamProvider(UsbCommStreamProvider_t provider)
{
    usb1.setStreamProvider(provider);
}

extern "C" uint8_t UsbComm_StartTxBenchmark(uint32_t durationMs)
{
    return usb1.startTxBenchmark(durationMs);
}

extern "C" uint8_t UsbComm_IsTxBenchmarkRunning(void)
{
    return usb1.isTxBenchmarkRunning();
}

// 返回当前系统毫秒计时，保持与 CAN 层 millis() 风格一致。
uint32_t UsbComm::millis(void) const
{
    return HAL_GetTick();
}

// 将任意长度限制到 usb1 调试缓存能够保存的最大长度。
uint16_t UsbComm::clampLen(uint32_t length) const
{
    if (length > USB_COMM_MAX_DATA_SIZE) {
        return USB_COMM_MAX_DATA_SIZE;
    }

    return static_cast<uint16_t>(length);
}

// 判断当前回调是否运行在中断上下文中，以便选择正确的 FreeRTOS 队列 API。
uint8_t UsbComm::isInInterrupt(void) const
{
    return (__get_IPSR() != 0U) ? 1U : 0U;
}

void UsbComm::notifyTask(void)
{
    if (taskHandle == nullptr) {
        return;
    }

    if (isInInterrupt() != 0U) {
        BaseType_t higherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(taskHandle, &higherPriorityTaskWoken);
        portYIELD_FROM_ISR(higherPriorityTaskWoken);
    } else {
        xTaskNotifyGive(taskHandle);
    }
}

// 标记 USB CDC 已初始化，同时设置默认虚拟串口参数。
void UsbComm::onInit(void)
{
    cdcInitialized = 1;
    hostConnected = 0;
    txBusy = 0;
    initCount++;
    notifyTask();

    if (lineDTERate == 0U) {
        lineDTERate = 115200U;
        lineCharFormat = 0;
        lineParityType = 0;
        lineDataBits = 8;
    }
}

// 标记 USB CDC 已关闭，发送 busy 状态随之清空。
void UsbComm::onDeInit(void)
{
    cdcInitialized = 0;
    hostConnected = 0;
    txBusy = 0;
    benchmarkPending = 0;
    benchmarkActive = 0;
    deinitCount++;
    notifyTask();
}

// 记录主机发来的 CDC 控制命令，并解析 SET_LINE_CODING 的串口参数。
void UsbComm::onControl(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
    lastControlCmd = cmd;
    lastControlLength = length;
    controlCount++;

    if ((cmd == CDC_SET_LINE_CODING) && (pbuf != nullptr) && (length >= 7U)) {
        lineDTERate = static_cast<uint32_t>(pbuf[0]) |
                      (static_cast<uint32_t>(pbuf[1]) << 8) |
                      (static_cast<uint32_t>(pbuf[2]) << 16) |
                      (static_cast<uint32_t>(pbuf[3]) << 24);
        lineCharFormat = pbuf[4];
        lineParityType = pbuf[5];
        lineDataBits = pbuf[6];
    } else if ((cmd == CDC_SET_CONTROL_LINE_STATE) && (pbuf != nullptr) && (length == 0U)) {
        const USBD_SetupReqTypedef *request = reinterpret_cast<const USBD_SetupReqTypedef *>(pbuf);
        controlLineState = request->wValue;
        hostConnected = ((controlLineState & 0x0001U) != 0U) ? 1U : 0U;
        if (hostConnected == 0U) {
            txBusy = 0;
            benchmarkPending = 0;
            benchmarkActive = 0;
        }
        notifyTask();
    }
}

// 填充 CDC GET_LINE_CODING 响应，让主机能读到当前虚拟串口参数。
void UsbComm::fillLineCoding(uint8_t *pbuf, uint16_t length) const
{
    if ((pbuf == nullptr) || (length < 7U)) {
        return;
    }

    pbuf[0] = static_cast<uint8_t>(lineDTERate & 0xFFU);
    pbuf[1] = static_cast<uint8_t>((lineDTERate >> 8) & 0xFFU);
    pbuf[2] = static_cast<uint8_t>((lineDTERate >> 16) & 0xFFU);
    pbuf[3] = static_cast<uint8_t>((lineDTERate >> 24) & 0xFFU);
    pbuf[4] = lineCharFormat;
    pbuf[5] = lineParityType;
    pbuf[6] = lineDataBits;
}

// 记录 USB 原始接收数据，覆盖 usbRxQueue，并准备一份 echo 数据给 USB 任务发送。
void UsbComm::onRx(uint8_t *data, uint32_t length)
{
    const uint16_t copyLen = clampLen(length);
    const uint32_t nowMs = millis();

    RXlen = copyLen;
    RXtimestamp = nowMs;
    RXpacketCount++;
    RXbyteCount += length;
    if (length > USB_COMM_MAX_DATA_SIZE) {
        RXtruncatedCount++;
    }

    std::memset(RXdata, 0, sizeof(RXdata));
    if ((data != nullptr) && (copyLen > 0U)) {
        std::memcpy(RXdata, data, copyLen);
    }

    UsbPacket_t packet = {};
    packet.len = copyLen;
    packet.timestamp = nowMs;
    if (copyLen > 0U) {
        std::memcpy(packet.payload, RXdata, copyLen);
    }

    if ((shared != nullptr) && (shared->usbRxQueue != nullptr)) {
        if (isInInterrupt() != 0U) {
            BaseType_t higherPriorityTaskWoken = pdFALSE;
            if (xQueueSendToBackFromISR(shared->usbRxQueue, &packet, &higherPriorityTaskWoken) != pdTRUE) {
                RXqueueDropCount++;
            }
            portYIELD_FROM_ISR(higherPriorityTaskWoken);
        } else {
            if (xQueueSendToBack(shared->usbRxQueue, &packet, 0) != pdTRUE) {
                UsbPacket_t droppedPacket = {};
                (void)xQueueReceive(shared->usbRxQueue, &droppedPacket, 0);
                if (xQueueSendToBack(shared->usbRxQueue, &packet, 0) != pdTRUE) {
                    RXqueueDropCount++;
                }
            }
        }
    }

    std::memset(echoData, 0, sizeof(echoData));
    if (copyLen > 0U) {
        std::memcpy(echoData, RXdata, copyLen);
    }
    echoLen = copyLen;
    echoPending = (copyLen > 0U) ? 1U : 0U;
    notifyTask();
}

// 记录 USB IN 端点发送完成事件，释放下一次发送。
void UsbComm::onTxComplete(uint8_t *data, uint32_t *length, uint8_t epnum)
{
    (void)data;
    (void)epnum;

    txBusy = 0;
    TXcompleteCount++;
    if (length != nullptr) {
        TXbyteCount += *length;
    }
    notifyTask();
}

// 通过 CDC 虚拟串口发送一段数据，同时保存到 usb1.TX* 字段供调试器查看。
uint8_t UsbComm::send(const uint8_t *data, uint16_t length)
{
    if ((cdcInitialized == 0U) || (data == nullptr) || (length == 0U)) {
        TXfailCount++;
        return USBD_FAIL;
    }

    if (txBusy != 0U) {
        TXbusyCount++;
        return USBD_BUSY;
    }

    const uint16_t copyLen = clampLen(length);
    TXlen = copyLen;
    TXtimestamp = millis();

    std::memset(TXdata, 0, sizeof(TXdata));
    std::memcpy(TXdata, data, copyLen);

    const uint8_t result = CDC_Transmit_HS(TXdata, copyLen);
    if (result == USBD_OK) {
        txBusy = 1;
        TXpacketCount++;
    } else if (result == USBD_BUSY) {
        TXbusyCount++;
    } else {
        TXfailCount++;
    }

    return result;
}

uint8_t UsbComm::sendDirect(uint8_t *data, uint16_t length)
{
    if ((cdcInitialized == 0U) || (hostConnected == 0U) || (data == nullptr) || (length == 0U)) {
        TXfailCount++;
        return USBD_FAIL;
    }

    if (txBusy != 0U) {
        TXbusyCount++;
        return USBD_BUSY;
    }

    TXlen = length;
    TXtimestamp = millis();
    const uint16_t debugLen = clampLen(length);
    std::memset(TXdata, 0, sizeof(TXdata));
    std::memcpy(TXdata, data, debugLen);

    const uint8_t result = CDC_Transmit_HS(data, length);
    if (result == USBD_OK) {
        txBusy = 1;
        TXpacketCount++;
    } else if (result == USBD_BUSY) {
        TXbusyCount++;
    } else {
        TXfailCount++;
    }

    return result;
}

uint8_t UsbComm::enqueueTx(const uint8_t *data, uint16_t length)
{
    if ((shared == nullptr) ||
        (shared->usbTxQueue == nullptr) ||
        (data == nullptr) ||
        (length == 0U) ||
        (length > USB_COMM_MAX_DATA_SIZE)) {
        TXqueueDropCount++;
        return 0U;
    }

    UsbPacket_t packet = {};
    packet.len = length;
    packet.timestamp = millis();
    std::memcpy(packet.payload, data, length);

    if (xQueueSendToBack(shared->usbTxQueue, &packet, 0) != pdTRUE) {
        TXqueueFullCount++;
        UsbPacket_t droppedPacket = {};
        (void)xQueueReceive(shared->usbTxQueue, &droppedPacket, 0);
        if (xQueueSendToBack(shared->usbTxQueue, &packet, 0) != pdTRUE) {
            TXqueueDropCount++;
            return 0U;
        }
    }

    notifyTask();
    return 1U;
}

void UsbComm::setStreamProvider(UsbCommStreamProvider_t provider)
{
    streamProvider = provider;
    notifyTask();
}

uint8_t UsbComm::startTxBenchmark(uint32_t durationMs)
{
    benchmarkPending = 0;
    benchmarkActive = 0;
    benchmarkDurationMs = durationMs;
    benchmarkPayloadBytes = 0;
    benchmarkPacketCount = 0;
    benchmarkSeq = 0;
    benchmarkStartMs = 0;
    benchmarkStopMs = millis();

    if (durationMs != 0U) {
        benchmarkStartAtMs = millis() + USB_COMM_TX_BENCHMARK_ACK_DELAY_MS;
        benchmarkPending = 1;
    }

    notifyTask();
    return 1U;
}

uint8_t UsbComm::isTxBenchmarkRunning(void) const
{
    return ((benchmarkPending != 0U) || (benchmarkActive != 0U)) ? 1U : 0U;
}

void UsbComm::updateBenchmarkState(void)
{
    const uint32_t nowMs = millis();

    if ((benchmarkPending != 0U) && timeReached(nowMs, benchmarkStartAtMs)) {
        benchmarkPending = 0;
        benchmarkActive = 1;
        benchmarkStartMs = nowMs;
        benchmarkStopMs = nowMs;
    }

    if ((benchmarkActive != 0U) && ((nowMs - benchmarkStartMs) >= benchmarkDurationMs)) {
        benchmarkActive = 0;
        benchmarkStopMs = nowMs;
    }
}

uint16_t UsbComm::fillBenchmarkData(uint8_t *buffer, uint16_t maxLen)
{
    if ((buffer == nullptr) || (maxLen == 0U)) {
        return 0U;
    }

    const uint32_t seq = benchmarkSeq++;
    buffer[0] = static_cast<uint8_t>(seq & 0xFFU);
    buffer[1] = static_cast<uint8_t>((seq >> 8) & 0xFFU);
    buffer[2] = static_cast<uint8_t>((seq >> 16) & 0xFFU);
    buffer[3] = static_cast<uint8_t>((seq >> 24) & 0xFFU);

    for (uint16_t i = 4U; i < maxLen; i++) {
        buffer[i] = static_cast<uint8_t>((seq + i) & 0xFFU);
    }

    return maxLen;
}

void UsbComm::processStreamProvider(void)
{
    if ((cdcInitialized == 0U) || (hostConnected == 0U) || (txBusy != 0U)) {
        return;
    }

    uint16_t streamLen = 0U;
    if (benchmarkActive != 0U) {
        streamLen = fillBenchmarkData(streamData, sizeof(streamData));
    } else if (streamProvider != nullptr) {
        streamLen = streamProvider(streamData, sizeof(streamData));
    }

    if (streamLen == 0U) {
        return;
    }

    if (sendDirect(streamData, streamLen) == USBD_OK) {
        if (benchmarkActive != 0U) {
            benchmarkPayloadBytes += streamLen;
            benchmarkPacketCount++;
        }
    }
}

// 如果开启 echo 测试，把最近一次 USB RX 数据原样发回主机。
void UsbComm::processEcho(void)
{
    if ((echoEnabled == 0U) || (echoPending == 0U) || (cdcInitialized == 0U)) {
        return;
    }

    uint8_t localEcho[USB_COMM_MAX_DATA_SIZE] = {};
    uint16_t localLen = 0;

    taskENTER_CRITICAL();
    if (echoPending != 0U) {
        localLen = echoLen;
        std::memcpy(localEcho, echoData, localLen);
        echoPending = 0;
    }
    taskEXIT_CRITICAL();

    if (localLen > 0U) {
        if (send(localEcho, localLen) == USBD_BUSY) {
            taskENTER_CRITICAL();
            std::memcpy(echoData, localEcho, localLen);
            echoLen = localLen;
            echoPending = 1;
            taskEXIT_CRITICAL();
        }
    }
}

// 从 usbTxQueue 取上层待发送数据，通过 CDC 虚拟串口发给主机。
void UsbComm::processTxQueue(void)
{
    if ((shared == nullptr) ||
        (shared->usbTxQueue == nullptr) ||
        (cdcInitialized == 0U) ||
        (hostConnected == 0U)) {
        return;
    }

    if (txBusy != 0U) {
        return;
    }

    UsbPacket_t packet = {};
    if (xQueueReceive(shared->usbTxQueue, &packet, 0) != pdTRUE) {
        return;
    }

    if (send(packet.payload, packet.len) == USBD_BUSY) {
        (void)xQueueSendToFront(shared->usbTxQueue, &packet, 0);
    }
}

// USB 通信任务主体：循环处理 echo 测试和上层 USB 发送队列。
void UsbComm::run(void *parameter)
{
    shared = static_cast<TaskSharedData_t *>(parameter);

    if (shared == nullptr) {
        vTaskDelete(nullptr);
        return;
    }

    taskHandle = xTaskGetCurrentTaskHandle();
    taskRunning = 1;
    cdcInitialized = 0;
    hostConnected = 0;
    echoEnabled = 1;
    txBusy = 0;
    echoPending = 0;
    RXlen = 0;
    RXtimestamp = 0;
    RXpacketCount = 0;
    RXbyteCount = 0;
    RXtruncatedCount = 0;
    RXqueueDropCount = 0;
    TXlen = 0;
    TXtimestamp = 0;
    TXpacketCount = 0;
    TXbyteCount = 0;
    TXcompleteCount = 0;
    TXbusyCount = 0;
    TXfailCount = 0;
    TXqueueFullCount = 0;
    TXqueueDropCount = 0;
    lastControlCmd = 0;
    lastControlLength = 0;
    controlCount = 0;
    lineDTERate = 115200U;
    lineCharFormat = 0;
    lineParityType = 0;
    lineDataBits = 8;
    controlLineState = 0;
    initCount = 0;
    deinitCount = 0;
    echoLen = 0;
    streamProvider = nullptr;
    benchmarkPending = 0;
    benchmarkActive = 0;
    benchmarkStartAtMs = 0;
    benchmarkStartMs = 0;
    benchmarkStopMs = 0;
    benchmarkDurationMs = 0;
    benchmarkPayloadBytes = 0;
    benchmarkPacketCount = 0;
    benchmarkSeq = 0;
    std::memset(RXdata, 0, sizeof(RXdata));
    std::memset(TXdata, 0, sizeof(TXdata));
    std::memset(echoData, 0, sizeof(echoData));
    std::memset(streamData, 0, sizeof(streamData));

    for (;;) {
        updateBenchmarkState();

        if (benchmarkActive == 0U) {
            processEcho();
        }

        if (txBusy == 0U) {
            if ((benchmarkActive != 0U) || (streamProvider != nullptr)) {
                processStreamProvider();
            }

            if ((txBusy == 0U) && (benchmarkActive == 0U)) {
                processTxQueue();
            }
        }

        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1));
    }
}
