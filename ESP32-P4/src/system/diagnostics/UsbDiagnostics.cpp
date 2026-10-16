/** @file UsbDiagnostics.cpp
 * @brief 汇总启动、任务和 USB 诊断，保留可跨软件复位的 RAM 快照。
 */
#include "UsbDiagnostics.h"
#include <Arduino.h>
#include <esp_attr.h>
#include <esp_system.h>
#include "../../hal/serial/HostSerialPort.h"
#include "../TaskRuntimeMetrics.h"
#include "../log/FirmwareLog.h"
#include "../../generated/ProductFingerprint.generated.h"
#include <stddef.h>

namespace p4::diagnostics {
namespace {
constexpr uint32_t kMagic = 0x55444231U;
// .noinit is not cleared by application startup. Validity is checked on every
// boot; power-on/brownout always discard it. ROM download/flash can overwrite
// this RAM, so this record is explicitly NOT a persistent crash dump.
struct RetainedRecord {
    uint32_t magic, boot, phase, uptimeMs;
    HostSerialStatistics usb;
    uint32_t heap, minHeap, loops[TASK_RUNTIME_COUNT], stack[TASK_RUNTIME_COUNT];
    uint32_t checksum;
};
__NOINIT_ATTR RetainedRecord retained;
RetainedRecord previous = {};
uint32_t boot = 1, phase = 0, resetReason = 0, lastSample = 0, lastReport = 0;
bool previousValid = false;
uint32_t checksum(const RetainedRecord& record) {
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&record);
    uint32_t value = 2166136261U;
    for (size_t i = 0; i < offsetof(RetainedRecord, checksum); ++i)
        value = (value ^ bytes[i]) * 16777619U;
    return value;
}
}

void beginUsbDiagnostics() {
    resetReason = (uint32_t)esp_reset_reason();
    previousValid = resetReason != ESP_RST_POWERON && resetReason != ESP_RST_BROWNOUT &&
                    retained.magic == kMagic && retained.checksum == checksum(retained);
    if (previousValid) { previous = retained; boot = previous.boot + 1; }
    markUsbBootPhase(1);
}

void markUsbBootPhase(uint32_t value) {
    __atomic_store_n(&phase, value, __ATOMIC_RELAXED);
    // RuntimeMonitor can already be running when assembly returns. It alone
    // owns retained snapshots after task creation; don't race its checksum.
    if (value >= 3) return;
    retained = {};
    retained.magic = kMagic;
    retained.boot = boot;
    retained.phase = value;
    retained.checksum = checksum(retained);
}

void sampleUsbDiagnostics(uint32_t nowMs) {
    if ((uint32_t)(nowMs - lastSample) < 1000U) return;
    lastSample = nowMs;
    RetainedRecord next = {};
    next.magic = kMagic; next.boot = boot;
    next.phase = __atomic_load_n(&phase, __ATOMIC_RELAXED); next.uptimeMs = nowMs;
    next.usb = HostSerialPort::statistics();
    next.heap = ESP.getFreeHeap(); next.minHeap = ESP.getMinFreeHeap();
    for (uint8_t id = 0; id < TASK_RUNTIME_COUNT; ++id) {
        TaskRuntimeSnapshot task = {};
        if (taskRuntimeGetSnapshot(id, &task)) {
            next.loops[id] = task.loopCount;
            next.stack[id] = task.stackMinFreeBytes;
        }
    }
    next.checksum = checksum(next);
    retained = next;
    if ((uint32_t)(nowMs - lastReport) < 5000U) return;
    lastReport = nowMs;
    // Low-rate, zero-wait log admission. No additional wire packet or protocol
    // version is needed: existing firmware-text decoding receives these lines.
    FW_LOG_INFO_PRINTF("[USB_BOOT] fw=%08lX boot=%lu reset=%lu phase=%lu retained=%u prevBoot=%lu prevPhase=%lu prevUp=%lu\n",
        (unsigned long)kProductFirmwareSourceCrc,
        (unsigned long)boot, (unsigned long)resetReason, (unsigned long)next.phase,
        previousValid, (unsigned long)previous.boot, (unsigned long)previous.phase,
        (unsigned long)previous.uptimeMs);
    FW_LOG_INFO_PRINTF("[USB_DIAG] up=%lu heap=%lu minHeap=%lu rx=%lu tx=%lu short=%lu reset=%lu lastRx=%lu lastTx=%lu cmd=%lu lastCmd=%lu\n",
        (unsigned long)nowMs, (unsigned long)next.heap, (unsigned long)next.minHeap,
        (unsigned long)next.usb.rxBytes, (unsigned long)next.usb.txBytes,
        (unsigned long)next.usb.shortWrites, (unsigned long)next.usb.busResets,
        (unsigned long)next.usb.lastRxMs, (unsigned long)next.usb.lastTxMs,
        (unsigned long)next.usb.commands, (unsigned long)next.usb.lastCommandMs);
    for (uint8_t id = 0; id < TASK_RUNTIME_COUNT; ++id)
        FW_LOG_INFO_PRINTF("[USB_TASK] id=%u loops=%lu stackMin=%lu\n", id,
            (unsigned long)next.loops[id], (unsigned long)next.stack[id]);
    FW_LOG_INFO_PRINTF("[USB_TX] zero=%lu maxUs=%lu\n",
        (unsigned long)next.usb.zeroWrites, (unsigned long)next.usb.maxWriteUs);
}
}
