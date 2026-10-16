/** @file UsbDiagnostics.h
 * @brief 声明无复位、无运动副作用的 USB 诊断采样接口。
 */
#pragma once
#include <stdint.h>

namespace p4::diagnostics {
// Diagnostic-only: no reset, flash writes, control commands or motion ownership.
void beginUsbDiagnostics();
void markUsbBootPhase(uint32_t phase);
void sampleUsbDiagnostics(uint32_t nowMs);
}
