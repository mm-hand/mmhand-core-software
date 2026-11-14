/**
 * @file TwaiCanDriver.cpp
 * @brief 实现 ESP32-P4 TWAI 安装、接收、状态读取和 Bus-Off 恢复。
 */
#include "TwaiCanDriver.h"

#include "../../config/BoardConfig.h"

#include <driver/twai.h>
#include <hal/twai_ll.h>
#include <esp_err.h>
#include <esp_ldo_regulator.h>
#include "../../system/rtos/FreeRtosKernel.h"
#include <string.h>

TwaiCanDriver::TwaiCanDriver()
    : installed_(false), started_(false), ioPowerHandle_(nullptr)
{
}

bool TwaiCanDriver::ensureIoPower()
{
    const bool usesVo4 = TWAI_TX_PIN == 47 || TWAI_TX_PIN == 48 ||
                         TWAI_RX_PIN == 47 || TWAI_RX_PIN == 48;
    if (!usesVo4 || ioPowerHandle_ != nullptr) {
        return true;
    }

    esp_ldo_channel_config_t config = {
        .chan_id = 4,
        .voltage_mv = 3300,
        .flags = {.adjustable = 1},
    };
    esp_ldo_channel_handle_t handle = nullptr;
    if (esp_ldo_acquire_channel(&config, &handle) != ESP_OK ||
        esp_ldo_channel_adjust_voltage(handle, 3300) != ESP_OK) {
        return false;
    }
    ioPowerHandle_ = handle;
    vTaskDelay(pdMS_TO_TICKS(20));
    return true;
}

bool TwaiCanDriver::begin()
{
    if (started_) {
        return true;
    }
    if (!ensureIoPower()) {
        return false;
    }
    if (!installed_) {
        twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
            (gpio_num_t)TWAI_TX_PIN, (gpio_num_t)TWAI_RX_PIN, TWAI_MODE_NORMAL);
        twai_timing_config_t timing = TWAI_TIMING_CONFIG_1MBITS();
        twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        general.rx_queue_len = 512;
        if (twai_driver_install(&general, &timing, &filter) != ESP_OK) {
            return false;
        }
        installed_ = true;
    }
    started_ = twai_start() == ESP_OK;
    return started_;
}

bool TwaiCanDriver::tryReceive(CanFrame* outFrame)
{
    if (!outFrame || !started_) {
        return false;
    }
    twai_message_t message;
    if (twai_receive(&message, 0) != ESP_OK) {
        return false;
    }
    outFrame->identifier = message.identifier;
    outFrame->dataLength = message.data_length_code;
    memcpy(outFrame->data, message.data, sizeof(outFrame->data));
    return true;
}

CanControllerStatus TwaiCanDriver::status() const
{
    CanControllerStatus result = {};
    if (!installed_) {
        return result;
    }
    twai_status_info_t info;
    if (twai_get_status_info(&info) != ESP_OK) {
        return result;
    }
    result.rxMissedCount = info.rx_missed_count;
    result.rxOverrunCount = info.rx_overrun_count;
    result.busErrorCount = info.bus_error_count;
    result.pendingRxCount = info.msgs_to_rx;
    result.rxErrorCounter = info.rx_error_counter;
    result.txErrorCounter = info.tx_error_counter;
    result.state = (uint8_t)info.state;
    result.valid = 1;
    result.running = info.state == TWAI_STATE_RUNNING;
    result.busOff = info.state == TWAI_STATE_BUS_OFF ? 1 : 0;
    result.stopped = info.state == TWAI_STATE_STOPPED ? 1 : 0;
    return result;
}

int32_t TwaiCanDriver::requestRecovery()
{
    return installed_ ? twai_initiate_recovery() : ESP_ERR_INVALID_STATE;
}

int32_t TwaiCanDriver::restart()
{
    const esp_err_t result = installed_ ? twai_start() : ESP_ERR_INVALID_STATE;
    if (result == ESP_OK) started_ = true;
    return result;
}

uint32_t TwaiCanDriver::rxFifoCount() const
{
    return twai_ll_get_rx_msg_count(TWAI_LL_GET_HW(0));
}

int32_t TwaiCanDriver::stop()
{
    const esp_err_t result = installed_ ? twai_stop() : ESP_ERR_INVALID_STATE;
    if (result == ESP_OK) started_ = false;
    return result;
}
