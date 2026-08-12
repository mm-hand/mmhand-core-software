/**
 * @file NvsBlobStore.cpp
 * @brief 实现 ESP-IDF NVS 的生命周期与基础键值访问。
 */
#include "NvsBlobStore.h"

#include <esp_err.h>
#include <nvs.h>
#include <nvs_flash.h>

namespace {
bool g_initialized = false; ///< 整个固件共享一次 NVS Flash 初始化结果。
}

NvsBlobStore::NvsBlobStore() : handle_(0) {}
NvsBlobStore::~NvsBlobStore() { close(); }

bool NvsBlobStore::initialize()
{
    if (g_initialized) return true;
    esp_err_t error = nvs_flash_init();
    if (error == ESP_ERR_NVS_NO_FREE_PAGES || error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        error = nvs_flash_erase();
        if (error == ESP_OK) error = nvs_flash_init();
    }
    g_initialized = error == ESP_OK;
    return g_initialized;
}

bool NvsBlobStore::open(const char* namespaceName, Mode mode)
{
    close();
    if (!initialize() || !namespaceName) return false;
    nvs_handle_t nativeHandle = 0;
    const nvs_open_mode_t nativeMode = mode == Mode::ReadOnly ? NVS_READONLY : NVS_READWRITE;
    if (nvs_open(namespaceName, nativeMode, &nativeHandle) != ESP_OK) return false;
    handle_ = (uint32_t)nativeHandle;
    return true;
}

void NvsBlobStore::close()
{
    if (handle_ != 0) {
        nvs_close((nvs_handle_t)handle_);
        handle_ = 0;
    }
}

bool NvsBlobStore::getBlob(const char* key, void* data, size_t* length) const
{
    return handle_ != 0 && key && length &&
           nvs_get_blob((nvs_handle_t)handle_, key, data, length) == ESP_OK;
}

bool NvsBlobStore::setBlob(const char* key, const void* data, size_t length)
{
    return handle_ != 0 && key && data &&
           nvs_set_blob((nvs_handle_t)handle_, key, data, length) == ESP_OK;
}

bool NvsBlobStore::getU8(const char* key, uint8_t* value) const
{
    return handle_ != 0 && key && value &&
           nvs_get_u8((nvs_handle_t)handle_, key, value) == ESP_OK;
}

bool NvsBlobStore::setU8(const char* key, uint8_t value)
{
    return handle_ != 0 && key && nvs_set_u8((nvs_handle_t)handle_, key, value) == ESP_OK;
}

bool NvsBlobStore::eraseKey(const char* key, bool* wasMissing)
{
    if (wasMissing) *wasMissing = false;
    if (handle_ == 0 || !key) return false;
    const esp_err_t error = nvs_erase_key((nvs_handle_t)handle_, key);
    if (error == ESP_ERR_NVS_NOT_FOUND) {
        if (wasMissing) *wasMissing = true;
        return true;
    }
    return error == ESP_OK;
}

bool NvsBlobStore::commit()
{
    return handle_ != 0 && nvs_commit((nvs_handle_t)handle_) == ESP_OK;
}
