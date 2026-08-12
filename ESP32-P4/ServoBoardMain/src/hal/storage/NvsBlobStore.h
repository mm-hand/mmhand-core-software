/**
 * @file NvsBlobStore.h
 * @brief 封装 ESP-IDF NVS namespace、blob、u8 和事务提交操作。
 * @details persistence 模块只处理快照版本、CRC 和 A/B 槽，不再直接包含 nvs.h。
 */
#ifndef P4_HAL_STORAGE_NVS_BLOB_STORE_H
#define P4_HAL_STORAGE_NVS_BLOB_STORE_H

#include <stddef.h>
#include <stdint.h>

class NvsBlobStore {
public:
    enum class Mode : uint8_t { ReadOnly, ReadWrite };

    NvsBlobStore();
    ~NvsBlobStore();

    static bool initialize();
    bool open(const char* namespaceName, Mode mode);
    void close();
    bool getBlob(const char* key, void* data, size_t* length) const;
    bool setBlob(const char* key, const void* data, size_t length);
    bool getU8(const char* key, uint8_t* value) const;
    bool setU8(const char* key, uint8_t value);
    bool eraseKey(const char* key, bool* wasMissing = nullptr);
    bool commit();

private:
    uint32_t handle_; ///< nvs_handle_t 的平台无关存储。
};

#endif // P4_HAL_STORAGE_NVS_BLOB_STORE_H
