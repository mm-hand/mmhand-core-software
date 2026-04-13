#ifndef HAL_ENCODERS_H
#define HAL_ENCODERS_H

#include <Arduino.h>
#include <SPI.h>
#include "driver/spi_master.h"
#include "../config/Config.h"

class HalEncoders {
public:
    static HalEncoders &getInstance() {
        static HalEncoders instance;
        return instance;
    }

    void begin();
    void getData(EncoderData &outData);
    EncoderData getData();

    static const char *getErrorString(uint16_t errorCode);

private:
    HalEncoders();

    static constexpr uint8_t GROUP_COUNT = 5;
    static constexpr uint8_t MAX_GROUP_SIZE = 8;

    static constexpr uint16_t AS5047P_REG_ERRFL = 0x0001;
    static constexpr uint16_t AS5047P_REG_DIAAGC = 0x3FFC;
    static constexpr uint16_t AS5047P_REG_ANGLECOM = 0x3FFF;

    static constexpr uint8_t MAX_RECOVERY = 3;

    spi_device_handle_t _spi;
    const uint8_t group_sizes_[GROUP_COUNT] = {4, 4, 4, 4, 5};

    EncoderFSMState _fsmStates[ENCODER_TOTAL_NUM];
    uint16_t _cmdPipeline[ENCODER_TOTAL_NUM];
    uint16_t _latchedErrorCodes[ENCODER_TOTAL_NUM];
    uint8_t _recoveryAttempts[ENCODER_TOTAL_NUM];

    void setupSPI();
    void setMux(uint8_t channel);
    bool transferOnce(uint8_t muxChannel, uint8_t count, const uint16_t *txBuf, uint16_t *rxBuf);

    void collectGroup138(uint8_t groupIndex, int groupStart, uint8_t count, EncoderData &outData);
    void collectGroup151(uint8_t groupIndex, int groupStart, uint8_t count, EncoderData &outData);
    void decodeFrame(
        EncoderData &outData,
        int encoderId,
        uint16_t rawVal,
        uint16_t issuedCmd,
        bool fullDiag
    );

    void markLinkLost(EncoderData &outData, int encoderId);
    uint16_t pickRegisterForState(int encoderId, bool fullDiag) const;
    uint8_t getGroupMuxChannel(uint8_t groupIndex) const;
    uint8_t get_parity(uint16_t n);
    uint16_t build_command_frame(uint16_t address, bool is_read);
    uint8_t remapGroupIndex(uint8_t groupCount, uint8_t transferIndex) const;
    int mapGlobalEncoderIndex(int groupStart, uint8_t groupCount, uint8_t transferIndex) const;
};

extern HalEncoders &encoders;

#endif
