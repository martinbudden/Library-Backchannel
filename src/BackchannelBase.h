#pragma once

#include <cstddef>
#include <cstdint>

class BackchannelTransceiverBase;

class BackchannelBase {
protected:
    BackchannelBase(BackchannelTransceiverBase& backchannelTransceiver);
public:
    void WAIT_FOR_DATA_RECEIVED();
    int sendData(const uint8_t* data, size_t len) const;

    virtual bool processedReceivedPacket() = 0;
    virtual bool sendPacket(uint8_t subCommand) = 0;
    bool sendPacket() { return sendPacket(0); }

protected:
    BackchannelTransceiverBase& _backchannelTransceiver;
    uint8_t* _transmitDataBufferPtr {};
    const size_t _transmitDataBufferSize {};
    uint8_t* _receivedDataBufferPtr {};
    const size_t _receivedDataBufferSize {};
};
