#pragma once

#include <cstddef>
#include <cstdint>
/*!
Virtual base class for the backchannel transceiver.
Can be subclassed to provide transceivers over ESPNOW or UDP (for example).
*/
class BackchannelTransceiverBase {
public:
    virtual int sendData(const uint8_t* data, size_t len) const = 0;
    virtual void WAIT_FOR_DATA_RECEIVED() = 0;
    virtual size_t getReceivedDataLength() const = 0;
    virtual void setReceivedDataLengthToZero() = 0;
    virtual uint32_t getTickCountDeltaAndReset() = 0;

    uint8_t* getTransmitDataBufferPtr() { return &_transmitDataBuffer[0]; }
    size_t getTransmitDataBufferSize() const { return sizeof(_transmitDataBuffer); }
    uint8_t* getReceivedDataBufferPtr() { return &_receivedDataBuffer[0]; }
    size_t getReceivedDataBufferSize() const { return sizeof(_receivedDataBuffer); }
protected:
    // If using MSP, then the MSP packets are packed into _transmitDataBuffer by MSP::processOutCommand,
    // so _transmitDataBuffer must be large enough to hold the largest MSP packet.
    // If the packet length exceeds ESP_NOW_MAX_DATA_LEN, then it is not sent,
    // but we don't know its length until we have unpacked it.
    uint8_t _transmitDataBuffer[512] {};
    uint8_t _receivedDataBuffer[256] {}; // must be >= ESP_NOW_MAX_DATA_LEN
};
