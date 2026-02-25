#pragma once

#include <cstddef>
#include <cstdint>
/*!
Virtual base class for the backchannel transceiver.
Can be subclassed to provide transceivers over ESPNOW or UDP (for example).
*/
class BackchannelTransceiverBase {
public:
    virtual ~BackchannelTransceiverBase() = default;
    virtual int send_data(const uint8_t* data, size_t len) const = 0;
    virtual void WAIT_FOR_DATA_RECEIVED() = 0;
    virtual size_t get_received_data_length() const = 0;
    virtual void set_received_data_length_to_zero() = 0;
    virtual uint32_t get_tick_count_delta_and_reset() = 0;

    uint8_t* get_transmit_data_buffer_ptr() { return &_transmit_data_buffer[0]; }
    size_t get_transmit_data_buffer_size() const { return sizeof(_transmit_data_buffer); }
    uint8_t* get_received_data_buffer_ptr() { return &_received_data_buffer[0]; }
    size_t get_received_data_buffer_size() const { return sizeof(_received_data_buffer); }
protected:
    // If using MSP, then the MSP packets are packed into _transmit_data_buffer by MSP::processOutCommand,
    // so _transmit_data_buffer must be large enough to hold the largest MSP packet.
    // If the packet length exceeds ESP_NOW_MAX_DATA_LEN, then it is not sent,
    // but we don't know its length until we have unpacked it.
    uint8_t _transmit_data_buffer[512] {};
    uint8_t _received_data_buffer[256] {}; // must be >= ESP_NOW_MAX_DATA_LEN
};
