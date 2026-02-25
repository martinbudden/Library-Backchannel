#pragma once

#include <cstddef>
#include <cstdint>

class BackchannelTransceiverBase;

class BackchannelBase {
protected:
    BackchannelBase(BackchannelTransceiverBase& backchannel_transceiver);
public:
    virtual ~BackchannelBase() = default;

    void WAIT_FOR_DATA_RECEIVED();
    int send_data(const uint8_t* data, size_t len) const;

    virtual bool processed_received_packet() = 0;
    virtual bool send_packet(uint8_t sub_command) = 0;
    bool send_packet() { return send_packet(0); }

protected:
    BackchannelTransceiverBase& _backchannel_transceiver;
    uint8_t* _transmit_data_buffer_ptr {};
    const size_t _transmit_data_buffer_size {};
    uint8_t* _received_data_buffer_ptr {};
    const size_t _received_data_buffer_size {};
};
