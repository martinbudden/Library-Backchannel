#pragma once

#include "command_packet.h"

#include <cstddef>
#include <cstdint>

class BackchannelTransceiverBase;
struct backchannel_context_t;


class BackchannelBase {
protected:
    BackchannelBase(BackchannelTransceiverBase& backchannel_transceiver, const uint8_t* backchannel_mac_address, const uint8_t* my_mac_address);
public:
    virtual ~BackchannelBase() = default;

    void WAIT_FOR_DATA_RECEIVED();
    int send_data(const uint8_t* data, size_t len) const;
    static uint32_t id_from_mac_address(const uint8_t* mac_address);

    virtual bool processed_received_packet(backchannel_context_t& pg);
    virtual bool send_packet(backchannel_context_t& pg, uint8_t sub_command);
    bool send_packet(backchannel_context_t& pg) { return send_packet(pg, 0); }
    virtual bool packet_request_data(backchannel_context_t& pg, const CommandPacketRequestData& packet);
    virtual bool packet_set_offset(backchannel_context_t& pg, const CommandPacketSetOffset& packet) = 0;
    virtual bool packet_control(backchannel_context_t& pg, const CommandPacketControl& packet) = 0;
    virtual bool packet_set_pid(backchannel_context_t& pg, const CommandPacketSetPid& packet) = 0;

protected:
    BackchannelTransceiverBase& _backchannel_transceiver;
    uint8_t* _transmit_data_buffer_ptr {};
    const size_t _transmit_data_buffer_size {};
    uint8_t* _received_data_buffer_ptr {};
    const size_t _received_data_buffer_size {};
    const uint32_t _backchannel_id;
    const uint32_t _telemetry_id;
    uint32_t _request_type { CommandPacketRequestData::REQUEST_STOP_SENDING_DATA }; // So on startup a reset screen packet is sent
    uint32_t _sequence_number {0};
};
