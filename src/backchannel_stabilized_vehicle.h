#pragma once

#include "backchannel_base.h"
#include "command_packet.h"


/*!
Backchannel that sends and receives packets that contain data for a stabilized vehicle.
*/
class BackchannelStabilizedVehicle : public BackchannelBase {
public:
    BackchannelStabilizedVehicle(
        BackchannelTransceiverBase& backchannel_transceiver,
        const uint8_t* backchannel_mac_address,
        const uint8_t* my_mac_address
    );
public:
    virtual bool send_packet(backchannel_parameter_group_t& pg, uint8_t sub_command) override;
    static uint32_t id_from_mac_address(const uint8_t* mac_address);
protected:
    virtual bool processed_received_packet(backchannel_parameter_group_t& pg) override;
    virtual bool packet_request_data(backchannel_parameter_group_t& pg, const CommandPacketRequestData& packet);
    virtual bool packet_set_offset(backchannel_parameter_group_t& pg, const CommandPacketSetOffset& packet);
    virtual bool packet_control(backchannel_parameter_group_t& pg, const CommandPacketControl& packet);
    virtual bool packet_set_pid(backchannel_parameter_group_t& pg, const CommandPacketSetPid& packet);
protected:
    const uint32_t _backchannel_id;
    const uint32_t _telemetry_id;
    uint32_t _request_type { CommandPacketRequestData::REQUEST_STOP_SENDING_DATA }; // So on startup a reset screen packet is sent
    uint32_t _sequence_number {0};
};
