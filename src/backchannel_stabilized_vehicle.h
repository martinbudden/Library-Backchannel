#pragma once

#include "backchannel_base.h"
#include "command_packet.h"

class Ahrs;
class AhrsTask;
class ReceiverBase;
class TaskBase;
class VehicleControllerBase;

/*!
Backchannel that sends and receives packets that contain data for a stabilized vehicle.
*/
class BackchannelStabilizedVehicle : public BackchannelBase {
public:
    BackchannelStabilizedVehicle(
        BackchannelTransceiverBase& backchannel_transceiver,
        const uint8_t* backchannel_mac_address,
        const uint8_t* my_mac_address,
        VehicleControllerBase& vehicle_controller,
        Ahrs& ahrs,
        const ReceiverBase& receiver,
        const TaskBase* main_task
    );
    BackchannelStabilizedVehicle(
        BackchannelTransceiverBase& backchannel_transceiver,
        const uint8_t* backchannel_mac_address,
        const uint8_t* my_mac_address,
        VehicleControllerBase& vehicle_controller,
        Ahrs& ahrs,
        const ReceiverBase& receiver
    );
public:
    virtual bool send_packet(uint8_t sub_command) override;
    static uint32_t idFrom_mac_address(const uint8_t* mac_address);
protected:
    virtual bool processed_received_packet() override;
    virtual bool packet_request_data(const CommandPacketRequestData& packet);
    virtual bool packet_set_offset(const CommandPacketSetOffset& packet);
    virtual bool packet_control(const CommandPacketControl& packet);
    virtual bool packet_set_pid(const CommandPacketSetPID& packet);
protected:
    VehicleControllerBase& _vehicle_controller;
    Ahrs& _ahrs;
    const ReceiverBase& _receiver;
    const TaskBase* _main_task;
    const uint32_t _backchannel_id;
    const uint32_t _telemetry_id;
    uint32_t _request_type { CommandPacketRequestData::REQUEST_STOP_SENDING_DATA }; // So on startup a reset screen packet is sent
    uint32_t _sequence_number {0};
};
