#pragma once

#include "BackchannelBase.h"
#include "CommandPacket.h"

class AHRS;
class AHRS_Task;
class ReceiverBase;
class TaskBase;
class VehicleControllerBase;

/*!
Backchannel that sends and receives packets that contain data for a stabilized vehicle.
*/
class BackchannelStabilizedVehicle : public BackchannelBase {
public:
    BackchannelStabilizedVehicle(
        BackchannelTransceiverBase& backchannelTransceiver,
        const uint8_t* backchannelMacAddress,
        const uint8_t* myMacAddress,
        VehicleControllerBase& vehicleController,
        AHRS& ahrs,
        const ReceiverBase& receiver,
        const TaskBase* mainTask
    );
    BackchannelStabilizedVehicle(
        BackchannelTransceiverBase& backchannelTransceiver,
        const uint8_t* backchannelMacAddress,
        const uint8_t* myMacAddress,
        VehicleControllerBase& vehicleController,
        AHRS& ahrs,
        const ReceiverBase& receiver
    );
public:
    virtual bool sendPacket(uint8_t subCommand) override;
    static uint32_t idFromMacAddress(const uint8_t* macAddress);
protected:
    virtual bool processedReceivedPacket() override;
    virtual bool packetRequestData(const CommandPacketRequestData& packet);
    virtual bool packetSetOffset(const CommandPacketSetOffset& packet);
    virtual bool packetControl(const CommandPacketControl& packet);
    virtual bool packetSetPID(const CommandPacketSetPID& packet);
protected:
    VehicleControllerBase& _vehicleController;
    AHRS& _ahrs;
    const ReceiverBase& _receiver;
    const TaskBase* _mainTask;
    const uint32_t _backchannelID;
    const uint32_t _telemetryID;
    uint32_t _requestType { CommandPacketRequestData::REQUEST_STOP_SENDING_DATA }; // So on startup a reset screen packet is sent
    uint32_t _sequenceNumber {0};
};
