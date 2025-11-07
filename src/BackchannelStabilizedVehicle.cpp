#include "BackchannelStabilizedVehicle.h"
#include "BackchannelTransceiverBase.h"

#include <AHRS.h>
#if defined(FRAMEWORK_ARDUINO_ESP32)
//#define USE_DEBUG_PRINTF_BACKCHANNEL
#if defined(USE_DEBUG_PRINTF_BACKCHANNEL)
#include <HardwareSerial.h>
#endif
#endif
#include <ReceiverTelemetry.h>
#include <ReceiverTelemetryData.h>
#include <SV_Telemetry.h>
#include <SV_TelemetryData.h>
#include <VehicleControllerBase.h>

BackchannelStabilizedVehicle::BackchannelStabilizedVehicle(
    BackchannelTransceiverBase& backchannelTransceiver,
    const uint8_t* backchannelMacAddress,
    const uint8_t* myMacAddress,
    VehicleControllerBase& vehicleController,
    AHRS& ahrs,
    const ReceiverBase& receiver
    ) :
    BackchannelStabilizedVehicle(backchannelTransceiver, backchannelMacAddress, myMacAddress, vehicleController, ahrs, receiver, nullptr)
{
}

BackchannelStabilizedVehicle::BackchannelStabilizedVehicle(
        BackchannelTransceiverBase& backchannelTransceiver,
        const uint8_t* backchannelMacAddress,
        const uint8_t* myMacAddress,
        VehicleControllerBase& vehicleController,
        AHRS& ahrs,
        const ReceiverBase& receiver,
        const TaskBase* mainTask
    ) :
    BackchannelBase(backchannelTransceiver),
    _vehicleController(vehicleController),
    _ahrs(ahrs),
    _receiver(receiver),
    _mainTask(mainTask),
    _backchannelID(idFromMacAddress(backchannelMacAddress)),
    _telemetryID(idFromMacAddress(myMacAddress))
{
#if !defined(ESP_NOW_MAX_DATA_LEN)
#define ESP_NOW_MAX_DATA_LEN (250)
#endif
    // NOTE: esp_now_send runs at a high priority, so shorter packets mean less blocking of the other tasks.
    static_assert(sizeof(TD_TASK_INTERVALS_EXTENDED) <= ESP_NOW_MAX_DATA_LEN); // 12
    static_assert(sizeof(TD_TASK_INTERVALS_EXTENDED) <= ESP_NOW_MAX_DATA_LEN); // 28
    static_assert(sizeof(TD_AHRS) <= ESP_NOW_MAX_DATA_LEN); // 60
    //static_assert(sizeof(TD_RECEIVER) <= ESP_NOW_MAX_DATA_LEN); // 40
}

uint32_t BackchannelStabilizedVehicle::idFromMacAddress(const uint8_t* macAddress)
{
    // use the last 4 bytes of th MacAddress as ID
    const uint8_t* pM = macAddress;
    const uint32_t ret =  (*(pM + 2U) << 24U) | (*(pM + 3U) << 16U) | (*(pM + 4U) << 8U) | *(pM + 5U); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise)

    return ret;
}

bool BackchannelStabilizedVehicle::packetSetOffset(const CommandPacketSetOffset& packet)
{
    IMU_Base::xyz_int32_t gyroOffset = _ahrs.getGyroOffsetMapped();
    IMU_Base::xyz_int32_t accOffset = _ahrs.getAccOffsetMapped();

#if defined(USE_DEBUG_PRINTF_BACKCHANNEL)
    Serial.printf("BSV packetSetOffset type:%d, len:%d value:%d, type:%d\r\n", packet.type, packet.len, packet.setType, packet.value);
#endif
    switch (packet.setType) {
    case CommandPacketSetOffset::SET_GYRO_OFFSET_X:
        gyroOffset.x = packet.value;
        _ahrs.setGyroOffsetMapped(gyroOffset);
        break;
    case CommandPacketSetOffset::SET_GYRO_OFFSET_Y:
        gyroOffset.y = packet.value;
        _ahrs.setGyroOffsetMapped(gyroOffset);
        break;
    case CommandPacketSetOffset::SET_GYRO_OFFSET_Z:
        gyroOffset.z = packet.value;
        _ahrs.setGyroOffsetMapped(gyroOffset);
        break;
    case CommandPacketSetOffset::SET_ACC_OFFSET_X:
        accOffset.x = packet.value;
        _ahrs.setAccOffsetMapped(accOffset);
        break;
    case CommandPacketSetOffset::SET_ACC_OFFSET_Y:
        accOffset.y = packet.value;
        _ahrs.setAccOffsetMapped(accOffset);
        break;
    case CommandPacketSetOffset::SET_ACC_OFFSET_Z:
        accOffset.z = packet.value;
        _ahrs.setAccOffsetMapped(accOffset);
        break;
    default:
#if defined(USE_DEBUG_PRINTF_BACKCHANNEL)
        Serial.printf("Backchannel::packetSetOffset invalid itemIndex:%d\r\n", packet.setType);
#endif
        return false;
    }

    return true;
}

bool BackchannelStabilizedVehicle::packetControl(const CommandPacketControl& packet)
{
    (void)packet;
    return false;
}

bool BackchannelStabilizedVehicle::packetSetPID(const CommandPacketSetPID& packet)
{
    (void)packet;
    return false;
}

bool BackchannelStabilizedVehicle::packetRequestData(const CommandPacketRequestData& packet)
{
#if defined(USE_DEBUG_PRINTF_BACKCHANNEL)
    Serial.printf("BSV packetRequestData packet type:%d, len:%d, requestType:%d, valueType:%d\r\n", packet.type, packet.len, packet.requestType, packet.valueType);
#endif
    _requestType = packet.requestType;
    sendPacket(packet.valueType);
    return true;
}

/*!
Called from within main task function.

Once _requestType has been set, this will continue to send packets until _requestType is set to NO_REQUEST
*/
bool BackchannelStabilizedVehicle::sendPacket(uint8_t subCommand)
{
#if defined(USE_DEBUG_PRINTF_BACKCHANNEL)
    if (_requestType != CommandPacketRequestData::NO_REQUEST) {
        //Serial.printf("BSV sendPacket requestType:%d, subCommand:%d\r\n", _requestType, subCommand);
    }
#endif
    (void)subCommand;

    switch (_requestType) {
    case CommandPacketRequestData::NO_REQUEST: {
        return false;
    }
    case CommandPacketRequestData::REQUEST_STOP_SENDING_DATA: {
        // send a minimal packet so the client can reset its screen
        const size_t len = packTelemetryData_Minimal(_transmitDataBufferPtr, _telemetryID, _sequenceNumber);
        sendData(_transmitDataBufferPtr, len);
        // set _requestType to NO_REQUEST so no further data sent
        _requestType = CommandPacketRequestData::NO_REQUEST;
        break;
    }
    case CommandPacketRequestData::REQUEST_TASK_INTERVAL_DATA: {
        const size_t len = packTelemetryData_TaskIntervals(_transmitDataBufferPtr, _telemetryID, _sequenceNumber,
            *_ahrs.getTask(),
            *_vehicleController.getTask(),
            _mainTask ?  _mainTask->getTickCountDelta() : 0,
            _backchannelTransceiver.getTickCountDeltaAndReset()
        );
        //Serial.printf("tiLen:%d\r\n", len);
        sendData(_transmitDataBufferPtr, len);
        break;
    }
    case CommandPacketRequestData::REQUEST_TASK_INTERVAL_EXTENDED_DATA: {
        const size_t len = packTelemetryData_TaskIntervalsExtended(_transmitDataBufferPtr, _telemetryID, _sequenceNumber,
            _ahrs,
            _vehicleController,
            _mainTask ? _mainTask->getTickCountDelta() : 0,
            _backchannelTransceiver.getTickCountDeltaAndReset(),
            static_cast<uint32_t>(_receiver.getDroppedPacketCountDelta())
        );
        //Serial.printf("tiLen:%d\r\n", len);
        sendData(_transmitDataBufferPtr, len);
        break;
    }
    case CommandPacketRequestData::REQUEST_AHRS_DATA: {
        const AHRS::ahrs_data_t& ahrsData {};
        const size_t len = packTelemetryData_AHRS(_transmitDataBufferPtr, _telemetryID, _sequenceNumber, _ahrs, ahrsData);
        //Serial.printf("ahrsLen:%d\r\n", len);
        sendData(_transmitDataBufferPtr, len);
        break;
    }
    case CommandPacketRequestData::REQUEST_RECEIVER_DATA: {
        const size_t len = packTelemetryData_Receiver(_transmitDataBufferPtr, _telemetryID, _sequenceNumber, _receiver);
        //Serial.printf("receiverLen:%d\r\n", len);
        sendData(_transmitDataBufferPtr, len);
        break;
    }
    default:
        return false;
    } // end switch
    return true;
}

/*!
If data was received then interpret it as a packet and return true.
Four types of packets may be received:

1. A command packet, for example a command to switch off the motors.
2. A request to transmit telemetry. In this case format the telemetry data and send it.
3. A request to set a PID value. In this case set the PID value and then send back a TD_PIDS packet for display.
4. A request to set an IMU offset value. In this case set the offset value, but don't send back an TD_AHRS packet for display,
   since the request will have come from within
*/
bool BackchannelStabilizedVehicle::processedReceivedPacket()
{
    //Serial.printf("update\r\n");
    const size_t receivedDataLength = _backchannelTransceiver.getReceivedDataLength();
    if (receivedDataLength > 0) {
        // We have a packet, so process it

        //Serial.printf("rdLen:%d\r\n", receivedDataLength);
        _backchannelTransceiver.setReceivedDataLengthToZero();

        const auto* const controlPacket = reinterpret_cast<const CommandPacketControl*>(_receivedDataBufferPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
#if defined(USE_DEBUG_PRINTF_BACKCHANNEL)
        Serial.printf("BSV processedReceivedPacket id:%x, type:%d, len:%d value:%d\r\n", controlPacket->id, controlPacket->type, controlPacket->len, controlPacket->value);
#endif
        if (controlPacket->id == _backchannelID) {
            // it's our packet, so process it

            //Serial.printf("Backchannel::update id:%x, type:%d, len:%d value:%d\r\n", controlPacket->id, controlPacket->type, controlPacket->len, controlPacket->value);
            switch (controlPacket->type) {
            case CommandPacketControl::TYPE: // NOLINT(bugprone-branch-clone) false positive
                packetControl(*reinterpret_cast<const CommandPacketControl*>(_receivedDataBufferPtr)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
                return true;
            case CommandPacketRequestData::TYPE: // NOLINT(bugprone-branch-clone) false positive
                packetRequestData(*reinterpret_cast<const CommandPacketRequestData*>(_receivedDataBufferPtr)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
                return true;
            case CommandPacketSetPID::TYPE: // NOLINT(bugprone-branch-clone) false positive
                packetSetPID(*reinterpret_cast<const CommandPacketSetPID*>(_receivedDataBufferPtr)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
                return true;
            case CommandPacketSetOffset::TYPE: // NOLINT(bugprone-branch-clone) false positive
                packetSetOffset(*reinterpret_cast<const CommandPacketSetOffset*>(_receivedDataBufferPtr)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
                return true;
            default:
                // do nothing
                break;
            } // end switch
        }
    }

    return false;
}
