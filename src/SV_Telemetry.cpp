#if !defined(LIBRARY_BACKCHANNEL_EXCLUDE_SV_TELEMETRY)

#include "SV_Telemetry.h"
#include "SV_TelemetryData.h"

#include <ahrs.h>
#include <task_base.h>
#include <vehicle_controller_base.h>


/*!
Packs the TD_Minimal packet with zeros. Returns the length of the packet.
*/
size_t packTelemetryData_Minimal(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber)
{
    TD_MINIMAL* td = reinterpret_cast<TD_MINIMAL*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;

    td->type = TD_MINIMAL::TYPE;
    td->len = sizeof(TD_MINIMAL);
    td->subType = 0;
    td->sequenceNumber = static_cast<uint8_t>(sequenceNumber);

    return td->len;
}

/*!
Packs the tick interval telemetry data into a TD_TASK_INTERVALS_EXTENDED packet. Returns the length of the packet.
*/
size_t packTelemetryData_TaskIntervals(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber,
        const TaskBase& ahrsTask,
        const TaskBase& vehicleControllerTask,
        uint32_t mainTaskTickCountDelta,
        uint32_t transceiverTickCountDelta)
{
    TD_TASK_INTERVALS* td = reinterpret_cast<TD_TASK_INTERVALS*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_TASK_INTERVALS::TYPE;
    td->len = sizeof(TD_TASK_INTERVALS);
    td->subType = 0;
    td->sequenceNumber = static_cast<uint8_t>(sequenceNumber);

    td->data = {
        .mainTaskIntervalTicks = static_cast<uint8_t>(mainTaskTickCountDelta),
        .ahrsTaskIntervalTicks = static_cast<uint8_t>(ahrsTask.get_tick_count_delta()),
        .vcTaskIntervalTicks = static_cast<uint8_t>(vehicleControllerTask.get_tick_count_delta()),
        .transceiverTickCountDelta = static_cast<uint8_t>(transceiverTickCountDelta)
    };

    return td->len;
}

/*!
Packs the tick interval telemetry data into a TD_TASK_INTERVALS_EXTENDED packet. Returns the length of the packet.
*/
size_t packTelemetryData_TaskIntervalsExtended(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber,
        const Ahrs& ahrs,
        const VehicleControllerBase& vehicleController,
        uint32_t mainTaskTickCountDelta,
        uint32_t transceiverTickCountDelta,
        uint32_t receiverDroppedPacketCount)
{
    TD_TASK_INTERVALS_EXTENDED* td = reinterpret_cast<TD_TASK_INTERVALS_EXTENDED*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_TASK_INTERVALS_EXTENDED::TYPE;
    td->len = sizeof(TD_TASK_INTERVALS_EXTENDED);
    td->subType = 0;
    td->sequenceNumber = static_cast<uint8_t>(sequenceNumber);

    const TaskBase* vehicleControllerTask = vehicleController.get_task();
    const TaskBase* ahrsTask = ahrs.get_task();

    td->data.mainTaskIntervalTicks = static_cast<uint8_t>(mainTaskTickCountDelta);
    td->data.ahrsTaskIntervalTicks = static_cast<uint8_t>(ahrsTask->get_tick_count_delta());
    td->data.vcTaskIntervalTicks = static_cast<uint8_t>(vehicleControllerTask->get_tick_count_delta());
    td->data.transceiverTickCountDelta = static_cast<uint8_t>(transceiverTickCountDelta);

    td->data.ahrsTaskIntervalMicroseconds = static_cast<uint16_t>(ahrsTask->get_time_microseconds_delta());

    static_assert(TD_TASK_INTERVALS_EXTENDED::TIME_CHECKS_COUNT == Ahrs::TIME_CHECKS_COUNT);
    for (size_t ii = 0; ii < TD_TASK_INTERVALS_EXTENDED::TIME_CHECKS_COUNT; ++ii) {
        td->data.ahrsTimeChecksMicroseconds[ii] = static_cast<uint16_t>(ahrs.get_time_checks_microseconds(ii));
    }

    td->data.vcTaskIntervalMicroseconds = static_cast<uint16_t>(vehicleControllerTask->get_time_microseconds_delta());
    td->data.vcOutputPowerTimeMicroseconds = static_cast<uint16_t>(vehicleController.get_output_power_time_microseconds());

    td->data.receiverDroppedPacketCount = static_cast<uint8_t>(receiverDroppedPacketCount);

    return td->len;
}

/*!
Packs the AHRS telemetry data into a TD_AHRS packet. Returns the length of the packet.
*/
size_t packTelemetryData_AHRS(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber, const Ahrs& ahrs, const ahrs_data_t& ahrsData)
{
    TD_AHRS* td = reinterpret_cast<TD_AHRS*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_AHRS::TYPE;
    td->len = sizeof(TD_AHRS);
    td->subType = 0;
    td->sequenceNumber = static_cast<uint8_t>(sequenceNumber);

    td->data = {
        // roll, pitch, and yaw to be filled in by caller
        .roll = 0.0F,
        .pitch = 0.0F,
        .yaw = 0.0F,
        .gyroRPS = ahrsData.acc_gyro_rps.gyro_rps,
        .acc = ahrsData.acc_gyro_rps.acc,
        .gyroOffset = ahrs.get_gyro_offset_mapped(),
        .accOffset = ahrs.get_acc_offset_mapped(),
    };

    td->taskIntervalTicks = 0; // to be filled in by the caller

    const uint32_t flags = ahrs.get_flags();
    td->flags = (flags & Ahrs::SENSOR_FUSION_REQUIRES_INITIALIZATION) ? TD_AHRS::SENSOR_FUSION_REQUIRES_INITIALIZATION : 0;
    if (flags & Ahrs::IMU_AUTO_CALIBRATES) {
        td->flags |= TD_AHRS::IMU_AUTO_CALIBRATES;
    }

    return td->len;
}

/*!
Packs the VehicleController PID telemetry data into a TD_PID packet. Returns the length of the packet.
*/
size_t packTelemetryData_PID(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber, const VehicleControllerBase& vehicleController, uint8_t pidProfile, uint8_t controlMode, float f0, float f1)
{
    TD_PID* td = reinterpret_cast<TD_PID*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_PID::TYPE;
    td->len = sizeof(TD_PID);
    td->subType = 0;
    td->sequenceNumber = static_cast<uint8_t>(sequenceNumber);

    const uint8_t pidCount =std::min(static_cast<uint8_t>(TD_PID::MAX_PID_COUNT), static_cast<uint8_t>(vehicleController.get_pid_count()));
    td->data.pidCount = pidCount;
    td->data.pidProfile = pidProfile;
    td->data.vehicleType = static_cast<uint8_t>(vehicleController.get_type());
    td->data.controlMode = controlMode;

    td->data.f0 = f0; // general purpose value f0 used for pitchBalanceAngleDegrees in self balancing robots
    td->data.f1 = f1;

    for (uint8_t ii = 0; ii < pidCount; ++ii) {
        const VehicleControllerBase::PIDF_uint16_t pid = vehicleController.get_pid_msp(ii);
        td->data.pids[ii].kp = pid.kp;
        td->data.pids[ii].ki = pid.ki;
        td->data.pids[ii].kd = pid.kd;
        td->data.pids[ii].ks = pid.ks;
        td->data.pids[ii].kk = pid.kk;
    }

    return td->len;
}

/*!
Packs the VehicleController PID telemetry data into a TD_PID_OUTPUTS packet. Returns the length of the packet.
*/
size_t packTelemetryData_PID_Outputs(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber, const VehicleControllerBase& vehicleController, uint8_t pidProfile, uint8_t controlMode)
{
    TD_PID_OUTPUTS* td = reinterpret_cast<TD_PID_OUTPUTS*>(telemetryDataPtr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_PID_OUTPUTS::TYPE;
    td->len = sizeof(TD_PID_OUTPUTS);
    td->subType = 0;
    td->sequenceNumber = static_cast<uint8_t>(sequenceNumber);

    const uint8_t pidCount =std::min(static_cast<uint8_t>(TD_PID_OUTPUTS::MAX_PID_COUNT), static_cast<uint8_t>(vehicleController.get_pid_count()));
    td->data.pidCount = pidCount;
    td->data.pidProfile = pidProfile;
    td->data.vehicleType = static_cast<uint8_t>(vehicleController.get_type()),
    td->data.controlMode = controlMode;

    for (uint8_t ii = 0; ii < pidCount; ++ii) {
        td->data.setpoints[ii] = vehicleController.get_pid_setpoint(ii);
        const VehicleControllerBase::PIDF_error_t error = vehicleController.get_pid_error(ii);
        td->data.errors[ii].P = error.P;
        td->data.errors[ii].I = error.I;
        td->data.errors[ii].D = error.D;
        td->data.errors[ii].S = error.S;
        td->data.errors[ii].K = error.K;
    }

    return td->len;
}

#endif // LIBRARY_BACKCHANNEL_EXCLUDE_SV_TELEMETRY
