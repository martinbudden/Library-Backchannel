#if !defined(LIBRARY_BACKCHANNEL_EXCLUDE_SV_TELEMETRY)

#include "sv_telemetry.h"
#include "sv_telemetry_data.h"

#include <ahrs.h>
#include <pid_controller.h>
#include <task_base.h>
#include <vehicle_controller_base.h>


/*!
Packs the TD_Minimal packet with zeros. Returns the length of the packet.
*/
size_t pack_telemetry_data_minimal(uint8_t* telemetry_data_ptr, uint32_t id, uint32_t sequence_number)
{
    TD_MINIMAL* td = reinterpret_cast<TD_MINIMAL*>(telemetry_data_ptr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;

    td->type = TD_MINIMAL::TYPE;
    td->len = sizeof(TD_MINIMAL);
    td->sub_type = 0;
    td->sequence_number = static_cast<uint8_t>(sequence_number);

    return td->len;
}

/*!
Packs the tick interval telemetry data into a TD_TASK_INTERVALS_EXTENDED packet. Returns the length of the packet.
*/
size_t pack_telemetry_data_task_intervals(uint8_t* telemetry_data_ptr, uint32_t id, uint32_t sequence_number,
        const TaskBase& ahrs_task,
        const TaskBase& vehicle_controller_task,
        uint32_t main_tasktick_count_delta,
        uint32_t transceiver_tick_count_delta)
{
    TD_TASK_INTERVALS* td = reinterpret_cast<TD_TASK_INTERVALS*>(telemetry_data_ptr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_TASK_INTERVALS::TYPE;
    td->len = sizeof(TD_TASK_INTERVALS);
    td->sub_type = 0;
    td->sequence_number = static_cast<uint8_t>(sequence_number);

    td->data = {
        .main_task_interval_ticks = static_cast<uint8_t>(main_tasktick_count_delta),
        .ahrs_task_interval_ticks = static_cast<uint8_t>(ahrs_task.get_tick_count_delta()),
        .vc_task_interval_ticks = static_cast<uint8_t>(vehicle_controller_task.get_tick_count_delta()),
        .transceiver_tick_count_delta = static_cast<uint8_t>(transceiver_tick_count_delta)
    };

    return td->len;
}

/*!
Packs the tick interval telemetry data into a TD_TASK_INTERVALS_EXTENDED packet. Returns the length of the packet.
*/
size_t pack_telemetry_data_task_intervals_extended(uint8_t* telemetry_data_ptr, uint32_t id, uint32_t sequence_number,
        const Ahrs& ahrs,
        const VehicleControllerBase& vehicle_controller,
        uint32_t main_tasktick_count_delta,
        uint32_t transceiver_tick_count_delta,
        uint32_t receiver_dropped_packet_count)
{
    TD_TASK_INTERVALS_EXTENDED* td = reinterpret_cast<TD_TASK_INTERVALS_EXTENDED*>(telemetry_data_ptr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_TASK_INTERVALS_EXTENDED::TYPE;
    td->len = sizeof(TD_TASK_INTERVALS_EXTENDED);
    td->sub_type = 0;
    td->sequence_number = static_cast<uint8_t>(sequence_number);

    const TaskBase* vehicle_controller_task = vehicle_controller.get_task();
    const TaskBase* ahrs_task = ahrs.get_task();

    td->data.main_task_interval_ticks = static_cast<uint8_t>(main_tasktick_count_delta);
    td->data.ahrs_task_interval_ticks = static_cast<uint8_t>(ahrs_task->get_tick_count_delta());
    td->data.vc_task_interval_ticks = static_cast<uint8_t>(vehicle_controller_task->get_tick_count_delta());
    td->data.transceiver_tick_count_delta = static_cast<uint8_t>(transceiver_tick_count_delta);

    td->data.ahrs_task_interval_microseconds = static_cast<uint16_t>(ahrs_task->get_time_microseconds_delta());

    static_assert(TD_TASK_INTERVALS_EXTENDED::TIME_CHECKS_COUNT == Ahrs::TIME_CHECKS_COUNT);
    for (size_t ii = 0; ii < TD_TASK_INTERVALS_EXTENDED::TIME_CHECKS_COUNT; ++ii) {
        td->data.ahrsTimeChecksMicroseconds[ii] = static_cast<uint16_t>(ahrs.get_time_checks_microseconds(ii));
    }

    td->data.vc_task_interval_microseconds = static_cast<uint16_t>(vehicle_controller_task->get_time_microseconds_delta());
    td->data.vc_output_power_time_microseconds = static_cast<uint16_t>(vehicle_controller.get_output_power_time_microseconds());

    td->data.receiver_dropped_packet_count = static_cast<uint8_t>(receiver_dropped_packet_count);

    return td->len;
}

/*!
Packs the AHRS telemetry data into a TD_AHRS packet. Returns the length of the packet.
*/
size_t pack_telemetry_data_ahrs(uint8_t* telemetry_data_ptr, uint32_t id, uint32_t sequence_number, const Ahrs& ahrs, const ahrs_data_t& ahrs_data)
{
    TD_AHRS* td = reinterpret_cast<TD_AHRS*>(telemetry_data_ptr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_AHRS::TYPE;
    td->len = sizeof(TD_AHRS);
    td->sub_type = 0;
    td->sequence_number = static_cast<uint8_t>(sequence_number);

    td->data = {
        // roll, pitch, and yaw to be filled in by caller
        .roll = 0.0F,
        .pitch = 0.0F,
        .yaw = 0.0F,
        .gyro_rps = ahrs_data.acc_gyro_rps.gyro_rps,
        .acc = ahrs_data.acc_gyro_rps.acc,
        .gyro_offset = ahrs.get_gyro_offset_mapped(),
        .acc_offset = ahrs.get_acc_offset_mapped(),
    };

    td->task_interval_ticks = 0; // to be filled in by the caller

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
size_t pack_telemetry_data_pid(uint8_t* telemetry_data_ptr, uint32_t id, uint32_t sequence_number, const VehicleControllerBase& vehicle_controller, uint8_t pid_profile, uint8_t control_mode, float f0, float f1)
{
    TD_PID* td = reinterpret_cast<TD_PID*>(telemetry_data_ptr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_PID::TYPE;
    td->len = sizeof(TD_PID);
    td->sub_type = 0;
    td->sequence_number = static_cast<uint8_t>(sequence_number);

    const uint8_t pid_count =std::min(static_cast<uint8_t>(TD_PID::MAX_PID_COUNT), static_cast<uint8_t>(vehicle_controller.get_pid_count()));
    td->data.pid_count = pid_count;
    td->data.pid_profile = pid_profile;
    td->data.vehicle_type = static_cast<uint8_t>(vehicle_controller.get_type());
    td->data.control_mode = control_mode;

    td->data.f0 = f0; // general purpose value f0 used for pitch_balance_angle_degrees in self balancing robots
    td->data.f1 = f1;

    for (uint8_t ii = 0; ii < pid_count; ++ii) {
        const pid_constants_uint16_t pid = vehicle_controller.get_pid_msp(ii);
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
size_t pack_telemetry_data_pid_outputs(uint8_t* telemetry_data_ptr, uint32_t id, uint32_t sequence_number, const VehicleControllerBase& vehicle_controller, uint8_t pid_profile, uint8_t control_mode)
{
    TD_PID_OUTPUTS* td = reinterpret_cast<TD_PID_OUTPUTS*>(telemetry_data_ptr); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,hicpp-use-auto,modernize-use-auto)

    td->id = id;
    td->type = TD_PID_OUTPUTS::TYPE;
    td->len = sizeof(TD_PID_OUTPUTS);
    td->sub_type = 0;
    td->sequence_number = static_cast<uint8_t>(sequence_number);

    const uint8_t pid_count =std::min(static_cast<uint8_t>(TD_PID_OUTPUTS::MAX_PID_COUNT), static_cast<uint8_t>(vehicle_controller.get_pid_count()));
    td->data.pid_count = pid_count;
    td->data.pid_profile = pid_profile;
    td->data.vehicle_type = static_cast<uint8_t>(vehicle_controller.get_type()),
    td->data.control_mode = control_mode;

    for (uint8_t ii = 0; ii < pid_count; ++ii) {
        td->data.setpoints[ii] = vehicle_controller.get_pid_setpoint(ii);
        const pid_error_t error = vehicle_controller.get_pid_error(ii);
        td->data.errors[ii].p = error.p;
        td->data.errors[ii].i = error.i;
        td->data.errors[ii].d = error.d;
        td->data.errors[ii].s = error.s;
        td->data.errors[ii].k = error.k;
    }

    return td->len;
}

#endif // LIBRARY_BACKCHANNEL_EXCLUDE_SV_TELEMETRY
