#pragma once

#include <cstddef>
#include <cstdint>

struct ahrs_data_t;
class Ahrs;
class AhrsTask;
class TaskBase;
class VehicleControllerBase;


size_t pack_telemetry_data_minimal(uint8_t* telemetry_data_ptr, uint32_t id, uint32_t sequence_number);

// NOLINTBEGIN(readability-avoid-const-params-in-decls)

size_t pack_telemetry_data_task_intervals(uint8_t* telemetry_data_ptr, uint32_t id, uint32_t sequence_number,
        const TaskBase& ahrs_task,
        const TaskBase& vehicle_controller_task,
        uint32_t main_tasktick_count_delta,
        uint32_t transceiver_tick_count_delta);

size_t pack_telemetry_data_task_intervals_extended(uint8_t* telemetry_data_ptr, uint32_t id, uint32_t sequence_number,
        const Ahrs& ahrs,
        const VehicleControllerBase& vehicle_controller,
        uint32_t main_tasktick_count_delta,
        uint32_t transceiver_tick_count_delta,
        uint32_t receiver_dropped_packet_count);

size_t pack_telemetry_data_ahrs(uint8_t* telemetry_data_ptr, uint32_t id, uint32_t sequence_number,
        const Ahrs& ahrs,
        const ahrs_data_t& ahrs_data);

size_t pack_telemetry_data_pid(uint8_t* telemetry_data_ptr, uint32_t id, uint32_t sequence_number,
    const VehicleControllerBase& vehicle_controller,
    uint8_t pid_profile,
    uint8_t control_mode,
    float f0,
    float f1);

size_t pack_telemetry_data_pid_outputs(uint8_t* telemetry_data_ptr, uint32_t id, uint32_t sequence_number,
    const VehicleControllerBase& vehicle_controller,
    uint8_t pid_profile,
    uint8_t control_mode);

// NOLINTEND(readability-avoid-const-params-in-decls)
