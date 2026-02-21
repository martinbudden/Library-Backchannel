#include "BackchannelBase.h"
#include "BackchannelTask.h"

#include <array>
#include <cassert>
#include <cstring>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/task.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>
#endif
#endif


BackchannelTask* BackchannelTask::create_task(BackchannelBase& backchannel, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds)
{
    task_info_t task_info{};
    return create_task(task_info, backchannel, priority, core, task_interval_microseconds);
}

BackchannelTask* BackchannelTask::create_task(task_info_t& task_info, BackchannelBase& backchannel, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds)
{
    static BackchannelTask backchannel_task(task_interval_microseconds, backchannel);

    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t task_parameters { // NOLINT(misc-const-correctness) false positive
        .task = &backchannel_task
    };
#if !defined(BACKCHANNEL_TASK_STACK_DEPTH_BYTES)
    enum { BACKCHANNEL_TASK_STACK_DEPTH_BYTES = 4096 }; // 2048 probably sufficient when not using Serial.printf statements
#endif
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32) || !defined(FRAMEWORK_USE_FREERTOS)
    static std::array <uint8_t, BACKCHANNEL_TASK_STACK_DEPTH_BYTES> stack;
#else
    static std::array <StackType_t, BACKCHANNEL_TASK_STACK_DEPTH_BYTES / sizeof(StackType_t)> stack;
#endif
    task_info = {
        .task_handle = nullptr,
        .name = "BackchannelTask", // max length 16, including zero terminator
        .stack_depth_bytes = BACKCHANNEL_TASK_STACK_DEPTH_BYTES,
        .stack_buffer = reinterpret_cast<uint8_t*>(&stack[0]), // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        .priority = priority,
        .core = core,
        .task_interval_microseconds = task_interval_microseconds
    };
#if defined(FRAMEWORK_USE_FREERTOS)
    assert(strlen(task_info.name) < configMAX_TASK_NAME_LEN && "Backchannel: taskname too long");
    assert(task_info.priority < configMAX_PRIORITIES && "Backchannel: priority too high");

    static StaticTask_t taskBuffer;
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
    task_info.task_handle = xTaskCreateStaticPinnedToCore(
        BackchannelTask::task_static,
        task_info.name,
        task_info.stack_depth_bytes / sizeof(StackType_t),
        &task_parameters,
        task_info.priority,
        &stack[0],
        &taskBuffer,
        task_info.core
    );
    assert(task_info.task_handle != nullptr && "Unable to create BackchannelTask");
#elif defined(FRAMEWORK_RPI_PICO) || defined(FRAMEWORK_ARDUINO_RPI_PICO)
    task_info.task_handle = xTaskCreateStaticAffinitySet(
        BackchannelTask::task_static,
        task_info.name,
        task_info.stack_depth_bytes / sizeof(StackType_t),
        &task_parameters,
        task_info.priority,
        &stack[0],
        &taskBuffer,
        task_info.core
    );
    assert(task_info.task_handle != nullptr && "Unable to create BackchannelTask");
#else
    task_info.task_handle = xTaskCreateStatic(
        BackchannelTask::task_static,
        task_info.name,
        task_info.stack_depth_bytes / sizeof(StackType_t),
        &task_parameters,
        task_info.priority,
        &stack[0],
        &taskBuffer
    );
    assert(task_info.task_handle != nullptr && "Unable to create BackchannelTask");
    // vTaskCoreAffinitySet(task_info.task_handle, task_info.core);
#endif
#else
    (void)task_parameters;
#endif // FRAMEWORK_USE_FREERTOS

    return &backchannel_task;
}
