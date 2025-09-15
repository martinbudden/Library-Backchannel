#include "BackchannelBase.h"
#include "BackchannelTask.h"

#include <array>
#include <cstring>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_USE_FREERTOS_SUBDIRECTORY)
#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/task.h>
#else
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>
#endif
#endif


BackchannelTask* BackchannelTask::createTask(BackchannelBase& backchannel, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroSeconds)
{
    task_info_t taskInfo{};
    return createTask(taskInfo, backchannel, priority, core, taskIntervalMicroSeconds);
}

BackchannelTask* BackchannelTask::createTask(task_info_t& taskInfo, BackchannelBase& backchannel, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroSeconds)
{
    static BackchannelTask backchannelTask(taskIntervalMicroSeconds, backchannel);

    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t taskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &backchannelTask
    };
#if !defined(BACKCHANNEL_TASK_STACK_DEPTH_BYTES)
    enum { BACKCHANNEL_TASK_STACK_DEPTH_BYTES = 4096 }; // 2048 probably sufficient when not using Serial.printf statements
#endif
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32) || defined(FRAMEWORK_TEST)
    static std::array <uint8_t, BACKCHANNEL_TASK_STACK_DEPTH_BYTES> stack;
#else
    static std::array <StackType_t, BACKCHANNEL_TASK_STACK_DEPTH_BYTES / sizeof(StackType_t)> stack;
#endif
    taskInfo = {
        .taskHandle = nullptr,
        .name = "BackchannelTask", // max length 16, including zero terminator
        .stackDepthBytes = BACKCHANNEL_TASK_STACK_DEPTH_BYTES,
        .stackBuffer = reinterpret_cast<uint8_t*>(&stack[0]), // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        .priority = priority,
        .core = core,
        .taskIntervalMicroSeconds = taskIntervalMicroSeconds
    };
#if defined(FRAMEWORK_USE_FREERTOS)
    assert(strlen(taskInfo.name) < configMAX_TASK_NAME_LEN && "Backchannel: taskname too long");
    assert(taskInfo.priority < configMAX_PRIORITIES && "Backchannel: priority too high");

    static StaticTask_t taskBuffer;
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
    taskInfo.taskHandle = xTaskCreateStaticPinnedToCore(
        BackchannelTask::Task,
        taskInfo.name,
        taskInfo.stackDepthBytes / sizeof(StackType_t),
        &taskParameters,
        taskInfo.priority,
        &stack[0],
        &taskBuffer,
        taskInfo.core
    );
    assert(taskInfo.taskHandle != nullptr && "Unable to create BackchannelTask.");
#else
    taskInfo.taskHandle = xTaskCreateStaticAffinitySet(
        BackchannelTask::Task,
        taskInfo.name,
        taskInfo.stackDepthBytes / sizeof(StackType_t),
        &taskParameters,
        taskInfo.priority,
        &stack[0],
        &taskBuffer,
        taskInfo.core
    );
    assert(taskInfo.taskHandle != nullptr && "Unable to create BackchannelTask.");
#endif
#else
    (void)taskParameters;
#endif // FRAMEWORK_USE_FREERTOS

    return &backchannelTask;
}
