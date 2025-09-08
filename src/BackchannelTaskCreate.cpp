#include "BackchannelBase.h"
#include "BackchannelTask.h"

#include <array>
#include <cstring>

#if defined(FRAMEWORK_USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/task.h>
#endif


BackchannelTask* BackchannelTask::createTask(BackchannelBase& backchannel, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds)
{
    task_info_t taskInfo{};
    return createTask(taskInfo, backchannel, priority, coreID, taskIntervalMicroSeconds);
}

BackchannelTask* BackchannelTask::createTask(task_info_t& taskInfo, BackchannelBase& backchannel, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds)
{
    static BackchannelTask backchannelTask(taskIntervalMicroSeconds, backchannel);

    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t taskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &backchannelTask
    };
#if !defined(BACKCHANNEL_TASK_STACK_DEPTH_BYTES)
    enum { BACKCHANNEL_TASK_STACK_DEPTH_BYTES = 4096 }; // 2048 probably sufficient when not using Serial.printf statements
#endif
    static std::array <uint8_t, BACKCHANNEL_TASK_STACK_DEPTH_BYTES> stack;
    taskInfo = {
        .taskHandle = nullptr,
        .name = "BackchannelTask", // max length 16, including zero terminator
        .stackDepth = BACKCHANNEL_TASK_STACK_DEPTH_BYTES,
        .stackBuffer = &stack[0],
        .priority = priority,
        .coreID = coreID,
        .taskIntervalMicroSeconds = taskIntervalMicroSeconds
    };
#if defined(FRAMEWORK_USE_FREERTOS)
    assert(strlen(taskInfo.name) < configMAX_TASK_NAME_LEN && "Backchannel: taskname too long");
    assert(taskInfo.priority < configMAX_PRIORITIES && "Backchannel: priority too high");

    static StaticTask_t taskBuffer;
    taskInfo.taskHandle = xTaskCreateStaticPinnedToCore(
        BackchannelTask::Task,
        taskInfo.name,
        taskInfo.stackDepth / sizeof(StackType_t),
        &taskParameters,
        taskInfo.priority,
        taskInfo.stackBuffer,
        &taskBuffer,
        taskInfo.coreID
    );
    assert(taskInfo.taskHandle != nullptr && "Unable to create BackchannelTask.");
#else
    (void)taskParameters;
#endif // FRAMEWORK_USE_FREERTOS

    return &backchannelTask;
}
