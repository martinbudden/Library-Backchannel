#include "BackchannelTask.h"
#include "BackchannelBase.h"
#include <TimeMicroseconds.h>
#include <cassert>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <task.h>
#endif
#endif


/*!
loop() function for when not using FREERTOS
*/
void BackchannelTask::loop()
{
    // calculate _tickCountDelta to get actual deltaT value, since we may have been delayed for more than taskIntervalTicks
#if defined(FRAMEWORK_USE_FREERTOS)
    const TickType_t tickCount = xTaskGetTickCount();
#else
    const uint32_t tickCount = timeMs();
#endif

    _tickCountDelta = tickCount - _tickCountPrevious;
    _tickCountPrevious = tickCount;
    // guard against the case of this while loop executing twice on the same tick interval
    if (_tickCountDelta > 0) { // cppcheck-suppress knownConditionTrueFalse
        if (_backchannel.processedReceivedPacket() == false) { // NOLINT(readability-simplify-boolean-expr)
            // we didn't receive a packet (which can trigger a subsequent send)
            // so send a telemetry packet, if any requests outstanding
            _backchannel.sendPacket();
        }
    }
}

/*!
Task function for the BackchannelTask.
*/
[[noreturn]] void BackchannelTask::task()
{
#if defined(FRAMEWORK_USE_FREERTOS)
    const uint32_t taskIntervalTicks = pdMS_TO_TICKS(_taskIntervalMicroseconds / 1000);
    assert(taskIntervalTicks > 0 && "BackchannelTask taskIntervalTicks is zero.");
    _previousWakeTimeTicks = xTaskGetTickCount();

    while (true) {
        // delay until the end of the next taskIntervalTicks
        vTaskDelayUntil(&_previousWakeTimeTicks, taskIntervalTicks);

        if (_backchannel.processedReceivedPacket() == false) {
            // we didn't receive a packet (which can trigger a subsequent send)
            // so send a telemetry packet, if any requests outstanding
            _backchannel.sendPacket();
        }
    }
#else
    while (true) {}
#endif // FRAMEWORK_USE_FREERTOS
}

/*!
Wrapper function for BackchannelTask::Task with the correct signature to be used in xTaskCreate.
*/
[[noreturn]] void BackchannelTask::Task(void* arg)
{
    const TaskBase::parameters_t* parameters = static_cast<TaskBase::parameters_t*>(arg);

    static_cast<BackchannelTask*>(parameters->task)->task(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast}
}
