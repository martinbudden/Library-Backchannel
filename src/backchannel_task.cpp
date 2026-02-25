#include "backchannel_task.h"
#include "backchannel_base.h"
#include <cassert>
#include <time_microseconds.h>

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
    // calculate _tick_count_delta to get actual deltaT value, since we may have been delayed for more than task_interval_ticks
#if defined(FRAMEWORK_USE_FREERTOS)
    const Tick_type_t tick_count = xTaskGetTickCount();
#else
    const uint32_t tick_count = time_ms();
#endif

    _tick_count_delta = tick_count - _tick_count_previous;
    _tick_count_previous = tick_count;
    // guard against the case of this while loop executing twice on the same tick interval
    if (_tick_count_delta > 0) { // cppcheck-suppress knownConditionTrueFalse
        if (_backchannel.processed_received_packet() == false) { // NOLINT(readability-simplify-boolean-expr)
            // we didn't receive a packet (which can trigger a subsequent send)
            // so send a telemetry packet, if any requests outstanding
            _backchannel.send_packet();
        }
    }
}

/*!
Task function for the BackchannelTask.
*/
[[noreturn]] void BackchannelTask::task()
{
#if defined(FRAMEWORK_USE_FREERTOS)
    const uint32_t task_interval_ticks = pdMS_TO_TICKS(_task_interval_microseconds / 1000);
    assert(task_interval_ticks > 0 && "BackchannelTask task_interval_ticks is zero.");
    _previous_wake_time_ticks = xTaskGetTickCount();

    while (true) {
        // delay until the end of the next task_interval_ticks
#if (tskKERNEL_VERSION_MAJOR > 10) || ((tskKERNEL_VERSION_MAJOR == 10) && (tskKERNEL_VERSION_MINOR >= 5))
            const Base_type_t was_delayed = xTaskDelayUntil(&_previous_wake_time_ticks, task_interval_ticks);
            if (was_delayed) {
                _was_delayed = true;
            }
#else
            vTaskDelayUntil(&_previous_wake_time_ticks, task_interval_ticks);
#endif
        if (_backchannel.processed_received_packet() == false) {
            // we didn't receive a packet (which can trigger a subsequent send)
            // so send a telemetry packet, if any requests outstanding
            _backchannel.send_packet();
        }
    }
#else
    while (true) {}
#endif // FRAMEWORK_USE_FREERTOS
}

/*!
Wrapper function for BackchannelTask::Task with the correct signature to be used in xTaskCreate.
*/
[[noreturn]] void BackchannelTask::task_static(void* arg)
{
    const TaskBase::parameters_t* parameters = static_cast<TaskBase::parameters_t*>(arg);

    static_cast<BackchannelTask*>(parameters->task)->task(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast}
}
