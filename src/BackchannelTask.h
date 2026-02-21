#pragma once

#include <task_base.h>

class BackchannelBase;

class BackchannelTask : public TaskBase {
public:
    BackchannelTask(uint32_t task_interval_microseconds, BackchannelBase& backchannel) :
        TaskBase(task_interval_microseconds),
        _backchannel(backchannel) {}
public:
    static BackchannelTask* create_task(task_info_t& task_info, BackchannelBase& backchannel, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
    static BackchannelTask* create_task(BackchannelBase& backchannel, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
public:
    [[noreturn]] static void task_static(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    BackchannelBase& _backchannel;
};
