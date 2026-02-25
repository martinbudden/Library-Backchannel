#pragma once

#include <task_base.h>

class BackchannelBase;
struct backchannel_parameter_group_t;


class BackchannelTask : public TaskBase {
public:
    BackchannelTask(uint32_t task_interval_microseconds, BackchannelBase& backchannel, backchannel_parameter_group_t& parameter_group) :
        TaskBase(task_interval_microseconds),
        _backchannel(backchannel),
        _parameter_group(parameter_group)
        {}
public:
    static BackchannelTask* create_task(task_info_t& task_info, BackchannelBase& backchannel, backchannel_parameter_group_t& parameter_group, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
    static BackchannelTask* create_task(BackchannelBase& backchannel, backchannel_parameter_group_t& parameter_group, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
public:
    [[noreturn]] static void task_static(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    BackchannelBase& _backchannel;
    backchannel_parameter_group_t& _parameter_group;
};
