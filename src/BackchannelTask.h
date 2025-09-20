#pragma once

#include <TaskBase.h>

class BackchannelBase;

class BackchannelTask : public TaskBase {
public:
    BackchannelTask(uint32_t taskIntervalMicroseconds, BackchannelBase& backchannel) :
        TaskBase(taskIntervalMicroseconds),
        _backchannel(backchannel) {}
public:
    static BackchannelTask* createTask(task_info_t& taskInfo, BackchannelBase& backchannel, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroseconds);
    static BackchannelTask* createTask(BackchannelBase& backchannel, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroseconds);
public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    BackchannelBase& _backchannel;
};
