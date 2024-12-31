#pragma once

#include <timers.h>

namespace imu{
    void task(void* pvParam);
    void timer_callback(TimerHandle_t xTimer);
}