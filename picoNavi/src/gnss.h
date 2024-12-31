#pragma once
#include <Arduino.h>
#include <timers.h>

namespace gnss{
    void pps_callback(uint gpio,uint32_t emask);
    void task(void* pvParam);
    void timer_callback(TimerHandle_t xTimer);
}