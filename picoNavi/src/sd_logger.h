#pragma once

#include <stdint.h>
#include <stddef.h>

namespace sd_logger
{
    void write_pkt(const uint8_t *buffer, size_t size);
    void task(void *pvParam);
    void set_timestamp_offset(int64_t val);
    void set_timestamp_offset(uint16_t year,uint8_t month,uint8_t day,uint8_t hour,uint8_t minute,uint8_t second);
}