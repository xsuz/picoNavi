#pragma once

#include <stdint.h>
#include <stddef.h>

namespace sd_logger
{
    void write_pkt(const uint8_t *buffer, size_t size);
    void task(void *pvParam);
    void set_timestamp_offset(int64_t val);
}