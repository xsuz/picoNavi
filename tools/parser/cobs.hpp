#pragma once

#include<cstdint>

namespace parser::cobs {
    constexpr size_t MAX_ENCODED_SIZE=256;
    size_t encode(const uint8_t* input, size_t input_size, uint8_t* output);
    size_t decode(const uint8_t* input, size_t input_size, uint8_t* output );
}