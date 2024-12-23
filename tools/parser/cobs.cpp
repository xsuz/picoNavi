#include "cobs.hpp"

/// @brief COBS encode
/// @param input  The input data
/// @param input_size  The size of the input data
/// @param output  The output buffer
/// @param output_size  The size of the output buffer
/// @return size_t(-1) if failed, otherwise the size of the encoded data
size_t parser::cobs::encode(const uint8_t* input, size_t input_size, uint8_t* output) {
    uint8_t cobs_buf[255],cobs_buf_idx=0;
    size_t out_idx=0;
    if((input_size==0)||(input_size>=MAX_ENCODED_SIZE)){
        return -1;
    }
    for(size_t i=0;i<input_size;i++){
        if(input[i]==0){
            output[out_idx]=cobs_buf_idx+1;
            out_idx++;
            for(uint8_t j=0;j<cobs_buf_idx;j++){
                output[out_idx]=cobs_buf[j];
                out_idx++;
            }
            cobs_buf_idx=0;
        }else{
            cobs_buf[cobs_buf_idx]=input[i];
            cobs_buf_idx++;
            if(cobs_buf_idx==0xFF){
                output[out_idx]=cobs_buf_idx+1;
                out_idx++;
                for(uint8_t j=0;j<cobs_buf_idx;j++){
                    output[out_idx]=cobs_buf[j];
                    out_idx++;
                }
                cobs_buf_idx=0;
            }
        }
    }
    output[out_idx]=cobs_buf_idx+1;
    out_idx++;
    for(uint8_t j=0;j<cobs_buf_idx;j++){
        output[out_idx]=cobs_buf[j];
        out_idx++;
    }
    output[out_idx]=0x00;
    out_idx++;
    return out_idx;
}

size_t parser::cobs::decode(const uint8_t* input, size_t input_size, uint8_t* output) {
    size_t enc_idx=0,next0_idx=0,out_idx=0;
    bool is_overhead=true,is_end=false;
    while(enc_idx < input_size && !is_end) {
        if(next0_idx == 0) {
            if(input[enc_idx] == 0) {
                is_end=true;
            } else {
                if(!is_overhead) {
                    output[out_idx]=0;
                    out_idx++;
                }
                next0_idx=input[enc_idx];
                enc_idx++;
                if(next0_idx == 0xFF){
                    is_overhead=true;
                }else{
                    is_overhead=false;
                }
            }
        }else{
            output[out_idx]=input[enc_idx];
            out_idx++;
            enc_idx++;
        }
        next0_idx--;
    }
    if(!is_end) {
        return 0;
    }
    return out_idx;
}