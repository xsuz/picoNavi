#include "cobs.hpp"
#include "SensorPacket.h"
#include <iostream>

int main()
{
    uint8_t encoded[256];
    uint8_t decoded[256];
    union
    {
        DataPacket::IMUData data;
        uint8_t bytes[sizeof(data)];
    } spkt;
    spkt.data.timestamp = 0;
    spkt.data.id = DataPacket::DeviceID::IMU;
    spkt.data.a_x = 0.0;
    spkt.data.a_y = 0.1;
    spkt.data.a_z = 0.2;
    spkt.data.w_x = 0.3;
    spkt.data.w_y = 0.4;
    spkt.data.w_z = 0.5;
    spkt.data.m_x = 0.6;
    spkt.data.m_y = 0.7;
    spkt.data.m_z = 0.8;
    spkt.data.q0 =  0.0;
    spkt.data.q1 =  0.1;
    spkt.data.q2 =  0.2;
    spkt.data.q3 =  0.3;
    uint8_t len = 6;
    size_t encoded_len = parser::cobs::encode(spkt.bytes, sizeof(spkt.data), encoded);
    spkt.data.timestamp = 0;
    spkt.data.id = DataPacket::DeviceID::IMU|0x01;
    spkt.data.a_x = 0.0;
    spkt.data.a_y = 0.1;
    spkt.data.a_z = 0.2;
    spkt.data.w_x = 0.3;
    spkt.data.w_y = 0.4;
    spkt.data.w_z = 0.5;
    spkt.data.m_x = 0.6;
    spkt.data.m_y = 0.7;
    spkt.data.m_z = 0.8;
    spkt.data.q0 =  0.0;
    spkt.data.q1 =  0.1;
    spkt.data.q2 =  0.2;
    spkt.data.q3 =  0.3;
    encoded_len += parser::cobs::encode(spkt.bytes, sizeof(spkt.data), encoded + encoded_len);
    std::cout << "Encoded length : " << encoded_len << std::endl;
    std::cout << "Encoded : [ ";
    for (size_t i = 0; i < encoded_len; i++)
    {
        std::cout << std::hex << int(encoded[i]) << " ";
    }
    std::cout << "]" << std::endl;
    size_t idx = 0, pkt_cnt = 0;
    std::cout << "Decoded : " << std::endl;
    while (idx < encoded_len)
    {
        size_t decoded_len = parser::cobs::decode(encoded + idx, encoded_len - idx, decoded);
        if (decoded_len == 0)
        {
            std::cout << "Decode failed" << std::endl;
            break;
        }
        pkt_cnt++;
        std::cout << pkt_cnt << " : [ ";
        switch (decoded[0]&0xF0)
        {
        case DataPacket::DeviceID::IMU:{
            std::cout << "IMUData : ";
            for (size_t i = 0; i < sizeof(DataPacket::IMUData); i++)
            {
                spkt.bytes[i] = decoded[i];
            }
            std::cout << "id:" << std::hex << int(spkt.data.id) << ", ";
            std::cout << "timestamp:" << std::hex << int(spkt.data.timestamp) << ", ";
            std::cout << "a_x:" << spkt.data.a_x << ", ";
            std::cout << "a_y:" << spkt.data.a_y << ", ";
            std::cout << "a_z:" << spkt.data.a_z << ", ";
            std::cout << "w_x:" << spkt.data.w_x << ", ";
            std::cout << "w_y:" << spkt.data.w_y << ", ";
            std::cout << "w_z:" << spkt.data.w_z << ", ";
            std::cout << "m_x:" << spkt.data.m_x << ", ";
            std::cout << "m_y:" << spkt.data.m_y << ", ";
            std::cout << "m_z:" << spkt.data.m_z << ", ";
            std::cout << "q0:" << spkt.data.q0 << ", ";
            std::cout << "q1:" << spkt.data.q1 << ", ";
            std::cout << "q2:" << spkt.data.q2 << ", ";
            std::cout << "q3:" << spkt.data.q3 << " ";
        }break;
        default:{
            std::cout << "Unknown : ";
            for (size_t i = 0; i < decoded_len; i++)
            {
                std::cout << std::hex << int(decoded[i]) << " ";
            }
        }break;
        }
        std::cout << "]" << std::endl;
        while ((encoded[idx] != 0) && (idx < encoded_len))
        {
            idx++;
        }
        idx++;
    }
    return 0;
}