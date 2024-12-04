#include <FreeRTOS.h>
#include <task.h>

#include "imu.h"

#include "SensorPacket.h"
#include "byte_utils.h"
#include "sd_logger.h"

#include <ICM_20948.h>
#include <SPI.h>

namespace imu
{
    ICM_20948_SPI icm20948;
    void task(void *pvParam)
    {
        pinMode(LED_BUILTIN,OUTPUT);

        digitalWrite(LED_BUILTIN,HIGH);
        // IMU setup
        SPI1.setRX(12);
        SPI1.setCS(13);
        SPI1.setSCK(14);
        SPI1.setTX(15);
        SPI1.begin();
        bool initialized = false;
        while (!initialized)
        {
            icm20948.begin(13, SPI1);
            if (icm20948.status != ICM_20948_Stat_Ok)
            {
                Serial.println("rerty...");
                vTaskDelay(100);
            }
            else
            {
                initialized = true;
            }
        }

        auto timerIMU = xTimerCreate("imu", 20, pdTRUE, 0, imu::timer_callback);
        xTimerStart(timerIMU, 0);
        while(1){
            vTaskDelay(100000);
        }
    }
    void timer_callback(TimerHandle_t xTimer)
    {
        union
        {
            IMUData data;
            uint8_t bytes[sizeof(data)];
        } spkt;
        digitalWrite(LED_BUILTIN, HIGH);
        icm20948.getAGMT();
        spkt.data.id = 0x40;
        spkt.data.timestamp = millis();
        spkt.data.a_x = icm20948.accX()*0.00980665f;
        spkt.data.a_y = icm20948.accY()*0.00980665f;
        spkt.data.a_z = icm20948.accZ()*0.00980665f;
        spkt.data.w_x = icm20948.gyrX();
        spkt.data.w_y = icm20948.gyrY();
        spkt.data.w_z = icm20948.gyrZ();

        swap32<uint32_t>(&spkt.data.timestamp);
        swap32<float>(&spkt.data.a_x);
        swap32<float>(&spkt.data.a_y);
        swap32<float>(&spkt.data.a_z);
        swap32<float>(&spkt.data.w_x);
        swap32<float>(&spkt.data.w_y);
        swap32<float>(&spkt.data.w_z);
        sd_logger::write_pkt(spkt.bytes, sizeof(spkt.bytes));
        digitalWrite(LED_BUILTIN, LOW);
    }
}