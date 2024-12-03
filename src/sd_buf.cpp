#include "sd_buf.h"

#include "byte_utils.h"

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <SD.h>
#include <SPI.h>

namespace sd_buf
{
    /* Semaphore for access control of buffer*/
    SemaphoreHandle_t xSemaphore = NULL;
    StaticSemaphore_t xMutexBuf;

    const size_t size = 512;
    uint8_t buf[256][size];
    int idx = 0;
    uint8_t row = 0, track = 0;
    uint32_t offset = 0;

    void inline write_raw(uint8_t);

    /// @brief uSDに保存するタスク
    /// @param pvParam 
    void task(void *pvParam)
    {
        sd_buf::xSemaphore = xSemaphoreCreateMutexStatic(&sd_buf::xMutexBuf);

        while (!SD.begin(17))
        {
            if (Serial)
            {
                // Serial.println("Card failed, or not present");
            }
            vTaskDelay(100);
        }

        while (1)
        {

            while (sd_buf::row != sd_buf::track)
            {
                File f = SD.open("log-new.bin", FILE_WRITE);
                if (f)
                {
                    f.write(sd_buf::buf[sd_buf::track], sd_buf::size);
                    f.close();
                }
                sd_buf::track++;
                // Serial.print("+");
            }
            vTaskDelay(10);
        }
    }

    void write_pkt(const uint8_t *buffer, size_t size)
    {

        union
        {
            uint32_t timestamp;
            uint8_t bytes[4];
        } t2u;

        uint8_t cobs_buf_idx = 0;
        uint8_t cobs_buf[256];

        if(offset==0){
            return;
        }

        t2u.timestamp = millis() + sd_buf::offset;
        swap32<uint32_t>(&t2u.timestamp);

        if (size == 0)
        {
            return;
        }
        xSemaphoreTake(sd_buf::xSemaphore, (TickType_t)portMAX_DELAY);
        for (uint8_t i = 0; i < 4; i++)
        {
            if (t2u.bytes[i] == 0x00)
            {
                sd_buf::write_raw(cobs_buf_idx + 1);
                for (uint8_t j = 0; j < cobs_buf_idx; j++)
                {
                    sd_buf::write_raw(cobs_buf[j]);
                }
                cobs_buf_idx = 0;
            }
            else
            {
                cobs_buf[cobs_buf_idx] = t2u.bytes[i];
                cobs_buf_idx++;
            }
        }
        for (uint8_t i = 0; i < size; i++)
        {
            if (buffer[i] == 0x00)
            {
                sd_buf::write_raw(cobs_buf_idx + 1);
                for (uint8_t j = 0; j < cobs_buf_idx; j++)
                {
                    sd_buf::write_raw(cobs_buf[j]);
                }
                cobs_buf_idx = 0;
            }
            else
            {
                cobs_buf[cobs_buf_idx] = buffer[i];
                cobs_buf_idx++;
            }
        }
        sd_buf::write_raw(cobs_buf_idx + 1);
        for (uint8_t j = 0; j < cobs_buf_idx; j++)
        {
            sd_buf::write_raw(cobs_buf[j]);
        }
        sd_buf::write_raw(0x00);
        xSemaphoreGive(sd_buf::xSemaphore);
    }

    void inline write_raw(uint8_t data)
    {
        sd_buf::buf[sd_buf::row][sd_buf::idx] = data;
        sd_buf::idx++;
        if (sd_buf::idx == sd_buf::size)
        {
            sd_buf::row++;
            sd_buf::idx = 0;
        }
    }

    void set_timestamp_offset(uint32_t val){
        offset=val;
    }
}