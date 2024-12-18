#include "sd_logger.h"

#include "byte_utils.h"

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <SD.h>
#include <SPI.h>

namespace sd_logger
{
    /* Semaphore for access control of buffer*/
    SemaphoreHandle_t xSemaphore = NULL;
    StaticSemaphore_t xMutexBuf;

    const size_t buf_size_col = 4096;
    const size_t buf_size_row = 16;
    uint8_t buf[buf_size_row][buf_size_col];
    int idx = 0;
    uint8_t row = 0, track = 0;
    int64_t offset = 0;

    void inline write_raw(uint8_t);

    /// @brief uSDに保存するタスク
    /// @param pvParam 
    void task(void *pvParam)
    {
        sd_logger::xSemaphore = xSemaphoreCreateMutexStatic(&sd_logger::xMutexBuf);

        while (!SD.begin(17))
        {
            if (Serial)
            {
                // Serial.println("Card failed, or not present");
            }
            vTaskDelay(100);
        }

        File f = SD.open("log.bin", FILE_WRITE);
        while (1)
        {

            while (sd_logger::row != sd_logger::track)
            {
                if (f)
                {
                    digitalWrite(LED_BUILTIN, HIGH);
                    f.write(sd_logger::buf[sd_logger::track], sd_logger::buf_size_col);
                    f.flush();
                    digitalWrite(LED_BUILTIN, LOW);
                }
                sd_logger::track++;
                if(sd_logger::track==sd_logger::buf_size_row){
                    sd_logger::track=0;
                }
            }
            vTaskDelay(10);
        }
    }

    void write_pkt(const uint8_t *buffer, size_t size)
    {

        union
        {
            int64_t timestamp;
            uint8_t bytes[8];
        } t2u;

        uint8_t cobs_buf_idx = 0;
        uint8_t cobs_buf[256];

        if(offset==0){
            return;
        }

        t2u.timestamp = millis() + sd_logger::offset;
        swap64<int64_t>(&t2u.timestamp);

        if (size == 0)
        {
            return;
        }
        xSemaphoreTake(sd_logger::xSemaphore, (TickType_t)portMAX_DELAY);
        for (uint8_t i = 0; i < sizeof(t2u.timestamp); i++)
        {
            if (t2u.bytes[i] == 0x00)
            {
                sd_logger::write_raw(cobs_buf_idx + 1);
                for (uint8_t j = 0; j < cobs_buf_idx; j++)
                {
                    sd_logger::write_raw(cobs_buf[j]);
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
                sd_logger::write_raw(cobs_buf_idx + 1);
                for (uint8_t j = 0; j < cobs_buf_idx; j++)
                {
                    sd_logger::write_raw(cobs_buf[j]);
                }
                cobs_buf_idx = 0;
            }
            else
            {
                cobs_buf[cobs_buf_idx] = buffer[i];
                cobs_buf_idx++;
            }
        }
        sd_logger::write_raw(cobs_buf_idx + 1);
        for (uint8_t j = 0; j < cobs_buf_idx; j++)
        {
            sd_logger::write_raw(cobs_buf[j]);
        }
        sd_logger::write_raw(0x00);
        xSemaphoreGive(sd_logger::xSemaphore);
    }

    void inline write_raw(uint8_t data)
    {
        sd_logger::buf[sd_logger::row][sd_logger::idx] = data;
        sd_logger::idx++;
        if (sd_logger::idx == sd_logger::buf_size_col)
        {
            sd_logger::row++;
            if (sd_logger::row == sd_logger::buf_size_row)
            {
                sd_logger::row = 0;
            }
            sd_logger::idx = 0;
        }
    }

    void set_timestamp_offset(int64_t val){
        offset=val;
    }
}