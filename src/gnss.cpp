#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>

#include "gnss.h"
#include "SensorPacket.h"
#include "byte_utils.h"
#include "sd_logger.h"

// #include <ctime>

#include <TinyGPSPlus.h>

namespace gnss
{
    /// @brief NMEAパーサー
    TinyGPSPlus gps;
    // std::tm timeinfo;
    int64_t t = 0;
    int64_t offset = 0;

    constexpr float deg2radf = PI / 180.0f;
    constexpr float threshold_hdop = 2.0;

    /// @brief PPS信号の割り込みハンドラ
    /// @param gpio
    /// @param emask
    void pps_callback(uint gpio, uint32_t emask)
    {
        gpio_set_irq_enabled(gpio, (GPIO_IRQ_EDGE_RISE), false);
        if (gps.time.isValid() && gps.date.isValid())
        {
            // t = std::time(nullptr);
            // timeinfo.tm_sec = gps.time.second();
            // timeinfo.tm_min = gps.time.minute();
            // timeinfo.tm_hour = gps.time.hour();
            // timeinfo.tm_mday = gps.date.day();
            // timeinfo.tm_mon = gps.date.month() - 1;
            // timeinfo.tm_year = gps.date.year() - 1900;
            // timeinfo.tm_isdst = 0;
            // t = std::mktime(&timeinfo);

            uint16_t year = gps.date.year();
            uint8_t month = gps.date.month();
            uint8_t day = gps.date.day();
            uint8_t hour = gps.time.hour();
            uint8_t minute = gps.time.minute();
            uint8_t second = gps.time.second();

            int dl = year / 4 - year / 100 + year / 400;
            constexpr int dl1970 = 1970 / 4 - 1970 / 100 + 1970 / 400;
            t = (year - 1970) * 365 + dl - dl1970;
            switch (month)
            {
            case 1:
                t += 0;
                break;
            case 2:
                t += 31;
                break;
            case 3:
                t += 59;
                break;
            case 4:
                t += 90;
                break;
            case 5:
                t += 120;
                break;
            case 6:
                t += 151;
                break;
            case 7:
                t += 181;
                break;
            case 8:
                t += 212;
                break;
            case 9:
                t += 243;
                break;
            case 10:
                t += 273;
                break;
            case 11:
                t += 304;
                break;
            case 12:
                t += 334;
                break;
            default:
                break;
            }
            t += day - 1;
            t *= 24;
            t += hour;
            t *= 60;
            t += minute;
            t *= 60;
            t += second;
            offset = t * 1000 - millis();
            sd_logger::set_timestamp_offset(offset);
            // sd_logger::set_timestamp_offset(((((gps.time.hour() + 9) % 24) * 60 + gps.time.minute()) * 60 + gps.time.second() + 1) * 1000 - millis());
        }
        gpio_set_irq_enabled(gpio, (GPIO_IRQ_EDGE_RISE), true);
    }

    /// @brief GPSレシーバの受信タスク
    /// @param pvParam
    void task(void *pvParam)
    {
        // UART0を初期化
        Serial1.setFIFOSize(2048);
        Serial1.begin(9600);
        delay(1000);                           // GPSレシーバの起動を待機
        Serial1.println("$PMTK251,115200*1F"); // Baudrateを115200に変更
        delay(1000);
        Serial1.flush();       // 無効なデータを破棄
        Serial1.begin(115200); // baudrate 115200で再度UART0を初期化
        delay(100);
        Serial1.println("$PMTK220,100*2F");     // 送信頻度を100ms間隔に変更
        Serial1.println("$PMTK353,1,0,1,1*36"); // GLONASSを無効化

        // PPSによる割り込み設定
        gpio_init(2);
        gpio_set_dir(2, GPIO_IN);
        gpio_set_irq_enabled_with_callback(2, GPIO_IRQ_EDGE_RISE, true, &pps_callback);

        //
        auto timerIMU = xTimerCreate("imu", 100, pdTRUE, 0, timer_callback);
        xTimerStart(timerIMU, 0);

        while (1)
        {
            while (Serial1.available() > 0)
            {
                uint8_t c = Serial1.read();
                gps.encode(c);
                Serial.write(c);
            }
            vTaskDelay(1);
        }
    }

    /// @brief GPSデータのサンプリング
    /// @param pvParam
    void timer_callback(TimerHandle_t xTimer)
    {
        union
        {
            GPSData data;
            uint8_t bytes[sizeof(data)];
        } spkt;
        if (gps.location.isValid() && (gps.hdop.hdop() < threshold_hdop))
        {
            spkt.data.id = 0x60;
            spkt.data.latitude = gps.location.lat();
            spkt.data.longitude = gps.location.lng();
            spkt.data.alt = gps.altitude.meters();
            spkt.data.ve = -gps.speed.mps() * sinf(gps.course.deg() * deg2radf);
            spkt.data.vn = gps.speed.mps() * cosf(gps.course.deg() * deg2radf);
            spkt.data.timestamp = millis();
            spkt.data.hdop = gps.hdop.hdop();
            swap64<double>(&spkt.data.latitude);
            swap64<double>(&spkt.data.longitude);
            swap32<float>(&spkt.data.alt);
            swap32<float>(&spkt.data.ve);
            swap32<float>(&spkt.data.vn);
            swap32<uint32_t>(&spkt.data.timestamp);
            swap32<float>(&spkt.data.hdop);

            sd_logger::write_pkt(spkt.bytes, sizeof(spkt.bytes));
        }
        else
        {
            digitalWrite(LED_BUILTIN, LOW);
        }
    }
}