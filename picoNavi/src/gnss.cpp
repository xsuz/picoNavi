#include <FreeRTOS.h>
#include <task.h>
#include <timers.h>
#include <queue.h>

#include "gnss.h"
#include "DeviceData.h"
#include "byte_utils.h"
#include "sd_logger.h"
#include "clock.h"

// #include <ctime>

#include <SEGGER_RTT.h>
#include <TinyGPSPlus.h>

namespace gnss
{
    /// @brief NMEAパーサ
    TinyGPSPlus gps;
    TinyGPSCustom PSTMPV_UTC(gps, "PSTMPV", 1);   // UTC time of position fix
    TinyGPSCustom PSTMPV_VelN(gps, "PSTMPV", 8);  // North velocity in m/s
    TinyGPSCustom PSTMPV_VelE(gps, "PSTMPV", 9);  // East velocity in m/s
    TinyGPSCustom PSTMPV_VelD(gps, "PSTMPV", 10); // Down velocity in m/s

    TinyGPSCustom PSTMTS_PRN(gps, "PSTMTS", 2);       // Satellite PRN number
    TinyGPSCustom PSTMTS_PsR(gps, "PSTMTS", 3);       // Satellite Pseudo range
    TinyGPSCustom PSTMTS_Freq(gps, "PSTMTS", 4);      // Satellite Frequency
    TinyGPSCustom PSTMTS_cp(gps, "PSTMTS", 5);        // Satellite Carrier phase measurement (cycles)
    TinyGPSCustom PSTMTS_flags(gps, "PSTMTS", 6);     // Satellite flags
    TinyGPSCustom PSTMTS_CN0(gps, "PSTMTS", 7);       // Satellite C/N0 (dB)
    TinyGPSCustom PSTMTS_Ttime(gps, "PSTMTS", 8);     // Satellite Tracking time (ms)
    TinyGPSCustom PSTMTS_SatX(gps, "PSTMTS", 10);     // Satellite X position (m)
    TinyGPSCustom PSTMTS_SatY(gps, "PSTMTS", 11);     // Satellite Y position (m)
    TinyGPSCustom PSTMTS_SatZ(gps, "PSTMTS", 12);     // Satellite Z position (m)
    TinyGPSCustom PSTMTS_VelX(gps, "PSTMTS", 13);     // Satellite X velocity (m/s)
    TinyGPSCustom PSTMTS_VelY(gps, "PSTMTS", 14);     // Satellite Y velocity (m/s)
    TinyGPSCustom PSTMTS_VelZ(gps, "PSTMTS", 15);     // Satellite Z velocity (m/s)
    TinyGPSCustom PSTMTS_Pr_Delta(gps, "PSTMTS", 19); // Satellite Pseudo range correction (m)
    TinyGPSCustom PSTMTS_Cp_Delta(gps, "PSTMTS", 20); // Satellite Carrier phase correction (cycles)

    QueueHandle_t gnssQueue;
    QueueHandle_t gnss_utcQueue;
    QueueHandle_t gnssSatelliteQueue;
    QueueHandle_t utcQueue;

    constexpr int LED = 11;

    void pps_callback(uint gpio, uint32_t emask)
    {
        gpio_set_irq_enabled(gpio, (GPIO_IRQ_EDGE_FALL), false);
        SEGGER_RTT_printf(0, "[%sDEBUG%s gnss] : pps_callback\n", RTT_CTRL_TEXT_BLUE, RTT_CTRL_RESET);
        if (gps.location.isValid() && gps.time.age() < 2000 && gps.date.isValid() && gps.date.age() < 2000)
        {
            sys_clock::calc_timestamp_offset(millis(), gps.date.year(), gps.date.month(), gps.date.day(),
                                             gps.time.hour(), gps.time.minute(), gps.time.second());
            SEGGER_RTT_printf(0, "[%sINFO%s gnss] : UTC time synchronized: %04d/%02d/%02d %02d:%02d:%02d.%02d\n",
                              RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET,
                              gps.date.year(), gps.date.month(), gps.date.day(),
                              gps.time.hour(), gps.time.minute(), gps.time.second(), gps.time.centisecond());
        }
        gpio_set_irq_enabled(gpio, (GPIO_IRQ_EDGE_FALL), true);
    }

    void pstmpv_callback()
    {
        digitalWrite(LED, HIGH);
        DeviceData::GPSData data;
        int64_t utc = TinyGPSPlus::parseDecimal(PSTMPV_UTC.value());

        auto lat = gps.location.rawLat();
        auto lng = gps.location.rawLng();
        data.latitude = (lat.deg * 1e7 + lat.billionths / 100) * (lat.negative ? -1 : 1);
        data.longitude = (lng.deg * 1e7 + lng.billionths / 100) * (lat.negative ? -1 : 1);
        data.altitude = gps.altitude.value() * 10; // cm -> mm
        data.velN = TinyGPSPlus::parseDecimal(PSTMPV_VelN.value()) * 10;
        data.velE = TinyGPSPlus::parseDecimal(PSTMPV_VelE.value()) * 10;
        data.velD = TinyGPSPlus::parseDecimal(PSTMPV_VelD.value()) * 10;
        data.timestamp = millis();

        SEGGER_RTT_printf(0, "[%sINFO%s gnss] : latitude: %d, longitude: %d, altitude: %d, velN: %d, velE: %d, velD: %d, fixType: %u\n",
                          RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET,
                          data.latitude, data.longitude, data.altitude, data.velN, data.velE, data.velD, data.fixType);

        xQueueSend(gnssQueue, &data, 0);
        xQueueSend(gnss_utcQueue, &utc, 0);

        digitalWrite(LED, LOW);
    }

    void pstmts_callback()
    {
        DeviceData::GNSSSatelliteInfo satinfo;
        satinfo.prn = atoi(PSTMTS_PRN.value());
        satinfo.psR = atof(PSTMTS_PsR.value());
        satinfo.freq = atof(PSTMTS_Freq.value());
        satinfo.cp = atof(PSTMTS_cp.value());
        satinfo.flags = atoi(PSTMTS_flags.value());
        satinfo.cn0 = atoi(PSTMTS_CN0.value());
        satinfo.tTime = atol(PSTMTS_Ttime.value());
        satinfo.satX = atof(PSTMTS_SatX.value());
        satinfo.satY = atof(PSTMTS_SatY.value());
        satinfo.satZ = atof(PSTMTS_SatZ.value());
        satinfo.velX = atof(PSTMTS_VelX.value());
        satinfo.velY = atof(PSTMTS_VelY.value());
        satinfo.velZ = atof(PSTMTS_VelZ.value());
        satinfo.prDelta = atof(PSTMTS_Pr_Delta.value());
        satinfo.cpDelta = atof(PSTMTS_Cp_Delta.value());

        int64_t utc=sys_clock::get_timestamp();
        xQueueSend(gnssSatelliteQueue, &satinfo, 0);
        xQueueSend(utcQueue, &utc, 0);
    }

    void task(void *pvParam)
    {
        union
        {
            DeviceData::GPSData data;
            uint8_t bytes[sizeof(data)];
        } spkt;
        union
        {
            DeviceData::GNSSSatelliteInfo data;
            uint8_t bytes[sizeof(data)];
        } spkt_satinfo;
        int64_t utc;
        DeviceData::CANPacket can_pkt;

        SEGGER_RTT_printf(0, "[%sINFO%s gnss] : GNSS task started.\n", RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET);
        pinMode(LED, OUTPUT);
        digitalWrite(LED, LOW);

        // UART0を初期化
        Serial1.setFIFOSize(8192);
        Serial1.begin(9600);
        delay(1000);                                // GPSレシーバの起動を待機
        Serial1.println("$PSTMSETPAR,1102,0xD*15"); // Baudrateを115200に変更
        Serial1.println("$PSTMSAVEPAR");
        Serial1.println("$PSTMSRR");
        Serial1.setFIFOSize(8192);
        Serial1.begin(921600); // baudrate 921600で再度UART0を初期化
        delay(100);
        Serial1.flush(); // 無効なデータを破棄

        // {
        //     Serial1.print('$');
        //     const char *cfg_msg = "PSTMSETPAR,1228,0x00000001";
        //     char checksum = 0;
        //     for (char *p = (char *)cfg_msg; p; p++)
        //     {
        //         Serial1.write(*p);
        //         checksum ^= *p;
        //     }
        //     Serial1.printf("*%02X\r\n", checksum);
        //     Serial1.println("$PSTMSAVEPAR");
        // }

        // Application ON/OFF
        Serial1.println("$PSTMSETPAR,1200,0x09C546C4*5A");
        Serial1.println("$PSTMSAVEPAR");

        // NMEA Port Msg-List (32bits low)
        // Serial1.println("$PSTMSETPAR,1201,0x00980056*53"); // default
        Serial1.println("$PSTMSETPAR,1201,0x00000342*54");
        // Serial1.println("$PSTMSETPAR,1201,0x0000004F*23");
        Serial1.println("$PSTMSAVEPAR");
        // NMEA Port Msg-List (32bits high)
        Serial1.println("$PSTMSETPAR,1228,0x00000001*5B");
        Serial1.println("$PSTMSAVEPAR");
        // Serial1.println("$PSTMSETPAR,1303,1*2B");// default
        // Serial1.println("$PSTMSETPAR,1303,0.1*35");
        // Serial1.println("$PSTMSETPAR,1303,0.125*32");
        // Serial1.println("$PSTMSETPAR,1303,0.2*36");
        Serial1.println("$PSTMSETPAR,1303,0.25*03");
        // Serial1.println("$PSTMSETPAR,1303,0.5*31");
        Serial1.println("$PSTMSAVEPAR");
        Serial1.println("$PSTMSRR");

        gnssQueue = xQueueCreate(5, sizeof(DeviceData::GPSData));
        gnss_utcQueue = xQueueCreate(5, sizeof(int64_t));

        gnssSatelliteQueue = xQueueCreate(20, sizeof(DeviceData::GNSSSatelliteInfo));
        utcQueue = xQueueCreate(20,sizeof(int64_t));

        if (gnssQueue == NULL)
        {
            SEGGER_RTT_printf(0, "[%sERROR%s gnss] : Failed to create GNSS queue.\n", RTT_CTRL_TEXT_RED, RTT_CTRL_RESET);
            while (1)
                ; // Halt if queue creation fails
        }
        SEGGER_RTT_printf(0, "[%sINFO%s gnss] : GNSS queue created successfully.\n", RTT_CTRL_TEXT_GREEN, RTT_CTRL_RESET);

        // PPSによる割り込み設定
        gpio_init(2);
        gpio_set_dir(2, GPIO_IN);
        gpio_set_irq_enabled_with_callback(2, GPIO_IRQ_EDGE_FALL, true, &pps_callback);

        // GNSSデータのポーリングを開始
        xTaskCreate(gnss::task_polling, "gps_polling", 256, NULL, 6, NULL);
        while (1)
        {
            
            while (uxQueueMessagesWaiting(gnssSatelliteQueue) > 0){
                xQueueReceive(gnssSatelliteQueue,&spkt_satinfo.data,0);
                xQueueReceive(utcQueue, &utc,0);
                sd_logger::write_pkt(DeviceData::SensorType::GPS|0x01, spkt_satinfo.bytes, sizeof(spkt_satinfo.bytes), utc);
            }
            while (uxQueueMessagesWaiting(gnssQueue) > 0)
            {
                xQueueReceive(gnssQueue, &spkt.data, 0);
                xQueueReceive(gnss_utcQueue, &utc, 0);

                // SEGGER_RTT_printf(0, "gnss : latitude: %d, longitude: %d, altitude: %d, velN: %d, velE: %d, velD: %d, hAcc: %u, vAcc: %u, fixType: %u, pDOP: %u\n",
                //                   spkt.data.latitude, spkt.data.longitude, spkt.data.altitude,
                //                   spkt.data.velN, spkt.data.velE, spkt.data.velD,
                //                   spkt.data.hAcc, spkt.data.vAcc,
                //                   spkt.data.fixType, spkt.data.pDOP);

                // バイトオーダーを変換
                u32::to_le(&spkt.data.latitude);
                u32::to_le(&spkt.data.longitude);
                u32::to_le(&spkt.data.altitude);
                u32::to_le(&spkt.data.velN);
                u32::to_le(&spkt.data.velE);
                u32::to_le(&spkt.data.velD);
                u32::to_le(&spkt.data.timestamp);
                sd_logger::write_pkt(DeviceData::SensorType::GPS, spkt.bytes, sizeof(spkt.bytes), utc);
            }
            vTaskDelay(10); // 10ms待機
        }
    }

    void task_polling(void *pvParam)
    {
        Serial.begin(115200);
        while (1)
        {
            if (Serial)
            {
                while (Serial.available() > 0)
                {
                    uint8_t c = Serial.read();
                    Serial1.write(c);
                }
            }
            while (Serial1.available() > 0)
            {
                uint8_t c = Serial1.read();
                gps.encode(c);
                if (Serial)
                {
                    Serial.write(c);
                }
                if (PSTMPV_UTC.isUpdated())
                {
                    pstmpv_callback();
                }
                if (PSTMTS_Pr_Delta.isUpdated()){
                    pstmts_callback();
                }
            }
            vTaskDelay(1);
        }
    }
}