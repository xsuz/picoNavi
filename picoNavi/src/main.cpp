#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

#include "sd_logger.h"
#include "gnss.h"
#include "imu.h"

TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);

    xTaskCreate(gnss::task,"gps_task",512,NULL,2,&Task2);
    xTaskCreate(imu::task,"imu_task",512,NULL,3,&Task3);
    xTaskCreate(sd_logger::task,"sd_buf_task",512,NULL,1,&Task1);
    vTaskStartScheduler();
}

void loop()
{
    // put your main code here, to run repeatedly:
}

void setup1(){
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop1(){
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
}
