#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include <map>

#include <SEGGER_RTT.h>

#include "sd_logger.h"
#include "gnss.h"

TaskHandle_t Task1;
TaskHandle_t Task2;

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);

    xTaskCreate(gnss::task,"gps",512,NULL,4,&Task2);
    xTaskCreate(sd_logger::task,"sd",256,NULL,1,&Task1);
    // vTaskStartScheduler();
}

void loop()
{
    // put your main code here, to run repeatedly:
}

void setup1(){
    // Core System
}

void loop1(){
    delay(1000);
}
