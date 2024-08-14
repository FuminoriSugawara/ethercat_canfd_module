//#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_log.h"
//#include <SPI.h>
//#include <ACAN2517FD.h>
#include "esp32_can.h"
#include "EasyCAT.h"
#include <RealmanMotorDriver.hpp>

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#define PIN_NUM_MISO_HSPI 12
#define PIN_NUM_MOSI_HSPI 13
#define PIN_NUM_CLK_HSPI  14
#define PIN_NUM_CS_HSPI   15

#define UART_NUM UART_NUM_0
#define BUF_SIZE (1024)


static const char *TAG = "SPI_EXAMPLE";
unsigned long previousMs = 0;
const int OUTPUTS_COUNT = 8;
// EasyCAT buffer
PROCBUFFER_OUT _EasyCAT_BufferOut;  // output process data buffer


CAN_FRAME_FD message;

realman_motor_driver::RealmanMotorDriver motorDriver{0x07, std::shared_ptr<CAN_COMMON>(&CAN1)};


//ACAN2517FD can(PIN_NUM_CS, SPI, 255);

int32_t position = 0;


bool easyCAT_setup()
{

    bool success = EasyCAT_Init(PIN_NUM_CS_HSPI, ASYNC);
    if (success)
    {
        printf("EasyCAT initialized\n");
    }
    else
    {
        printf("EasyCAT initialization failed\n");
    }
    return success;
}

void canfd_task(void *pvParameters) {
    motorDriver.initialize();
    motorDriver.setDriverEnabled();
    motorDriver.setPositionControlMode();

    //uint8_t counter = 0;
    while (1) {
        if (CAN1.readFD(message))
        {
            motorDriver.processCANFDMessage(message);
        }
        float target_position = (float) (_EasyCAT_BufferOut.Cust.Position_1) / 10000.0;
        motorDriver.setTargetPosition(target_position);
        motorDriver.updateCurrentPosition();
        position = (int32_t) (motorDriver.getCurrentPosition() * 10000.0);
        
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }



}

void EasyCAT_Application ()                                        

{
  #define ECAT_APP_CYCLE_TIME_MS 10

  unsigned long nowMs = millis();
  if (nowMs - previousMs >= ECAT_APP_CYCLE_TIME_MS)
  {      
    previousMs = nowMs;
    

    _EasyCAT_BufferOut = EasyCAT_BufferOut;


    EasyCAT_BufferIn.Cust.ActualPosition_1 = position;
    EasyCAT_BufferIn.Cust.ActualPosition_2 = 0;
    EasyCAT_BufferIn.Cust.ActualPosition_3 = 0;
    EasyCAT_BufferIn.Cust.ActualPosition_4 = 0;
    EasyCAT_BufferIn.Cust.ActualPosition_5 = 0;
    EasyCAT_BufferIn.Cust.ActualPosition_6 = 0;

  }
}

void easyCAT_task(void *pvParameters) {
    bool success = easyCAT_setup();  // EasyCAT初期化
    if (!success) {
        printf("EasyCAT initialization failed\n");
        return;
    }

    printf("EasyCAT task started\n");
    while (1) {
        unsigned char status = EasyCAT_MainTask();
        // printf("EasyCAT status: %d\n", status);
        EasyCAT_Application();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

extern "C" void app_main(void)
{

    Serial.begin(115200);


    printf("Starting CAN FD task\n");
    if (CAN1.beginFD(1000000, 5000000))
    {
        printf("MCP2517FD Init OK ...\n");
    }
    else
    {
        printf("MCP2517FD Init Failed ...\n");
    }
	CAN1.watchFor(); 

    xTaskCreate(canfd_task, "canfd_task", 4096, NULL, 5, NULL);
    xTaskCreate(easyCAT_task, "easyCAT_task", 4096, NULL, 5, NULL);

}