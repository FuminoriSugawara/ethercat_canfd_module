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
#define MOTOR_DRIVER_COUNT 7
//#define MOTOR_DRIVER_COUNT 4
//#define MOTOR_DRIVER_COUNT 1
#define CONTROL_MODE RMTR_SERVO_MODE_CUR
//#define CONTROL_MODE RMTR_SERVO_MODE_POS

unsigned long previousMs = 0;
const int OUTPUTS_COUNT = 8;
// EasyCAT buffer
PROCBUFFER_OUT _EasyCAT_BufferOut;  // output process data buffer


CAN_FRAME_FD message;
realman_motor_driver::RealmanMotorDriver motor_drivers[MOTOR_DRIVER_COUNT] = {
    realman_motor_driver::RealmanMotorDriver(0x01, std::shared_ptr<CAN_COMMON>(&CAN1)),
    realman_motor_driver::RealmanMotorDriver(0x02, std::shared_ptr<CAN_COMMON>(&CAN1)),
    realman_motor_driver::RealmanMotorDriver(0x03, std::shared_ptr<CAN_COMMON>(&CAN1)),
    realman_motor_driver::RealmanMotorDriver(0x04, std::shared_ptr<CAN_COMMON>(&CAN1)),
    realman_motor_driver::RealmanMotorDriver(0x05, std::shared_ptr<CAN_COMMON>(&CAN1)),
    realman_motor_driver::RealmanMotorDriver(0x06, std::shared_ptr<CAN_COMMON>(&CAN1)),
    realman_motor_driver::RealmanMotorDriver(0x07, std::shared_ptr<CAN_COMMON>(&CAN1)),
};

//realman_motor_driver::RealmanMotorDriver motorDriver{0x07, std::shared_ptr<CAN_COMMON>(&CAN1)};


//ACAN2517FD can(PIN_NUM_CS, SPI, 255);

int32_t actual_positions[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int32_t actual_velocities[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int32_t actual_torques[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint16_t status_words[8] = {0, 0, 0, 0, 0, 0, 0, 0};

int32_t target_positions[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int32_t target_velocities[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int32_t target_torques[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint16_t control_words[8] = {0, 0, 0, 0, 0, 0, 0, 0};


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

bool motorDriverSetup()
{
        // initialize motor drivers
        for (int i = 0; i < MOTOR_DRIVER_COUNT; i++)
        {
            Serial.printf("Initializing motor driver %d\n", i);
            motor_drivers[i].setConnectionOnline();
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        // for (int i = 0; i < MOTOR_DRIVER_COUNT; i++)
        // {
        //     motor_drivers[i].loadCurrentState();
        //     vTaskDelay(100 / portTICK_PERIOD_MS);
        // }
        //for (int i = 0; i < MOTOR_DRIVER_COUNT; i++)
        //{
        //    motor_drivers[i].setPositionControlMode();
        //    vTaskDelay(100 / portTICK_PERIOD_MS);
        //    motor_drivers[i].setZeroPosition();
        //    vTaskDelay(100 / portTICK_PERIOD_MS);
        //    motor_drivers[i].clearJointError();
        //    vTaskDelay(100 / portTICK_PERIOD_MS);
        //    motor_drivers[i].setDriverEnabled();
        //    vTaskDelay(100 / portTICK_PERIOD_MS);
        //}
        for (int i = 0; i < MOTOR_DRIVER_COUNT; i++)
        {
            motor_drivers[i].setDriverDisabled();
            vTaskDelay(10 / portTICK_PERIOD_MS);
            motor_drivers[i].setZeroPosition();
            vTaskDelay(10 / portTICK_PERIOD_MS);
            motor_drivers[i].clearJointError();
            vTaskDelay(10 / portTICK_PERIOD_MS);

            if (CONTROL_MODE == RMTR_SERVO_MODE_POS)
            {
                // Switch to position control mode
                motor_drivers[i].setPositionControlMode();
            }
            else if (CONTROL_MODE == RMTR_SERVO_MODE_VEL)
            {
                // Switch to velocity control mode
                motor_drivers[i].setVelocityControlMode();
            }
            else if (CONTROL_MODE == RMTR_SERVO_MODE_CUR)
            {
                // Switch to current control mode
                motor_drivers[i].setCurrentControlMode();
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
            motor_drivers[i].setDriverEnabled();
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    return true;
}

void canfd_send_task(void *pvParameters) {
    motorDriverSetup();
    //while (1) {
    //    vTaskDelay(10 / portTICK_PERIOD_MS);
    //}
    while (1) {
        for (int i = 0; i < MOTOR_DRIVER_COUNT; i++)
        {
            motor_drivers[i].loadCurrentCurrent();
            //vTaskDelay(10 / portTICK_PERIOD_MS);
            //motor_drivers[i].loadCurrentVelocity();
            vTaskDelay(1 / portTICK_PERIOD_MS);
            motor_drivers[i].loadCurrentPosition();
            vTaskDelay(1 / portTICK_PERIOD_MS);
            if (CONTROL_MODE == RMTR_SERVO_MODE_POS)
            {
                Serial.printf("Target position %d: %d\n", i, target_positions[i]);
                motor_drivers[i].setTargetPosition(target_positions[i]);
            }
            else if (CONTROL_MODE == RMTR_SERVO_MODE_VEL)
            {
                Serial.printf("Target velocity %d: %d\n", i, target_velocities[i]);
                motor_drivers[i].setTargetVelocity(target_velocities[i]);
            }
            else if (CONTROL_MODE == RMTR_SERVO_MODE_CUR)
            {
                motor_drivers[i].setTargetCurrent(target_torques[i]);
                Serial.printf("Target torque %d: %d\n", i, target_torques[i]);
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void canfd_receive_task(void *pvParameters) {

    while (1) {
        if (CAN1.readFD(message))
        {
            CAN_FRAME_FD _message = message;
            for (int i = 0; i < MOTOR_DRIVER_COUNT; i++)
            {
                motor_drivers[i].processCANFDMessage(_message);
                // update actual position
                actual_positions[i] = (int32_t)(motor_drivers[i].getCurrentPosition());
                // Serial.printf("Actual position %d: %d\n", i, actual_positions[i]);
                // update actual velocity
                actual_velocities[i] = (int32_t)(motor_drivers[i].getCurrentVelocity());
                // update actual torque
                actual_torques[i] = (int32_t)(motor_drivers[i].getCurrentTorque());
            }
        }
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

    // update target position
    target_positions[0] = _EasyCAT_BufferOut.Cust.Position_1;
    target_positions[1] = _EasyCAT_BufferOut.Cust.Position_2;
    target_positions[2] = _EasyCAT_BufferOut.Cust.Position_3;
    target_positions[3] = _EasyCAT_BufferOut.Cust.Position_4;
    target_positions[4] = _EasyCAT_BufferOut.Cust.Position_5;
    target_positions[5] = _EasyCAT_BufferOut.Cust.Position_6;
    target_positions[6] = _EasyCAT_BufferOut.Cust.Position_7;
    target_positions[7] = _EasyCAT_BufferOut.Cust.Position_8;
    // update target velocity
    target_velocities[0] = _EasyCAT_BufferOut.Cust.Velocity_1;
    target_velocities[1] = _EasyCAT_BufferOut.Cust.Velocity_2;
    target_velocities[2] = _EasyCAT_BufferOut.Cust.Velocity_3;
    target_velocities[3] = _EasyCAT_BufferOut.Cust.Velocity_4;
    target_velocities[4] = _EasyCAT_BufferOut.Cust.Velocity_5;
    target_velocities[5] = _EasyCAT_BufferOut.Cust.Velocity_6;
    target_velocities[6] = _EasyCAT_BufferOut.Cust.Velocity_7;
    target_velocities[7] = _EasyCAT_BufferOut.Cust.Velocity_8;
    // update target torque
    target_torques[0] = _EasyCAT_BufferOut.Cust.Torque_1;
    target_torques[1] = _EasyCAT_BufferOut.Cust.Torque_2;
    target_torques[2] = _EasyCAT_BufferOut.Cust.Torque_3;
    target_torques[3] = _EasyCAT_BufferOut.Cust.Torque_4;
    target_torques[4] = _EasyCAT_BufferOut.Cust.Torque_5;
    target_torques[5] = _EasyCAT_BufferOut.Cust.Torque_6;
    target_torques[6] = _EasyCAT_BufferOut.Cust.Torque_7;
    target_torques[7] = _EasyCAT_BufferOut.Cust.Torque_8;
    // update control word
    control_words[0] = _EasyCAT_BufferOut.Cust.ControlWord_1;
    control_words[1] = _EasyCAT_BufferOut.Cust.ControlWord_2;
    control_words[2] = _EasyCAT_BufferOut.Cust.ControlWord_3;
    control_words[3] = _EasyCAT_BufferOut.Cust.ControlWord_4;
    control_words[4] = _EasyCAT_BufferOut.Cust.ControlWord_5;
    control_words[5] = _EasyCAT_BufferOut.Cust.ControlWord_6;
    control_words[6] = _EasyCAT_BufferOut.Cust.ControlWord_7;
    control_words[7] = _EasyCAT_BufferOut.Cust.ControlWord_8;

    EasyCAT_BufferIn.Cust.ActualPosition_1 = actual_positions[0];
    EasyCAT_BufferIn.Cust.ActualPosition_2 = actual_positions[1];
    EasyCAT_BufferIn.Cust.ActualPosition_3 = actual_positions[2];
    EasyCAT_BufferIn.Cust.ActualPosition_4 = actual_positions[3];
    EasyCAT_BufferIn.Cust.ActualPosition_5 = actual_positions[4];
    EasyCAT_BufferIn.Cust.ActualPosition_6 = actual_positions[5];
    EasyCAT_BufferIn.Cust.ActualPosition_7 = actual_positions[6];
    EasyCAT_BufferIn.Cust.ActualPosition_8 = actual_positions[7];
    EasyCAT_BufferIn.Cust.ActualVelocity_1 = actual_velocities[0];
    EasyCAT_BufferIn.Cust.ActualVelocity_2 = actual_velocities[1];
    EasyCAT_BufferIn.Cust.ActualVelocity_3 = actual_velocities[2];
    EasyCAT_BufferIn.Cust.ActualVelocity_4 = actual_velocities[3];
    EasyCAT_BufferIn.Cust.ActualVelocity_5 = actual_velocities[4];
    EasyCAT_BufferIn.Cust.ActualVelocity_6 = actual_velocities[5];
    EasyCAT_BufferIn.Cust.ActualVelocity_7 = actual_velocities[6];
    EasyCAT_BufferIn.Cust.ActualVelocity_8 = actual_velocities[7];
    EasyCAT_BufferIn.Cust.ActualTorque_1 = actual_torques[0];
    EasyCAT_BufferIn.Cust.ActualTorque_2 = actual_torques[1];
    EasyCAT_BufferIn.Cust.ActualTorque_3 = actual_torques[2];
    EasyCAT_BufferIn.Cust.ActualTorque_4 = actual_torques[3];
    EasyCAT_BufferIn.Cust.ActualTorque_5 = actual_torques[4];
    EasyCAT_BufferIn.Cust.ActualTorque_6 = actual_torques[5];
    EasyCAT_BufferIn.Cust.ActualTorque_7 = actual_torques[6];
    EasyCAT_BufferIn.Cust.ActualTorque_8 = actual_torques[7];
    EasyCAT_BufferIn.Cust.StatusWord_1 = status_words[0];
    EasyCAT_BufferIn.Cust.StatusWord_2 = status_words[1];
    EasyCAT_BufferIn.Cust.StatusWord_3 = status_words[2];
    EasyCAT_BufferIn.Cust.StatusWord_4 = status_words[3];
    EasyCAT_BufferIn.Cust.StatusWord_5 = status_words[4];
    EasyCAT_BufferIn.Cust.StatusWord_6 = status_words[5];
    EasyCAT_BufferIn.Cust.StatusWord_7 = status_words[6];
    EasyCAT_BufferIn.Cust.StatusWord_8 = status_words[7];

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


    xTaskCreate(canfd_send_task, "canfd_send_task", 4096, NULL, 5, NULL);
    xTaskCreate(canfd_receive_task, "canfd_receive_task", 4096, NULL, 5, NULL);
    xTaskCreate(easyCAT_task, "easyCAT_task", 4096, NULL, 5, NULL);

}