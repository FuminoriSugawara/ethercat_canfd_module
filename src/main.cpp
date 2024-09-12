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
#include "esp_task_wdt.h"

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
//#define CONTROL_MODE RMTR_SERVO_MODE_CUR
#define CONTROL_MODE RMTR_SERVO_MODE_POS

unsigned long previousMs = 0;
const int OUTPUTS_COUNT = 8;
boolean DEBUG_MODE = true;
boolean DRY_RUN = false;
// EasyCAT buffer
PROCBUFFER_OUT _EasyCAT_BufferOut;  // output process data buffer
// canfd send count;
uint32_t canfd_send_count = 0;
// canfd receive count;
uint32_t canfd_receive_count = 0;


CAN_FRAME_FD message;
realman_motor_driver::RealmanMotorDriver motor_drivers[MOTOR_DRIVER_COUNT] = {
    realman_motor_driver::RealmanMotorDriver(0x01, std::shared_ptr<CAN_COMMON>(&CAN1), DEBUG_MODE, DRY_RUN),
    realman_motor_driver::RealmanMotorDriver(0x02, std::shared_ptr<CAN_COMMON>(&CAN1), DEBUG_MODE, DRY_RUN),
    realman_motor_driver::RealmanMotorDriver(0x03, std::shared_ptr<CAN_COMMON>(&CAN1), DEBUG_MODE, DRY_RUN),
    realman_motor_driver::RealmanMotorDriver(0x04, std::shared_ptr<CAN_COMMON>(&CAN1), DEBUG_MODE, DRY_RUN),
    realman_motor_driver::RealmanMotorDriver(0x05, std::shared_ptr<CAN_COMMON>(&CAN1), DEBUG_MODE, DRY_RUN),
    realman_motor_driver::RealmanMotorDriver(0x06, std::shared_ptr<CAN_COMMON>(&CAN1), DEBUG_MODE, DRY_RUN),
    realman_motor_driver::RealmanMotorDriver(0x07, std::shared_ptr<CAN_COMMON>(&CAN1), DEBUG_MODE, DRY_RUN),
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
        vTaskDelay(100 / portTICK_PERIOD_MS);
        bool motor_driver_connection_states[MOTOR_DRIVER_COUNT]; 
        bool all_drivers_connected = false;
        uint8_t retry_count = 5;
        while (!all_drivers_connected)
        {
            all_drivers_connected = true;
            for (int i = 0; i < MOTOR_DRIVER_COUNT; i++)
            {
                // verify all motor drivers are connected
                bool connection_state = motor_drivers[i].getConnectionState();
                motor_driver_connection_states[i] = connection_state;
                if (!connection_state)
                {
                    motor_drivers[i].setConnectionOnline();
                    all_drivers_connected = false;
                }
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            if (all_drivers_connected)
            {
                Serial.println("All motor drivers connected");
                break;
            }
            retry_count--;
            if (retry_count == 0)
            {
                Serial.println("Motor driver connection failed");
                for (int i = 0; i < MOTOR_DRIVER_COUNT; i++)
                {
                    Serial.printf("Motor driver connection state: %d\n", motor_driver_connection_states[i]);
                }
                while (1)
                {
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
                break;
            }   
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            Serial.printf("Retry count %d\n", retry_count);
        }

        vTaskDelay(200 / portTICK_PERIOD_MS);
        // check driver error
        for (int i = 0; i < MOTOR_DRIVER_COUNT; i++)
        {
            motor_drivers[i].loadCurrentState();
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
        bool all_drivers_error_free = true;
        for (int i = 0; i < MOTOR_DRIVER_COUNT; i++)
        {
            uint16_t error_state = motor_drivers[i].getErrorState();
            Serial.printf("Error state %d: 0x%04X\n", i, error_state);
            if (error_state != 0)
            {
                all_drivers_error_free = false;
            }
        }
        if (!all_drivers_error_free)
        {
            Serial.println("Motor driver error detected");
            Serial.println("Resetting motor drivers");
            for (int i = 0; i < MOTOR_DRIVER_COUNT; i++)
            {
                if (motor_drivers[i].getErrorState() != 0)
                {
                    motor_drivers[i].setDriverDisabled();
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    motor_drivers[i].setZeroPosition();
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    motor_drivers[i].clearJointError();
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    motor_drivers[i].setDriverEnabled();
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                }
            }
            // check driver error
            for (int i = 0; i < MOTOR_DRIVER_COUNT; i++)
            {
                motor_drivers[i].loadCurrentState();
            }
            vTaskDelay(200 / portTICK_PERIOD_MS);
            for (int i = 0; i < MOTOR_DRIVER_COUNT; i++)
            {
                uint16_t error_state = motor_drivers[i].getErrorState();
                Serial.printf("Error state %d: 0x%04X\n", i, error_state);
            }
            while (1)
            {
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
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
            // motor_drivers[i].setDriverDisabled();
            // vTaskDelay(100 / portTICK_PERIOD_MS);
            // motor_drivers[i].setZeroPosition();
            // vTaskDelay(100 / portTICK_PERIOD_MS);
            // motor_drivers[i].clearJointError();
            // vTaskDelay(100 / portTICK_PERIOD_MS);

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
            vTaskDelay(100 / portTICK_PERIOD_MS);
            //motor_drivers[i].setDriverEnabled();
            //vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    return true;
}

void canfd_receive_task(void *pvParameters)
{
    TickType_t last_wdt_reset = xTaskGetTickCount();
    const TickType_t wdt_reset_period = pdMS_TO_TICKS(1000); 
    while (1)
    {
        CAN_FRAME_FD rxFrame;
        if (CAN1.readFD(rxFrame))
        {
            uint8_t module_id = (rxFrame.id & 0x00FF);
            //Serial.printf("Received Module ID: 0x%02X\n", module_id);
            motor_drivers[module_id - 1].processCANFDMessage(rxFrame);
            canfd_receive_count++;
            //Serial.printf("ID: 0x%04X\n", rxFrame.id);
            //for (int i = 0; i < rxFrame.length; i++)
            //{
            //    Serial.printf("0x%02X ", rxFrame.data.uint8[i]);
            //}
            //Serial.printf("\n");
        }
        TickType_t now = xTaskGetTickCount();
        if ((now - last_wdt_reset) >= wdt_reset_period)
        {
            vTaskDelay(1 / portTICK_PERIOD_MS);
            last_wdt_reset = now;

            // オプション: リセットしたことをログに記録
            ESP_LOGI("WDT", "WDT reset at %llu ms", (uint64_t)now * portTICK_PERIOD_MS);
        }
    }
}

void send_control_tic_message() {
    CAN_FRAME_FD control_tic_message;
    control_tic_message.id = 0x7F0;
    control_tic_message.fdMode = 1;
    control_tic_message.rrs = 0;
    control_tic_message.length = 0;
    CAN1.sendFrameFD(control_tic_message);
}

void canfd_task(void *pvParameters) {
    motorDriverSetup();
    //while (1) {
    //    vTaskDelay(10 / portTICK_PERIOD_MS);
    //}
    while (1) {
        for (int i = 0; i < MOTOR_DRIVER_COUNT; i++)
        {
            motor_drivers[i].loadCurrentPosition();
            //canfd_send_count++;
            motor_drivers[i].loadCurrentCurrent();
            //canfd_send_count++;
            motor_drivers[i].loadCurrentVelocity();
            //canfd_send_count++;
            
            if (CONTROL_MODE == RMTR_SERVO_MODE_POS)
            {
                //Serial.printf("Target position %d: %d\n", i, target_positions[i]);
                motor_drivers[i].setTargetPosition(target_positions[i]);
            }
            else if (CONTROL_MODE == RMTR_SERVO_MODE_VEL)
            {
                //Serial.printf("Target velocity %d: %d\n", i, target_velocities[i]);
                motor_drivers[i].setTargetVelocity(target_velocities[i]);
            }
            else if (CONTROL_MODE == RMTR_SERVO_MODE_CUR)
            {
                motor_drivers[i].setTargetCurrent(target_torques[i]);
                //Serial.printf("Target torque %d: %d\n", i, target_torques[i]);
            }
            //canfd_send_count++;
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        send_control_tic_message();
    }
}

void sendEtherCATDataAsCANFD(const PROCBUFFER_OUT& ethercat_data, uint32_t canId) {
    CAN_FRAME_FD response;
    response.id = canId;
    response.fdMode = 1;   // Set FD mode
    response.extended = 0; // Using standard ID
    response.rrs = 0;

    // Determine the maximum amount of data we can send
    const size_t max_data_size = sizeof(response.data.uint8);
    size_t data_size = 0;

    // Copy position data
    size_t pos_size = std::min(max_data_size - data_size, 8 * sizeof(int32_t));
    memcpy(response.data.uint8 + data_size, &ethercat_data.Cust.Position_1, pos_size);
    data_size += pos_size;

    // Copy velocity data if there's space
    if (data_size < max_data_size) {
        size_t vel_size = std::min(max_data_size - data_size, 8 * sizeof(int32_t));
        memcpy(response.data.uint8 + data_size, &ethercat_data.Cust.Velocity_1, vel_size);
        data_size += vel_size;
    }

    // Copy torque data if there's space
    if (data_size < max_data_size) {
        size_t torque_size = std::min(max_data_size - data_size, 8 * sizeof(int32_t));
        memcpy(response.data.uint8 + data_size, &ethercat_data.Cust.Torque_1, torque_size);
        data_size += torque_size;
    }

    // Copy control words if there's space (assuming little-endian system)
    if (data_size < max_data_size) {
        size_t control_size = std::min(max_data_size - data_size, 8 * sizeof(uint16_t));
        memcpy(response.data.uint8 + data_size, &ethercat_data.Cust.ControlWord_1, control_size);
        data_size += control_size;
    }

    response.length = data_size;  // Set the actual data length

    // Send the frame
    CAN1.sendFrameFD(response);
}

void canfd_health_check_task(void *pvParameters) {
    // 送信カウント数と受信カウント数を1秒ごとに比較
    while (1) {
        // カウント数を比較
        if (canfd_send_count != canfd_receive_count) {
            // エラー処理
            ESP_LOGE("CANFD", "Send count: %u, Receive count: %u", canfd_send_count, canfd_receive_count);
        } else {
            // 正常
            ESP_LOGI("CANFD", "Send count: %u, Receive count: %u", canfd_send_count, canfd_receive_count);
        }

        // カウント数をリセット
        canfd_send_count = 0;
        canfd_receive_count = 0;

        // 1秒待機
        vTaskDelay(1000 / portTICK_PERIOD_MS);
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

    for (int i = 0; i < MOTOR_DRIVER_COUNT; i++)
    {

        actual_positions[i] = (int32_t)(motor_drivers[i].getCurrentPosition());
        actual_torques[i] = (int32_t)(motor_drivers[i].getCurrentTorque());
        actual_velocities[i] = (int32_t)(motor_drivers[i].getCurrentVelocity());
    }

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

    sendEtherCATDataAsCANFD(EasyCAT_BufferOut, 0x600);
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

    //esp_task_wdt_init(5, true);

    Serial.begin(115200);
    //CAN1.setDebuggingMode(true);


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


    xTaskCreatePinnedToCore(canfd_task, "canfd_send_task", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(canfd_receive_task, "canfd_receive_task", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(easyCAT_task, "easyCAT_task", 4096, NULL, 5, NULL, 0);
    //xTaskCreatePinnedToCore(canfd_health_check_task, "canfd_health_check_task", 4096, NULL, 5, NULL, 0);

}