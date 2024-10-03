//#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/timer.h"
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
#define EASYCAT_INT_PIN 26

#define TIME_LOG_BUFFER_SIZE 1000

typedef struct {
    int64_t easyCATApplication;
    int64_t loadTargetValuesFromEtherCAT;
    int64_t motorControl;
    int64_t loadCurrentValuesFromMotorDrivers;
    int64_t setCurrentValuesToEasyCATBuffer;
} TimeLog;

typedef struct {
    TimeLog average;
    TimeLog min;
    TimeLog max;
} TimeLogStats;

TimeLog timeLog[TIME_LOG_BUFFER_SIZE];
int timeLogIndex = 0;


unsigned long previousMs = 0;
const int OUTPUTS_COUNT = 8;
boolean DEBUG_MODE = false;
boolean DRY_RUN = false;
// EasyCAT buffer
PROCBUFFER_OUT _EasyCAT_BufferOut;  // output process data buffer
// canfd send count;
volatile uint32_t canfd_send_count = 0;
// canfd receive count;
volatile uint32_t canfd_receive_count = 0;

volatile uint32_t easycat_count = 0;

// Timer configuration
#define TIMER_DIVIDER         2  // Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL_SEC   (0.002) // 1ms interval

TaskHandle_t canfdTaskHandle = NULL;
TaskHandle_t easyCATTaskHandle = NULL;

int32_t easycat_index = 0;


void IRAM_ATTR EasyCAT_IntHandler()
{
    // 1msごとにINTが発生する
    // TIMER_INTERVAL_SEC の間隔で呼び出す。
    if (easycat_index >= 1)
    {

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(easyCATTaskHandle, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken)
        {
            portYIELD_FROM_ISR();
        }
        easycat_index = 0;
    }
    else
    {
        easycat_index++;
    }
}




CAN_FRAME_FD message;
realman_motor_driver::RealmanMotorDriver motor_drivers[MOTOR_DRIVER_COUNT] = {
    realman_motor_driver::RealmanMotorDriver(0x01, std::shared_ptr<MCP2517FD>(&CAN1), DEBUG_MODE, DRY_RUN),
    realman_motor_driver::RealmanMotorDriver(0x02, std::shared_ptr<MCP2517FD>(&CAN1), DEBUG_MODE, DRY_RUN),
    realman_motor_driver::RealmanMotorDriver(0x03, std::shared_ptr<MCP2517FD>(&CAN1), DEBUG_MODE, DRY_RUN),
    realman_motor_driver::RealmanMotorDriver(0x04, std::shared_ptr<MCP2517FD>(&CAN1), DEBUG_MODE, DRY_RUN),
    realman_motor_driver::RealmanMotorDriver(0x05, std::shared_ptr<MCP2517FD>(&CAN1), DEBUG_MODE, DRY_RUN),
    realman_motor_driver::RealmanMotorDriver(0x06, std::shared_ptr<MCP2517FD>(&CAN1), DEBUG_MODE, DRY_RUN),
    realman_motor_driver::RealmanMotorDriver(0x07, std::shared_ptr<MCP2517FD>(&CAN1), DEBUG_MODE, DRY_RUN),
};

//ACAN2517FD can(PIN_NUM_CS, SPI, 255);

int32_t actual_positions[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int32_t actual_velocities[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int32_t actual_torques[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint16_t status_words[8] = {0, 0, 0, 0, 0, 0, 0, 0};

int32_t target_positions[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int32_t target_velocities[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int32_t target_torques[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint16_t control_words[8] = {0, 0, 0, 0, 0, 0, 0, 0};

class MyCANListener : public CANListener {
public:

    void gotFrameFD(CAN_FRAME_FD *frame, int mailbox) override {
        CAN_FRAME_FD rxFrame;
        CAN1.readFD(rxFrame);
        uint8_t module_id = (rxFrame.id & 0x00FF);
        motor_drivers[module_id - 1].processCANFDMessage(rxFrame);
        canfd_receive_count++;
    }   
};

MyCANListener myCanListener;

int64_t measureTime( void (*func)(void) )
{
    int64_t start = esp_timer_get_time();
    func();
    int64_t end = esp_timer_get_time();
    return end - start;
}

TimeLogStats calcStats() {
    TimeLogStats stats;

    int64_t easyCATApplicationMin = INT64_MAX;
    int64_t loadTargetValuesFromEtherCATMin = INT64_MAX;
    int64_t motorControlMin = INT64_MAX;
    int64_t loadCurrentValuesFromMotorDriversMin = INT64_MAX;
    int64_t setCurrentValuesToEasyCATBufferMin = INT64_MAX;

    int64_t easyCATApplicationMax = 0;
    int64_t loadTargetValuesFromEtherCATMax = 0;
    int64_t motorControlMax = 0;
    int64_t loadCurrentValuesFromMotorDriversMax = 0;
    int64_t setCurrentValuesToEasyCATBufferMax = 0;


    int64_t easyCATApplicationSum = 0;
    int64_t loadTargetValuesFromEtherCATSum = 0;
    int64_t motorControlSum = 0;
    int64_t loadCurrentValuesFromMotorDriversSum = 0;
    int64_t setCurrentValuesToEasyCATBufferSum = 0;
    if (timeLogIndex == 0)
    {
        return stats;
    }
    int32_t size = ((timeLogIndex + 1)< TIME_LOG_BUFFER_SIZE) ? (timeLogIndex + 1) : TIME_LOG_BUFFER_SIZE;
    for (int i = 0; i < size; i++)
    {
        easyCATApplicationSum += timeLog[i].easyCATApplication;
        loadTargetValuesFromEtherCATSum += timeLog[i].loadTargetValuesFromEtherCAT;
        motorControlSum += timeLog[i].motorControl;
        loadCurrentValuesFromMotorDriversSum += timeLog[i].loadCurrentValuesFromMotorDrivers;
        setCurrentValuesToEasyCATBufferSum += timeLog[i].setCurrentValuesToEasyCATBuffer;

        if (timeLog[i].easyCATApplication < easyCATApplicationMin)
        {
            easyCATApplicationMin = timeLog[i].easyCATApplication;
        }
        if (timeLog[i].loadTargetValuesFromEtherCAT < loadTargetValuesFromEtherCATMin)
        {
            loadTargetValuesFromEtherCATMin = timeLog[i].loadTargetValuesFromEtherCAT;
        }

        if (timeLog[i].motorControl < motorControlMin)
        {
            motorControlMin = timeLog[i].motorControl;
        }

        if (timeLog[i].loadCurrentValuesFromMotorDrivers < loadCurrentValuesFromMotorDriversMin)
        {
            loadCurrentValuesFromMotorDriversMin = timeLog[i].loadCurrentValuesFromMotorDrivers;
        }

        if (timeLog[i].setCurrentValuesToEasyCATBuffer < setCurrentValuesToEasyCATBufferMin)
        {
            setCurrentValuesToEasyCATBufferMin = timeLog[i].setCurrentValuesToEasyCATBuffer;
        }

        if (timeLog[i].easyCATApplication > easyCATApplicationMax)
        {
            easyCATApplicationMax = timeLog[i].easyCATApplication;
        }

        if (timeLog[i].loadTargetValuesFromEtherCAT > loadTargetValuesFromEtherCATMax)
        {
            loadTargetValuesFromEtherCATMax = timeLog[i].loadTargetValuesFromEtherCAT;
        }

        if (timeLog[i].motorControl > motorControlMax)
        {
            motorControlMax = timeLog[i].motorControl;
        }

        if (timeLog[i].loadCurrentValuesFromMotorDrivers > loadCurrentValuesFromMotorDriversMax)
        {
            loadCurrentValuesFromMotorDriversMax = timeLog[i].loadCurrentValuesFromMotorDrivers;
        }

        if (timeLog[i].setCurrentValuesToEasyCATBuffer > setCurrentValuesToEasyCATBufferMax)
        {
            setCurrentValuesToEasyCATBufferMax = timeLog[i].setCurrentValuesToEasyCATBuffer;
        }

    }
    int64_t easyCATApplicationAvg = easyCATApplicationSum / (size);
    int64_t loadTargetValuesFromEtherCATAvg = loadTargetValuesFromEtherCATSum / size;
    int64_t motorControlAvg = motorControlSum / size;
    int64_t loadCurrentValuesFromMotorDriversAvg = loadCurrentValuesFromMotorDriversSum / size;
    int64_t setCurrentValuesToEasyCATBufferAvg = setCurrentValuesToEasyCATBufferSum / size;

    stats.average.easyCATApplication = easyCATApplicationAvg;
    stats.average.loadTargetValuesFromEtherCAT = loadTargetValuesFromEtherCATAvg;
    stats.average.motorControl = motorControlAvg;
    stats.average.loadCurrentValuesFromMotorDrivers = loadCurrentValuesFromMotorDriversAvg;
    stats.average.setCurrentValuesToEasyCATBuffer = setCurrentValuesToEasyCATBufferAvg;

    stats.min.easyCATApplication = easyCATApplicationMin;
    stats.min.loadTargetValuesFromEtherCAT = loadTargetValuesFromEtherCATMin;
    stats.min.motorControl = motorControlMin;
    stats.min.loadCurrentValuesFromMotorDrivers = loadCurrentValuesFromMotorDriversMin;
    stats.min.setCurrentValuesToEasyCATBuffer = setCurrentValuesToEasyCATBufferMin;

    stats.max.easyCATApplication = easyCATApplicationMax;
    stats.max.loadTargetValuesFromEtherCAT = loadTargetValuesFromEtherCATMax;
    stats.max.motorControl = motorControlMax;
    stats.max.loadCurrentValuesFromMotorDrivers = loadCurrentValuesFromMotorDriversMax;
    stats.max.setCurrentValuesToEasyCATBuffer = setCurrentValuesToEasyCATBufferMax;

    return stats;
}


bool easyCATSetup()
{

    bool success = EasyCAT_Init(PIN_NUM_CS_HSPI, DC_SYNC);
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
            uint8_t connected_count = 0;
            for (int i = 0; i < MOTOR_DRIVER_COUNT; i++)
            {
                // verify all motor drivers are connected
                bool connection_state = motor_drivers[i].getConnectionState();
                motor_driver_connection_states[i] = connection_state;
                if (connection_state) {
                    Serial.printf("Motor driver %d connected\n", i+1);
                    connected_count++;
                    continue;
                } else 
                {
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    motor_drivers[i].setConnectionOnline();
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
            }
            if (connected_count == MOTOR_DRIVER_COUNT)
            {
                Serial.println("All motor drivers connected");
                all_drivers_connected = true;
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
            //vTaskDelay(1000 / portTICK_PERIOD_MS);
            //Serial.printf("Retry count %d\n", retry_count);
        }

        vTaskDelay(200 / portTICK_PERIOD_MS);
        // check driver error
        for (int i = 0; i < MOTOR_DRIVER_COUNT; i++)
        {
            motor_drivers[i].loadCurrentState();
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
        ESP_LOGI("MotorDriver", "Checking driver error");
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

        ESP_LOGI("MotorDriver", "Setting control mode");

        for (int i = 0; i < MOTOR_DRIVER_COUNT; i++)
        {
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

        ESP_LOGI("MotorDriver", "Motor driver setup completed");
    return true;
}

//void canfd_receive_task(void *pvParameters)
//{
//    TickType_t last_wdt_reset = xTaskGetTickCount();
//    //const TickType_t wdt_reset_period = pdMS_TO_TICKS(1000); 
//    while (1)
//    {
//        CAN_FRAME_FD rxFrame;
//        if (CAN1.readFD(rxFrame))
//        {
//            uint8_t module_id = (rxFrame.id & 0x00FF);
//            //Serial.printf("Received Module ID: 0x%02X\n", module_id);
//            motor_drivers[module_id - 1].processCANFDMessage(rxFrame);
//            canfd_receive_count++;
//            //Serial.printf("ID: 0x%04X\n", rxFrame.id);
//            //for (int i = 0; i < rxFrame.length; i++)
//            //{
//            //    Serial.printf("0x%02X ", rxFrame.data.uint8[i]);
//            //}
//            //Serial.printf("\n");
//        }
//        vTaskDelayUntil(&last_wdt_reset, 1 / portTICK_PERIOD_MS);
//        //vTaskDelay(1 / portTICK_PERIOD_MS);
//        //TickType_t now = xTaskGetTickCount();
//        //if ((now - last_wdt_reset) >= wdt_reset_period)
//        //{
//        //    vTaskDelay(1 / portTICK_PERIOD_MS);
//        //    last_wdt_reset = now;
//        //    uint16_t rx_queue_available = CAN1.available();
//        //    ESP_LOGI("Queue", "RX Queue available: %u", rx_queue_available);
//
//        //    // オプション: リセットしたことをログに記録
//        //    //ESP_LOGI("WDT", "WDT reset at %llu ms", (uint64_t)now * portTICK_PERIOD_MS);
//        //}
//    }
//}

void sendControlTicMessage() {
    CAN_FRAME_FD control_tic_message;
    control_tic_message.id = 0x7F0;
    control_tic_message.fdMode = 1;
    control_tic_message.rrs = 0;
    control_tic_message.length = 0;
    CAN1.WriteFrame(control_tic_message);
}

void motorControl()
{
    //std::vector<int32_t> target_values(MOTOR_DRIVER_COUNT);

    if (CONTROL_MODE == RMTR_SERVO_MODE_POS)
    {
        std::vector<std::pair<uint16_t, CAN_FRAME_FD>> messages;
        messages.reserve(MOTOR_DRIVER_COUNT);

        for (size_t i = 0; i < MOTOR_DRIVER_COUNT; ++i) {
            CAN_FRAME_FD frame;
            frame.id = (i + 1) | MESSAGE_TYPE_CMD_POS;
            frame.fdMode = 1;
            frame.rrs = 0;
            frame.length = 4;

            uint32_t target_position_int = static_cast<uint32_t>(target_positions[i]);
            frame.data.uint8[0] = (target_position_int & 0x000000FF);
            frame.data.uint8[1] = (target_position_int & 0x0000FF00) >> 8;
            frame.data.uint8[2] = (target_position_int & 0x00FF0000) >> 16;
            frame.data.uint8[3] = (target_position_int & 0xFF000000) >> 24;

            messages.emplace_back(frame.id, frame);
        }
        CAN1.WriteMultipleFrameBuffers(messages);
        //std::copy(target_positions, target_positions + MOTOR_DRIVER_COUNT, target_values.begin());
        //realman_motor_driver::RealmanMotorDriver::sendMultipleTargetPositions(target_values);
        canfd_send_count += MOTOR_DRIVER_COUNT;
    }
    //else if (CONTROL_MODE == RMTR_SERVO_MODE_VEL)
    //{
    //    std::copy(target_velocities, target_velocities + MOTOR_DRIVER_COUNT, target_values.begin());
    //    realman_motor_driver::RealmanMotorDriver::sendMultipleTargetVelocities(target_values);
    //    canfd_send_count += MOTOR_DRIVER_COUNT;
    //}
    //else if (CONTROL_MODE == RMTR_SERVO_MODE_CUR)
    //{
    //    std::copy(target_torques, target_torques + MOTOR_DRIVER_COUNT, target_values.begin());
    //    realman_motor_driver::RealmanMotorDriver::sendMultipleTargetCurrents(target_values);
    //    canfd_send_count += MOTOR_DRIVER_COUNT;
    //}

    sendControlTicMessage();
    
}

//void canfd_task(void *pvParameters) {
//    //while (1) {
//    //    vTaskDelay(10 / portTICK_PERIOD_MS);
//    //}
//    while (1) {
//        // Wait for notification from EasyCAT task
//        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//        //ESP_LOGI("CANFD", "Received EasyCAT notification");
//        motorControl();
//        //vTaskDelay(1 / portTICK_PERIOD_MS);
//    }
//}

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
    CAN1.WriteFrame(response);
}

void canfdHealthCheckTask(void *pvParameters) {
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
        ESP_LOGI("CANFD", "task_MCPIntFD count: %u", CAN1.task_MCPIntFD_count);
        ESP_LOGI("CANFD", "INT Handler count: %u", CAN1.int_handler_count);
        ESP_LOGI("CANFD", "Handle dispatch count: %u", CAN1.handle_dispatch_count);
        ESP_LOGI("CANFD", "RX Queue count: %u", CAN1.rx_queue_count);
        ESP_LOGI("CANFD", "RX Queue available: %u", CAN1.available());
        //ESP_LOGI("CANFD", "Send count: %u", CAN1.send_count);
        ESP_LOGI("CANFD", "EasyCAT count: %u", easycat_count);
        TimeLogStats stats = calcStats();

        printf("EasyCAT application time: min %lld us, max %lld us, avg %lld us\n", stats.min.easyCATApplication, stats.max.easyCATApplication, stats.average.easyCATApplication);
        printf("Load target values from EtherCAT time: min %lld us, max %lld us, avg %lld us\n", stats.min.loadTargetValuesFromEtherCAT, stats.max.loadTargetValuesFromEtherCAT, stats.average.loadTargetValuesFromEtherCAT);
        printf("Motor control time: min %lld us, max %lld us, avg %lld us\n", stats.min.motorControl, stats.max.motorControl, stats.average.motorControl);
        printf("Load current values from motor drivers time: min %lld us, max %lld us, avg %lld us\n", stats.min.loadCurrentValuesFromMotorDrivers, stats.max.loadCurrentValuesFromMotorDrivers, stats.average.loadCurrentValuesFromMotorDrivers);
        printf("Set current values to EasyCAT buffer time: min %lld us, max %lld us, avg %lld us\n", stats.min.setCurrentValuesToEasyCATBuffer, stats.max.setCurrentValuesToEasyCATBuffer, stats.average.setCurrentValuesToEasyCATBuffer);

        // カウント数をリセット
        canfd_send_count = 0;
        canfd_receive_count = 0;
        CAN1.task_MCPIntFD_count = 0;
        CAN1.int_handler_count = 0;
        CAN1.handle_dispatch_count = 0;
        CAN1.rx_queue_count = 0;
        //CAN1.send_count = 0;
        easycat_count = 0;
        timeLogIndex = 0;

        // 1秒待機
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
}



void loadTargetValuesFromEtherCAT() {
    easycat_count++;
    // update target position
    target_positions[0] = EasyCAT_BufferOut.Cust.Position_1;
    target_positions[1] = EasyCAT_BufferOut.Cust.Position_2;
    target_positions[2] = EasyCAT_BufferOut.Cust.Position_3;
    target_positions[3] = EasyCAT_BufferOut.Cust.Position_4;
    target_positions[4] = EasyCAT_BufferOut.Cust.Position_5;
    target_positions[5] = EasyCAT_BufferOut.Cust.Position_6;
    target_positions[6] = EasyCAT_BufferOut.Cust.Position_7;
    target_positions[7] = EasyCAT_BufferOut.Cust.Position_8;
    // update target velocity
    target_velocities[0] = EasyCAT_BufferOut.Cust.Velocity_1;
    target_velocities[1] = EasyCAT_BufferOut.Cust.Velocity_2;
    target_velocities[2] = EasyCAT_BufferOut.Cust.Velocity_3;
    target_velocities[3] = EasyCAT_BufferOut.Cust.Velocity_4;
    target_velocities[4] = EasyCAT_BufferOut.Cust.Velocity_5;
    target_velocities[5] = EasyCAT_BufferOut.Cust.Velocity_6;
    target_velocities[6] = EasyCAT_BufferOut.Cust.Velocity_7;
    target_velocities[7] = EasyCAT_BufferOut.Cust.Velocity_8;
    // update target torque
    target_torques[0] = EasyCAT_BufferOut.Cust.Torque_1;
    target_torques[1] = EasyCAT_BufferOut.Cust.Torque_2;
    target_torques[2] = EasyCAT_BufferOut.Cust.Torque_3;
    target_torques[3] = EasyCAT_BufferOut.Cust.Torque_4;
    target_torques[4] = EasyCAT_BufferOut.Cust.Torque_5;
    target_torques[5] = EasyCAT_BufferOut.Cust.Torque_6;
    target_torques[6] = EasyCAT_BufferOut.Cust.Torque_7;
    target_torques[7] = EasyCAT_BufferOut.Cust.Torque_8;
    // update control word
    control_words[0] = EasyCAT_BufferOut.Cust.ControlWord_1;
    control_words[1] = EasyCAT_BufferOut.Cust.ControlWord_2;
    control_words[2] = EasyCAT_BufferOut.Cust.ControlWord_3;
    control_words[3] = EasyCAT_BufferOut.Cust.ControlWord_4;
    control_words[4] = EasyCAT_BufferOut.Cust.ControlWord_5;
    control_words[5] = EasyCAT_BufferOut.Cust.ControlWord_6;
    control_words[6] = EasyCAT_BufferOut.Cust.ControlWord_7;
    control_words[7] = EasyCAT_BufferOut.Cust.ControlWord_8;
}

// load current values from motor drivers
void loadCurrentValuesFromMotorDrivers() {
    for (int i = 0; i < MOTOR_DRIVER_COUNT; i++)
    {
        actual_positions[i] = (int32_t)(motor_drivers[i].getCurrentPosition());
        actual_torques[i] = (int32_t)(motor_drivers[i].getCurrentTorque());
        actual_velocities[i] = (int32_t)(motor_drivers[i].getCurrentVelocity());
    }
}

// set current values to EasyCAT buffer
void setCurrentValuesToEasyCATBuffer() {
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



void easyCATApplication ()                                        
{
    EasyCAT_MainTask();




    //sendEtherCATDataAsCANFD(EasyCAT_BufferOut, 0x600);
}


void easyCAT_task(void *pvParameters) {
    
    while (1) {
        // Wait for notification from ISR
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        //ESP_LOGI("EasyCAT", "EasyCAT task started");
        timeLog[timeLogIndex].easyCATApplication = measureTime(easyCATApplication);
        //easyCATApplication();
        //EasyCAT_MainTask();
        // printf("EasyCAT status: %d\n", status);
        timeLog[timeLogIndex].loadTargetValuesFromEtherCAT = measureTime(loadTargetValuesFromEtherCAT);
        //loadTargetValuesFromEtherCAT();
        //vTaskDelay(1 / portTICK_PERIOD_MS);
        //xTaskNotifyGive(canfdTaskHandle);
        timeLog[timeLogIndex].motorControl = measureTime(motorControl);
        //motorControl();

        timeLog[timeLogIndex].loadCurrentValuesFromMotorDrivers = measureTime(loadCurrentValuesFromMotorDrivers);
        //loadCurrentValuesFromMotorDrivers();
        timeLog[timeLogIndex].setCurrentValuesToEasyCATBuffer = measureTime(setCurrentValuesToEasyCATBuffer);
        //setCurrentValuesToEasyCATBuffer();
        //vTaskDelay(1 / portTICK_PERIOD_MS);
        // if (timeLogIndex >= TIME_LOG_BUFFER_SIZE - 1) {
            // Calculate average time
            // int64_t easyCATApplicationSum = 0;
            // int64_t loadTargetValuesFromEtherCATSum = 0;
            // int64_t motorControlSum = 0;
            // int64_t loadCurrentValuesFromMotorDriversSum = 0;
            // int64_t setCurrentValuesToEasyCATBufferSum = 0;
            // for (int i = 0; i < TIME_LOG_BUFFER_SIZE; i++) {
            //     easyCATApplicationSum += timeLog[i].easyCATApplication;
            //     loadTargetValuesFromEtherCATSum += timeLog[i].loadTargetValuesFromEtherCAT;
            //     motorControlSum += timeLog[i].motorControl;
            //     loadCurrentValuesFromMotorDriversSum += timeLog[i].loadCurrentValuesFromMotorDrivers;
            //     setCurrentValuesToEasyCATBufferSum += timeLog[i].setCurrentValuesToEasyCATBuffer;
            // }
            // int64_t easyCATApplicationAvg = easyCATApplicationSum / TIME_LOG_BUFFER_SIZE;
            // int64_t loadTargetValuesFromEtherCATAvg = loadTargetValuesFromEtherCATSum / TIME_LOG_BUFFER_SIZE;
            // int64_t motorControlAvg = motorControlSum / TIME_LOG_BUFFER_SIZE;
            // int64_t loadCurrentValuesFromMotorDriversAvg = loadCurrentValuesFromMotorDriversSum / TIME_LOG_BUFFER_SIZE;
            // int64_t setCurrentValuesToEasyCATBufferAvg = setCurrentValuesToEasyCATBufferSum / TIME_LOG_BUFFER_SIZE;
            // printf("EasyCAT application time: %lld us\n", easyCATApplicationAvg);
            // printf("Load target values from EtherCAT time: %lld us\n", loadTargetValuesFromEtherCATAvg);
            // printf("Motor control time: %lld us\n", motorControlAvg);
            // printf("Load current values from motor drivers time: %lld us\n", loadCurrentValuesFromMotorDriversAvg);
            // printf("Set current values to EasyCAT buffer time: %lld us\n", setCurrentValuesToEasyCATBufferAvg);
            // printf("Total time: %lld us\n", easyCATApplicationAvg + loadTargetValuesFromEtherCATAvg + motorControlAvg + loadCurrentValuesFromMotorDriversAvg + setCurrentValuesToEasyCATBufferAvg);
            // TimeLogStats stats = calcStats();
            // timeLogIndex = 0;

            // printf("EasyCAT application time: min %lld us, max %lld us, avg %lld us\n", stats.min.easyCATApplication, stats.max.easyCATApplication, stats.average.easyCATApplication);
            // printf("Load target values from EtherCAT time: min %lld us, max %lld us, avg %lld us\n", stats.min.loadTargetValuesFromEtherCAT, stats.max.loadTargetValuesFromEtherCAT, stats.average.loadTargetValuesFromEtherCAT);
            // printf("Motor control time: min %lld us, max %lld us, avg %lld us\n", stats.min.motorControl, stats.max.motorControl, stats.average.motorControl);
            // printf("Load current values from motor drivers time: min %lld us, max %lld us, avg %lld us\n", stats.min.loadCurrentValuesFromMotorDrivers, stats.max.loadCurrentValuesFromMotorDrivers, stats.average.loadCurrentValuesFromMotorDrivers);
            // printf("Set current values to EasyCAT buffer time: min %lld us, max %lld us, avg %lld us\n", stats.min.setCurrentValuesToEasyCATBuffer, stats.max.setCurrentValuesToEasyCATBuffer, stats.average.setCurrentValuesToEasyCATBuffer);

        // } else {
        timeLogIndex++;
        // }
    }
}


// Timer ISR callback
static bool IRAM_ATTR timer_group_isr_callback(void *args) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Notify Task A to read data from Device A
    vTaskNotifyGiveFromISR(easyCATTaskHandle, &xHigherPriorityTaskWoken);

    // Reset the timer counter value
    TIMERG0.hw_timer[TIMER_0].update = 1;
    TIMERG0.int_clr_timers.t0 = 1;

    return xHigherPriorityTaskWoken == pdTRUE;
}

void init_timer() {
    // タイマーの設定構造体を初期化
    timer_config_t config = {
        .alarm_en = TIMER_ALARM_EN,       // アラーム（割り込み）を有効化
        .counter_en = TIMER_PAUSE,        // 初期状態は停止
        .intr_type = TIMER_INTR_LEVEL,    // レベルトリガー割り込み
        .counter_dir = TIMER_COUNT_UP,    // カウンターの方向（上昇）
        .auto_reload = TIMER_AUTORELOAD_EN, // 自動リロードを有効化
        .divider = TIMER_DIVIDER          // タイマーのクロック分周比
    };

    // タイマーを初期化
    ESP_ERROR_CHECK(timer_init(TIMER_GROUP_0, TIMER_0, &config));

    // カウンター初期値を0に設定
    ESP_ERROR_CHECK(timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0));

    // アラーム値を設定（これが割り込みのタイミングを決定）
    ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_SEC * TIMER_SCALE));

    // 割り込みを有効化
    ESP_ERROR_CHECK(timer_enable_intr(TIMER_GROUP_0, TIMER_0));

    // 割り込みハンドラを設定
    ESP_ERROR_CHECK(timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timer_group_isr_callback, NULL, 0));

    // タイマーを開始
    ESP_ERROR_CHECK(timer_start(TIMER_GROUP_0, TIMER_0));
}

extern "C" void app_main(void)
{

    //esp_task_wdt_init(5, true);

    Serial.begin(115200);
    //CAN1.setDebuggingMode(true);

    bool easyCATSetupSuccess = easyCATSetup(); // EasyCAT初期化
    if (!easyCATSetupSuccess)
    {
        printf("EasyCAT initialization failed\n");
        return;
    }

    printf("EasyCAT task started\n");


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

    CAN1.attachObj(&myCanListener);

    motorDriverSetup();

    pinMode(EASYCAT_INT_PIN,INPUT_PULLUP);



    //xTaskCreatePinnedToCore(canfd_receive_task, "canfd_receive_task", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(easyCAT_task, "easyCAT-task", 4096, NULL, 6, &easyCATTaskHandle , 0);
    //xTaskCreatePinnedToCore(canfd_task, "canfd-send-task", 4096, NULL, 5, &canfdTaskHandle , 0);
    xTaskCreatePinnedToCore(canfdHealthCheckTask, "canfd_health_check_task", 4096, NULL, 1, NULL, 0);

    attachInterrupt(digitalPinToInterrupt(EASYCAT_INT_PIN),EasyCAT_IntHandler, FALLING);
    //init_timer();

}