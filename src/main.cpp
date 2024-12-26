//#include <Arduino.h>
#include <map>
#include <memory>
#include <array>
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

//#define MOTOR_DRIVER_COUNT 7
//#define MOTOR_DRIVER_COUNT 4
//#define MOTOR_DRIVER_COUNT 1
#define CONTROL_MODE RMTR_SERVO_MODE_POS
//#define CONTROL_MODE RMTR_SERVO_MODE_VEL
//#define CONTROL_MODE RMTR_SERVO_MODE_CUR
#define EASYCAT_INT_PIN 26

#define TIME_LOG_BUFFER_SIZE 1000

typedef struct {
    int64_t easyCATRead;
    int64_t easyCATWrite;
    int64_t loadTargetValuesFromEtherCAT;
    int64_t motorControl;
    int64_t loadRxQueue;
    int64_t loadCurrentValuesFromMotorDrivers;
    int64_t setCurrentValuesToEasyCATBuffer;
    int64_t easyCATReadTaskTotal;
    int64_t easyCATWriteTaskTotal;
    int64_t canfdReadTaskTotal;
    int64_t canfdWriteTaskTotal;
} TimeLog;

typedef struct {
    TimeLog average;
    TimeLog min;
    TimeLog max;
    uint32_t easyCATReadTaskOverflowCount;
    uint32_t easyCATWriteTaskOverflowCount;
    uint32_t canfdReadTaskOverflowCount;
    uint32_t canfdWriteTaskOverflowCount;
} TimeLogStats;

TimeLog timeLog[TIME_LOG_BUFFER_SIZE];
int timeLogIndex = 0;


unsigned long previousMs = 0;
boolean DEBUG_MODE = false;
boolean DRY_RUN = false;
// EasyCAT buffer
PROCBUFFER_OUT _EasyCAT_BufferOut;  // output process data buffer
// canfd send count;
volatile uint32_t canfd_send_count = 0;
// canfd receive count;
volatile uint32_t canfd_receive_count = 0;

volatile uint32_t easycat_count = 0;

volatile uint32_t motor_control_count = 0;
volatile uint32_t rx_queue_count = 0;
constexpr std::array<uint8_t, 7> USED_MODULE_IDS = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
//constexpr std::array<uint8_t, 6> USED_MODULE_IDS = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
//constexpr std::array<uint8_t, 5> USED_MODULE_IDS = {0x01, 0x02, 0x03, 0x04, 0x05};
//constexpr std::array<uint8_t, 4> USED_MODULE_IDS = {0x01, 0x02, 0x03, 0x04};
//constexpr std::array<uint8_t, 3> USED_MODULE_IDS = {0x01, 0x02, 0x03};
//constexpr std::array<uint8_t, 2> USED_MODULE_IDS = {0x01, 0x02};
//constexpr std::array<uint8_t, 1> USED_MODULE_IDS = {0x01};

std::map<uint8_t, uint8_t> module_id_to_easycat_index_map = {
    {0x01, 0},
    {0x02, 1},
    {0x03, 2},
    {0x04, 3},
    {0x05, 4},
    {0x06, 5},
    {0x07, 6}
};

std::map<uint8_t, realman_motor_driver::RealmanMotorDriver> motor_drivers_map;
std::map<uint8_t, uint32_t> canfd_send_counts_map;
std::map<uint8_t, uint32_t> canfd_receive_counts_map;
std::map<uint8_t, bool> motor_driver_connection_states_map;
std::map<uint8_t, int32_t> actual_positions_map;
std::map<uint8_t, int32_t> actual_velocities_map;
std::map<uint8_t, int32_t> actual_currents_map;
std::map<uint8_t, uint16_t> encoder_diffs_map;
std::map<uint8_t, int32_t> target_positions_map;
std::map<uint8_t, int32_t> target_velocities_map;
std::map<uint8_t, int32_t> target_currents_map;
std::map<uint8_t, uint16_t> control_words_map;


// Timer configuration
#define TIMER_DIVIDER         2  // Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL_SEC   (0.002) // 1ms interval

TaskHandle_t canfdReadTaskHandle = NULL;
TaskHandle_t canfdWriteTaskHandle = NULL;
TaskHandle_t easyCATReadTaskHandle = NULL;
TaskHandle_t easyCATWriteTaskHandle = NULL;

uint8_t cycle_ms = 4;
uint8_t offset_ms = cycle_ms / 2;



void IRAM_ATTR EasyCAT_IntHandler()
{
    static uint32_t count = 0;
    // 1msごとにINTが発生する
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t local_count = count % cycle_ms;
    if (local_count == 0)
    {
        vTaskNotifyGiveFromISR(easyCATWriteTaskHandle, &xHigherPriorityTaskWoken);
        vTaskNotifyGiveFromISR(canfdWriteTaskHandle, &xHigherPriorityTaskWoken);
    }
    else if (local_count == offset_ms)
    {
        vTaskNotifyGiveFromISR(easyCATReadTaskHandle, &xHigherPriorityTaskWoken);
        vTaskNotifyGiveFromISR(canfdReadTaskHandle, &xHigherPriorityTaskWoken);
        timeLogIndex++;
    }

    count++;
    
    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}




void initializeMotorDriversMap() {
    std::shared_ptr<MCP2517FD> can1_ptr = std::make_shared<MCP2517FD>(CAN1);

    for (uint8_t module_id : USED_MODULE_IDS) {
        motor_drivers_map.emplace(
            module_id,
            realman_motor_driver::RealmanMotorDriver(module_id, can1_ptr, DEBUG_MODE, DRY_RUN)
        );
        canfd_send_counts_map.emplace(module_id, 0);
        canfd_receive_counts_map.emplace(module_id, 0);
        motor_driver_connection_states_map.emplace(module_id, false);
        actual_positions_map.emplace(module_id, 0);
        actual_velocities_map.emplace(module_id, 0);
        actual_currents_map.emplace(module_id, 0);
        encoder_diffs_map.emplace(module_id, 0);
        target_positions_map.emplace(module_id, 0);
        target_velocities_map.emplace(module_id, 0);
        target_currents_map.emplace(module_id, 0);
        control_words_map.emplace(module_id, 0);
    }
}


//int32_t actual_positions[8] = {0, 0, 0, 0, 0, 0, 0, 0};
//int32_t actual_velocities[8] = {0, 0, 0, 0, 0, 0, 0, 0};
//int32_t actual_currents[8] = {0, 0, 0, 0, 0, 0, 0, 0};
//uint16_t encoder_diffs[8] = {0, 0, 0, 0, 0, 0, 0, 0};
//
//int32_t target_positions[8] = {0, 0, 0, 0, 0, 0, 0, 0};
//int32_t target_velocities[8] = {0, 0, 0, 0, 0, 0, 0, 0};
//int32_t target_currents[8] = {0, 0, 0, 0, 0, 0, 0, 0};
//uint16_t control_words[8] = {0, 0, 0, 0, 0, 0, 0, 0};

class MyCANListener : public CANListener {
public:

    void gotFrameFD(CAN_FRAME_FD *frame, int mailbox) override {
        CAN_FRAME_FD rxFrame;
        CAN1.readFD(rxFrame);
        uint8_t module_id = (rxFrame.id & 0x00FF);
        auto it = motor_drivers_map.find(module_id);
        if (it == motor_drivers_map.end()) {
            return;
        }
        motor_drivers_map.at(module_id).processCANFDMessage(rxFrame);
        canfd_receive_counts_map[module_id]++;
        canfd_receive_count++;
    }   
};

void loadRxQueue() {
    CAN1.intHandler();
    rx_queue_count ++;
    uint32_t rxQueueCount = CAN1.waitingRxQueueCount();
    for (uint32_t i = 0; i < rxQueueCount; i++) {
        CAN_FRAME_FD rxFrame;
        if (CAN1.readFD(rxFrame)) {
            uint8_t module_id = (rxFrame.id & 0x00FF);
            auto it = motor_drivers_map.find(module_id);
            if (it == motor_drivers_map.end()) {
                return;
            }
            motor_drivers_map.at(module_id).processCANFDMessage(rxFrame);
            canfd_receive_count++;
            canfd_receive_counts_map[module_id]++;
        }
    }
}

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

    int64_t easyCATReadMin = INT64_MAX;
    int64_t easyCATWriteMin = INT64_MAX;
    int64_t loadTargetValuesFromEtherCATMin = INT64_MAX;
    int64_t motorControlMin = INT64_MAX;
    int64_t loadCurrentValuesFromMotorDriversMin = INT64_MAX;
    int64_t setCurrentValuesToEasyCATBufferMin = INT64_MAX;
    int64_t loadRxQueueMin = INT64_MAX;
    int64_t easyCatReadTaskTotalMin = INT64_MAX;
    int64_t easyCatWriteTaskTotalMin = INT64_MAX;
    int64_t canfdReadTaskTotalMin = INT64_MAX;
    int64_t canfdWriteTaskTotalMin = INT64_MAX;

    int64_t easyCATReadMax = 0;
    int64_t easyCATWriteMax = 0;
    int64_t loadTargetValuesFromEtherCATMax = 0;
    int64_t motorControlMax = 0;
    int64_t loadCurrentValuesFromMotorDriversMax = 0;
    int64_t setCurrentValuesToEasyCATBufferMax = 0;
    int64_t loadRxQueueMax = 0;
    int64_t easyCatReadTaskTotalMax = 0;
    int64_t easyCatWriteTaskTotalMax = 0;
    int64_t canfdReadTaskTotalMax = 0;
    int64_t canfdWriteTaskTotalMax = 0;


    int64_t easyCATReadSum = 0;
    int64_t easyCATWriteSum = 0;
    int64_t loadTargetValuesFromEtherCATSum = 0;
    int64_t motorControlSum = 0;
    int64_t loadCurrentValuesFromMotorDriversSum = 0;
    int64_t setCurrentValuesToEasyCATBufferSum = 0;
    int64_t loadRxQueueSum = 0;
    int64_t easyCatReadTaskTotalSum = 0;
    int64_t easyCatWriteTaskTotalSum = 0;
    int64_t canfdReadTaskTotalSum = 0;
    int64_t canfdWriteTaskTotalSum = 0;

    //int32_t overflowCount = 0;
    int32_t easyCATReadTaskOverflowCount = 0;
    int32_t easyCATWriteTaskOverflowCount = 0;
    int32_t canfdReadTaskOverflowCount = 0;
    int32_t canfdWriteTaskOverflowCount = 0;

    if (timeLogIndex == 0)
    {
        return stats;
    }
    int32_t size = ((timeLogIndex + 1)< TIME_LOG_BUFFER_SIZE) ? (timeLogIndex + 1) : TIME_LOG_BUFFER_SIZE;
    for (int i = 0; i < size; i++)
    {
        timeLog[i].easyCATReadTaskTotal = timeLog[i].easyCATRead + timeLog[i].loadTargetValuesFromEtherCAT;
        timeLog[i].canfdReadTaskTotal = timeLog[i].loadRxQueue;

        timeLog[i].easyCATWriteTaskTotal = timeLog[i].easyCATWrite + timeLog[i].loadCurrentValuesFromMotorDrivers + timeLog[i].setCurrentValuesToEasyCATBuffer; 
        timeLog[i].canfdWriteTaskTotal = timeLog[i].motorControl;

        if (timeLog[i].easyCATReadTaskTotal > 1000 * offset_ms)
        {
            easyCATReadTaskOverflowCount++;
        }

        if (timeLog[i].easyCATWriteTaskTotal > 1000 * offset_ms)
        {
            easyCATWriteTaskOverflowCount++;
        }

        if (timeLog[i].canfdReadTaskTotal > 1000 * offset_ms)
        {
            canfdReadTaskOverflowCount++;
        }

        if (timeLog[i].canfdWriteTaskTotal > 1000 * offset_ms)
        {
            canfdWriteTaskOverflowCount++;
        }


        easyCATReadSum += timeLog[i].easyCATRead;
        easyCATWriteSum += timeLog[i].easyCATWrite;
        loadTargetValuesFromEtherCATSum += timeLog[i].loadTargetValuesFromEtherCAT;
        motorControlSum += timeLog[i].motorControl;
        loadCurrentValuesFromMotorDriversSum += timeLog[i].loadCurrentValuesFromMotorDrivers;
        setCurrentValuesToEasyCATBufferSum += timeLog[i].setCurrentValuesToEasyCATBuffer;
        loadRxQueueSum += timeLog[i].loadRxQueue;

        easyCatReadTaskTotalSum += timeLog[i].easyCATReadTaskTotal;
        easyCatWriteTaskTotalSum += timeLog[i].easyCATWriteTaskTotal;
        canfdReadTaskTotalSum += timeLog[i].canfdReadTaskTotal;
        canfdWriteTaskTotalSum += timeLog[i].canfdWriteTaskTotal;


        //if (timeLog[i].easyCATApplication < easyCATApplicationMin)
        //{
        //    easyCATApplicationMin = timeLog[i].easyCATApplication;
        //}
        if (timeLog[i].easyCATRead > 0 && timeLog[i].easyCATRead < easyCATReadMin)
        {
            easyCATReadMin = timeLog[i].easyCATRead;
        }

        if (timeLog[i].easyCATWrite > 0 && timeLog[i].easyCATWrite < easyCATWriteMin)
        {
            easyCATWriteMin = timeLog[i].easyCATWrite;
        }

        if (timeLog[i].loadTargetValuesFromEtherCAT > 0 && timeLog[i].loadTargetValuesFromEtherCAT < loadTargetValuesFromEtherCATMin)
        {
            loadTargetValuesFromEtherCATMin = timeLog[i].loadTargetValuesFromEtherCAT;
        }

        if (timeLog[i].motorControl > 0 && timeLog[i].motorControl < motorControlMin)
        {
            motorControlMin = timeLog[i].motorControl;
        }

        if (timeLog[i].loadCurrentValuesFromMotorDrivers > 0 && timeLog[i].loadCurrentValuesFromMotorDrivers < loadCurrentValuesFromMotorDriversMin)
        {
            loadCurrentValuesFromMotorDriversMin = timeLog[i].loadCurrentValuesFromMotorDrivers;
        }

        if (timeLog[i].setCurrentValuesToEasyCATBuffer > 0 && timeLog[i].setCurrentValuesToEasyCATBuffer < setCurrentValuesToEasyCATBufferMin)
        {
            setCurrentValuesToEasyCATBufferMin = timeLog[i].setCurrentValuesToEasyCATBuffer;
        }

        if (timeLog[i].loadRxQueue > 0 && timeLog[i].loadRxQueue < loadRxQueueMin)
        {
            loadRxQueueMin = timeLog[i].loadRxQueue;
        }

        if (timeLog[i].easyCATReadTaskTotal > 0 && timeLog[i].easyCATReadTaskTotal < easyCatReadTaskTotalMin)
        {
            easyCatReadTaskTotalMin = timeLog[i].easyCATReadTaskTotal;
        }

        if (timeLog[i].easyCATWriteTaskTotal > 0 && timeLog[i].easyCATWriteTaskTotal < easyCatWriteTaskTotalMin)
        {
            easyCatWriteTaskTotalMin = timeLog[i].easyCATWriteTaskTotal;
        }

        if (timeLog[i].canfdReadTaskTotal > 0 && timeLog[i].canfdReadTaskTotal < canfdReadTaskTotalMin)
        {
            canfdReadTaskTotalMin = timeLog[i].canfdReadTaskTotal;
        }

        if (timeLog[i].canfdWriteTaskTotal > 0 && timeLog[i].canfdWriteTaskTotal < canfdWriteTaskTotalMin)
        {
            canfdWriteTaskTotalMin = timeLog[i].canfdWriteTaskTotal;
        }

        if (timeLog[i].easyCATRead > easyCATReadMax)
        {
            easyCATReadMax = timeLog[i].easyCATRead;
        }

        if (timeLog[i].easyCATWrite > easyCATWriteMax)
        {
            easyCATWriteMax = timeLog[i].easyCATWrite;
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

        if (timeLog[i].loadRxQueue > loadRxQueueMax)
        {
            loadRxQueueMax = timeLog[i].loadRxQueue;
        }

        if (timeLog[i].easyCATReadTaskTotal > easyCatReadTaskTotalMax)
        {
            easyCatReadTaskTotalMax = timeLog[i].easyCATReadTaskTotal;
        }

        if (timeLog[i].easyCATWriteTaskTotal > easyCatWriteTaskTotalMax)
        {
            easyCatWriteTaskTotalMax = timeLog[i].easyCATWriteTaskTotal;
        }

        if (timeLog[i].canfdReadTaskTotal > canfdReadTaskTotalMax)
        {
            canfdReadTaskTotalMax = timeLog[i].canfdReadTaskTotal;
        }

        if (timeLog[i].canfdWriteTaskTotal > canfdWriteTaskTotalMax)
        {
            canfdWriteTaskTotalMax = timeLog[i].canfdWriteTaskTotal;
        }

    }
    int64_t easyCATReadAvg = easyCATReadSum / (size);
    int64_t easyCATWriteAvg = easyCATWriteSum / (size);
    int64_t loadTargetValuesFromEtherCATAvg = loadTargetValuesFromEtherCATSum / size;
    int64_t motorControlAvg = motorControlSum / size;
    int64_t loadCurrentValuesFromMotorDriversAvg = loadCurrentValuesFromMotorDriversSum / size;
    int64_t setCurrentValuesToEasyCATBufferAvg = setCurrentValuesToEasyCATBufferSum / size;
    int64_t loadRxQueueAvg = loadRxQueueSum / size;
    int64_t easyCatReadTaskTotalAvg = easyCatReadTaskTotalSum / size;
    int64_t easyCatWriteTaskTotalAvg = easyCatWriteTaskTotalSum / size;
    int64_t canfdReadTaskTotalAvg = canfdReadTaskTotalSum / size;
    int64_t canfdWriteTaskTotalAvg = canfdWriteTaskTotalSum / size;

    stats.average.easyCATRead = easyCATReadAvg;
    stats.average.easyCATWrite = easyCATWriteAvg;
    stats.average.loadTargetValuesFromEtherCAT = loadTargetValuesFromEtherCATAvg;
    stats.average.motorControl = motorControlAvg;
    stats.average.loadCurrentValuesFromMotorDrivers = loadCurrentValuesFromMotorDriversAvg;
    stats.average.setCurrentValuesToEasyCATBuffer = setCurrentValuesToEasyCATBufferAvg;
    stats.average.loadRxQueue = loadRxQueueAvg;
    stats.average.easyCATReadTaskTotal = easyCatReadTaskTotalAvg;
    stats.average.easyCATWriteTaskTotal = easyCatWriteTaskTotalAvg;
    stats.average.canfdReadTaskTotal = canfdReadTaskTotalAvg;
    stats.average.canfdWriteTaskTotal = canfdWriteTaskTotalAvg;

    stats.min.easyCATRead = easyCATReadMin;
    stats.min.easyCATWrite = easyCATWriteMin;
    stats.min.loadTargetValuesFromEtherCAT = loadTargetValuesFromEtherCATMin;
    stats.min.motorControl = motorControlMin;
    stats.min.loadCurrentValuesFromMotorDrivers = loadCurrentValuesFromMotorDriversMin;
    stats.min.setCurrentValuesToEasyCATBuffer = setCurrentValuesToEasyCATBufferMin;
    stats.min.loadRxQueue = loadRxQueueMin;
    stats.min.easyCATReadTaskTotal = easyCatReadTaskTotalMin;
    stats.min.easyCATWriteTaskTotal = easyCatWriteTaskTotalMin;
    stats.min.canfdReadTaskTotal = canfdReadTaskTotalMin;
    stats.min.canfdWriteTaskTotal = canfdWriteTaskTotalMin;

    stats.max.easyCATRead = easyCATReadMax;
    stats.max.easyCATWrite = easyCATWriteMax;
    stats.max.loadTargetValuesFromEtherCAT = loadTargetValuesFromEtherCATMax;
    stats.max.motorControl = motorControlMax;
    stats.max.loadCurrentValuesFromMotorDrivers = loadCurrentValuesFromMotorDriversMax;
    stats.max.setCurrentValuesToEasyCATBuffer = setCurrentValuesToEasyCATBufferMax;
    stats.max.loadRxQueue = loadRxQueueMax;
    stats.max.easyCATReadTaskTotal = easyCatReadTaskTotalMax;
    stats.max.easyCATWriteTaskTotal = easyCatWriteTaskTotalMax;
    stats.max.canfdReadTaskTotal = canfdReadTaskTotalMax;
    stats.max.canfdWriteTaskTotal = canfdWriteTaskTotalMax;

    stats.easyCATReadTaskOverflowCount = easyCATReadTaskOverflowCount;
    stats.easyCATWriteTaskOverflowCount = easyCATWriteTaskOverflowCount;
    stats.canfdReadTaskOverflowCount = canfdReadTaskOverflowCount;
    stats.canfdWriteTaskOverflowCount = canfdWriteTaskOverflowCount;    

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
        bool all_drivers_connected = false;
        uint8_t retry_count = 5;
        while (!all_drivers_connected)
        {
            uint8_t connected_count = 0;
            for (uint8_t module_id : USED_MODULE_IDS)
            {
                // verify all motor drivers are connected
                bool connection_state = motor_drivers_map.at(module_id).getConnectionState();
                motor_driver_connection_states_map[module_id] = connection_state;
                if (connection_state) {
                    Serial.printf("Motor driver %d connected\n", module_id);
                    connected_count++;
                    continue;
                } else 
                {
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                    motor_drivers_map.at(module_id).setConnectionOnline();
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
            }
            if (connected_count == USED_MODULE_IDS.size())
            {
                Serial.println("All motor drivers connected");
                all_drivers_connected = true;
                break;
            }
            retry_count--;
            if (retry_count == 0)
            {
                Serial.println("Motor driver connection failed");
                for (uint8_t module_id : USED_MODULE_IDS)
                {
                    Serial.printf("Motor driver connection state: %d\n", motor_driver_connection_states_map[module_id]);
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
        for (uint8_t module_id : USED_MODULE_IDS)
        {
            motor_drivers_map.at(module_id).loadState();
        }
        vTaskDelay(200 / portTICK_PERIOD_MS);
        ESP_LOGI("MotorDriver", "Checking driver error");
        bool all_drivers_error_free = true;
        for (uint8_t module_id : USED_MODULE_IDS)
        {
            uint16_t error_state = motor_drivers_map.at(module_id).getErrorState();
            ESP_LOGE("MotorDriver","Error state %d: 0x%04X", module_id, error_state);
            if (error_state != 0)
            {
                all_drivers_error_free = false;
            }
        }
        if (!all_drivers_error_free)
        {
            ESP_LOGE("MotorDriver","Motor driver error detected");
            ESP_LOGE("MotorDriver","Resetting motor drivers");
            for (uint8_t module_id : USED_MODULE_IDS)
            {
                motor_drivers_map.at(module_id).setDriverDisabled();
                vTaskDelay(100 / portTICK_PERIOD_MS);
                motor_drivers_map.at(module_id).clearJointError();
                vTaskDelay(100 / portTICK_PERIOD_MS);
                motor_drivers_map.at(module_id).setDriverEnabled();
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            // check driver error
            for (uint8_t module_id : USED_MODULE_IDS)
            {
                motor_drivers_map.at(module_id).loadState();
            }
            vTaskDelay(200 / portTICK_PERIOD_MS);
            for (uint8_t module_id : USED_MODULE_IDS)
            {
                uint16_t error_state = motor_drivers_map.at(module_id).getErrorState();
                ESP_LOGI("MotorDriver","Error state %d: 0x%04X\n", module_id, error_state);
            }
            while (1)
            {
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }

        ESP_LOGI("MotorDriver", "Setting control mode");

        for (uint8_t module_id : USED_MODULE_IDS)
        {
            if (CONTROL_MODE == RMTR_SERVO_MODE_POS)
            {
                // Switch to position control mode
                motor_drivers_map.at(module_id).setPositionControlMode();
            }
            else if (CONTROL_MODE == RMTR_SERVO_MODE_VEL)
            {
                // Switch to velocity control mode
                motor_drivers_map.at(module_id).setVelocityControlMode();
            }
            else if (CONTROL_MODE == RMTR_SERVO_MODE_CUR)
            {
                // Switch to current control mode
                motor_drivers_map.at(module_id).setCurrentControlMode();
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

void loadPosition()
{
    for (uint8_t module_id : USED_MODULE_IDS)
    {
        motor_drivers_map.at(module_id).loadOutputShaftPosition();
        canfd_send_count++;
        canfd_send_counts_map[module_id]++;
    }

}

void motorControl()
{
    motor_control_count++;
    for (uint8_t module_id : USED_MODULE_IDS)
    {
        if (CONTROL_MODE == RMTR_SERVO_MODE_POS)
        {
            // Serial.printf("Target position %d: %d\n", i, target_positions[i]);
            motor_drivers_map.at(module_id).setTargetPosition(target_positions_map.at(module_id));
            canfd_send_count++;
            canfd_send_counts_map[module_id]++;
        }
        else if (CONTROL_MODE == RMTR_SERVO_MODE_VEL)
        {
            // Serial.printf("Target velocity %d: %d\n", i, target_velocities[i]);
            motor_drivers_map.at(module_id).setTargetVelocity(target_velocities_map.at(module_id));
            canfd_send_count++;
            canfd_send_counts_map[module_id]++;
        }
        else if (CONTROL_MODE == RMTR_SERVO_MODE_CUR)
        {
            motor_drivers_map.at(module_id).setTargetCurrent(target_currents_map.at(module_id));
            canfd_send_count++;
            canfd_send_counts_map[module_id]++;
            // Serial.printf("Target current %d: %d\n", i, target_currents[i]);
        }
        ets_delay_us(50);
        // 出力側のエンコーダカウントを取得
        motor_drivers_map.at(module_id).loadEncoderCount();
        canfd_send_count++;
        canfd_send_counts_map[module_id]++;
        ets_delay_us(50);
    }
    
}

void canfdReadTask(void *pvParameters) {
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        timeLog[timeLogIndex].loadRxQueue = measureTime(loadRxQueue);
    }
}

void canfdWriteTask(void *pvParameters) {
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        timeLog[timeLogIndex].motorControl = measureTime(motorControl);
    }
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

    // Copy current data if there's space
    if (data_size < max_data_size) {
        size_t current_size = std::min(max_data_size - data_size, 8 * sizeof(int32_t));
        memcpy(response.data.uint8 + data_size, &ethercat_data.Cust.Current_1, current_size);
        data_size += current_size;
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
        ESP_LOGI("main", "EasyCAT count: %u", easycat_count);
        ESP_LOGI("main", "Motor control count: %u", motor_control_count);
        ESP_LOGI("main", "Send count: %u, Receive count: %u", canfd_send_count, canfd_receive_count);
        for (uint8_t module_id : USED_MODULE_IDS) {
            ESP_LOGI("main", "module_id: %d, Send count: %u, Receive count: %u", module_id, canfd_send_counts_map.at(module_id), canfd_receive_counts_map.at(module_id));
            ESP_LOGI("main", "module_id: %d, Output shaft count: %d, Motor shaft count: %d", module_id, motor_drivers_map.at(module_id).getOutputShaftEncoderCount(), motor_drivers_map.at(module_id).getMotorShaftEncoderCount());
        }
        ESP_LOGI("main", "RX Queue count: %u", rx_queue_count);
        ESP_LOGI("CANFD", "int_pin_count: %u", CAN1.int_pin_count);
        ESP_LOGI("CANFD", "task_MCPIntFD count: %u", CAN1.task_MCPIntFD_count);
        ESP_LOGI("CANFD", "INT Handler count: %u", CAN1.int_handler_count);
        ESP_LOGI("CANFD", "Handle dispatch count: %u", CAN1.handle_dispatch_count);
        ESP_LOGI("CANFD", "RX Queue count: %u", CAN1.rx_queue_count);
        ESP_LOGI("CANFD", "RX Queue available: %u", CAN1.available());
        ESP_LOGI("CANFD", "Send count: %u", CAN1.send_count);
        ESP_LOGE("CANFD", "Receive error count: %u", CAN1.receiveErrorCount);
        ESP_LOGE("CANFD", "Transmit error count: %u", CAN1.transmitErrorCount);
        ESP_LOGE("CANFD", "ErrorRegister: 0x%08X", CAN1.transmitRecceiveErrorCountResister);
        ESP_LOGI("CANFD", "Interrupt code: 0x%08X", CAN1.interruptCode);
        ESP_LOGI("CANFD", "Fifo status: 0x%08X", CAN1.fifoStatus);
        ESP_LOGI("CANFD", "Receive overflow interrupt status: 0x%08X", CAN1.receiveOverflowInterruptStatus);
        ESP_LOGE("CANFD", "CRC 0x%08X", CAN1.crc);
        ESP_LOGI("CANFD", "CiINT: 0x%08X", CAN1.ci_int);
        ESP_LOGI("CANFD", "Int flag log index: %u\n", CAN1.intFlagLogIndex);
        //for (int i = 0; i < CAN1.intFlagLogIndex; i++)
        //{
        //    ESP_LOGI("CANFD","0x%08X", CAN1.intFlagLog[i]);
        //    ESP_LOGI("CANFD","CiCON 0x%08X", CAN1.ciConLog[i]);
        //}

        TimeLogStats stats = calcStats();
        ESP_LOGI("Stats", "EasyCAT read time: min %lld us, max %lld us, avg %lld us", stats.min.easyCATRead, stats.max.easyCATRead, stats.average.easyCATRead);
        ESP_LOGI("Stats", "EasyCAT write time: min %lld us, max %lld us, avg %lld us", stats.min.easyCATWrite, stats.max.easyCATWrite, stats.average.easyCATWrite);
        ESP_LOGI("Stats", "Load target values from EtherCAT time: min %lld us, max %lld us, avg %lld us", stats.min.loadTargetValuesFromEtherCAT, stats.max.loadTargetValuesFromEtherCAT, stats.average.loadTargetValuesFromEtherCAT);
        ESP_LOGI("Stats", "Motor control time: min %lld us, max %lld us, avg %lld us", stats.min.motorControl, stats.max.motorControl, stats.average.motorControl);
        ESP_LOGI("Stats", "Load current values from motor drivers time: min %lld us, max %lld us, avg %lld us", stats.min.loadCurrentValuesFromMotorDrivers, stats.max.loadCurrentValuesFromMotorDrivers, stats.average.loadCurrentValuesFromMotorDrivers);
        ESP_LOGI("Stats", "Set current values to EasyCAT buffer time: min %lld us, max %lld us, avg %lld us", stats.min.setCurrentValuesToEasyCATBuffer, stats.max.setCurrentValuesToEasyCATBuffer, stats.average.setCurrentValuesToEasyCATBuffer);
        ESP_LOGI("Stats", "Load RX queue time: min %lld us, max %lld us, avg %lld us", stats.min.loadRxQueue, stats.max.loadRxQueue, stats.average.loadRxQueue);
        ESP_LOGI("Stats", "EasyCAT read task total time: min %lld us, max %lld us, avg %lld us", stats.min.easyCATReadTaskTotal, stats.max.easyCATReadTaskTotal, stats.average.easyCATReadTaskTotal);
        ESP_LOGI("Stats", "EasyCAT write task total time: min %lld us, max %lld us, avg %lld us", stats.min.easyCATWriteTaskTotal, stats.max.easyCATWriteTaskTotal, stats.average.easyCATWriteTaskTotal);
        ESP_LOGI("Stats", "CANFD read task total time: min %lld us, max %lld us, avg %lld us", stats.min.canfdReadTaskTotal, stats.max.canfdReadTaskTotal, stats.average.canfdReadTaskTotal);
        ESP_LOGI("Stats", "CANFD write task total time: min %lld us, max %lld us, avg %lld us", stats.min.canfdWriteTaskTotal, stats.max.canfdWriteTaskTotal, stats.average.canfdWriteTaskTotal);
        ESP_LOGI("Stats", "EasyCAT read task overflow count: %u", stats.easyCATReadTaskOverflowCount);
        ESP_LOGI("Stats", "EasyCAT write task overflow count: %u", stats.easyCATWriteTaskOverflowCount);
        ESP_LOGI("Stats", "CANFD read task overflow count: %u", stats.canfdReadTaskOverflowCount);
        ESP_LOGI("Stats", "CANFD write task overflow count: %u", stats.canfdWriteTaskOverflowCount);

        // カウント数をリセット
        motor_control_count = 0;
        canfd_send_count = 0;
        for (uint8_t module_id : USED_MODULE_IDS) {
            canfd_send_counts_map[module_id] = 0;
            canfd_receive_counts_map[module_id] = 0;
        }
        canfd_receive_count = 0;
        rx_queue_count = 0;
        easycat_count = 0;
        timeLogIndex = 0;

        CAN1.task_MCPIntFD_count = 0;
        CAN1.int_handler_count = 0;
        CAN1.handle_dispatch_count = 0;
        CAN1.rx_queue_count = 0;
        CAN1.intFlagLogIndex = 0;
        CAN1.send_count = 0;
        CAN1.int_pin_count = 0;

        // 1秒待機
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
}

void loadTargetValuesFromEtherCAT()
{
    for (uint8_t module_id : USED_MODULE_IDS)
    {
        uint8_t easycat_index = module_id_to_easycat_index_map.at(module_id);
        int32_t target_position = 0;
        int32_t target_velocity = 0;
        int32_t target_current = 0;
        uint16_t control_word = 0;
        switch (easycat_index)
        {
        case 0:
            target_position = EasyCAT_BufferOut.Cust.Position_1;
            target_velocity = EasyCAT_BufferOut.Cust.Velocity_1;
            target_current = EasyCAT_BufferOut.Cust.Current_1;
            control_word = EasyCAT_BufferOut.Cust.ControlWord_1;
            break;
        case 1:
            target_position = EasyCAT_BufferOut.Cust.Position_2;
            target_velocity = EasyCAT_BufferOut.Cust.Velocity_2;
            target_current = EasyCAT_BufferOut.Cust.Current_2;
            control_word = EasyCAT_BufferOut.Cust.ControlWord_2;
            break;
        case 2:
            target_position = EasyCAT_BufferOut.Cust.Position_3;
            target_velocity = EasyCAT_BufferOut.Cust.Velocity_3;
            target_current = EasyCAT_BufferOut.Cust.Current_3;
            control_word = EasyCAT_BufferOut.Cust.ControlWord_3;
            break;

        case 3:
            target_position = EasyCAT_BufferOut.Cust.Position_4;
            target_velocity = EasyCAT_BufferOut.Cust.Velocity_4;
            target_current = EasyCAT_BufferOut.Cust.Current_4;
            control_word = EasyCAT_BufferOut.Cust.ControlWord_4;
            break;
        case 4:
            target_position = EasyCAT_BufferOut.Cust.Position_5;
            target_velocity = EasyCAT_BufferOut.Cust.Velocity_5;
            target_current = EasyCAT_BufferOut.Cust.Current_5;
            control_word = EasyCAT_BufferOut.Cust.ControlWord_5;
            break;
        case 5:
            target_position = EasyCAT_BufferOut.Cust.Position_6;
            target_velocity = EasyCAT_BufferOut.Cust.Velocity_6;
            target_current = EasyCAT_BufferOut.Cust.Current_6;
            control_word = EasyCAT_BufferOut.Cust.ControlWord_6;
            break;
        case 6:
            target_position = EasyCAT_BufferOut.Cust.Position_7;
            target_velocity = EasyCAT_BufferOut.Cust.Velocity_7;
            target_current = EasyCAT_BufferOut.Cust.Current_7;
            control_word = EasyCAT_BufferOut.Cust.ControlWord_7;
            break;
        case 7:
            target_position = EasyCAT_BufferOut.Cust.Position_8;
            target_velocity = EasyCAT_BufferOut.Cust.Velocity_8;
            target_current = EasyCAT_BufferOut.Cust.Current_8;
            control_word = EasyCAT_BufferOut.Cust.ControlWord_8;
            break;
        }

        target_positions_map[module_id] = target_position;
        target_velocities_map[module_id] = target_velocity;
        target_currents_map[module_id] = target_current;
        control_words_map[module_id] = control_word;
    }
}

// load current values from motor drivers
void loadCurrentValuesFromMotorDrivers() {
    for (uint8_t module_id : USED_MODULE_IDS) {
        actual_positions_map[module_id] = (int32_t)(motor_drivers_map.at(module_id).getOutputShaftPosition());
        actual_currents_map[module_id] = (int32_t)(motor_drivers_map.at(module_id).getCurrent());
        actual_velocities_map[module_id] = (int32_t)(motor_drivers_map.at(module_id).getVelocity());
        encoder_diffs_map[module_id] = motor_drivers_map.at(module_id).getDifferenceBetweenMotorAndOutputShaftPosition();
    }
}

// set current values to EasyCAT buffer
void setCurrentValuesToEasyCATBuffer() {
    for (uint8_t module_id : USED_MODULE_IDS) {
        uint8_t index = module_id_to_easycat_index_map.at(module_id);
        switch(index) {
        case 0:
            EasyCAT_BufferIn.Cust.ActualPosition_1 = actual_positions_map.at(module_id);
            EasyCAT_BufferIn.Cust.ActualVelocity_1 = actual_velocities_map.at(module_id);
            EasyCAT_BufferIn.Cust.ActualCurrent_1 = actual_currents_map.at(module_id);
            EasyCAT_BufferIn.Cust.EncoderDiff_1 = encoder_diffs_map.at(module_id);
            break;

        case 1:
            EasyCAT_BufferIn.Cust.ActualPosition_2 = actual_positions_map.at(module_id);
            EasyCAT_BufferIn.Cust.ActualVelocity_2 = actual_velocities_map.at(module_id);
            EasyCAT_BufferIn.Cust.ActualCurrent_2 = actual_currents_map.at(module_id);
            EasyCAT_BufferIn.Cust.EncoderDiff_2 = encoder_diffs_map.at(module_id);
            break;
        
        case 2:
            EasyCAT_BufferIn.Cust.ActualPosition_3 = actual_positions_map.at(module_id);
            EasyCAT_BufferIn.Cust.ActualVelocity_3 = actual_velocities_map.at(module_id);
            EasyCAT_BufferIn.Cust.ActualCurrent_3 = actual_currents_map.at(module_id);
            EasyCAT_BufferIn.Cust.EncoderDiff_3 = encoder_diffs_map.at(module_id);
            break;

        case 3:
            EasyCAT_BufferIn.Cust.ActualPosition_4 = actual_positions_map.at(module_id);
            EasyCAT_BufferIn.Cust.ActualVelocity_4 = actual_velocities_map.at(module_id);
            EasyCAT_BufferIn.Cust.ActualCurrent_4 = actual_currents_map.at(module_id);
            EasyCAT_BufferIn.Cust.EncoderDiff_4 = encoder_diffs_map.at(module_id);
            break;
        
        case 4:
            EasyCAT_BufferIn.Cust.ActualPosition_5 = actual_positions_map.at(module_id);
            EasyCAT_BufferIn.Cust.ActualVelocity_5 = actual_velocities_map.at(module_id);
            EasyCAT_BufferIn.Cust.ActualCurrent_5 = actual_currents_map.at(module_id);
            EasyCAT_BufferIn.Cust.EncoderDiff_5 = encoder_diffs_map.at(module_id);
            break;

        case 5:
            EasyCAT_BufferIn.Cust.ActualPosition_6 = actual_positions_map.at(module_id);
            EasyCAT_BufferIn.Cust.ActualVelocity_6 = actual_velocities_map.at(module_id);
            EasyCAT_BufferIn.Cust.ActualCurrent_6 = actual_currents_map.at(module_id);
            EasyCAT_BufferIn.Cust.EncoderDiff_6 = encoder_diffs_map.at(module_id);
            break;
        
        case 6:
            EasyCAT_BufferIn.Cust.ActualPosition_7 = actual_positions_map.at(module_id);
            EasyCAT_BufferIn.Cust.ActualVelocity_7 = actual_velocities_map.at(module_id);
            EasyCAT_BufferIn.Cust.ActualCurrent_7 = actual_currents_map.at(module_id);
            EasyCAT_BufferIn.Cust.EncoderDiff_7 = encoder_diffs_map.at(module_id);
            break;
        
        case 7:
            EasyCAT_BufferIn.Cust.ActualPosition_8 = actual_positions_map.at(module_id);
            EasyCAT_BufferIn.Cust.ActualVelocity_8 = actual_velocities_map.at(module_id);
            EasyCAT_BufferIn.Cust.ActualCurrent_8 = actual_currents_map.at(module_id);
            EasyCAT_BufferIn.Cust.EncoderDiff_8 = encoder_diffs_map.at(module_id);
            break;
        }

        index++;
    }
}



void easyCATRead()
{
    EasyCAT_ReadTask();
}

void easyCATWrite()
{
    EasyCAT_WriteTask();
}


void easyCATReadTask(void *pvParameters) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        timeLog[timeLogIndex].easyCATRead = measureTime(easyCATRead);
        timeLog[timeLogIndex].loadTargetValuesFromEtherCAT = measureTime(loadTargetValuesFromEtherCAT);
    }
}


void easyCATWriteTask(void *pvParameters) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        timeLog[timeLogIndex].loadCurrentValuesFromMotorDrivers = measureTime(loadCurrentValuesFromMotorDrivers);
        timeLog[timeLogIndex].setCurrentValuesToEasyCATBuffer = measureTime(setCurrentValuesToEasyCATBuffer);
        timeLog[timeLogIndex].easyCATWrite = measureTime(easyCATWrite); 
    }
}


// Timer ISR callback
//static bool IRAM_ATTR timer_group_isr_callback(void *args) {
//    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//
//    // Notify Task A to read data from Device A
//    vTaskNotifyGiveFromISR(easyCATTaskHandle, &xHigherPriorityTaskWoken);
//    vTaskNotifyGiveFromISR(canfdTaskHandle, &xHigherPriorityTaskWoken);
//
//    // Reset the timer counter value
//    TIMERG0.hw_timer[TIMER_0].update = 1;
//    TIMERG0.int_clr_timers.t0 = 1;
//
//    return xHigherPriorityTaskWoken == pdTRUE;
//}

//void init_timer() {
//    // タイマーの設定構造体を初期化
//    timer_config_t config = {
//        .alarm_en = TIMER_ALARM_EN,       // アラーム（割り込み）を有効化
//        .counter_en = TIMER_PAUSE,        // 初期状態は停止
//        .intr_type = TIMER_INTR_LEVEL,    // レベルトリガー割り込み
//        .counter_dir = TIMER_COUNT_UP,    // カウンターの方向（上昇）
//        .auto_reload = TIMER_AUTORELOAD_EN, // 自動リロードを有効化
//        .divider = TIMER_DIVIDER          // タイマーのクロック分周比
//    };
//
//    // タイマーを初期化
//    ESP_ERROR_CHECK(timer_init(TIMER_GROUP_0, TIMER_0, &config));
//
//    // カウンター初期値を0に設定
//    ESP_ERROR_CHECK(timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0));
//
//    // アラーム値を設定（これが割り込みのタイミングを決定）
//    ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_SEC * TIMER_SCALE));
//
//    // 割り込みを有効化
//    ESP_ERROR_CHECK(timer_enable_intr(TIMER_GROUP_0, TIMER_0));
//
//    // 割り込みハンドラを設定
//    ESP_ERROR_CHECK(timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timer_group_isr_callback, NULL, 0));
//
//    // タイマーを開始
//    ESP_ERROR_CHECK(timer_start(TIMER_GROUP_0, TIMER_0));
//}

extern "C" void app_main(void)
{
    initializeMotorDriversMap();

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
    CAN1.enableInterrupts = true;
    CAN1.enableListener = true;

    motorDriverSetup();

    ESP_LOGI("main", "Starting EasyCAT task");
    pinMode(EASYCAT_INT_PIN,INPUT_PULLUP);
    CAN1.enableInterrupts = false;
    CAN1.enableListener = false;





    //xTaskCreatePinnedToCore(canfd_receive_task, "canfd_receive_task", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(easyCATReadTask, "easyCAT-read-task", 4096, NULL, 6, &easyCATReadTaskHandle , 0);
    xTaskCreatePinnedToCore(easyCATWriteTask, "easyCAT-write-task", 4096, NULL, 6, &easyCATWriteTaskHandle , 0);
    //xTaskCreatePinnedToCore(canfd_task, "canfd-send-task", 4096, NULL, 5, &canfdTaskHandle , 0);
    xTaskCreatePinnedToCore(canfdReadTask, "canfd-read-task", 4096, NULL, 6, &canfdReadTaskHandle, 1);
    xTaskCreatePinnedToCore(canfdWriteTask, "canfd-write-task", 4096, NULL, 6, &canfdWriteTaskHandle, 1);
    xTaskCreatePinnedToCore(canfdHealthCheckTask, "canfd_health_check_task", 4096, NULL, 1, NULL, 0);

    attachInterrupt(digitalPinToInterrupt(EASYCAT_INT_PIN),EasyCAT_IntHandler, FALLING);
    //init_timer();

}