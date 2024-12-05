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
#include "Logger.hpp"

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
#define MAX_MODULE_SIZE 8

//#define MOTOR_DRIVER_COUNT 7
//#define MOTOR_DRIVER_COUNT 4
//#define MOTOR_DRIVER_COUNT 1
//#define CONTROL_MODE RMTR_SERVO_MODE_POS
//#define CONTROL_MODE RMTR_SERVO_MODE_VEL
//#define CONTROL_MODE RMTR_SERVO_MODE_CUR
#define EASYCAT_INT_PIN 26

#define MAX_MODULE_SIZE 8
realman_motor_driver::RealmanMotorDriver motor_driver = realman_motor_driver::RealmanMotorDriver(
    std::make_shared<MCP2517FD>(CAN1)
); 

unsigned long previousMs = 0;
// EasyCAT buffer
PROCBUFFER_OUT _EasyCAT_BufferOut;  // output process data buffer


std::map<uint8_t, uint8_t> module_id_index_map;
std::array<uint32_t, MAX_MODULE_SIZE> canfd_send_counts;
std::array<uint32_t, MAX_MODULE_SIZE> canfd_receive_counts;
std::array<int32_t, MAX_MODULE_SIZE> actual_positions;
std::array<int32_t, MAX_MODULE_SIZE> actual_velocities;
std::array<int32_t, MAX_MODULE_SIZE> actual_currents;
std::array<uint16_t, MAX_MODULE_SIZE> status_words;
std::array<int32_t, MAX_MODULE_SIZE> target_positions;
std::array<int32_t, MAX_MODULE_SIZE> target_velocities;
std::array<int32_t, MAX_MODULE_SIZE> target_currents;
std::array<uint16_t, MAX_MODULE_SIZE> control_words;


// Timer configuration
#define TIMER_DIVIDER         2  // Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL_SEC   (0.002) // 1ms interval

TaskHandle_t canfdReadTaskHandle = NULL;
TaskHandle_t canfdWriteTaskHandle = NULL;
TaskHandle_t easyCATReadTaskHandle = NULL;
TaskHandle_t easyCATWriteTaskHandle = NULL;

int32_t easycat_task_index = 0;

typedef union ControlWord
{
    uint16_t word;
    struct
    {
        bool enable : 1;
        bool reserved_1 : 3;
        uint8_t control_mode : 2;
        uint8_t reserved_2 : 2;
        uint8_t control_state : 4;
        uint8_t module_id : 4;
    } bits;

} ControlWord;

typedef union StatusWord
{
    uint16_t word;
    struct
    {
        bool enable : 1;
        bool brake : 1;
        bool error : 1;
        bool reserved_1 : 1;
        uint8_t control_mode : 2;
        uint8_t reserved_2 : 2;
        uint8_t control_state : 4;
        uint8_t reserved_3 : 4;
    } bits;
} StatusWord;

 enum class ControllerState
    {
        STARTING = 0,
        INITIALIZING = 1,
        CONNECTION_WAITING = 2,
        INITIAL_MODE_SETTING = 3,
        IDLING = 4,
        HOMING = 5,
        CONTROLLING = 6,
        MODE_CHANGING = 7,
        STOPPING = 8,
        ENABLING = 9,
    };

void IRAM_ATTR EasyCAT_IntHandler()
{
    Logger::easycat_count++;
    // 1msごとにINTが発生する
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (easycat_task_index >= 1)
    {
        // 2msごとにEasyCATタスクに通知
        vTaskNotifyGiveFromISR(easyCATWriteTaskHandle, &xHigherPriorityTaskWoken);
        vTaskNotifyGiveFromISR(canfdWriteTaskHandle, &xHigherPriorityTaskWoken);
        easycat_task_index = 0;
    }
    else
    {
        vTaskNotifyGiveFromISR(easyCATReadTaskHandle, &xHigherPriorityTaskWoken);
        vTaskNotifyGiveFromISR(canfdReadTaskHandle, &xHigherPriorityTaskWoken);
        easycat_task_index++;
        Logger::timeLogIndex++;
    }
    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

void initializeMotorDriversMap() {

    for (uint8_t i = 0; i < MAX_MODULE_SIZE; i++)
    {
        canfd_send_counts[i] = 0;
        canfd_receive_counts[i] = 0;
        actual_positions[i] = 0;
        actual_velocities[i] = 0;
        actual_currents[i] = 0;
        status_words[i] = 0;
        target_positions[i] = 0;
        target_velocities[i] = 0;
        target_currents[i] = 0;
        control_words[i] = 0;
    }
}

void processMessage(CAN_FRAME_FD &rxFrame)
{
    uint8_t module_id = (rxFrame.id & 0x00FF);
    uint16_t message_type = (rxFrame.id & 0x0F00);
    if (module_id_index_map.find(module_id) == module_id_index_map.end())
    {
        ESP_LOGE("CANFD", "Module ID %d not found", module_id);
        return;
    }
    uint8_t index = module_id_index_map.at(module_id);
    ControlWord control_word;
    control_word.word = control_words[index];
    StatusWord status_word;
    status_word.word = status_words[index];
    status_word.bits.control_mode = control_word.bits.control_mode;
    status_word.bits.control_state = control_word.bits.control_state;

    auto response = motor_driver.parseCANFDMessage(rxFrame);
    if (message_type == MESSAGE_TYPE_RES_SERVO)
    {
        actual_positions[index] = response.servo_response.position;
        actual_velocities[index] = response.servo_response.velocity;
        actual_currents[index] = response.servo_response.current;
        status_word.bits.enable = response.servo_response.enable_state;
        status_word.bits.error = response.servo_response.error_code > 0;
        status_word.bits.brake = response.servo_response.error_code > 0;
    }
    else if (message_type == MESSAGE_TYPE_RES_COMMON)
    {
        auto command_index = response.common_response.command_index;
        uint8_t data = response.common_response.data.uint8[0];

        if (command_index == 0x49 && data == 0x01)
        {
            status_word.bits.enable = true;
        }
        else if (command_index == 0x30)
        {
            status_word.bits.control_mode = control_word.bits.control_mode;
        }
    }
    else if (message_type == MESSAGE_TYPE_RES_JSTATE)
    {
        auto error_code = response.joint_state_response.error_code;
        auto system_voltage = response.joint_state_response.system_voltage;
        auto system_temperature = response.joint_state_response.system_temperature;
        auto enable_state = response.joint_state_response.enable_state;
        auto brake_state = response.joint_state_response.brake_state;
        auto position = response.joint_state_response.position;
        auto current = response.joint_state_response.current;
        status_word.bits.enable = enable_state;
        status_word.bits.brake = brake_state;
        status_word.bits.error = error_code > 0;
        actual_positions[index] = position;
        actual_currents[index] = current;
    }
    status_words[index] = status_word.word;

    canfd_receive_counts[index]++;
    Logger::canfd_receive_count++;
}



class MyCANListener : public CANListener {
public:
    void gotFrameFD(CAN_FRAME_FD *frame, int mailbox) override {
        CAN_FRAME_FD rxFrame;
        if (CAN1.readFD(rxFrame))
        {
            processMessage(rxFrame);
        }
    }   
};

void loadRxQueue() {
    CAN1.intHandler();
    Logger::rx_queue_count ++;
    uint32_t rxQueueCount = CAN1.waitingRxQueueCount();
    for (uint32_t i = 0; i < rxQueueCount; i++) {
        CAN_FRAME_FD rxFrame;
        if (CAN1.readFD(rxFrame)) {
            processMessage(rxFrame);
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


void motorControl()
{
    Logger::motor_control_count++;
    for (auto const &tuple : module_id_index_map)
    {
        uint8_t module_id = tuple.first;
        uint8_t index = tuple.second;
        ControlWord control_word;
        control_word.word = control_words[index];

        uint8_t control_mode = control_word.bits.control_mode;
        uint8_t control_state = control_word.bits.control_state;
        ControllerState state = static_cast<ControllerState>(control_state);
        bool control_enable = control_word.bits.enable;
        if (!control_enable) {
            continue;
        }
        switch(state) {
            case ControllerState::STARTING:
                break;
            case ControllerState::INITIALIZING:
                motor_driver.writeConnectionOnline(module_id);
                Logger::canfd_send_count++;
                canfd_send_counts[index]++;
                break;
            case ControllerState::CONNECTION_WAITING:
                break;
            case ControllerState::INITIAL_MODE_SETTING:
            case ControllerState::MODE_CHANGING:
                motor_driver.writeControlMode(module_id, control_mode);
                Logger::canfd_send_count++;
                canfd_send_counts[index]++;
                break;
            case ControllerState::IDLING:
                motor_driver.readCurrentState(module_id);
                Logger::canfd_send_count++;
                canfd_send_counts[index]++;
                break;
            case ControllerState::HOMING:
            case ControllerState::STOPPING:
            case ControllerState::CONTROLLING:
                if (control_mode == 3)
                {
                    motor_driver.writeTargetPosition(module_id, target_positions[index]);
                    Logger::canfd_send_count++;
                    canfd_send_counts[index]++;
                }
                else if (control_mode == 2)
                {
                    motor_driver.writeTargetVelocity(module_id, target_velocities[index]);
                    Logger::canfd_send_count++;
                    canfd_send_counts[index]++;
                }
                else if (control_mode == 1)
                {
                    motor_driver.writeTargetCurrent(module_id, target_currents[index]);
                    Logger::canfd_send_count++;
                    canfd_send_counts[index]++;
                }
                break;
            case ControllerState::ENABLING:
                motor_driver.writeDriverEnabled(module_id);
                Logger::canfd_send_count++;
                canfd_send_counts[index]++;
                break;
        }
        ets_delay_us(50);
    }
    
}

void canfdReadTask(void *pvParameters) {
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        Logger::timeLog[Logger::timeLogIndex].loadRxQueue = measureTime(loadRxQueue);
    }
}

void canfdWriteTask(void *pvParameters) {
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        Logger::timeLog[Logger::timeLogIndex].motorControl = measureTime(motorControl);
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
        memcpy(response.data.uint8 + data_size, &ethercat_data.Cust.Torque_1, current_size);
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
        
        //for (int i = 0; i < CAN1.intFlagLogIndex; i++)
        //{
        //    ESP_LOGI("CANFD","0x%08X", CAN1.intFlagLog[i]);
        //    ESP_LOGI("CANFD","CiCON 0x%08X", CAN1.ciConLog[i]);
        //}
        Logger::consoleLog(
            CAN1,
            canfd_send_counts,
            canfd_receive_counts,
            module_id_index_map,
            control_words,
            status_words,
            target_positions,
            actual_positions
        );

        
        

        // 1秒待機
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
}

void loadControlWordsFromEtherCAT()
{
    for (int i = 0; i < control_words.size(); i++)
    {
        uint16_t control_word = control_words[i];
        ControlWord cw;
        cw.word = control_word;
        uint8_t module_id = cw.bits.module_id;
        module_id_index_map.emplace(module_id, i);
    }

}

void loadTargetValuesFromEtherCAT()
{

    // moddule_id_index_mapに基づいて、EasyCATバッファからターゲット値をロード
    for (int i = 0 ; i < MAX_MODULE_SIZE; i++)
    {
        //uint8_t index = module_id_index[index];
        int32_t target_position = 0;
        int32_t target_velocity = 0;
        int32_t target_current = 0;
        uint16_t control_word = 0;
        switch (i)
        {
        case 0:
            target_position = EasyCAT_BufferOut.Cust.Position_1;
            target_velocity = EasyCAT_BufferOut.Cust.Velocity_1;
            target_current = EasyCAT_BufferOut.Cust.Torque_1;
            control_word = EasyCAT_BufferOut.Cust.ControlWord_1;
            break;
        case 1:
            target_position = EasyCAT_BufferOut.Cust.Position_2;
            target_velocity = EasyCAT_BufferOut.Cust.Velocity_2;
            target_current = EasyCAT_BufferOut.Cust.Torque_2;
            control_word = EasyCAT_BufferOut.Cust.ControlWord_2;
            break;
        case 2:
            target_position = EasyCAT_BufferOut.Cust.Position_3;
            target_velocity = EasyCAT_BufferOut.Cust.Velocity_3;
            target_current = EasyCAT_BufferOut.Cust.Torque_3;
            control_word = EasyCAT_BufferOut.Cust.ControlWord_3;
            break;
        case 3:
            target_position = EasyCAT_BufferOut.Cust.Position_4;
            target_velocity = EasyCAT_BufferOut.Cust.Velocity_4;
            target_current = EasyCAT_BufferOut.Cust.Torque_4;
            control_word = EasyCAT_BufferOut.Cust.ControlWord_4;
            break;
        case 4:
            target_position = EasyCAT_BufferOut.Cust.Position_5;
            target_velocity = EasyCAT_BufferOut.Cust.Velocity_5;
            target_current = EasyCAT_BufferOut.Cust.Torque_5;
            control_word = EasyCAT_BufferOut.Cust.ControlWord_5;
            break;
        case 5:
            target_position = EasyCAT_BufferOut.Cust.Position_6;
            target_velocity = EasyCAT_BufferOut.Cust.Velocity_6;
            target_current = EasyCAT_BufferOut.Cust.Torque_6;
            control_word = EasyCAT_BufferOut.Cust.ControlWord_6;
            break;
        case 6:
            target_position = EasyCAT_BufferOut.Cust.Position_7;
            target_velocity = EasyCAT_BufferOut.Cust.Velocity_7;
            target_current = EasyCAT_BufferOut.Cust.Torque_7;
            control_word = EasyCAT_BufferOut.Cust.ControlWord_7;
            break;
        case 7:
            target_position = EasyCAT_BufferOut.Cust.Position_8;
            target_velocity = EasyCAT_BufferOut.Cust.Velocity_8;
            target_current = EasyCAT_BufferOut.Cust.Torque_8;
            control_word = EasyCAT_BufferOut.Cust.ControlWord_8;
            break;
        }

        target_positions[i] = target_position;
        target_velocities[i] = target_velocity;
        target_currents[i] = target_current;
        control_words[i] = control_word;
    }
}



// set current values to EasyCAT buffer
void setCurrentValuesToEasyCATBuffer() {
    for (auto const &tuple : module_id_index_map)
    {
        uint8_t module_id = tuple.first;
        uint8_t index = tuple.second;
        switch(index) {
        case 0:
            EasyCAT_BufferIn.Cust.ActualPosition_1 = actual_positions[index];
            EasyCAT_BufferIn.Cust.ActualVelocity_1 = actual_velocities[index];
            EasyCAT_BufferIn.Cust.ActualTorque_1 = actual_currents[index];
            EasyCAT_BufferIn.Cust.StatusWord_1 = status_words[index];
            break;

        case 1:
            EasyCAT_BufferIn.Cust.ActualPosition_2 = actual_positions[index];
            EasyCAT_BufferIn.Cust.ActualVelocity_2 = actual_velocities[index];
            EasyCAT_BufferIn.Cust.ActualTorque_2 = actual_currents[index];
            EasyCAT_BufferIn.Cust.StatusWord_2 = status_words[index];
            break;
        
        case 2:
            EasyCAT_BufferIn.Cust.ActualPosition_3 = actual_positions[index];
            EasyCAT_BufferIn.Cust.ActualVelocity_3 = actual_velocities[index];
            EasyCAT_BufferIn.Cust.ActualTorque_3 = actual_currents[index];
            EasyCAT_BufferIn.Cust.StatusWord_3 = status_words[index];
            break;

        case 3:
            EasyCAT_BufferIn.Cust.ActualPosition_4 = actual_positions[index];
            EasyCAT_BufferIn.Cust.ActualVelocity_4 = actual_velocities[index];
            EasyCAT_BufferIn.Cust.ActualTorque_4 = actual_currents[index];
            EasyCAT_BufferIn.Cust.StatusWord_4 = status_words[index];
            break;
        
        case 4:
            EasyCAT_BufferIn.Cust.ActualPosition_5 = actual_positions[index];
            EasyCAT_BufferIn.Cust.ActualVelocity_5 = actual_velocities[index];
            EasyCAT_BufferIn.Cust.ActualTorque_5 = actual_currents[index];
            EasyCAT_BufferIn.Cust.StatusWord_5 = status_words[index];
            break;

        case 5:
            EasyCAT_BufferIn.Cust.ActualPosition_6 = actual_positions[index];
            EasyCAT_BufferIn.Cust.ActualVelocity_6 = actual_velocities[index];
            EasyCAT_BufferIn.Cust.ActualTorque_6 = actual_currents[index];
            EasyCAT_BufferIn.Cust.StatusWord_6 = status_words[index];
            break;
        
        case 6:
            EasyCAT_BufferIn.Cust.ActualPosition_7 = actual_positions[index];
            EasyCAT_BufferIn.Cust.ActualVelocity_7 = actual_velocities[index];
            EasyCAT_BufferIn.Cust.ActualTorque_7 = actual_currents[index];
            EasyCAT_BufferIn.Cust.StatusWord_7 = status_words[index];
            break;
        
        case 7:
            EasyCAT_BufferIn.Cust.ActualPosition_8 = actual_positions[index];
            EasyCAT_BufferIn.Cust.ActualVelocity_8 = actual_velocities[index];
            EasyCAT_BufferIn.Cust.ActualTorque_8 = actual_currents[index];
            EasyCAT_BufferIn.Cust.StatusWord_8 = status_words[index];
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
        Logger::timeLog[Logger::timeLogIndex].easyCATRead = measureTime(easyCATRead);
        Logger::timeLog[Logger::timeLogIndex].loadTargetValuesFromEtherCAT = measureTime(loadTargetValuesFromEtherCAT);
        loadControlWordsFromEtherCAT();
    }
}


void easyCATWriteTask(void *pvParameters) {
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        Logger::timeLog[Logger::timeLogIndex].setCurrentValuesToEasyCATBuffer = measureTime(setCurrentValuesToEasyCATBuffer);
        Logger::timeLog[Logger::timeLogIndex].easyCATWrite = measureTime(easyCATWrite); 
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

    //motorDriverSetup();

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