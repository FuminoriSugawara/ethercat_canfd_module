#ifndef _LOGGER_HPP
#define _LOGGER_HPP
#include <map>
#include "esp32_can.h"

#define TIME_LOG_BUFFER_SIZE 1000
// このクラスでは、CANFDの送受信カウント数や、各処理にかかる時間を集計する機能を提供する
namespace Logger
{

    typedef struct
    {
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

    typedef struct
    {
        TimeLog average;
        TimeLog min;
        TimeLog max;
        uint32_t easyCATReadTaskOverflowCount;
        uint32_t easyCATWriteTaskOverflowCount;
        uint32_t canfdReadTaskOverflowCount;
        uint32_t canfdWriteTaskOverflowCount;
    } TimeLogStats;

    extern TimeLog timeLog[TIME_LOG_BUFFER_SIZE];
    extern int timeLogIndex;
    extern uint32_t canfd_send_count;
    extern uint32_t canfd_receive_count;
    extern uint32_t easycat_count;
    extern uint32_t motor_control_count;
    extern uint32_t rx_queue_count;

    void consoleLog(
        MCP2517FD &CAN1,
        std::array<uint32_t, 8> &canfd_send_counts,
        std::array<uint32_t, 8> &canfd_receive_counts,
        std::map<uint8_t, uint8_t> &module_id_index_map,
        std::array<uint16_t, 8> &control_words,
        std::array<uint16_t, 8> &status_words,
        std::array<int32_t, 8> &target_positions,
        std::array<int32_t, 8> &actual_positions
        );

    TimeLogStats calcStats();
}

#endif