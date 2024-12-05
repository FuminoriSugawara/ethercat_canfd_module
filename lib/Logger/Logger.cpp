#include "Logger.hpp"
namespace Logger {

    TimeLog timeLog[TIME_LOG_BUFFER_SIZE];
    int timeLogIndex = 0;
    uint32_t canfd_send_count = 0;
    uint32_t canfd_receive_count = 0;
    uint32_t easycat_count = 0;
    uint32_t motor_control_count = 0;
    uint32_t rx_queue_count = 0;


    void consoleLog(
        MCP2517FD &CAN1,
        std::array<uint32_t, 8> &canfd_send_counts,
        std::array<uint32_t, 8> &canfd_receive_counts,
        std::map<uint8_t, uint8_t> &module_id_index_map,
        std::array<uint16_t, 8> &control_words,
        std::array<uint16_t, 8> &status_words,
        std::array<int32_t, 8> &target_positions,
        std::array<int32_t, 8> &actual_positions
        )
    {
        ESP_LOGI("---", "----------------------------------------");
        ESP_LOGI("main", "EasyCAT count: %u", easycat_count);
        ESP_LOGI("main", "Motor control count: %u", motor_control_count);
        ESP_LOGI("main", "Send count: %u, Receive count: %u", canfd_send_count, canfd_receive_count);
        for (auto const &tuple : module_id_index_map)
        {
            uint8_t module_id = tuple.first;
            uint8_t index = tuple.second;
            ESP_LOGI("main", "module_id: %d, Send count: %u, Receive count: %u", module_id, canfd_send_counts[index], canfd_receive_counts[index]);
            ESP_LOGI("main", "module_id: %d, Control word: 0x%04X", module_id, control_words[index]);
            ESP_LOGI("main", "module_id: %d, Status word: 0x%04X", module_id, status_words[index]);
            ESP_LOGI("main", "module_id: %d, Target position: %d", module_id, target_positions[index]);
            ESP_LOGI("main", "module_id: %d, Actual position: %d", module_id, actual_positions[index]);
        }
        //ESP_LOGI("main", "RX Queue count: %u", rx_queue_count);
        //ESP_LOGI("CANFD", "int_pin_count: %u", CAN1.int_pin_count);
        //ESP_LOGI("CANFD", "task_MCPIntFD count: %u", CAN1.task_MCPIntFD_count);
        //ESP_LOGI("CANFD", "INT Handler count: %u", CAN1.int_handler_count);
        //ESP_LOGI("CANFD", "Handle dispatch count: %u", CAN1.handle_dispatch_count);
        //ESP_LOGI("CANFD", "RX Queue count: %u", CAN1.rx_queue_count);
        //ESP_LOGI("CANFD", "RX Queue available: %u", CAN1.available());
        //ESP_LOGI("CANFD", "Send count: %u", CAN1.send_count);
        //ESP_LOGE("CANFD", "Receive error count: %u", CAN1.receiveErrorCount);
        //ESP_LOGE("CANFD", "Transmit error count: %u", CAN1.transmitErrorCount);
        //ESP_LOGE("CANFD", "ErrorRegister: 0x%08X", CAN1.transmitRecceiveErrorCountResister);
        //ESP_LOGI("CANFD", "Interrupt code: 0x%08X", CAN1.interruptCode);
        //ESP_LOGI("CANFD", "Fifo status: 0x%08X", CAN1.fifoStatus);
        //ESP_LOGI("CANFD", "Receive overflow interrupt status: 0x%08X", CAN1.receiveOverflowInterruptStatus);
        //ESP_LOGE("CANFD", "CRC 0x%08X", CAN1.crc);
        //ESP_LOGI("CANFD", "CiINT: 0x%08X", CAN1.ci_int);
        //ESP_LOGI("CANFD", "Int flag log index: %u\n", CAN1.intFlagLogIndex);

        TimeLogStats stats = calcStats();
        //ESP_LOGI("Stats", "EasyCAT read time: min %lld us, max %lld us, avg %lld us", stats.min.easyCATRead, stats.max.easyCATRead, stats.average.easyCATRead);
        //ESP_LOGI("Stats", "EasyCAT write time: min %lld us, max %lld us, avg %lld us", stats.min.easyCATWrite, stats.max.easyCATWrite, stats.average.easyCATWrite);
        //ESP_LOGI("Stats", "Load target values from EtherCAT time: min %lld us, max %lld us, avg %lld us", stats.min.loadTargetValuesFromEtherCAT, stats.max.loadTargetValuesFromEtherCAT, stats.average.loadTargetValuesFromEtherCAT);
        //ESP_LOGI("Stats", "Motor control time: min %lld us, max %lld us, avg %lld us", stats.min.motorControl, stats.max.motorControl, stats.average.motorControl);
        //ESP_LOGI("Stats", "Load current values from motor drivers time: min %lld us, max %lld us, avg %lld us", stats.min.loadCurrentValuesFromMotorDrivers, stats.max.loadCurrentValuesFromMotorDrivers, stats.average.loadCurrentValuesFromMotorDrivers);
        //ESP_LOGI("Stats", "Set current values to EasyCAT buffer time: min %lld us, max %lld us, avg %lld us", stats.min.setCurrentValuesToEasyCATBuffer, stats.max.setCurrentValuesToEasyCATBuffer, stats.average.setCurrentValuesToEasyCATBuffer);
        //ESP_LOGI("Stats", "Load RX queue time: min %lld us, max %lld us, avg %lld us", stats.min.loadRxQueue, stats.max.loadRxQueue, stats.average.loadRxQueue);
        //ESP_LOGI("Stats", "EasyCAT read task total time: min %lld us, max %lld us, avg %lld us", stats.min.easyCATReadTaskTotal, stats.max.easyCATReadTaskTotal, stats.average.easyCATReadTaskTotal);
        //ESP_LOGI("Stats", "EasyCAT write task total time: min %lld us, max %lld us, avg %lld us", stats.min.easyCATWriteTaskTotal, stats.max.easyCATWriteTaskTotal, stats.average.easyCATWriteTaskTotal);
        //ESP_LOGI("Stats", "CANFD read task total time: min %lld us, max %lld us, avg %lld us", stats.min.canfdReadTaskTotal, stats.max.canfdReadTaskTotal, stats.average.canfdReadTaskTotal);
        //ESP_LOGI("Stats", "CANFD write task total time: min %lld us, max %lld us, avg %lld us", stats.min.canfdWriteTaskTotal, stats.max.canfdWriteTaskTotal, stats.average.canfdWriteTaskTotal);
        //ESP_LOGI("Stats", "EasyCAT read task overflow count: %u", stats.easyCATReadTaskOverflowCount);
        //ESP_LOGI("Stats", "EasyCAT write task overflow count: %u", stats.easyCATWriteTaskOverflowCount);
        //ESP_LOGI("Stats", "CANFD read task overflow count: %u", stats.canfdReadTaskOverflowCount);
        //ESP_LOGI("Stats", "CANFD write task overflow count: %u", stats.canfdWriteTaskOverflowCount);
        ESP_LOGI("---", "----------------------------------------");

        // カウント数をリセット
        motor_control_count = 0;
        canfd_send_count = 0;
        for (auto const &tuple : module_id_index_map)
        {
            uint8_t index = tuple.second;
            canfd_send_counts[index] = 0;
            canfd_receive_counts[index] = 0;
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
    }

    TimeLogStats calcStats()
    {
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

        // int32_t overflowCount = 0;
        int32_t easyCATReadTaskOverflowCount = 0;
        int32_t easyCATWriteTaskOverflowCount = 0;
        int32_t canfdReadTaskOverflowCount = 0;
        int32_t canfdWriteTaskOverflowCount = 0;

        if (timeLogIndex == 0)
        {
            return stats;
        }
        int32_t size = ((timeLogIndex + 1) < TIME_LOG_BUFFER_SIZE) ? (timeLogIndex + 1) : TIME_LOG_BUFFER_SIZE;
        for (int i = 0; i < size; i++)
        {
            timeLog[i].easyCATReadTaskTotal = timeLog[i].easyCATRead + timeLog[i].loadTargetValuesFromEtherCAT;
            timeLog[i].canfdReadTaskTotal = timeLog[i].loadRxQueue;

            timeLog[i].easyCATWriteTaskTotal = timeLog[i].easyCATWrite + timeLog[i].loadCurrentValuesFromMotorDrivers + timeLog[i].setCurrentValuesToEasyCATBuffer;
            timeLog[i].canfdWriteTaskTotal = timeLog[i].motorControl;

            if (timeLog[i].easyCATReadTaskTotal > 1000)
            {
                easyCATReadTaskOverflowCount++;
            }

            if (timeLog[i].easyCATWriteTaskTotal > 1000)
            {
                easyCATWriteTaskOverflowCount++;
            }

            if (timeLog[i].canfdReadTaskTotal > 1000)
            {
                canfdReadTaskOverflowCount++;
            }

            if (timeLog[i].canfdWriteTaskTotal > 1000)
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

            // if (timeLog[i].easyCATApplication < easyCATApplicationMin)
            //{
            //     easyCATApplicationMin = timeLog[i].easyCATApplication;
            // }
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
}