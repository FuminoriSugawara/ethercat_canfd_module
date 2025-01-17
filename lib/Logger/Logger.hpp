#ifndef _LOGGER_HPP
#define _LOGGER_HPP
#include <map>
#include "esp32_can.h"

#define TIME_LOG_BUFFER_SIZE 1000
// このクラスでは、CANFDの送受信カウント数や、各処理にかかる時間を集計する機能を提供する
namespace logger
{

    typedef struct
    {
        int32_t easyCATRead;
        int32_t easyCATWrite;
        int32_t loadTargetValuesFromEtherCAT;
        int32_t motorControl;
        int32_t loadEncoderCount;
        int32_t loadRxQueue;
        int32_t loadCurrentValuesFromMotorDrivers;
        int32_t setCurrentValuesToEasyCATBuffer;
        int32_t easyCATReadTaskTotal;
        int32_t easyCATWriteTaskTotal;
        int32_t canfdReadTaskTotal;
        int32_t canfdWriteTaskTotal;
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

    uint32_t timeLogIndex = 0;
    TimeLog timeLog[TIME_LOG_BUFFER_SIZE];

    TimeLogStats calcStats();


    
    
    template <std::size_t N>
    class Logger
    {
    public:
        explicit Logger(const std::array<uint8_t, N> &module_ids) : module_ids(module_ids) {
            int i = 0;
            for (uint8_t module_id : module_ids)
            {
                id_to_index[module_id] = i;
                canfd_send_counts[i] = 0;
                canfd_receive_counts[i] = 0;
                i++;
            }
            canfd_send_count = 0;
            canfd_receive_count = 0;
            easycat_int_count = 0;
            motor_control_count = 0;
            load_encoder_count_count = 0;
            rx_queue_count = 0;
            odd_count = 0;
            even_count = 0;
        }
        void outputCounts();
        void outputStats();
        void resetCounts();
        void outputProcessingTime();

        uint32_t canfd_send_count;
        uint32_t canfd_receive_count;
        uint32_t easycat_int_count;
        uint32_t motor_control_count;
        uint32_t load_encoder_count_count;
        uint32_t rx_queue_count;
        uint32_t odd_count;
        uint32_t even_count;
        std::array<uint32_t, N> canfd_send_counts;
        std::array<uint32_t, N> canfd_receive_counts;
        std::array<int64_t, N> motor_control_times;
        std::array<int64_t, N> load_encoder_count_times;
        std::array<int, 16> id_to_index;
    private:
        std::array<uint8_t, N>module_ids;
        
    };

    template <std::size_t N>
    void Logger<N>::resetCounts()
    {
        motor_control_count = 0;
        load_encoder_count_count = 0;
        canfd_send_count = 0;
        canfd_receive_count = 0;
        rx_queue_count = 0;
        easycat_int_count = 0;
        timeLogIndex = 0;
        odd_count = 0;
        even_count = 0;
        for (uint8_t module_id : module_ids)
        {
            int index = id_to_index[module_id];
            canfd_send_counts[index] = 0;
            canfd_receive_counts[index] = 0;
            motor_control_times[index] = 0;
            load_encoder_count_times[index] = 0;
        }

        
        
    }

    template <std::size_t N>
    void Logger<N>::outputCounts()
    {
        ESP_LOGI("---", "----------------------------------------");
        ESP_LOGI("main", "EasyCAT interrupt count: %u", easycat_int_count);
        ESP_LOGI("main", "Odd count: %u, Even count: %u", odd_count, even_count);
        ESP_LOGI("main", "Motor control count: %u", motor_control_count);
        ESP_LOGI("main", "Load encoder count count: %u", load_encoder_count_count);
        ESP_LOGI("main", "Send count: %u, Receive count: %u", canfd_send_count, canfd_receive_count);
        ESP_LOGI("main", "RX Queue count: %u", rx_queue_count);
        for (uint8_t module_id : module_ids)
        {
            int index = id_to_index[module_id];
            ESP_LOGI("main", "module_id, %u, Send count, %u, Receive count, %u", module_id, canfd_send_counts[index], canfd_receive_counts[index]);
        }
    }

    template <std::size_t N>
    void Logger<N>::outputProcessingTime()
    {
        for (uint8_t module_id : module_ids)
        {
            int index = id_to_index[module_id];
            float motor_control_time = motor_control_count > 0 ? motor_control_times[index] / motor_control_count : 0;
            float load_encoder_count_time = load_encoder_count_count > 0 ? load_encoder_count_times[index] / load_encoder_count_count : 0;
            ESP_LOGI("motor_process_time", "module_id, %d, Motor control time(us), %d, Load encoder count time(us), %d", module_id, (int) motor_control_time, (int) load_encoder_count_time);
        }
    }

    template <std::size_t N>
    void Logger<N>::outputStats()
    {
        outputProcessingTime();
        TimeLogStats stats = logger::calcStats();
        //ESP_LOGI("Stats", "EasyCAT read time, min(us), %d, max(us), %d, avg(us), %d", stats.min.easyCATRead, stats.max.easyCATRead, stats.average.easyCATRead);
        //ESP_LOGI("Stats", "EasyCAT write time, min(us), %d, max(us), %d, avg(us), %d", stats.min.easyCATWrite, stats.max.easyCATWrite, stats.average.easyCATWrite);
        //ESP_LOGI("Stats", "Load target values from EtherCAT time, min(us), %d, max(us), %d, avg(us), %d", stats.min.loadTargetValuesFromEtherCAT, stats.max.loadTargetValuesFromEtherCAT, stats.average.loadTargetValuesFromEtherCAT);
        ESP_LOGI("Stats", "Motor control time, min(us), %d, max(us), %d, avg(us), %d", stats.min.motorControl, stats.max.motorControl, stats.average.motorControl);
        ESP_LOGI("Stats", "Load encoder count time, min(us), %d, max(us), %d, avg(us), %d", stats.min.loadEncoderCount, stats.max.loadEncoderCount, stats.average.loadEncoderCount);
        //ESP_LOGI("Stats", "Load current values from motor drivers time, min(us), %d, max(us), %d, avg(us), %d", stats.min.loadCurrentValuesFromMotorDrivers, stats.max.loadCurrentValuesFromMotorDrivers, stats.average.loadCurrentValuesFromMotorDrivers);
        //ESP_LOGI("Stats", "Set current values to EasyCAT buffer time, min(us), %d, max(us), %d, avg(us), %d", stats.min.setCurrentValuesToEasyCATBuffer, stats.max.setCurrentValuesToEasyCATBuffer, stats.average.setCurrentValuesToEasyCATBuffer);
        ESP_LOGI("Stats", "Load RX queue time, min(us), %d, max(us), %d, avg(us), %d", stats.min.loadRxQueue, stats.max.loadRxQueue, stats.average.loadRxQueue);
        ESP_LOGI("Stats", "EasyCAT read task total time, min(us), %d, max(us), %d, avg(us), %d", stats.min.easyCATReadTaskTotal, stats.max.easyCATReadTaskTotal, stats.average.easyCATReadTaskTotal);
        ESP_LOGI("Stats", "EasyCAT write task total time, min(us), %d, max(us), %d, avg(us), %d", stats.min.easyCATWriteTaskTotal, stats.max.easyCATWriteTaskTotal, stats.average.easyCATWriteTaskTotal);
        ESP_LOGI("Stats", "CANFD read task total time, min(us), %d, max(us), %d, avg(us), %d", stats.min.canfdReadTaskTotal, stats.max.canfdReadTaskTotal, stats.average.canfdReadTaskTotal);
        ESP_LOGI("Stats", "CANFD write task total time, min(us), %d, max(us), %d, avg(us), %d", stats.min.canfdWriteTaskTotal, stats.max.canfdWriteTaskTotal, stats.average.canfdWriteTaskTotal);
        //ESP_LOGI("Stats", "EasyCAT read task overflow count, %u", stats.easyCATReadTaskOverflowCount);
        //ESP_LOGI("Stats", "EasyCAT write task overflow count, %u", stats.easyCATWriteTaskOverflowCount);
        //ESP_LOGI("Stats", "CANFD read task overflow count, %u", stats.canfdReadTaskOverflowCount);
        //ESP_LOGI("Stats", "CANFD write task overflow count, %u", stats.canfdWriteTaskOverflowCount);

    }

    TimeLogStats calcStats()
    {
        TimeLogStats stats;

        int64_t easyCATReadMin = INT64_MAX;
        int64_t easyCATWriteMin = INT64_MAX;
        int64_t loadTargetValuesFromEtherCATMin = INT64_MAX;
        int64_t motorControlMin = INT64_MAX;
        int64_t loadEncoderCountMin = INT64_MAX;
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
        int64_t loadEncoderCountMax = 0;
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
        int64_t loadEncoderCountSum = 0;
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
            timeLog[i].canfdWriteTaskTotal = timeLog[i].motorControl + timeLog[i].loadEncoderCount;

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
            loadEncoderCountSum += timeLog[i].loadEncoderCount;
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

            if (timeLog[i].loadEncoderCount > 0 && timeLog[i].loadEncoderCount < loadEncoderCountMin)
            {
                loadEncoderCountMin = timeLog[i].loadEncoderCount;
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

            if (timeLog[i].loadEncoderCount > loadEncoderCountMax)
            {
                loadEncoderCountMax = timeLog[i].loadEncoderCount;
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
        int64_t loadEncoderCountAvg = loadEncoderCountSum / size;
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
        stats.average.loadEncoderCount = loadEncoderCountAvg;
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
        stats.min.loadEncoderCount = loadEncoderCountMin;
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
        stats.max.loadEncoderCount = loadEncoderCountMax;
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

    //void consoleLog(
    //    MCP2517FD &CAN1,
    //    std::array<uint32_t, 8> &canfd_send_counts,
    //    std::array<uint32_t, 8> &canfd_receive_counts,
    //    std::map<uint8_t, uint8_t> &module_id_index_map,
    //    std::array<uint16_t, 8> &control_words,
    //    std::array<uint16_t, 8> &status_words,
    //    std::array<int32_t, 8> &target_positions,
    //    std::array<int32_t, 8> &actual_positions
    //    );

    //TimeLogStats calcStats();
}

#endif