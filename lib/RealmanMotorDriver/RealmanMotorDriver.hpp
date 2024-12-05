#ifndef _REALMAN_MOTOR_DRIVER_HPP
#define _REALMAN_MOTOR_DRIVER_HPP
#include <memory>
#include "esp32_can.h"

// **********************************************************
// * definitions
// **********************************************************
// ***************************************
// * message type
// ***************************************
#define MESSAGE_TYPE_RES_COMMON (0x0100) // 共通コマンド応答
#define MESSAGE_TYPE_RES_SERVO (0x0500)  // サーボコマンド応答
#define MESSAGE_TYPE_RES_JSTATE (0x0700) // ジョイントステータス応答
#define MESSAGE_TYPE_CMD_COMMON (0x0000) // 共通コマンド
#define MESSAGE_TYPE_CMD_POS (0x0200)    // 位置サーボコマンド
#define MESSAGE_TYPE_CMD_VEL (0x0300)    // 速度サーボコマンド
#define MESSAGE_TYPE_CMD_CUR (0x0400)    // 電流サーボコマンド
#define MESSAGE_TYPE_CMD_JSTATE (0x0600) // ジョイントステータス要求
#define MESSAGE_TYPE_RES_DEBUG (0x0900) // デバッグ情報応答

// ***************************************
// * servo mode
// ***************************************
#define RMTR_SERVO_MODE_OPN (0x00) // 開ループ
#define RMTR_SERVO_MODE_CUR (0x01) // 電流制御モード
#define RMTR_SERVO_MODE_VEL (0x02) // 速度制御モード
#define RMTR_SERVO_MODE_POS (0x03) // 位置制御モード

// ***************************************
// * status value
// ***************************************
#define RMTR_STATUS_OFF (0)
#define RMTR_STATUS_ON (1)

// ***************************************
// * send value
// ***************************************
#define RMTR_SEND_OFF (0)
#define RMTR_SEND_ON (1)

// ***************************************
// * check value
// ***************************************
#define RMTR_CHECK_NG (0)
#define RMTR_CHECK_OK (1)

// ***************************************
// * step value
// ***************************************
#define RMTR_STEP_STAY (0)
#define RMTR_STEP_DONE (1)
#define RMTR_STEP_ERRO (-1)

// ***************************************
// * state machine
// ***************************************


namespace realman_motor_driver
{
    typedef enum
    {
        OFFLINE,
        ONLINE,
    } CONNECTION_STATE;

    typedef enum
    {
        DRIVER_DISABLED,
        DRIVER_ENABLED
    } DRIVER_STATE;

    typedef enum
    {
        BRAKE_OFF,
        BRAKE_ON
    } BRAKE_STATE;

    typedef enum: uint8_t
    {
        POSITION_CONTROL = 0x03, // 0x03
        VELOCITY_CONTROL = 0x02, // 0x02
        CURRENT_CONTROL = 0x01, // 0x01
        OPEN_CONTROL = 0x00,// 0x00
    } CONTROL_MODE;

    typedef union ControlWord {
        uint16_t word;
        struct
        {
            bool enable : 1;
            bool reserved_1 : 3;
            uint8_t control_mode : 2;
            bool reserved_2 : 2;
            uint8_t module_id : 4;
            bool reserved_3 : 4;
        } bits;

    } ControlWord;

    typedef union StatusWord {
        uint16_t word;
        struct
        {
            bool enable : 1;
            bool brake : 1;
            bool error : 1;
            bool reserved_1 : 1;
            uint8_t control_mode : 2;
            bool reserved_2 : 2;
            bool reserved_3 : 4;
            bool reserved_4 : 4;
        } bits;
    } StatusWord;

    typedef struct MessageHeader
    {
        uint8_t module_id;
        uint16_t message_type;
    } MessageHeader;

    
    typedef struct ServoResponse
    {
        uint8_t module_id;
        uint16_t message_type;
        uint32_t timestamp;
        int32_t current;
        int32_t velocity;
        int32_t position;
        uint16_t enable_state;
        uint16_t error_code;
    } ServoResponse;

    typedef struct JointStateResponse
    {
        uint8_t module_id;
        uint16_t message_type;
        uint32_t timestamp;
        uint16_t error_code;
        int16_t system_voltage;
        int16_t system_temperature;
        bool enable_state;
        bool brake_state;
        int32_t position;
        int32_t current;
    } JointStateResponse;

    typedef union CommonResponseData
    {
        uint8_t uint8[4];
        int8_t int8[4];
        uint16_t uint16[2];
        int16_t int16[2];
        uint32_t uint32;
        int32_t int32;
    } CommonResponseData;

    typedef struct CommonResponse
    {
        uint8_t operation_type;
        uint8_t command_index;
        CommonResponseData data;
    } CommonResponse;

    typedef union ParsedResponse
    {
        ServoResponse servo_response;
        JointStateResponse joint_state_response;
        CommonResponse common_response;
    } ParsedResponse;
    
    class RealmanMotorDriver
    {
    public:
        RealmanMotorDriver(std::shared_ptr<MCP2517FD> can_handler);
        ParsedResponse parseCANFDMessage(CAN_FRAME_FD &message);
        CommonResponse processCommonMessage(CAN_FRAME_FD &message);
        JointStateResponse processStateMessage(CAN_FRAME_FD &message);
        ServoResponse processServoMessage(CAN_FRAME_FD &message);
        void writeClearJointError(uint8_t module_id);
        void writeDriverEnabled(uint8_t module_id);
        void writeDriverDisabled(uint8_t module_id);
        void writeConnectionOnline(uint8_t module_id);
        void writeControlMode(uint8_t module_id, uint8_t control_mode);
        void writePositionControlMode(uint8_t module_id);
        void writeVelocityControlMode(uint8_t module_id);
        void writeCurrentControlMode(uint8_t module_id);
        void writeTargetPosition(uint8_t module_id, int32_t target_position);
        void writeTargetVelocity(uint8_t module_id, int32_t target_velocity);
        void writeTargetCurrent(uint8_t module_id, int32_t target_current);
        void writeZeroPosition(uint8_t module_id);
        void readCurrentState(uint8_t module_id);
        void readCurrentCurrent(uint8_t module_id);
        void readCurrentVelocity(uint8_t module_id);
        void readCurrentPosition(uint8_t module_id);


    private:
        std::shared_ptr<MCP2517FD> can_handler;
        // データ送信
        void _transmitMessage(uint8_t module_id, uint16_t message_type, BytesUnion_FD data, uint8_t length);
        MessageHeader _parseArbitrationId(uint16_t arbitration_id);

    };
}

#endif