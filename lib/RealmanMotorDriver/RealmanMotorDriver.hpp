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

    typedef enum
    {
        POSITION_CONTROL,
        VELOCITY_CONTROL,
        CURRENT_CONTROL
    } CONTROL_MODE;
    class RealmanMotorDriver
    {
    public:
        RealmanMotorDriver(uint8_t module_id, std::shared_ptr<CAN_COMMON> can_handler, boolean debug_mode = false);
        void processCANFDMessage(CAN_FRAME_FD &message);
        void processCommonMessage(CAN_FRAME_FD &message);
        void processStateMessage(CAN_FRAME_FD &message);
        void processServoMessage(CAN_FRAME_FD &message);
        void initialize(void);
        void clearJointError(void);
        void setDriverEnabled(void);
        void setDriverDisabled(void);
        void setConnectionOnline(void);
        void switchControlMode(CONTROL_MODE control_mode);
        void setPositionControlMode(void);
        void setVelocityControlMode(void);
        void setCurrentControlMode(void);
        void setTargetPosition(int32_t target_position);
        void setTargetVelocity(int32_t target_velocity);
        void setTargetCurrent(int32_t target_current);
        void setZeroPosition(void);
        void loadCurrentState(void);
        void loadCurrentCurrent(void);
        void loadCurrentVelocity(void);
        void loadCurrentPosition(void);
        float getCurrentPosition(void);
        float getCurrentVelocity(void);
        float getCurrentTorque(void);

    private:
        CONNECTION_STATE connection_state = OFFLINE;
        DRIVER_STATE driver_state = DRIVER_DISABLED;
        BRAKE_STATE brake_state = BRAKE_ON;
        CONTROL_MODE control_mode = POSITION_CONTROL;
        uint8_t error_state = 0;
        uint8_t module_id;
        float current_position = 0.0;
        float current_velocity = 0.0;
        float current_torque = 0.0;
        boolean IS_DEBUG = false;

        std::shared_ptr<CAN_COMMON> can_handler;
        // データ送信
        CAN_FRAME_FD transmitMessage(uint16_t message_type, BytesUnion_FD data, uint8_t length);
    };
}

#endif