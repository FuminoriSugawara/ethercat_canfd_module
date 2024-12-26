#include <cstring>
#include <cmath>
#include "RealmanMotorDriver.hpp"
using namespace realman_motor_driver;

RealmanMotorDriver::RealmanMotorDriver(uint8_t module_id, std::shared_ptr<MCP2517FD> can_handler, boolean debug_mode, boolean dry_run) {
    this->can_handler = can_handler;
    this->module_id = module_id;
    this->IS_DEBUG = debug_mode;
    this->DRY_RUN = dry_run;
}


void RealmanMotorDriver::initialize(void)
{

    this->setConnectionOnline();
    this->clearJointError();
}

void RealmanMotorDriver::clearJointError(void)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x02;
    data.uint8[1] = 0x0F;
    data.uint8[2] = 0x01;
    this->transmitMessage(MESSAGE_TYPE_CMD_COMMON, data, 3);
}

void RealmanMotorDriver::loadState(void)
{
    BytesUnion_FD empty_data;
    this->transmitMessage(MESSAGE_TYPE_CMD_JSTATE, empty_data, 0);
}



void RealmanMotorDriver::setDriverEnabled(void)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x02;
    data.uint8[1] = 0x0A;
    data.uint8[2] = 0x01;
    this->transmitMessage(MESSAGE_TYPE_CMD_COMMON, data, 3);
    this->driver_state = DRIVER_ENABLED;
}

void RealmanMotorDriver::setDriverDisabled(void)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x02;
    data.uint8[1] = 0x0A;
    data.uint8[2] = 0x00;
    this->transmitMessage(MESSAGE_TYPE_CMD_COMMON, data, 3);
    this->driver_state = DRIVER_DISABLED;
}

void RealmanMotorDriver::setConnectionOnline(void)
{
    Serial.printf("ID: 0x%02X, IAP update start\n", this->module_id);
    BytesUnion_FD data;
    data.uint8[0] = 0x02;
    data.uint8[1] = 0x49;
    data.uint8[2] = 0x00;
    this->transmitMessage(MESSAGE_TYPE_CMD_COMMON, data, 3);
}

void RealmanMotorDriver::switchControlMode(CONTROL_MODE control_mode)
{
    switch (control_mode)
    {
    case POSITION_CONTROL:
        this->setPositionControlMode();
        break;
    case VELOCITY_CONTROL:
        this->setVelocityControlMode();
        break;
    case CURRENT_CONTROL:
        this->setCurrentControlMode();
        break;
    };
}

void RealmanMotorDriver::setPositionControlMode(void)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x02;
    data.uint8[1] = 0x30;
    data.uint8[2] = 0x03;
    this->transmitMessage(MESSAGE_TYPE_CMD_COMMON, data, 3);
}

void RealmanMotorDriver::setVelocityControlMode(void)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x02;
    data.uint8[1] = 0x30;
    data.uint8[2] = 0x02;
    this->transmitMessage(MESSAGE_TYPE_CMD_COMMON, data, 3);
}

void RealmanMotorDriver::setCurrentControlMode(void)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x02;
    data.uint8[1] = 0x30;
    data.uint8[2] = 0x01;
    this->transmitMessage(MESSAGE_TYPE_CMD_COMMON, data, 3);
}

uint16_t bytesToUint16(uint8_t *bytes)
{
    return (bytes[1] << 8) | bytes[0];
}

uint32_t bytesToUint32(uint8_t *bytes)
{
    return (bytes[3] << 24) | (bytes[2] << 16) | (bytes[1] << 8) | bytes[0];
}

int32_t bytesToInt32(uint8_t *bytes)
{
    return (bytes[3] << 24) | (bytes[2] << 16) | (bytes[1] << 8) | bytes[0];
}


void RealmanMotorDriver::processCommonMessage(CAN_FRAME_FD &message)
{
    //for (int i = 0; i < message.length; i++)
    //{
    //    Serial.printf("0x%02X ", message.data.uint8[i]);
    //}
    uint8_t command_type = message.data.uint8[0];
    uint8_t command_index = message.data.uint8[1];
    //Serial.printf("Command Type: 0x%02X, Command Index: 0x%02X\n", command_type, command_index);
    
    if (command_index == 0x10) // current current
    {
        this->current = bytesToInt32(&message.data.uint8[2]);
        //Serial.printf("current %d\n", current_current);
    }
    else if (command_index == 0x12) // current velocity
    {
        this->velocity = bytesToInt32(&message.data.uint8[2]);
        //Serial.printf("velocity %d\n", current_velocity);
    }
    else if (command_index == 0x14) // current output shaft position
    { 
        this->output_shaft_position = bytesToInt32(&message.data.uint8[2]);
        //Serial.printf("Current Position in common message: %d\n", this->current_output_shaft_position);
    }
    else if (command_index == 0x16)
    {
        this->motor_shaft_position = bytesToInt32(&message.data.uint8[2]);
    }
    else if (command_index == 0x26)
    {
        this->output_shaft_encoder_count = bytesToInt32(&message.data.uint8[2]);
        this->motor_shaft_encoder_count = bytesToInt32(&message.data.uint8[6]);
    }
    else if (command_index == 0x49) // IAP update success
    {
        Serial.printf("ID: 0x%02X, IAP update success\n", this->module_id);
        this->connection_state = ONLINE;
    }
    else if (command_index == 0x30) // Control mode
    {
        uint8_t control_mode = message.data.uint8[2];
        if (control_mode == 0x01)
        {
            this->control_mode = CURRENT_CONTROL;
        }
        else if (control_mode == 0x02)
        {
            this->control_mode = VELOCITY_CONTROL;
        }
        else if (control_mode == 0x03)
        {
            this->control_mode = POSITION_CONTROL;
        }
    }
}


void RealmanMotorDriver::processStateMessage(CAN_FRAME_FD &message)
{
    if (message.length != 16) {
        Serial.println("Message length is not 16");
        Serial.println(message.length);
        return;
    }
    Serial.println("-----------------");
    //uint16_t error_code = (message.data.uint8[1] << 8) | message.data.uint8[0];
    uint16_t error_code = bytesToUint16(&message.data.uint8[0]);
    this->error_state = error_code;
    Serial.printf("Error Code: 0x%04X\n", error_code);
    //uint16_t system_voltage = (message.data.uint8[3] << 8) | message.data.uint8[2];
    // uint16_t system_voltage = bytesToUint16(&message.data.uint8[2]);
    float system_voltage = (float) (bytesToUint16(&message.data.uint8[2])) / 100;
    Serial.printf("System Voltage: %f\n", system_voltage);
    //uint16_t system_temperature = (message.data.uint8[5] << 8) | message.data.uint8[4];
    float system_temperature = (float) (bytesToUint16(&message.data.uint8[4])) / 10;
    Serial.printf("System Temperature: %f\n", system_temperature);
    uint8_t enable_state = message.data.uint8[6];
    Serial.printf("Enable State: %d\n", enable_state);
    uint8_t brake_state = message.data.uint8[7];
    Serial.printf("Brake State: %d\n", brake_state);
    // float current_position = (message.data.uint8[11] << 24) | (message.data.uint8[10] << 16) | (message.data.uint8[9] << 8) | message.data.uint8[8];
    float current_position = (float) (bytesToInt32(&message.data.uint8[8])) / 10000;
    Serial.printf("Current Position: %f\n", current_position);
    // float current_current = (message.data.uint8[15] << 24) | (message.data.uint8[14] << 16) | (message.data.uint8[13] << 8) | message.data.uint8[12];
    float current_current = (float) (bytesToInt32(&message.data.uint8[12])) / 1000;
    Serial.printf("Current Current: %f\n", current_current);
}

void RealmanMotorDriver::processServoMessage(CAN_FRAME_FD &message)
{
    if (message.length != 16) {
        return;
    }
    this->current = bytesToInt32(&message.data.uint8[0]);
    this->velocity = bytesToInt32(&message.data.uint8[4]);
    this->output_shaft_position = bytesToInt32(&message.data.uint8[8]);
    this->error_state = bytesToUint16(&message.data.uint8[14]);
    //uint16_t enable_state = bytesToUint16(&message.data.uint8[12]);
    //Serial.printf("Current Current: %f\n", current_current);
    //Serial.printf("Current Velocity: %f\n", current_velocity);
    //Serial.printf("Current Position: %f\n", current_position);
    //Serial.printf("Enable State: 0x%04X\n", enable_state);
    //Serial.printf("Error State: 0x%04X\n", error_state);
}


void RealmanMotorDriver::loadCurrent(void)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x01;
    data.uint8[1] = 0x10;
    data.uint8[2] = 0x02;
    this->transmitMessage(MESSAGE_TYPE_CMD_COMMON, data, 3);
}

void RealmanMotorDriver::loadVelocity(void)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x01;
    data.uint8[1] = 0x12;
    data.uint8[2] = 0x02;
    this->transmitMessage(MESSAGE_TYPE_CMD_COMMON, data, 3);
}

void RealmanMotorDriver::loadOutputShaftPosition(void)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x01;
    data.uint8[1] = 0x14;
    data.uint8[2] = 0x02;
    this->transmitMessage(MESSAGE_TYPE_CMD_COMMON, data, 3);
}

void RealmanMotorDriver::loadMotorShaftPosition(void)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x01;
    data.uint8[1] = 0x16;
    data.uint8[2] = 0x02;
    this->transmitMessage(MESSAGE_TYPE_CMD_COMMON, data, 3);
}

void RealmanMotorDriver::loadEncoderCount(void)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x01;
    data.uint8[1] = 0x26;
    data.uint8[2] = 0x05;
    this->transmitMessage(MESSAGE_TYPE_CMD_COMMON, data, 3);
}


int32_t RealmanMotorDriver::getOutputShaftPosition(void)
{
    return this->output_shaft_position;
}

int32_t RealmanMotorDriver::getVelocity(void)
{
    return this->velocity;
}

int32_t RealmanMotorDriver::getCurrent(void)
{
    return this->current;
}

uint16_t RealmanMotorDriver::getErrorState(void)
{
    return this->error_state;
}

bool RealmanMotorDriver::getConnectionState(void)
{
    return (this->connection_state == ONLINE);
}

int32_t RealmanMotorDriver::getMotorShaftEncoderCount(void)
{
    return this->motor_shaft_encoder_count;
}

int32_t RealmanMotorDriver::getOutputShaftEncoderCount(void)
{
    return this->output_shaft_encoder_count;
}

int16_t RealmanMotorDriver::getDifferenceBetweenMotorAndOutputShaftPosition(void)
{
    // モーター側のエンコーダ分解能
    int32_t motor_shaft_encoder_resolution = 66636;
    // 出力軸側のエンコーダ分解能
    int32_t output_shaft_encoder_resolution = 262144;
    // 減速比の設定（あとで修正する）
    double reduce_ratio = 100.0;
    switch(this->module_id)
    {
        case 0x01:
            reduce_ratio = 100.0;
            break;
        case 0x02:
        case 0x03:
        case 0x04:
        case 0x05:
        case 0x06:
        case 0x07:
            reduce_ratio = 80.0;
            break;
        default:
            reduce_ratio = 80.0;
            break;
    }
    double ratio = round(motor_shaft_encoder_resolution * reduce_ratio / output_shaft_encoder_resolution);

    double diff = this->motor_shaft_encoder_count / ratio - this->output_shaft_encoder_count;
    
    return static_cast<int16_t>(diff);
}

void RealmanMotorDriver::setTargetPosition(int32_t target_position)
{
    //if (this->control_mode != CONTROL_MODE::POSITION_CONTROL)
    //{
    //    Serial.println("State is not position control");
    //    return;
    //}
    BytesUnion_FD data;
    uint32_t target_position_int = (uint32_t) (target_position);
    data.uint8[0] = (target_position_int & 0x000000FF);
    data.uint8[1] = (target_position_int & 0x0000FF00) >> 8;
    data.uint8[2] = (target_position_int & 0x00FF0000) >> 16;
    data.uint8[3] = (target_position_int & 0xFF000000) >> 24;
    // Serial.println("Target Position");
    // for (int i = 0; i < 4; i++)
    // {
    //     Serial.printf("0x%02X ", data.uint8[i]);
    // }
    if (this->DRY_RUN)
    {
        this->transmitDummyMessage(MESSAGE_TYPE_CMD_COMMON, data, 6);
        return;
    }
    this->transmitMessage(MESSAGE_TYPE_CMD_POS, data, 4);
}

void RealmanMotorDriver::setTargetVelocity(int32_t target_velocity)
{
    //if (this->control_mode != CONTROL_MODE::VELOCITY_CONTROL)
    //{
    //    Serial.println("State is not velocity control");
    //    return;
    //}
    BytesUnion_FD data;
    int32_t target_velocity_int = (int32_t) (target_velocity);
    data.uint8[0] = (target_velocity_int & 0x000000FF);
    data.uint8[1] = (target_velocity_int & 0x0000FF00) >> 8;
    data.uint8[2] = (target_velocity_int & 0x00FF0000) >> 16;
    data.uint8[3] = (target_velocity_int & 0xFF000000) >> 24;
    //Serial.println("Target Velocity");
    //for (int i = 0; i < 6; i++)
    //{
    //    Serial.printf("0x%02X ", data.uint8[i]);
    //}
    if (this->DRY_RUN)
    {
        this->transmitDummyMessage(MESSAGE_TYPE_CMD_COMMON, data, 6);
        return;
    }
    this->transmitMessage(MESSAGE_TYPE_CMD_VEL, data, 4);
}

void RealmanMotorDriver::setTargetCurrent(int32_t target_current)
{
    
    BytesUnion_FD data;
    int32_t target_current_int = (int32_t) (target_current);
    
    data.uint8[0] = (target_current_int & 0x000000FF);
    data.uint8[1] = (target_current_int & 0x0000FF00) >> 8;
    data.uint8[2] = (target_current_int & 0x00FF0000) >> 16;
    data.uint8[3] = (target_current_int & 0xFF000000) >> 24;
    if (this->DRY_RUN)
    {
        this->transmitDummyMessage(MESSAGE_TYPE_CMD_COMMON, data, 6);
        return;
    }
    this->transmitMessage(MESSAGE_TYPE_CMD_CUR, data, 4);
}

void RealmanMotorDriver::setZeroPosition(void)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x02;
    data.uint8[1] = 0x0E;
    data.uint8[2] = 0x01;
    this->transmitMessage(MESSAGE_TYPE_CMD_COMMON, data, 3);
}


void RealmanMotorDriver::transmitMessage(uint16_t message_type, BytesUnion_FD data, uint8_t length)
{
    CAN_FRAME_FD txFrame;
    txFrame.rrs = 0;
    txFrame.id = this->module_id | message_type;
    txFrame.fdMode = 1;
    txFrame.data = data;
    txFrame.length = length;
    //this->can_handler->sendFrameFD(txFrame);
    this->can_handler->WriteFrame(txFrame);
}

void RealmanMotorDriver::transmitDummyMessage(uint16_t message_type, BytesUnion_FD data, uint8_t length)
{
    CAN_FRAME_FD txFrame;
    txFrame.rrs = 0;
    txFrame.id = 0x00 | message_type;
    txFrame.fdMode = 1;
    txFrame.data = data;
    txFrame.length = length;
    this->can_handler->sendFrameFD(txFrame);
}



void RealmanMotorDriver::transmitDebugMessage(CAN_FRAME_FD &message)
{
    CAN_FRAME_FD response;
    response.id = 0x700;
    uint8_t combined_data[64];
    int index =0;
    combined_data[index++] = (message.id >> 8) & 0xFF;
    combined_data[index++] = message.id & 0xFF;
    // 元のメッセージデータをコピー
    memcpy(combined_data + index, message.data.uint8, message.length);
    index += message.length;

    // responseの設定
    response.length = index;
    memcpy(response.data.uint8, combined_data, index);
    response.fdMode = 1;
    response.rrs = 0;
    this->can_handler->sendFrameFD(response);
}


void RealmanMotorDriver::processCANFDMessage(CAN_FRAME_FD &message)
{
    uint8_t module_id = (message.id & 0x00FF); // Mask for binary 0000000011111111
    uint16_t message_type = (message.id & 0x0F00); // Mask for binary 0000111100000000
    // Serial.printf("Module ID: 0x%02X Message Type: 0x%04X\n", module_id, message_type);
    // Serial.printf("Module ID: 0x%02X\n", module_id);
    // Serial.printf("driver module ID: 0x%02X\n", this->module_id);
    if (module_id != this->module_id)
    {
        Serial.printf("Module ID does not match, expected: 0x%02X, received: 0x%02X\n", this->module_id, module_id);
        return;
    }
    //Serial.printf("Processing message\n");
    
    switch (message_type)
    {
    case MESSAGE_TYPE_RES_COMMON:
        //Serial.printf("Processing response message for common\n");
        processCommonMessage(message);
        break;
    case MESSAGE_TYPE_RES_JSTATE:
        //Serial.printf("Processing response message for state\n");
        processStateMessage(message);
        break;
    case MESSAGE_TYPE_RES_SERVO:
        //Serial.printf("Processing response message for servo\n");
        processServoMessage(message);
        break;
    case MESSAGE_TYPE_CMD_COMMON:
    case MESSAGE_TYPE_CMD_JSTATE:
    case MESSAGE_TYPE_CMD_POS:
    case MESSAGE_TYPE_CMD_VEL:
    case MESSAGE_TYPE_CMD_CUR:
        Serial.printf("Other command message\n");
        //Serial.printf("Processing command message\n");
        break;
    };

    if (this->IS_DEBUG) {
        this->transmitDebugMessage(message);
    }
    
    //Serial.printf("Response sent 0x%04X\n", response.id);
    //Serial.printf("ID: 0x%02X\n", message.id);
    //Serial.printf("Message Type: 0x%04X\n", message_type);
    //for (int i = 0; i < message.length; i++)
    //{
    //    Serial.printf("0x%02X ", message.data.uint8[i]);
    //}
    
    // CAN_FRAME_FD txFrame;
    // txFrame.rrs = 0;
    // txFrame.id = 0x099;
    // txFrame.fdMode = 1;
    // txFrame.length = message.length;
    // for (int i = 0; i < message.length; i++)
    // {
    //     txFrame.data.int8[i] = message.data.int8[i];
    // }
    // //CAN1.sendFrameFD(txFrame);
    // this->can_handler->sendFrameFD(txFrame);
    // Serial.printf("Processing message\n");
    // Serial.printf("ID: 0x%04X\n", message.id);
    // Serial.printf("Module ID: 0x%04X\n", module_id);
    // Serial.printf("Length: %d\n", message.length);
    // Serial.printf("Data: ");
    // for (int i = 0; i < message.length; i++)
    // {
    //     Serial.printf("0x%02X ", message.data.int8[i]);
    // }
    // Serial.printf("\n");
}