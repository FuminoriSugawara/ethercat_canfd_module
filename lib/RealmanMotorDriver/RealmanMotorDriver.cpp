#include <cstring>
#include "RealmanMotorDriver.hpp"
using namespace realman_motor_driver;

RealmanMotorDriver::RealmanMotorDriver(uint8_t module_id, std::shared_ptr<CAN_COMMON> can_handler, boolean debug_mode) {
    this->can_handler = can_handler;
    this->module_id = module_id;
    this->IS_DEBUG = debug_mode;
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
    CAN_FRAME_FD response =
        this->transmitMessage(MESSAGE_TYPE_CMD_COMMON, data, 3);
    this->processCANFDMessage(response);
}

void RealmanMotorDriver::loadCurrentState(void)
{
    BytesUnion_FD empty_data;
    CAN_FRAME_FD response =
        this->transmitMessage(MESSAGE_TYPE_CMD_JSTATE, empty_data, 0);
    this->processCANFDMessage(response);
}



void RealmanMotorDriver::setDriverEnabled(void)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x02;
    data.uint8[1] = 0x0A;
    data.uint8[2] = 0x01;
    CAN_FRAME_FD response =
        this->transmitMessage(MESSAGE_TYPE_CMD_COMMON, data, 3);
    this->processCANFDMessage(response);
    this->driver_state = DRIVER_ENABLED;
}

void RealmanMotorDriver::setDriverDisabled(void)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x02;
    data.uint8[1] = 0x0A;
    data.uint8[2] = 0x00;
    CAN_FRAME_FD response =
        this->transmitMessage(MESSAGE_TYPE_CMD_COMMON, data, 3);
    this->processCANFDMessage(response);
    this->driver_state = DRIVER_DISABLED;
}

void RealmanMotorDriver::setConnectionOnline(void)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x02;
    data.uint8[1] = 0x49;
    data.uint8[2] = 0x00;
    CAN_FRAME_FD response =
        this->transmitMessage(MESSAGE_TYPE_CMD_COMMON, data, 3);
    this->processCANFDMessage(response);
    this->connection_state = ONLINE;
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
    CAN_FRAME_FD response =
        this->transmitMessage(MESSAGE_TYPE_CMD_COMMON, data, 3);
    this->processCANFDMessage(response);
    this->control_mode = POSITION_CONTROL;
}

void RealmanMotorDriver::setVelocityControlMode(void)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x02;
    data.uint8[1] = 0x30;
    data.uint8[2] = 0x02;

    CAN_FRAME_FD response =
        this->transmitMessage(MESSAGE_TYPE_CMD_COMMON, data, 3);
    this->processCANFDMessage(response);
    this->control_mode = VELOCITY_CONTROL;
}

void RealmanMotorDriver::setCurrentControlMode(void)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x02;
    data.uint8[1] = 0x30;
    data.uint8[2] = 0x01;
    CAN_FRAME_FD response =
        this->transmitMessage(MESSAGE_TYPE_CMD_COMMON, data, 3);
    this->processCANFDMessage(response);
    this->control_mode = CURRENT_CONTROL;
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
    if (command_index == 0x14)
    { // current position
        this->current_position = bytesToInt32(&message.data.uint8[2]);
        //Serial.printf("Current Position in common message: %f\n", this->current_position);
    }
    else if (command_index == 0x10)
    {
        this->current_torque = bytesToInt32(&message.data.uint8[2]);
        //Serial.printf("current %d\n", current_current);
    }
    else if (command_index == 0x12)
    {
        this->current_velocity = bytesToInt32(&message.data.uint8[2]);
        //Serial.printf("velocity %d\n", current_velocity);
    }
}


void RealmanMotorDriver::processStateMessage(CAN_FRAME_FD &message)
{
    if (message.length != 16) {
        Serial.println("Message length is not 16");
        Serial.println(message.length);
        return;
    }
    //uint16_t error_code = (message.data.uint8[1] << 8) | message.data.uint8[0];
    uint16_t error_code = bytesToUint16(&message.data.uint8[0]);
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
    //this->current_position = current_position;
    Serial.printf("Current Position: %f\n", current_position);
    // float current_current = (message.data.uint8[15] << 24) | (message.data.uint8[14] << 16) | (message.data.uint8[13] << 8) | message.data.uint8[12];
    float current_current = (float) (bytesToUint32(&message.data.uint8[12])) / 1000;
    Serial.printf("Current Current: %f\n", current_current);
}

void RealmanMotorDriver::processServoMessage(CAN_FRAME_FD &message)
{
    if (message.length != 16) {
        return;
    }
    float current_current = (float)(bytesToInt32(&message.data.uint8[0]));
    float current_velocity = (float) (bytesToInt32(&message.data.uint8[4])) / 50;
    float current_position = (float) (bytesToInt32(&message.data.uint8[8])) / 10000.0;
    uint16_t enable_state = bytesToUint16(&message.data.uint8[12]);
    uint16_t error_state = bytesToUint16(&message.data.uint8[14]);
    Serial.printf("Current Current: %f\n", current_current);
    Serial.printf("Current Velocity: %f\n", current_velocity);
    Serial.printf("Current Position: %f\n", current_position);
    Serial.printf("Enable State: 0x%04X\n", enable_state);
    Serial.printf("Error State: 0x%04X\n", error_state);
}


void RealmanMotorDriver::loadCurrentCurrent(void)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x01;
    data.uint8[1] = 0x10;
    data.uint8[2] = 0x02;
    CAN_FRAME_FD response =
        this->transmitMessage(MESSAGE_TYPE_CMD_COMMON, data, 3);
    this->processCANFDMessage(response);
}

void RealmanMotorDriver::loadCurrentVelocity(void)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x01;
    data.uint8[1] = 0x12;
    data.uint8[2] = 0x02;
    CAN_FRAME_FD response =
        this->transmitMessage(MESSAGE_TYPE_CMD_COMMON, data, 3);
    this->processCANFDMessage(response);
}

void RealmanMotorDriver::loadCurrentPosition(void)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x01;
    data.uint8[1] = 0x14;
    data.uint8[2] = 0x02;
    CAN_FRAME_FD response =
        this->transmitMessage(MESSAGE_TYPE_CMD_COMMON, data, 3);
    this->processCANFDMessage(response);
}

float RealmanMotorDriver::getCurrentPosition(void)
{
    return this->current_position;
}

float RealmanMotorDriver::getCurrentVelocity(void)
{
    return this->current_velocity;
}

float RealmanMotorDriver::getCurrentTorque(void)
{
    return this->current_torque;
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
    CAN_FRAME_FD response =
        this->transmitMessage(MESSAGE_TYPE_CMD_POS, data, 4);
    this->processCANFDMessage(response);
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
    data.uint8[0] = 0x02;
    data.uint8[1] = 0x34;
    data.uint8[2] = (target_velocity_int & 0x000000FF);
    data.uint8[3] = (target_velocity_int & 0x0000FF00) >> 8;
    data.uint8[4] = (target_velocity_int & 0x00FF0000) >> 16;
    data.uint8[5] = (target_velocity_int & 0xFF000000) >> 24;
    Serial.println("Target Velocity");
    for (int i = 0; i < 6; i++)
    {
        Serial.printf("0x%02X ", data.uint8[i]);
    }
    CAN_FRAME_FD response = this->transmitMessage(MESSAGE_TYPE_CMD_COMMON, data, 6);
    this->processCANFDMessage(response);
}

void RealmanMotorDriver::setTargetCurrent(int32_t target_current)
{
    
    BytesUnion_FD data;
    int32_t target_current_int = (int32_t) (target_current);
    // For safety, limit the current to 1000
    if (target_current_int > 2000)
    {
        target_current_int = 2000;
    }
    data.uint8[0] = 0x02;
    data.uint8[1] = 0x32;
    data.uint8[2] = (target_current_int & 0x000000FF);
    data.uint8[3] = (target_current_int & 0x0000FF00) >> 8;
    data.uint8[4] = (target_current_int & 0x00FF0000) >> 16;
    data.uint8[5] = (target_current_int & 0xFF000000) >> 24;
    CAN_FRAME_FD response = this->transmitMessage(MESSAGE_TYPE_CMD_COMMON, data, 6);
    this->processCANFDMessage(response);
}

void RealmanMotorDriver::setZeroPosition(void)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x02;
    data.uint8[1] = 0x0E;
    data.uint8[2] = 0x01;
    CAN_FRAME_FD response = this->transmitMessage(MESSAGE_TYPE_CMD_COMMON, data, 3);
    this->processCANFDMessage(response);
}


CAN_FRAME_FD RealmanMotorDriver::transmitMessage(uint16_t message_type, BytesUnion_FD data, uint8_t length)
{
    CAN_FRAME_FD txFrame;
    txFrame.rrs = 0;
    txFrame.id = this->module_id | message_type;
    txFrame.fdMode = 1;
    txFrame.data = data;
    txFrame.length = length;
    this->can_handler->sendFrameFD(txFrame);

    //Get response and return
    CAN_FRAME_FD rxFrame;
    this->can_handler->readFD(rxFrame);
    if (this->IS_DEBUG) {
        Serial.printf("ID: 0x%04X\n", rxFrame.id);
        for (int i = 0; i < rxFrame.length; i++)
        {
            Serial.printf("0x%02X ", rxFrame.data.uint8[i]);
        }
        Serial.printf("\n");
    }
    return rxFrame;
}


void RealmanMotorDriver::processCANFDMessage(CAN_FRAME_FD &message)
{
    uint8_t module_id = (message.id & 0x00FF); // Mask for binary 0000000011111111
    uint16_t message_type = (message.id & 0x0F00); // Mask for binary 0000111100000000
    //Serial.printf("Module ID: 0x%02X Message Type: 0x%04X\n", module_id, message_type);
    //Serial.printf("Module ID: 0x%02X\n", module_id);
    //Serial.printf("driver module ID: 0x%02X\n", this->module_id);
    if (module_id != this->module_id)
    {
        //Serial.printf("Module ID does not match, expected: 0x%02X, received: 0x%02X\n", this->module_id, module_id);
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
        //processServoMessage(message);
        break;
    case MESSAGE_TYPE_CMD_COMMON:
    case MESSAGE_TYPE_CMD_JSTATE:
    case MESSAGE_TYPE_CMD_POS:
    case MESSAGE_TYPE_CMD_VEL:
    case MESSAGE_TYPE_CMD_CUR:
        //Serial.printf("Processing command message\n");
        break;
    };
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