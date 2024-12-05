#include <cstring>
#include "RealmanMotorDriver.hpp"
using namespace realman_motor_driver;



uint16_t _bytesToUint16(uint8_t *bytes)
{
    return (bytes[1] << 8) | bytes[0];
}

int16_t _bytesToInt16(uint8_t *bytes)
{
    return (bytes[1] << 8) | bytes[0];
}

uint32_t _bytesToUint32(uint8_t *bytes)
{
    return (bytes[3] << 24) | (bytes[2] << 16) | (bytes[1] << 8) | bytes[0];
}

int32_t _bytesToInt32(uint8_t *bytes)
{
    return (bytes[3] << 24) | (bytes[2] << 16) | (bytes[1] << 8) | bytes[0];
}

RealmanMotorDriver::RealmanMotorDriver(std::shared_ptr<MCP2517FD> can_handler) {
    this->can_handler = can_handler;
}

// arbitration_id から module_idとmessage_typeを取得する
MessageHeader RealmanMotorDriver::_parseArbitrationId(uint16_t arbitration_id)
{
    MessageHeader header;
    header.module_id = arbitration_id & 0x00FF; // Mask for binary 0000000011111111
    header.message_type = arbitration_id & 0x0F00; // Mask for binary 0000111100000000
    return header;
}

void RealmanMotorDriver::_transmitMessage(uint8_t module_id, uint16_t message_type, BytesUnion_FD data, uint8_t length)
{
    CAN_FRAME_FD txFrame;
    txFrame.rrs = 0;
    txFrame.id = module_id | message_type;
    txFrame.fdMode = 1;
    txFrame.data = data;
    txFrame.length = length;
    //this->can_handler->sendFrameFD(txFrame);
    this->can_handler->WriteFrame(txFrame);
}


void RealmanMotorDriver::writeClearJointError(uint8_t module_id)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x02;
    data.uint8[1] = 0x0F;
    data.uint8[2] = 0x01;
    this->_transmitMessage(module_id, MESSAGE_TYPE_CMD_COMMON, data, 3);
}

void RealmanMotorDriver::readCurrentState(uint8_t module_id)
{
    BytesUnion_FD empty_data;
    this->_transmitMessage(module_id, MESSAGE_TYPE_CMD_JSTATE, empty_data, 0);
}



void RealmanMotorDriver::writeDriverEnabled(uint8_t module_id)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x02;
    data.uint8[1] = 0x0A;
    data.uint8[2] = 0x01;
    this->_transmitMessage(module_id, MESSAGE_TYPE_CMD_COMMON, data, 3);
}

void RealmanMotorDriver::writeDriverDisabled(uint8_t module_id)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x02;
    data.uint8[1] = 0x0A;
    data.uint8[2] = 0x00;
    this->_transmitMessage(module_id, MESSAGE_TYPE_CMD_COMMON, data, 3);
}

void RealmanMotorDriver::writeConnectionOnline(uint8_t module_id)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x02;
    data.uint8[1] = 0x49;
    data.uint8[2] = 0x00;
    this->_transmitMessage(module_id, MESSAGE_TYPE_CMD_COMMON, data, 3);
}


void RealmanMotorDriver::writeControlMode(uint8_t module_id, uint8_t control_mode)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x02;
    data.uint8[1] = 0x30;
    data.uint8[2] = control_mode;
    this->_transmitMessage(module_id, MESSAGE_TYPE_CMD_COMMON, data, 3);
}

void RealmanMotorDriver::writePositionControlMode(uint8_t module_id)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x02;
    data.uint8[1] = 0x30;
    data.uint8[2] = 0x03;
    this->_transmitMessage(module_id, MESSAGE_TYPE_CMD_COMMON, data, 3);
}

void RealmanMotorDriver::writeVelocityControlMode(uint8_t module_id)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x02;
    data.uint8[1] = 0x30;
    data.uint8[2] = 0x02;
    this->_transmitMessage(module_id, MESSAGE_TYPE_CMD_COMMON, data, 3);
}

void RealmanMotorDriver::writeCurrentControlMode(uint8_t module_id)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x02;
    data.uint8[1] = 0x30;
    data.uint8[2] = 0x01;
    this->_transmitMessage(module_id, MESSAGE_TYPE_CMD_COMMON, data, 3);
}


CommonResponse RealmanMotorDriver::processCommonMessage(CAN_FRAME_FD &message)
{
    uint8_t operation_type = message.data.uint8[0];
    uint8_t command_index = message.data.uint8[1];
    CommonResponse response;

    response.operation_type = operation_type;
    response.command_index = command_index;

    // data のサイズによって処理を分岐
    if (message.length == 6)
    {
        response.data.int32 = _bytesToInt32(&message.data.uint8[2]);
    }
    else
    {
        response.data.int16[0] = _bytesToInt16(&message.data.uint8[2]);
    }

    return response;
}

JointStateResponse RealmanMotorDriver::processStateMessage(CAN_FRAME_FD &message)
{
    auto header = this->_parseArbitrationId(message.id);
    JointStateResponse response;
    response.module_id = header.module_id;
    response.message_type = header.message_type;
    response.timestamp = message.timestamp;
    response.error_code = _bytesToUint16(&message.data.uint8[0]);
    response.system_voltage = _bytesToInt32(&message.data.uint8[2]);
    response.system_temperature = _bytesToInt32(&message.data.uint8[4]);
    response.enable_state = message.data.uint8[6];
    response.brake_state = message.data.uint8[7];
    response.position = _bytesToInt32(&message.data.uint8[8]);
    response.current = _bytesToInt32(&message.data.uint8[12]);
    return response;

}

ServoResponse RealmanMotorDriver::processServoMessage(CAN_FRAME_FD &message)
{
    auto header = this->_parseArbitrationId(message.id);
    ServoResponse response;
    response.module_id = header.module_id;
    response.message_type = header.message_type;
    response.timestamp = message.timestamp;
    response.current = _bytesToInt32(&message.data.uint8[0]);
    response.velocity = _bytesToInt32(&message.data.uint8[4]);
    response.position = _bytesToInt32(&message.data.uint8[8]);
    response.enable_state = _bytesToUint16(&message.data.uint8[12]);
    response.error_code = _bytesToUint16(&message.data.uint8[14]);
    return response;

}


void RealmanMotorDriver::readCurrentCurrent(uint8_t module_id)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x01;
    data.uint8[1] = 0x10;
    data.uint8[2] = 0x02;
    this->_transmitMessage(module_id, MESSAGE_TYPE_CMD_COMMON, data, 3);
}

void RealmanMotorDriver::readCurrentVelocity(uint8_t module_id)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x01;
    data.uint8[1] = 0x12;
    data.uint8[2] = 0x02;
    this->_transmitMessage(module_id, MESSAGE_TYPE_CMD_COMMON, data, 3);
}

void RealmanMotorDriver::readCurrentPosition(uint8_t module_id)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x01;
    data.uint8[1] = 0x14;
    data.uint8[2] = 0x02;
    this->_transmitMessage(module_id, MESSAGE_TYPE_CMD_COMMON, data, 3);
}


void RealmanMotorDriver::writeTargetPosition(uint8_t module_id, int32_t target_position)
{
    BytesUnion_FD data;

    data.uint8[0] = (target_position & 0x000000FF);
    data.uint8[1] = (target_position & 0x0000FF00) >> 8;
    data.uint8[2] = (target_position & 0x00FF0000) >> 16;
    data.uint8[3] = (target_position & 0xFF000000) >> 24;

    this->_transmitMessage(module_id, MESSAGE_TYPE_CMD_POS, data, 4);
}

void RealmanMotorDriver::writeTargetVelocity(uint8_t module_id, int32_t target_velocity)
{
    BytesUnion_FD data;

    data.uint8[0] = (target_velocity & 0x000000FF);
    data.uint8[1] = (target_velocity & 0x0000FF00) >> 8;
    data.uint8[2] = (target_velocity & 0x00FF0000) >> 16;
    data.uint8[3] = (target_velocity & 0xFF000000) >> 24;

    this->_transmitMessage(module_id, MESSAGE_TYPE_CMD_VEL, data, 4);
}

void RealmanMotorDriver::writeTargetCurrent(uint8_t module_id, int32_t target_current)
{
    BytesUnion_FD data;
    
    data.uint8[0] = (target_current & 0x000000FF);
    data.uint8[1] = (target_current & 0x0000FF00) >> 8;
    data.uint8[2] = (target_current & 0x00FF0000) >> 16;
    data.uint8[3] = (target_current & 0xFF000000) >> 24;
    
    this->_transmitMessage(module_id, MESSAGE_TYPE_CMD_CUR, data, 4);
}

void RealmanMotorDriver::writeZeroPosition(uint8_t module_id)
{
    BytesUnion_FD data;
    data.uint8[0] = 0x02;
    data.uint8[1] = 0x0E;
    data.uint8[2] = 0x01;
    this->_transmitMessage(module_id, MESSAGE_TYPE_CMD_COMMON, data, 3);
}


ParsedResponse RealmanMotorDriver::parseCANFDMessage(CAN_FRAME_FD &message)
{
    auto message_header = this->_parseArbitrationId(message.id);
    uint16_t message_type = message_header.message_type;

    ParsedResponse response;
    
    switch (message_type)
    {
    case MESSAGE_TYPE_RES_COMMON:
        response.common_response = this->processCommonMessage(message);
        break;
    case MESSAGE_TYPE_RES_JSTATE:
        response.joint_state_response = this->processStateMessage(message);
        break;
    case MESSAGE_TYPE_RES_SERVO:
        response.servo_response = this->processServoMessage(message);
        break;
    case MESSAGE_TYPE_CMD_COMMON:
    case MESSAGE_TYPE_CMD_JSTATE:
    case MESSAGE_TYPE_CMD_POS:
    case MESSAGE_TYPE_CMD_VEL:
    case MESSAGE_TYPE_CMD_CUR:
        break;
    };

    return response;

}