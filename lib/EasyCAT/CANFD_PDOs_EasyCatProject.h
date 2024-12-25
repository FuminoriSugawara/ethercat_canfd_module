#ifndef CUSTOM_PDO_NAME_H
#define CUSTOM_PDO_NAME_H

//-------------------------------------------------------------------//
//                                                                   //
//     This file has been created by the Easy Configurator tool      //
//                                                                   //
//     Easy Configurator project CANFD_PDOs_EasyCatProject.prj
//                                                                   //
//-------------------------------------------------------------------//


#define CUST_BYTE_NUM_OUT	112
#define CUST_BYTE_NUM_IN	112
#define TOT_BYTE_NUM_ROUND_OUT	112
#define TOT_BYTE_NUM_ROUND_IN	112


typedef union												//---- output buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_OUT];
	struct
	{
		int32_t     Position_1;
		int32_t     Position_2;
		int32_t     Position_3;
		int32_t     Position_4;
		int32_t     Position_5;
		int32_t     Position_6;
		int32_t     Position_7;
		int32_t     Position_8;
		int32_t     Velocity_1;
		int32_t     Velocity_2;
		int32_t     Velocity_3;
		int32_t     Velocity_4;
		int32_t     Velocity_5;
		int32_t     Velocity_6;
		int32_t     Velocity_7;
		int32_t     Velocity_8;
		int32_t     Current_1;
		int32_t     Current_2;
		int32_t     Current_3;
		int32_t     Current_4;
		int32_t     Current_5;
		int32_t     Current_6;
		int32_t     Current_7;
		int32_t     Current_8;
		uint16_t    ControlWord_1;
		uint16_t    ControlWord_2;
		uint16_t    ControlWord_3;
		uint16_t    ControlWord_4;
		uint16_t    ControlWord_5;
		uint16_t    ControlWord_6;
		uint16_t    ControlWord_7;
		uint16_t    ControlWord_8;
	}Cust;
} PROCBUFFER_OUT;


typedef union												//---- input buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_IN];
	struct
	{
		int32_t     ActualPosition_1;
		int32_t     ActualPosition_2;
		int32_t     ActualPosition_3;
		int32_t     ActualPosition_4;
		int32_t     ActualPosition_5;
		int32_t     ActualPosition_6;
		int32_t     ActualPosition_7;
		int32_t     ActualPosition_8;
		int32_t     ActualVelocity_1;
		int32_t     ActualVelocity_2;
		int32_t     ActualVelocity_3;
		int32_t     ActualVelocity_4;
		int32_t     ActualVelocity_5;
		int32_t     ActualVelocity_6;
		int32_t     ActualVelocity_7;
		int32_t     ActualVelocity_8;
		int32_t     ActualCurrent_1;
		int32_t     ActualCurrent_2;
		int32_t     ActualCurrent_3;
		int32_t     ActualCurrent_4;
		int32_t     ActualCurrent_5;
		int32_t     ActualCurrent_6;
		int32_t     ActualCurrent_7;
		int32_t     ActualCurrent_8;
		uint16_t    EncoderDiff_1;
		uint16_t    EncoderDiff_2;
		uint16_t    EncoderDiff_3;
		uint16_t    EncoderDiff_4;
		uint16_t    EncoderDiff_5;
		uint16_t    EncoderDiff_6;
		uint16_t    EncoderDiff_7;
		uint16_t    EncoderDiff_8;
	}Cust;
} PROCBUFFER_IN;

#endif