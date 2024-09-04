#ifndef CUSTOM_PDO_NAME_H
#define CUSTOM_PDO_NAME_H

//-------------------------------------------------------------------//
//                                                                   //
//     This file has been created by the Easy Configurator tool      //
//                                                                   //
//     Easy Configurator project DS402_PDOs_EasyCatProject.prj
//                                                                   //
//-------------------------------------------------------------------//


#define CUST_BYTE_NUM_OUT	26
#define CUST_BYTE_NUM_IN	26
#define TOT_BYTE_NUM_ROUND_OUT	28
#define TOT_BYTE_NUM_ROUND_IN	28


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
		uint16_t    ControlWord_0x6040;
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
		uint16_t    StatusWord_0x6041;
	}Cust;
} PROCBUFFER_IN;

#endif