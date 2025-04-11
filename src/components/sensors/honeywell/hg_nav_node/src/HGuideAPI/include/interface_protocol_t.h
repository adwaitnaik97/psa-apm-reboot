#ifndef __HGuideAPI_interface_protocol_t_h__
#define __HGuideAPI_interface_protocol_t_h__
#pragma once

// List of Interface protocols
enum interface_protocol_t
{
	protocol_No_Change                     = 0, //  No Change to interface
	protocol_HGNSI_DICD_Formatted_Messages = 1, //  HGNSI Interface
	protocol_Legacy_IMU_Formatted_Messages = 2, //  IMU Message Types
	protocol_NMEA                          = 3, //  NMEA Messages
	protocol_GNSSR_Navstorm                = 4, //  Spare
	protocol_GNSSR_Navfire                 = 5, //  Spare
	protocol_GNSSR_AsteRx_4                = 6, //  Spare
	protocol_GNSSR_AsteRx_M2A              = 7, //  Spare
	protocol_GNSSR_NovAtel_OEM7            = 8, //  Spare
	protocol_GNSSR_Trimble_C1216           = 9, //  Spare
	protocol_GNSSR_uBlox_NEO               = 10, //  Spare
	// [IMU] CAN bus with 11-bit ID 
	// [INS] Reserved protocol_GNSSR_Swift
	protocol_CAN_11_bit                    = 11,
	protocol_HGNSI_CONSOLE                 = 12, //  HGuide Console Commands
	protocol_CAN_29_bit                    = 29 //  [IMU] CAN bus with 29-bit ID
};

#endif // __HGuideAPI_interface_protocol_t_h__
