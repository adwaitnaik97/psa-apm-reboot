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
	// [IMU] CAN bus with 11-bit ID 
	// [INS] Reserved protocol_GNSSR_Swift
	protocol_CAN_11_bit                    = 11,
	protocol_HGNSI_CONSOLE                 = 12, //  HGuide Console Commands
	protocol_CAN_29_bit                    = 29 //  [IMU] CAN bus with 29-bit ID
};

#endif // __HGuideAPI_interface_protocol_t_h__
