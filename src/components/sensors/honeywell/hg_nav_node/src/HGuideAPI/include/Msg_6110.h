#ifndef __HGuideAPI_Msg_6110_h__
#define __HGuideAPI_Msg_6110_h__
#pragma once

#include <cstdint>

#include <include/ins_gnss_summary_t.h>
#include <include/odo_body_vel_status_t.h>


// 0x6110 : Odometer Measurement Output
//
// Sent out as a reply to the 0x4110 DMI message
// 
class HGUIDE_DLL Msg_6110
{
public:
	Msg_6110();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 96;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of message flag
	static const uint32_t MessageId = 0x6110; // Message ID
	static const uint32_t MessageLength = 24; // Message length
	uint32_t Checksum; // Checksum
	double systemTov; // [sec] System Time of Validity
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	float Odo_X_Body_Distance; // [m] Distance travelled X (from DMI input)
	float Odo_Y_Body_Distance; // [m] Distance travelled Y (from DMI input)
	float Odo_Z_Body_Distance; // [m] Distance travelled Z (from DMI input)
	float Odo_X_Body_Distance_Compensated; // RESERVED [m] Compensated distance travelled X
	float Odo_Y_Body_Distance_Compensated; // RESERVED [m] Compensated distance travelled Y
	float Odo_Z_Body_Distance_Compensated; // RESERVED [m] Compensated distance travelled Z
	odo_body_vel_status_t Odo_status; // Mirrored 0x4110 input status
	float uart_latency; // [s] The UART Latency value added by the INS SW
	uint32_t customer_latency; // [s] The value of the TimeOfValidityAdjustment in the 0x4110 message.
	ins_gnss_summary_t InsGnssSummary; // INS / GNSS summary word
};

#endif // __HGuideAPI_Msg_6110_h__
