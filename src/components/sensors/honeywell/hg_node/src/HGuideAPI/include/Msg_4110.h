#ifndef __HGuideAPI_Msg_4110_h__
#define __HGuideAPI_Msg_4110_h__
#pragma once

#include <cstdint>

#include <include/odo_body_vel_status_t.h>


// 0x4110 : Odometer Measurement Input Message
//
// The message 0x4110 is used to provide DMI (odometer) or velocity measurements to the HGuide.
// 
class HGUIDE_DLL Msg_4110
{
public:
	Msg_4110();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 96;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x4110; // Message ID
	static const uint32_t MessageLength = 24; // Message Length
	uint32_t Checksum; // Checksum
	
	// [s] positive number to inform about a delay between sensor sampling and message transmission
	// (Maximum adjustment is 0.5s)
	double TimeOfValidityAdjustment;
	double GPS_TOV; // [s] GPS time of validity (Seconds into week)
	float Body_Velocity_X; // Experimental Body Velocity X [m/s] maximum velocity of 200 m/s
	float Body_Velocity_Y; // Experimental Body Velocity Y [m/s] maximum velocity of 200 m/s
	float Body_Velocity_Z; // Experimental Body Velocity Z [m/s] maximum velocity of 200 m/s
	int32_t Odo_Pulses; // The total cumulative sum of the number of pulses.
	float Distance_per_Pulse; // The distance traveled per pulse. [m/pulse]
	odo_body_vel_status_t Odo_Body_Vel_Status; // Configuration Bit field
	uint32_t Message_Counter; // The total number of 0x4110 messages sent. The data will only be used when the message counter increments.
	float X_Y_Velocity_STDV; // Estimated velocity error in Body Velocity X and Y measurements. (1-sigma) [m/s]
	float Z_Velocity_STDV; // Estimated velocity error in Body Velocity Z measurements. (1-sigma) [m/s]
};

#endif // __HGuideAPI_Msg_4110_h__
