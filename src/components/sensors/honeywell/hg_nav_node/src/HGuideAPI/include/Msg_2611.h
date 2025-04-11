#ifndef __HGuideAPI_Msg_2611_h__
#define __HGuideAPI_Msg_2611_h__
#pragma once

#include <cstdint>


// 0x2611 : Built In Test (BIT) History Output
//
// When a 0x2611 message is requested the entire fault history 
// stored in memory will be sent out via consecutive 0x2611
// messages.
// 
class HGUIDE_DLL Msg_2611
{
public:
	Msg_2611();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 228;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x2611; // Message ID
	static const uint32_t MessageLength = 57; // Message Length
	uint32_t Checksum; // Checksum
	uint32_t Last_Message; // If equal to 1, then the last 0x2611 message to be sent. The entire BIT history has been provided.
	int32_t messageNumber_a; // Fault history record #
	int32_t faultID_a; // Fault ID
	int32_t BITmode_a; // BIT mode when fault was logged.
	int32_t systemMode_a; // System mode when fault was logged.
	uint32_t powerCycleCount_a; // Power cycle count when fault was logged.
	int32_t faultValue_a; // The value of the fault criteria when fault was logged.
	int32_t channelSubsystem_a; // Not used
	float eti_a; // Elapsed Time when fault was logged.
	float angularRateX_a; // Angular rate when fault was logged.
	float angularRateY_a; // Angular rate when fault was logged.
	float angularRateZ_a; // Angular rate when fault was logged.
	float accelerationX_a; // Acceleration when fault was logged.
	float accelerationY_a; // Acceleration when fault was logged.
	float accelerationZ_a; // Acceleration when fault was logged.
	float velocityX_a; // Velocity when fault was logged.
	float velocityY_a; // Velocity when fault was logged.
	float velocityZ_a; // Velocity when fault was logged.
	float insTemperature_a; // Temperature when fault was logged.
	float altitude_a; // Altitude when fault was logged.
	int32_t messageNumber_b; // Fault history record #
	int32_t faultID_b; // Fault ID
	int32_t BITmode_b; // BIT mode when fault was logged.
	int32_t systemMode_b; // System mode when fault was logged.
	uint32_t powerCycleCount_b; // Power cycle count when fault was logged.
	int32_t faultValue_b; // The value of the fault criteria when fault was logged.
	int32_t channelSubsystem_b; // Not used
	float eti_b; // Elapsed Time when fault was logged.
	float angularRateX_b; // Angular rate when fault was logged.
	float angularRateY_b; // Angular rate when fault was logged.
	float angularRateZ_b; // Angular rate when fault was logged.
	float accelerationX_b; // Acceleration when fault was logged.
	float accelerationY_b; // Acceleration when fault was logged.
	float accelerationZ_b; // Acceleration when fault was logged.
	float velocityX_b; // Velocity when fault was logged.
	float velocityY_b; // Velocity when fault was logged.
	float velocityZ_b; // Velocity when fault was logged.
	float insTemperature_b; // Temperature when fault was logged.
	float altitude_b; // Altitude when fault was logged.
};

#endif // __HGuideAPI_Msg_2611_h__
