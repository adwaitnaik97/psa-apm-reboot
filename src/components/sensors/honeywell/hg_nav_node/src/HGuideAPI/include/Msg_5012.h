#ifndef __HGuideAPI_Msg_5012_h__
#define __HGuideAPI_Msg_5012_h__
#pragma once

#include <cstdint>


// 0x5012 : GNSS Receiver Hardware Status Information
//
// Detailed status of the GNSS Receiver hardware.
// 
class HGUIDE_DLL Msg_5012
{
public:
	Msg_5012();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 80;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x5012; // Message ID
	static const uint32_t MessageLength = 20; // Message Length in 32-bit Words
	uint32_t Checksum; // Checksum
	float systemTov; // GNSS receiver system time
	float Temperature; // [deg C]
	float RF1_SupplyVoltage; // [VDC]
	float RF1_SupplyCurrent; // [Amperes]
	float RF2_SupplyVoltage; // [VDC]
	float RF2_SupplyCurrent; // [Amperes]
	float Voltage3V3; // [VDC] Actual Voltage on the 3V3 rail
	float Voltage1V8; // [VDC] Actual Voltage on the 1V8 rail
	float Voltage1V2; // [VDC] Actual Voltage on the 1V2 rail
	float Voltage5V0; // [VDC] Actual Voltage on the 5V0 rail
	uint8_t ptc; // GNSS receiver Percent Throughput - Current [Scaled to represent 0-100%]
	uint8_t ptm; // GNSS receiver Percent Throughput - Maximum [Scaled to represent 0-100%]
	uint8_t memUsage; // GNSS receiver RAM Usage [Scaled to represent 0-100%]
	uint32_t mode; // GPS Receiver Mode: 0 = Test | 1 = Init | 2 = Track | 3 = Powerup
	int32_t power_cycle_count; // Power Cycle Count
	double eti; // [sec] Total Device Elapsed Time Indicator (ETI)
};

#endif // __HGuideAPI_Msg_5012_h__
