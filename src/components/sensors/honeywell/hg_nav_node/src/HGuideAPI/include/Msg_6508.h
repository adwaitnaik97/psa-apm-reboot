#ifndef __HGuideAPI_Msg_6508_h__
#define __HGuideAPI_Msg_6508_h__
#pragma once

#include <cstdint>

#include <include/ins_gnss_summary_t.h>

// Detailed information about the amount of satellites
struct sat_block_t
{
	uint16_t TotalSats; // Total satellites in this block
	uint8_t NumGps; // Number of GPS satellites
	uint8_t NumGlo; // Number of GLONASS satellites
	uint8_t NumGal; // Number of Galileo satellites
	uint8_t NumBds; // Number of Beidou satellites
	uint8_t NumSbas; // Number of SBAS satellites
	uint8_t NumQzss; // Number of QZSS satellites
	uint8_t NumIrnss; // Number of IRNSS satellites
	uint8_t NumLband; // Number of L-Band satellites

	void Default()
	{
		TotalSats = 0;
		NumGps = 0;
		NumGlo = 0;
		NumGal = 0;
		NumBds = 0;
		NumSbas = 0;
		NumQzss = 0;
		NumIrnss = 0;
		NumLband = 0;
	}
};


// 0x6508 : Antenna Connected Information
//
// Message 0x6508 is used to provide the number of satellites in track and in lock
// on each RF channel for the system. This can be used to infer if each antenna is 
// properly connected and working correctly. 
// This message is used to inform the user about the functionality of each antenna
// 
class HGUIDE_DLL Msg_6508
{
public:
	Msg_6508();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 104;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message Flag
	static const uint32_t MessageId = 0x6508; // Message ID
	static const uint32_t MessageLength = 26; // Message Length in 32-bit word
	uint32_t Checksum; // Checksum
	double systemTov; // [sec] System Time of Validity
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32_t gps_week; // GPS week number
	ins_gnss_summary_t InsGnssSummary; // INS / GNSS summary word
	sat_block_t RF1AntSatSearch; // RF1 Antenna Tracked satellites
	sat_block_t RF1AntSatLock; // RF1 Antenna Locked satellites
	sat_block_t RF2AntSatSearch; // RF2 Antenna Tracked satellites
	sat_block_t RF2AntSatLock; // RF2 Antenna Locked satellites
};

#endif // __HGuideAPI_Msg_6508_h__
