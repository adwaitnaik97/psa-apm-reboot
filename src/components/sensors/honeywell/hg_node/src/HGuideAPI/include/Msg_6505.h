#ifndef __HGuideAPI_Msg_6505_h__
#define __HGuideAPI_Msg_6505_h__
#pragma once

#include <cstdint>

#include <include/ins_gnss_summary_t.h>
#include <include/constellation_enum_t.h>


// 0x6505 : Skymap Information
//
// Individual message per satellite, holds skymap data including elevation, azimuth, and signal C/No values.
// Message is being sent in batches. One should condense the information until 'Final_msg_in_set=1'
// Only satellites in view are reported - not the whole ephemeris
// 
class HGUIDE_DLL Msg_6505
{
public:
	Msg_6505();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 64;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message Flag
	static const uint32_t MessageId = 0x6505; // Message ID
	static const uint32_t MessageLength = 16; // Message Length in 32-bit word
	uint32_t Checksum; // Checksum
	constellation_enum_t Constellation; // Constellation that the satellite being described belongs to.
	bool Used_in_soln; // Used in the solution: 0=not used in GNSS solution | 1=used in GNSS solution
	bool Final_msg_in_set; // Final message to be sent: 0=not final message in set | 1= final message int set
	uint8_t prn_slot; // Prn/slot of the satellite
	float elevation; // Elevation angle of the satellite in degrees. (Horizon = 0 degrees)
	float azimuth; // Azimuth angle of the satellite in degrees. (North = 0 degrees, increasing towards East)
	uint8_t freq_band_1; // Frequency band of this signal
	uint8_t channel_1; // Channel number for this signal (GLONASS specific)
	float carrier_noise_1; // Carrier/noise for this signal, 0 if not tracked
	uint8_t freq_band_2; // Frequency band of this signal
	uint8_t channel_2; // Channel number for this signal (GLONASS specific)
	float carrier_noise_2; // Carrier/noise for this signal, 0 if not tracked
	uint8_t freq_band_3; // Frequency band of this signal
	uint8_t channel_3; // Channel number for this signal (GLONASS specific)
	float carrier_noise_3; // Carrier/noise for this signal, 0 if not tracked
	uint8_t freq_band_4; // Frequency band of this signal
	uint8_t channel_4; // Channel number for this signal (GLONASS specific)
	float carrier_noise_4; // Carrier/noise for this signal, 0 if not tracked
	uint8_t freq_band_5; // Frequency band of this signal
	uint8_t channel_5; // Channel number for this signal (GLONASS specific)
	float carrier_noise_5; // Carrier/noise for this signal, 0 if not tracked
	uint8_t freq_band_6; // Frequency band of this signal
	uint8_t channel_6; // Channel number for this signal (GLONASS specific)
	float carrier_noise_6; // Carrier/noise for this signal, 0 if not tracked
	ins_gnss_summary_t InsGnssSummary; // INS / GNSS Mode Summary
	int32_t gps_week; // GPS Week
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
};

#endif // __HGuideAPI_Msg_6505_h__
