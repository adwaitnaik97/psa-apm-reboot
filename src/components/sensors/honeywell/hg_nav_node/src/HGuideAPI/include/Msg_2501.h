#ifndef __HGuideAPI_Msg_2501_h__
#define __HGuideAPI_Msg_2501_h__
#pragma once

#include <cstdint>

// GPS Channel Status
//
// A custom data type which should be used to define a message which contains
// GPS Channel Status
//
struct channel_status_t
{
	uint8_t satellite_num; // Satellite number
	uint8_t hrdw_channel_num; // Channel number
	
	// Channel State:
 	// 0 = STANDBY
 	// 1 = NORMAL
 	// 2 = Reserved
 	// 3 = CODE LOCK
 	// 4 = CARRIER LOCK
 	// 5 = CARRIER TRACK 
 	// 6 = Reserved
 	// 7 = REACQISITION
	uint8_t channel_state;
	uint8_t channel_code; // Channel Code (0 = Not Tracking | 1 = C/A | 2 = C | 3 = P)
	bool used_in_pvt_sln; // true = Used in PVT | false = Not used in PVT
	float carrier_to_noise; // [dB-Hz] carrier to noise value
	uint8_t channel_fqcy; // FREQ_NOT_TRACKING = 0 | L1 = 1 | L2 = 2
	uint8_t delta_range_valid; // Delta Range Validity (1 = Valid | 0 = Invalid)
	uint8_t pseudo_range_valid; // Pseudo Range Validity (1 = Valid | 0 = Invalid)
	float azimuth; // [rad] satellite azimuth
	float elevation; // [rad] satellite elevation
	uint8_t iono_comp_data_src; // Iono compensation data source (UNKNOWN = 0 | IONO MODEL = 1 | DUAL FREQUENCY = 2 | INPUT EXTERNAL SOURCE = 3)
	uint8_t sat_sel_desel; // SEL_CRIT_NOT_REC = 0 | SVID_SEL = 1 | SVID_DES = 2
	float pseudo_range_meas; // Pseudo Range
	float delta_range_meas; // Delta Range
	float sv_pos_x;
	float sv_pos_y;
	float sv_pos_z;

	void Default()
	{
		satellite_num = 0;
		hrdw_channel_num = 0;
		channel_state = 0;
		channel_code = 0;
		used_in_pvt_sln = 0;
		carrier_to_noise = 0;
		channel_fqcy = 0;
		delta_range_valid = 0;
		pseudo_range_valid = 0;
		azimuth = 0;
		elevation = 0;
		iono_comp_data_src = 0;
		sat_sel_desel = 0;
		pseudo_range_meas = 0;
		delta_range_meas = 0;
		sv_pos_x = 0;
		sv_pos_y = 0;
		sv_pos_z = 0;
	}
};


// 0x2501 : GPS Channel Status Output
//
// Reporting of all 25 GPS channels. Each instance contains 6 Channels, message information has to be acumulated over 4 instances 
// (repeat the cycle each time the ChannelIdentifier = 1)
// 
class HGUIDE_DLL Msg_2501
{
public:
	Msg_2501();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 280;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x2501; // Message ID
	static const uint32_t MessageLength = 70; // Message Length
	uint32_t Checksum; // Checksum
	double systemTov; // Time since Power Up - Always Valid [sec]
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32_t gps_week; // Number of weeks since Jan 6, 1980 GPS Week = -1 indicates GPS Week Unknown/Invalid
	uint32_t ChannelIdentifier; // Start Channel ID number (1, 7, 13, 19)
	channel_status_t channel_status_1; // Status of channel # = ChannelIdentifier+0
	channel_status_t channel_status_2; // Status of channel # = ChannelIdentifier+1
	channel_status_t channel_status_3; // Status of channel # = ChannelIdentifier+2
	channel_status_t channel_status_4; // Status of channel # = ChannelIdentifier+3
	channel_status_t channel_status_5; // Status of channel # = ChannelIdentifier+4
	channel_status_t channel_status_6; // Status of channel # = ChannelIdentifier+5
};

#endif // __HGuideAPI_Msg_2501_h__
