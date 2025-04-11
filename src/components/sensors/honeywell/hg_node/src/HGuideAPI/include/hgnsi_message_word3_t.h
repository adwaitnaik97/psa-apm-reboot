#ifndef __HGuideAPI_hgnsi_message_word3_t_h__
#define __HGuideAPI_hgnsi_message_word3_t_h__
#pragma once

// Word 3 of message configuration list
//
// A custom data type defining the list of possible output messages of the INS
// the Output messages 32-bit word 3.
//
struct hgnsi_message_word3_t
{
	bool LED_STATUS_MESSAGE_6601; // Bit 0 : Enable 0x6601 - Virtual LED Status
	bool HGUIDE_INSTALL_6003; // Bit 1 : Enable 0x6003 - HGuide Sensor Installation Settings
	bool ANTENNA_CONNECTED_6508; // Bit 2 : Enable 0x6508 - Antenna Connected
	bool DEVICE_IDENTIFICATION_6001; // Bit 3 : Enable 0x6001 - Device Identification [Rate = one shot]
	bool NTRIP_CONFIG_6009; // Bit 4 : Enable 0x6009 - NTRIP Configuration [rate = one shot]
	bool NTRIP_STATUS_6609; // Bit 5 : Enable 0x6609 - NTRIP Status [rate = 1 Hz]
	bool MESSAGE_WORD_3_BIT_6; // Bit 6 : DO NOT USE
	bool MESSAGE_WORD_3_BIT_7; // Bit 7 : DO NOT USE
	bool MESSAGE_WORD_3_BIT_8; // Bit 8 : DO NOT USE
	bool REAL_TIME_GOOD_TO_GO_6651; // Bit 9 : Enable 0x6651 - Post Processing Good to Go
	bool MESSAGE_WORD_3_BIT_10; // Bit 10 : DO NOT USE
	bool MESSAGE_WORD_3_BIT_11; // Bit 11 : DO NOT USE
	bool MESSAGE_WORD_3_BIT_12; // Bit 12 : DO NOT USE
	bool MESSAGE_WORD_3_BIT_13; // Bit 13 : DO NOT USE
	bool MESSAGE_WORD_3_BIT_14; // Bit 14 : DO NOT USE
	bool NORTEK_DVL_BOTTOM_TRACK_6721; // Bit 15 : Enable 0x6721 - DVL Bottom Track Ping
	bool NORTEK_DVL_WATER_TRACK_6722; // Bit 16 : Enable 0x6722 - DVL Water Track Ping
	bool NORTEK_DVL_CURRENT_PROFILE_6723; // Bit 17 : Enable 0x6723 - DVL Current Profile Ping
	bool NORTEK_DVL_CURRENT_PROFILE_VELOCITY_DATA_6724; // Bit 18 : Enable 0x6724 - DVL Current Profile Velocity Data
	bool NORTEK_DVL_CURRENT_PROFILE_AMPLITUDE_DATA_6725; // Bit 19 : Enable 0x6725 - DVL Current Profile Amplitude Data
	bool NORTEK_DVL_CURRENT_PROFILE_CORRELATION_DATA_6726; // Bit 20 : Enable 0x6726 - DVL Current Profile Correlation Data
	bool NORTEK_DVL_NORM_MEAS_RESID_STATES_ERR_EST_6738; // Bit 21 : Enable 0x6738 - DVL Norm Meas Resid States Err Est
	bool MESSAGE_WORD_3_BIT_22; // Bit 22 : DO NOT USE
	bool MESSAGE_WORD_3_BIT_23; // Bit 23 : DO NOT USE
	bool MESSAGE_WORD_3_BIT_24; // Bit 24 : DO NOT USE
	bool MESSAGE_WORD_3_BIT_25; // Bit 25 : DO NOT USE
	bool MESSAGE_WORD_3_BIT_26; // Bit 26 : DO NOT USE
	bool MESSAGE_WORD_3_BIT_27; // Bit 27 : DO NOT USE
	bool MESSAGE_WORD_3_BIT_28; // Bit 28 : DO NOT USE
	bool MESSAGE_WORD_3_BIT_29; // Bit 29 : DO NOT USE
	bool MESSAGE_WORD_3_BIT_30; // Bit 30 : DO NOT USE
	bool PROCESS_LOAD_9910; // Bit 31 : Enable 0x9910 - Process Loading

	void Default()
	{
		LED_STATUS_MESSAGE_6601 = 0;
		HGUIDE_INSTALL_6003 = 1;
		ANTENNA_CONNECTED_6508 = 0;
		DEVICE_IDENTIFICATION_6001 = 0;
		NTRIP_CONFIG_6009 = 0;
		NTRIP_STATUS_6609 = 0;
		MESSAGE_WORD_3_BIT_6 = 0;
		MESSAGE_WORD_3_BIT_7 = 0;
		MESSAGE_WORD_3_BIT_8 = 0;
		REAL_TIME_GOOD_TO_GO_6651 = 0;
		MESSAGE_WORD_3_BIT_10 = 0;
		MESSAGE_WORD_3_BIT_11 = 0;
		MESSAGE_WORD_3_BIT_12 = 0;
		MESSAGE_WORD_3_BIT_13 = 0;
		MESSAGE_WORD_3_BIT_14 = 0;
		NORTEK_DVL_BOTTOM_TRACK_6721 = 1;
		NORTEK_DVL_WATER_TRACK_6722 = 1;
		NORTEK_DVL_CURRENT_PROFILE_6723 = 1;
		NORTEK_DVL_CURRENT_PROFILE_VELOCITY_DATA_6724 = 1;
		NORTEK_DVL_CURRENT_PROFILE_AMPLITUDE_DATA_6725 = 1;
		NORTEK_DVL_CURRENT_PROFILE_CORRELATION_DATA_6726 = 1;
		NORTEK_DVL_NORM_MEAS_RESID_STATES_ERR_EST_6738 = 1;
		MESSAGE_WORD_3_BIT_22 = 0;
		MESSAGE_WORD_3_BIT_23 = 0;
		MESSAGE_WORD_3_BIT_24 = 0;
		MESSAGE_WORD_3_BIT_25 = 0;
		MESSAGE_WORD_3_BIT_26 = 0;
		MESSAGE_WORD_3_BIT_27 = 0;
		MESSAGE_WORD_3_BIT_28 = 0;
		MESSAGE_WORD_3_BIT_29 = 0;
		MESSAGE_WORD_3_BIT_30 = 0;
		PROCESS_LOAD_9910 = 0;
	}
};

#endif // __HGuideAPI_hgnsi_message_word3_t_h__
