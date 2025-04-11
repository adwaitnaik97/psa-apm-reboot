#ifndef __HGuideAPI_hgnsi_message_word4_t_h__
#define __HGuideAPI_hgnsi_message_word4_t_h__
#pragma once

// Word 4 of message configuration list
//
// A custom data type defining the list of possible output messages of the GNSS receiver
// the Output messages 32-bit word 4.
//
struct hgnsi_message_word4_t
{
	bool GNSS_RECEIVER_VERSION_5001; // Bit 0 : Enable 0x5001 - GNSS Receiver Version Information
	bool MESSAGE_WORD_4_BIT_2; // Bit 1 : DO NOT USE
	bool MESSAGE_WORD_4_BIT_3; // Bit 2 : DO NOT USE
	bool GNSS_RECEIVER_STATUS_5012; // Bit 3 : Enable 0x5012 - GNSS Receiver Hardware Status Information
	bool MESSAGE_WORD_4_BIT_4; // Bit 4 : DO NOT USE
	bool GNSS_RANGE_5101_1Hz; // Bit 5 : Enable 0x5101 - GNSS Range Data [Rate = 1 Hz]
	bool GNSS_RANGE_5101_5Hz; // Bit 6 : Enable 0x5101 - GNSS Range Data [Rate = 5 Hz]
	bool GNSS_RANGE_5101_10Hz; // Bit 7 : Enable 0x5101 - GNSS Range Data [Rate = 10 Hz]
	bool MESSAGE_WORD_4_BIT_8; // Bit 8 : DO NOT USE
	bool GNSS_SATELLITE_POSITION_5102_1Hz; // Bit 9 : Enable 0x5102 - GNSS Satellite Position [Rate = 1 Hz]
	bool GNSS_SATELLITE_POSITION_5102_5Hz; // Bit 10 : Enable 0x5102 - GNSS Satellite Position [Rate = 5 Hz]
	bool GNSS_SATELLITE_POSITION_5102_10Hz; // Bit 11 : Enable 0x5102 - GNSS Satellite Position [Rate = 10 Hz]
	bool MESSAGE_WORD_4_BIT_12; // Bit 12 : DO NOT USE
	bool GPS_EPHEMERIS_5103; // Bit 13 : Enable 0x5103 - GPS Ephemeris Message
	bool GLONASS_EPHEMERIS_5104; // Bit 14 : Enable 0x5104 - GLONASS Ephemeris Message
	bool GALILEO_EPHEMERIS_5105; // Bit 15 : Enable 0x5105 - GALILEO Ephemeris Message
	bool BEIDOU_EPHEMERIS_5106; // Bit 16 : Enable 0x5106 - BEIDOU Ephemeris Message
	bool MESSAGE_WORD_4_BIT_17; // Bit 17 : DO NOT USE
	bool MESSAGE_WORD_4_BIT_18; // Bit 18 : DO NOT USE
	bool GNSS_PVT_MEASUREMENT_5108_1Hz; // Bit 19 : Enable 0x5108 - GNSS Receiver PVT Measurement Output [Rate = 1 Hz]
	bool GNSS_PVT_MEASUREMENT_5108_5Hz; // Bit 20 : Enable 0x5108 - GNSS Receiver PVT Measurement Output [Rate = 5 Hz]
	bool GNSS_PVT_MEASUREMENT_5108_10Hz; // Bit 21 : Enable 0x5108 - GNSS Receiver PVT Measurement Output [Rate = 10 Hz]
	bool GNSS_PVT_MEASUREMENT_5108_20Hz; // Bit 22 : Enable 0x5108 - GNSS Receiver PVT Measurement Output [Rate = 20 Hz]
	bool MESSAGE_WORD_4_BIT_23; // Bit 23 : DO NOT USE
	bool GNSS_ATTITUDE_MEASUREMENT_5109_1Hz; // Bit 24 : Enable 0x5109 - GNSS Receiver Attitude/Heading Measurement Output [Rate = 1 Hz]
	bool GNSS_ATTITUDE_MEASUREMENT_5109_5Hz; // Bit 25 : Enable 0x5109 - GNSS Receiver Attitude/Heading Measurement Output [Rate = 5 Hz]
	bool GNSS_ATTITUDE_MEASUREMENT_5109_10Hz; // Bit 26 : Enable 0x5109 - GNSS Receiver Attitude/Heading Measurement Output [Rate = 10 Hz]
	bool GNSS_ATTITUDE_MEASUREMENT_5109_20Hz; // Bit 27 : Enable 0x5109 - GNSS Receiver Attitude/Heading Measurement Output [Rate = 20 Hz]
	bool MESSAGE_WORD_4_BIT_28; // Bit 28 : DO NOT USE
	bool MESSAGE_WORD_4_BIT_29; // Bit 29 : DO NOT USE
	bool GNSS_PPS_TIME_5201; // Bit 30 : Enable 0x5201 - PPS GNSS Time
	bool MESSAGE_WORD_4_BIT_31; // Bit 31 : DO NOT USE

	void Default()
	{
		GNSS_RECEIVER_VERSION_5001 = 0;
		MESSAGE_WORD_4_BIT_2 = 0;
		MESSAGE_WORD_4_BIT_3 = 0;
		GNSS_RECEIVER_STATUS_5012 = 0;
		MESSAGE_WORD_4_BIT_4 = 0;
		GNSS_RANGE_5101_1Hz = 0;
		GNSS_RANGE_5101_5Hz = 0;
		GNSS_RANGE_5101_10Hz = 0;
		MESSAGE_WORD_4_BIT_8 = 0;
		GNSS_SATELLITE_POSITION_5102_1Hz = 0;
		GNSS_SATELLITE_POSITION_5102_5Hz = 0;
		GNSS_SATELLITE_POSITION_5102_10Hz = 0;
		MESSAGE_WORD_4_BIT_12 = 0;
		GPS_EPHEMERIS_5103 = 0;
		GLONASS_EPHEMERIS_5104 = 0;
		GALILEO_EPHEMERIS_5105 = 0;
		BEIDOU_EPHEMERIS_5106 = 0;
		MESSAGE_WORD_4_BIT_17 = 0;
		MESSAGE_WORD_4_BIT_18 = 0;
		GNSS_PVT_MEASUREMENT_5108_1Hz = 0;
		GNSS_PVT_MEASUREMENT_5108_5Hz = 0;
		GNSS_PVT_MEASUREMENT_5108_10Hz = 0;
		GNSS_PVT_MEASUREMENT_5108_20Hz = 0;
		MESSAGE_WORD_4_BIT_23 = 0;
		GNSS_ATTITUDE_MEASUREMENT_5109_1Hz = 0;
		GNSS_ATTITUDE_MEASUREMENT_5109_5Hz = 0;
		GNSS_ATTITUDE_MEASUREMENT_5109_10Hz = 0;
		GNSS_ATTITUDE_MEASUREMENT_5109_20Hz = 0;
		MESSAGE_WORD_4_BIT_28 = 0;
		MESSAGE_WORD_4_BIT_29 = 0;
		GNSS_PPS_TIME_5201 = 0;
		MESSAGE_WORD_4_BIT_31 = 0;
	}
};

#endif // __HGuideAPI_hgnsi_message_word4_t_h__
