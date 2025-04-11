#ifndef __HGuideAPI_hgnsi_message_word2_t_h__
#define __HGuideAPI_hgnsi_message_word2_t_h__
#pragma once

// Word 2 of message configuration list
//
// A custom data type defining the list of possible output messages of the INS
// the Output messages 32-bit word 2.
//
struct hgnsi_message_word2_t
{
	bool GPS_CONFIG_2002; // Bit 0 : Enable 0x2002 - DO NOT USE
	bool MOTION_DETECT_6111; // Bit 1 : Enable 0x6111 - Motion Detection Algorithm detailed results
	bool MAG_AID_MEAS_ERR_EST_2427_5HZ; // Bit 2 : Enable 0x2427 - DO NOT USE [Rate = 5 Hz]
	bool MAG_AID_MEAS_ERR_EST_2427_1HZ; // Bit 3 : Enable 0x2427 - DO NOT USE [Rate = 1 Hz]
	bool BAROMETRIC_AID_MEAS_ERR_EST_2426_5HZ; // Bit 4 : Enable 0x2426 - DO NOT USE [Rate = 5 Hz]
	bool BAROMETRIC_AID_MEAS_ERR_EST_2426_1HZ; // Bit 5 : Enable 0x2426 - DO NOT USE [Rate = 1 Hz]
	bool INERTIAL_SENSOR_ERR_STDV_2423_5HZ; // Bit 6 : Enable 0x2423 - DO NOT USE [Rate = 5 Hz]
	bool INERTIAL_SENSOR_ERR_STDV_2423_1HZ; // Bit 7 : Enable 0x2423 - DO NOT USE [Rate = 1 Hz]
	bool GPS_CHANNEL_STATUS_2501_SET; // Bit 8 : Enable 0x2501 - DO NOT USE
	bool GPS_ALMANAC_2511_SET; // Bit 9 : Enable 0x2511 - DO NOT USE
	bool GPS_EPHEMERIS_2512_SET; // Bit 10 : Enable 0x2512 - DO NOT USE
	bool GPS_ONAV_2513_SET; // Bit 11 : Enable 0x2513 - DO NOT USE
	bool GPS_IONO_2514_SET; // Bit 12 : Enable 0x2514 - DO NOT USE
	bool GNSS_STATES_ERR_EST_6424_5HZ; // Bit 13 : Enable 0x6424 - DO NOT USE
	bool GNSS_NORM_MEAS_RESID_6428_5HZ; // Bit 14 : Enable 0x6428 - DO NOT USE
	bool ODO_NORM_MEAS_STATES_6438_5HZ; // Bit 15 : Enable 0x6438 - Odometer Measurement Residuals [Rate = 5 Hz]
	bool IBIT_RESULTS_2601; // Bit 16 : Enable 0x2601 - DO NOT USE
	bool BIT_HISTORY_2611; // Bit 17 : Enable 0x2611 - DO NOT USE
	bool GNSS_PVT_6108; // Bit 18 : Enable 0x6108 - GNSS Position, Velocity and Time
	bool GEODETIC_POSITION_6403; // Bit 19 : Enable 0x6403 - Geodetic Position
	bool EULER_ATTITUDE_6405; // Bit 20 : Enable 0x6405 - Attitude
	bool NED_VELOCITY_6504; // Bit 21 : Enable 0x6504 - North, East, Down Velocity
	bool GNSS_ATT_6109; // Bit 22 : Enable 0x6109 - GNSS Attitude
	bool ODO_DPOS_6110; // Bit 23 : Enable 0x6110 - Odometer Measurement - reply to 0x4110 aiding
	bool APP_SP_OUTPUT_1_2901; // Bit 24 : Enable 0x2901 - DO NOT USE
	bool GNSS_SINGLE_ANT_ATT_6112; // Bit 25 : Enable 0x6112 - GNSS Single Antenna Initialization
	bool VEHICLE_BODY_RATES_AND_ACCELS_6406_100HZ; // Bit 26 : Enable 0x6406 - Vehicle Body Rates and Linear Acceleration [Rate = 100 Hz]
	bool GPS_NORM_MEAS_RESID_2428_5HZ; // Bit 27 : Enable 0x2428 - DO NOT USE [Rate = 5 Hz]
	bool GPS_NORM_MEAS_RESID_2428_1HZ; // Bit 28 : Enable 0x2428 - DO NOT USE [Rate = 1 Hz]
	bool TR_NORM_MEAS_RESID_2429_5HZ; // Bit 29 : Enable 0x2429 - DO NOT USE [Rate = 5 Hz]
	bool TR_NORM_MEAS_RESID_2429_1HZ; // Bit 30 : Enable 0x2429 - DO NOT USE [Rate = 1 Hz]
	bool LOG_MESSAGE_9900; // Bit 31 : Enable 0x9900 - Log Message

	void Default()
	{
		GPS_CONFIG_2002 = 0;
		MOTION_DETECT_6111 = 1;
		MAG_AID_MEAS_ERR_EST_2427_5HZ = 0;
		MAG_AID_MEAS_ERR_EST_2427_1HZ = 0;
		BAROMETRIC_AID_MEAS_ERR_EST_2426_5HZ = 0;
		BAROMETRIC_AID_MEAS_ERR_EST_2426_1HZ = 0;
		INERTIAL_SENSOR_ERR_STDV_2423_5HZ = 0;
		INERTIAL_SENSOR_ERR_STDV_2423_1HZ = 0;
		GPS_CHANNEL_STATUS_2501_SET = 0;
		GPS_ALMANAC_2511_SET = 0;
		GPS_EPHEMERIS_2512_SET = 0;
		GPS_ONAV_2513_SET = 0;
		GPS_IONO_2514_SET = 0;
		GNSS_STATES_ERR_EST_6424_5HZ = 1;
		GNSS_NORM_MEAS_RESID_6428_5HZ = 1;
		ODO_NORM_MEAS_STATES_6438_5HZ = 1;
		IBIT_RESULTS_2601 = 0;
		BIT_HISTORY_2611 = 0;
		GNSS_PVT_6108 = 1;
		GEODETIC_POSITION_6403 = 1;
		EULER_ATTITUDE_6405 = 1;
		NED_VELOCITY_6504 = 1;
		GNSS_ATT_6109 = 1;
		ODO_DPOS_6110 = 1;
		APP_SP_OUTPUT_1_2901 = 0;
		GNSS_SINGLE_ANT_ATT_6112 = 0;
		VEHICLE_BODY_RATES_AND_ACCELS_6406_100HZ = 0;
		GPS_NORM_MEAS_RESID_2428_5HZ = 1;
		GPS_NORM_MEAS_RESID_2428_1HZ = 0;
		TR_NORM_MEAS_RESID_2429_5HZ = 1;
		TR_NORM_MEAS_RESID_2429_1HZ = 0;
		LOG_MESSAGE_9900 = 0;
	}
};

#endif // __HGuideAPI_hgnsi_message_word2_t_h__
