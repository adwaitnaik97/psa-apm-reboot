#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_1001.h>
#include <hg_nav_node/hgnsi_message_word1_t.h>
#include <hg_nav_node/hgnsi_message_word2_t.h>
#include <hg_nav_node/hgnsi_message_word3_t.h>
#include <hg_nav_node/hgnsi_message_word4_t.h>
hg_nav_node::Msg_1001 msgStruct_1001;

ros::Subscriber Msg_1001_sub;
void init_1001(ros::NodeHandle * n){
	Msg_1001_sub = n->subscribe(MSG_1001_PATH, 5, Msg_1001_sub_callback);
	ROS_INFO("Starting sub %s",MSG_1001_PATH);
	return;
}

void stop_1001(void){
	Msg_1001_sub.shutdown();
	ROS_INFO("0x1001 stopped");
	return;
}

// Msg_1001 to Topic
void convert(Msg_1001 messageIn, hg_nav_node::Msg_1001 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;

	messageOut->MessageWord1.INS_CONFIG_2001 = messageIn.MessageWord1.INS_CONFIG_2001;
	messageOut->MessageWord1.INS_MODE_STATUS_2011_1HZ = messageIn.MessageWord1.INS_MODE_STATUS_2011_1HZ;
	messageOut->MessageWord1.TIMEMARK_EVENT_IN_6201 = messageIn.MessageWord1.TIMEMARK_EVENT_IN_6201;
	messageOut->MessageWord1.INS_INIT_STATUS_2021_1HZ = messageIn.MessageWord1.INS_INIT_STATUS_2021_1HZ;
	messageOut->MessageWord1.SKYMAP_DATA_6505 = messageIn.MessageWord1.SKYMAP_DATA_6505;
	messageOut->MessageWord1.GPS_PVT_OUT_2108_1HZ = messageIn.MessageWord1.GPS_PVT_OUT_2108_1HZ;
	messageOut->MessageWord1.EVENT_IN_GEODETIC_POSITION_6202 = messageIn.MessageWord1.EVENT_IN_GEODETIC_POSITION_6202;
	messageOut->MessageWord1.EVENT_IN_NED_VELOCITY_6203 = messageIn.MessageWord1.EVENT_IN_NED_VELOCITY_6203;
	messageOut->MessageWord1.TIMEMARK_PPS_OUT_2201_1HZ = messageIn.MessageWord1.TIMEMARK_PPS_OUT_2201_1HZ;
	messageOut->MessageWord1.TIMEMARK_BLOCK4_2211_1HZ = messageIn.MessageWord1.TIMEMARK_BLOCK4_2211_1HZ;
	messageOut->MessageWord1.EVENT_IN_EULER_ATTITUDE_6204 = messageIn.MessageWord1.EVENT_IN_EULER_ATTITUDE_6204;
	messageOut->MessageWord1.EVENT_IN_FULL_6205 = messageIn.MessageWord1.EVENT_IN_FULL_6205;
	messageOut->MessageWord1.AUTOPILOT_FLT_CTRL_2301_FC = messageIn.MessageWord1.AUTOPILOT_FLT_CTRL_2301_FC;
	messageOut->MessageWord1.UNFILTERED_INS_DATA_USR_REF_6311_NC = messageIn.MessageWord1.UNFILTERED_INS_DATA_USR_REF_6311_NC;
	messageOut->MessageWord1.UNFILTERED_INS_DATA_2311_NC = messageIn.MessageWord1.UNFILTERED_INS_DATA_2311_NC;
	messageOut->MessageWord1.SAVE_CONFIGURATION_TO_FLASH = messageIn.MessageWord1.SAVE_CONFIGURATION_TO_FLASH;
	messageOut->MessageWord1.NAV_OUT_2401_100HZ = messageIn.MessageWord1.NAV_OUT_2401_100HZ;
	messageOut->MessageWord1.NAV_OUT_2401_50HZ = messageIn.MessageWord1.NAV_OUT_2401_50HZ;
	messageOut->MessageWord1.NAV_OUT_2401_1HZ = messageIn.MessageWord1.NAV_OUT_2401_1HZ;
	messageOut->MessageWord1.SMOOTH_NAV_OUT_2402_100HZ = messageIn.MessageWord1.SMOOTH_NAV_OUT_2402_100HZ;
	messageOut->MessageWord1.SMOOTH_NAV_OUT_2402_50HZ = messageIn.MessageWord1.SMOOTH_NAV_OUT_2402_50HZ;
	messageOut->MessageWord1.SMOOTH_NAV_OUT_2402_1HZ = messageIn.MessageWord1.SMOOTH_NAV_OUT_2402_1HZ;
	messageOut->MessageWord1.INS_ERR_EST_2411_5HZ = messageIn.MessageWord1.INS_ERR_EST_2411_5HZ;
	messageOut->MessageWord1.INS_ERR_EST_2411_1HZ = messageIn.MessageWord1.INS_ERR_EST_2411_1HZ;
	messageOut->MessageWord1.KF_NAV_MEAS_ST_2421_5HZ = messageIn.MessageWord1.KF_NAV_MEAS_ST_2421_5HZ;
	messageOut->MessageWord1.KF_NAV_MEAS_ST_2421_1HZ = messageIn.MessageWord1.KF_NAV_MEAS_ST_2421_1HZ;
	messageOut->MessageWord1.KF_NAV_SOL_ST_DEV_2422_5HZ = messageIn.MessageWord1.KF_NAV_SOL_ST_DEV_2422_5HZ;
	messageOut->MessageWord1.KF_NAV_SOL_ST_DEV_2422_1HZ = messageIn.MessageWord1.KF_NAV_SOL_ST_DEV_2422_1HZ;
	messageOut->MessageWord1.GPS_AID_MEAS_ERR_EST_2424_5HZ = messageIn.MessageWord1.GPS_AID_MEAS_ERR_EST_2424_5HZ;
	messageOut->MessageWord1.GPS_AID_MEAS_ERR_EST_2424_1HZ = messageIn.MessageWord1.GPS_AID_MEAS_ERR_EST_2424_1HZ;
	messageOut->MessageWord1.TR_ALIGN_AID_MEAS_ERR_EST_2425_5HZ = messageIn.MessageWord1.TR_ALIGN_AID_MEAS_ERR_EST_2425_5HZ;
	messageOut->MessageWord1.TR_ALIGN_AID_MEAS_ERR_EST_2425_1HZ = messageIn.MessageWord1.TR_ALIGN_AID_MEAS_ERR_EST_2425_1HZ;

	messageOut->MessageWord2.GPS_CONFIG_2002 = messageIn.MessageWord2.GPS_CONFIG_2002;
	messageOut->MessageWord2.MOTION_DETECT_6111 = messageIn.MessageWord2.MOTION_DETECT_6111;
	messageOut->MessageWord2.MAG_AID_MEAS_ERR_EST_2427_5HZ = messageIn.MessageWord2.MAG_AID_MEAS_ERR_EST_2427_5HZ;
	messageOut->MessageWord2.MAG_AID_MEAS_ERR_EST_2427_1HZ = messageIn.MessageWord2.MAG_AID_MEAS_ERR_EST_2427_1HZ;
	messageOut->MessageWord2.BAROMETRIC_AID_MEAS_ERR_EST_2426_5HZ = messageIn.MessageWord2.BAROMETRIC_AID_MEAS_ERR_EST_2426_5HZ;
	messageOut->MessageWord2.BAROMETRIC_AID_MEAS_ERR_EST_2426_1HZ = messageIn.MessageWord2.BAROMETRIC_AID_MEAS_ERR_EST_2426_1HZ;
	messageOut->MessageWord2.INERTIAL_SENSOR_ERR_STDV_2423_5HZ = messageIn.MessageWord2.INERTIAL_SENSOR_ERR_STDV_2423_5HZ;
	messageOut->MessageWord2.INERTIAL_SENSOR_ERR_STDV_2423_1HZ = messageIn.MessageWord2.INERTIAL_SENSOR_ERR_STDV_2423_1HZ;
	messageOut->MessageWord2.GPS_CHANNEL_STATUS_2501_SET = messageIn.MessageWord2.GPS_CHANNEL_STATUS_2501_SET;
	messageOut->MessageWord2.GPS_ALMANAC_2511_SET = messageIn.MessageWord2.GPS_ALMANAC_2511_SET;
	messageOut->MessageWord2.GPS_EPHEMERIS_2512_SET = messageIn.MessageWord2.GPS_EPHEMERIS_2512_SET;
	messageOut->MessageWord2.GPS_ONAV_2513_SET = messageIn.MessageWord2.GPS_ONAV_2513_SET;
	messageOut->MessageWord2.GPS_IONO_2514_SET = messageIn.MessageWord2.GPS_IONO_2514_SET;
	messageOut->MessageWord2.GNSS_STATES_ERR_EST_6424_5HZ = messageIn.MessageWord2.GNSS_STATES_ERR_EST_6424_5HZ;
	messageOut->MessageWord2.GNSS_NORM_MEAS_RESID_6428_5HZ = messageIn.MessageWord2.GNSS_NORM_MEAS_RESID_6428_5HZ;
	messageOut->MessageWord2.ODO_NORM_MEAS_STATES_6438_5HZ = messageIn.MessageWord2.ODO_NORM_MEAS_STATES_6438_5HZ;
	messageOut->MessageWord2.IBIT_RESULTS_2601 = messageIn.MessageWord2.IBIT_RESULTS_2601;
	messageOut->MessageWord2.BIT_HISTORY_2611 = messageIn.MessageWord2.BIT_HISTORY_2611;
	messageOut->MessageWord2.GNSS_PVT_6108 = messageIn.MessageWord2.GNSS_PVT_6108;
	messageOut->MessageWord2.GEODETIC_POSITION_6403 = messageIn.MessageWord2.GEODETIC_POSITION_6403;
	messageOut->MessageWord2.EULER_ATTITUDE_6405 = messageIn.MessageWord2.EULER_ATTITUDE_6405;
	messageOut->MessageWord2.NED_VELOCITY_6504 = messageIn.MessageWord2.NED_VELOCITY_6504;
	messageOut->MessageWord2.GNSS_ATT_6109 = messageIn.MessageWord2.GNSS_ATT_6109;
	messageOut->MessageWord2.ODO_DPOS_6110 = messageIn.MessageWord2.ODO_DPOS_6110;
	messageOut->MessageWord2.APP_SP_OUTPUT_1_2901 = messageIn.MessageWord2.APP_SP_OUTPUT_1_2901;
	messageOut->MessageWord2.GNSS_SINGLE_ANT_ATT_6112 = messageIn.MessageWord2.GNSS_SINGLE_ANT_ATT_6112;
	messageOut->MessageWord2.VEHICLE_BODY_RATES_AND_ACCELS_6406_100HZ = messageIn.MessageWord2.VEHICLE_BODY_RATES_AND_ACCELS_6406_100HZ;
	messageOut->MessageWord2.GPS_NORM_MEAS_RESID_2428_5HZ = messageIn.MessageWord2.GPS_NORM_MEAS_RESID_2428_5HZ;
	messageOut->MessageWord2.GPS_NORM_MEAS_RESID_2428_1HZ = messageIn.MessageWord2.GPS_NORM_MEAS_RESID_2428_1HZ;
	messageOut->MessageWord2.TR_NORM_MEAS_RESID_2429_5HZ = messageIn.MessageWord2.TR_NORM_MEAS_RESID_2429_5HZ;
	messageOut->MessageWord2.TR_NORM_MEAS_RESID_2429_1HZ = messageIn.MessageWord2.TR_NORM_MEAS_RESID_2429_1HZ;
	messageOut->MessageWord2.LOG_MESSAGE_9900 = messageIn.MessageWord2.LOG_MESSAGE_9900;

	messageOut->MessageWord3.LED_STATUS_MESSAGE_6601 = messageIn.MessageWord3.LED_STATUS_MESSAGE_6601;
	messageOut->MessageWord3.HGUIDE_INSTALL_6003 = messageIn.MessageWord3.HGUIDE_INSTALL_6003;
	messageOut->MessageWord3.ANTENNA_CONNECTED_6508 = messageIn.MessageWord3.ANTENNA_CONNECTED_6508;
	messageOut->MessageWord3.DEVICE_IDENTIFICATION_6001 = messageIn.MessageWord3.DEVICE_IDENTIFICATION_6001;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_4 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_4;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_5 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_5;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_6 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_6;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_7 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_7;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_8 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_8;
	messageOut->MessageWord3.REAL_TIME_GOOD_TO_GO_6651 = messageIn.MessageWord3.REAL_TIME_GOOD_TO_GO_6651;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_10 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_10;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_11 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_11;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_12 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_12;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_13 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_13;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_14 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_14;
	messageOut->MessageWord3.NORTEK_DVL_BOTTOM_TRACK_6721 = messageIn.MessageWord3.NORTEK_DVL_BOTTOM_TRACK_6721;
	messageOut->MessageWord3.NORTEK_DVL_WATER_TRACK_6722 = messageIn.MessageWord3.NORTEK_DVL_WATER_TRACK_6722;
	messageOut->MessageWord3.NORTEK_DVL_CURRENT_PROFILE_6723 = messageIn.MessageWord3.NORTEK_DVL_CURRENT_PROFILE_6723;
	messageOut->MessageWord3.NORTEK_DVL_CURRENT_PROFILE_VELOCITY_DATA_6724 = messageIn.MessageWord3.NORTEK_DVL_CURRENT_PROFILE_VELOCITY_DATA_6724;
	messageOut->MessageWord3.NORTEK_DVL_CURRENT_PROFILE_AMPLITUDE_DATA_6725 = messageIn.MessageWord3.NORTEK_DVL_CURRENT_PROFILE_AMPLITUDE_DATA_6725;
	messageOut->MessageWord3.NORTEK_DVL_CURRENT_PROFILE_CORRELATION_DATA_6726 = messageIn.MessageWord3.NORTEK_DVL_CURRENT_PROFILE_CORRELATION_DATA_6726;
	messageOut->MessageWord3.NORTEK_DVL_NORM_MEAS_RESID_STATES_ERR_EST_6738 = messageIn.MessageWord3.NORTEK_DVL_NORM_MEAS_RESID_STATES_ERR_EST_6738;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_22 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_22;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_23 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_23;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_24 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_24;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_25 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_25;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_26 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_26;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_27 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_27;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_28 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_28;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_29 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_29;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_30 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_30;
	messageOut->MessageWord3.PROCESS_LOAD_9910 = messageIn.MessageWord3.PROCESS_LOAD_9910;

	messageOut->MessageWord4.GNSS_RECEIVER_VERSION_5001 = messageIn.MessageWord4.GNSS_RECEIVER_VERSION_5001;
	messageOut->MessageWord4.MESSAGE_WORD_4_BIT_2 = messageIn.MessageWord4.MESSAGE_WORD_4_BIT_2;
	messageOut->MessageWord4.MESSAGE_WORD_4_BIT_3 = messageIn.MessageWord4.MESSAGE_WORD_4_BIT_3;
	messageOut->MessageWord4.GNSS_RECEIVER_STATUS_5012 = messageIn.MessageWord4.GNSS_RECEIVER_STATUS_5012;
	messageOut->MessageWord4.MESSAGE_WORD_4_BIT_4 = messageIn.MessageWord4.MESSAGE_WORD_4_BIT_4;
	messageOut->MessageWord4.GNSS_RANGE_5101_1Hz = messageIn.MessageWord4.GNSS_RANGE_5101_1Hz;
	messageOut->MessageWord4.GNSS_RANGE_5101_5Hz = messageIn.MessageWord4.GNSS_RANGE_5101_5Hz;
	messageOut->MessageWord4.GNSS_RANGE_5101_10Hz = messageIn.MessageWord4.GNSS_RANGE_5101_10Hz;
	messageOut->MessageWord4.MESSAGE_WORD_4_BIT_8 = messageIn.MessageWord4.MESSAGE_WORD_4_BIT_8;
	messageOut->MessageWord4.GNSS_SATELLITE_POSITION_5102_1Hz = messageIn.MessageWord4.GNSS_SATELLITE_POSITION_5102_1Hz;
	messageOut->MessageWord4.GNSS_SATELLITE_POSITION_5102_5Hz = messageIn.MessageWord4.GNSS_SATELLITE_POSITION_5102_5Hz;
	messageOut->MessageWord4.GNSS_SATELLITE_POSITION_5102_10Hz = messageIn.MessageWord4.GNSS_SATELLITE_POSITION_5102_10Hz;
	messageOut->MessageWord4.MESSAGE_WORD_4_BIT_12 = messageIn.MessageWord4.MESSAGE_WORD_4_BIT_12;
	messageOut->MessageWord4.GPS_EPHEMERIS_5103 = messageIn.MessageWord4.GPS_EPHEMERIS_5103;
	messageOut->MessageWord4.GLONASS_EPHEMERIS_5104 = messageIn.MessageWord4.GLONASS_EPHEMERIS_5104;
	messageOut->MessageWord4.GALILEO_EPHEMERIS_5105 = messageIn.MessageWord4.GALILEO_EPHEMERIS_5105;
	messageOut->MessageWord4.BEIDOU_EPHEMERIS_5106 = messageIn.MessageWord4.BEIDOU_EPHEMERIS_5106;
	messageOut->MessageWord4.MESSAGE_WORD_4_BIT_17 = messageIn.MessageWord4.MESSAGE_WORD_4_BIT_17;
	messageOut->MessageWord4.MESSAGE_WORD_4_BIT_18 = messageIn.MessageWord4.MESSAGE_WORD_4_BIT_18;
	messageOut->MessageWord4.GNSS_PVT_MEASUREMENT_5108_1Hz = messageIn.MessageWord4.GNSS_PVT_MEASUREMENT_5108_1Hz;
	messageOut->MessageWord4.GNSS_PVT_MEASUREMENT_5108_5Hz = messageIn.MessageWord4.GNSS_PVT_MEASUREMENT_5108_5Hz;
	messageOut->MessageWord4.GNSS_PVT_MEASUREMENT_5108_10Hz = messageIn.MessageWord4.GNSS_PVT_MEASUREMENT_5108_10Hz;
	messageOut->MessageWord4.GNSS_PVT_MEASUREMENT_5108_20Hz = messageIn.MessageWord4.GNSS_PVT_MEASUREMENT_5108_20Hz;
	messageOut->MessageWord4.MESSAGE_WORD_4_BIT_23 = messageIn.MessageWord4.MESSAGE_WORD_4_BIT_23;
	messageOut->MessageWord4.GNSS_ATTITUDE_MEASUREMENT_5109_1Hz = messageIn.MessageWord4.GNSS_ATTITUDE_MEASUREMENT_5109_1Hz;
	messageOut->MessageWord4.GNSS_ATTITUDE_MEASUREMENT_5109_5Hz = messageIn.MessageWord4.GNSS_ATTITUDE_MEASUREMENT_5109_5Hz;
	messageOut->MessageWord4.GNSS_ATTITUDE_MEASUREMENT_5109_10Hz = messageIn.MessageWord4.GNSS_ATTITUDE_MEASUREMENT_5109_10Hz;
	messageOut->MessageWord4.GNSS_ATTITUDE_MEASUREMENT_5109_20Hz = messageIn.MessageWord4.GNSS_ATTITUDE_MEASUREMENT_5109_20Hz;
	messageOut->MessageWord4.MESSAGE_WORD_4_BIT_28 = messageIn.MessageWord4.MESSAGE_WORD_4_BIT_28;
	messageOut->MessageWord4.MESSAGE_WORD_4_BIT_29 = messageIn.MessageWord4.MESSAGE_WORD_4_BIT_29;
	messageOut->MessageWord4.GNSS_PPS_TIME_5201 = messageIn.MessageWord4.GNSS_PPS_TIME_5201;
	messageOut->MessageWord4.MESSAGE_WORD_4_BIT_31 = messageIn.MessageWord4.MESSAGE_WORD_4_BIT_31;
}

// Topic to Msg_1001
void convert(hg_nav_node::Msg_1001 messageIn, Msg_1001 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;

	messageOut->MessageWord1.INS_CONFIG_2001 = messageIn.MessageWord1.INS_CONFIG_2001;
	messageOut->MessageWord1.INS_MODE_STATUS_2011_1HZ = messageIn.MessageWord1.INS_MODE_STATUS_2011_1HZ;
	messageOut->MessageWord1.TIMEMARK_EVENT_IN_6201 = messageIn.MessageWord1.TIMEMARK_EVENT_IN_6201;
	messageOut->MessageWord1.INS_INIT_STATUS_2021_1HZ = messageIn.MessageWord1.INS_INIT_STATUS_2021_1HZ;
	messageOut->MessageWord1.SKYMAP_DATA_6505 = messageIn.MessageWord1.SKYMAP_DATA_6505;
	messageOut->MessageWord1.GPS_PVT_OUT_2108_1HZ = messageIn.MessageWord1.GPS_PVT_OUT_2108_1HZ;
	messageOut->MessageWord1.EVENT_IN_GEODETIC_POSITION_6202 = messageIn.MessageWord1.EVENT_IN_GEODETIC_POSITION_6202;
	messageOut->MessageWord1.EVENT_IN_NED_VELOCITY_6203 = messageIn.MessageWord1.EVENT_IN_NED_VELOCITY_6203;
	messageOut->MessageWord1.TIMEMARK_PPS_OUT_2201_1HZ = messageIn.MessageWord1.TIMEMARK_PPS_OUT_2201_1HZ;
	messageOut->MessageWord1.TIMEMARK_BLOCK4_2211_1HZ = messageIn.MessageWord1.TIMEMARK_BLOCK4_2211_1HZ;
	messageOut->MessageWord1.EVENT_IN_EULER_ATTITUDE_6204 = messageIn.MessageWord1.EVENT_IN_EULER_ATTITUDE_6204;
	messageOut->MessageWord1.EVENT_IN_FULL_6205 = messageIn.MessageWord1.EVENT_IN_FULL_6205;
	messageOut->MessageWord1.AUTOPILOT_FLT_CTRL_2301_FC = messageIn.MessageWord1.AUTOPILOT_FLT_CTRL_2301_FC;
	messageOut->MessageWord1.UNFILTERED_INS_DATA_USR_REF_6311_NC = messageIn.MessageWord1.UNFILTERED_INS_DATA_USR_REF_6311_NC;
	messageOut->MessageWord1.UNFILTERED_INS_DATA_2311_NC = messageIn.MessageWord1.UNFILTERED_INS_DATA_2311_NC;
	messageOut->MessageWord1.SAVE_CONFIGURATION_TO_FLASH = messageIn.MessageWord1.SAVE_CONFIGURATION_TO_FLASH;
	messageOut->MessageWord1.NAV_OUT_2401_100HZ = messageIn.MessageWord1.NAV_OUT_2401_100HZ;
	messageOut->MessageWord1.NAV_OUT_2401_50HZ = messageIn.MessageWord1.NAV_OUT_2401_50HZ;
	messageOut->MessageWord1.NAV_OUT_2401_1HZ = messageIn.MessageWord1.NAV_OUT_2401_1HZ;
	messageOut->MessageWord1.SMOOTH_NAV_OUT_2402_100HZ = messageIn.MessageWord1.SMOOTH_NAV_OUT_2402_100HZ;
	messageOut->MessageWord1.SMOOTH_NAV_OUT_2402_50HZ = messageIn.MessageWord1.SMOOTH_NAV_OUT_2402_50HZ;
	messageOut->MessageWord1.SMOOTH_NAV_OUT_2402_1HZ = messageIn.MessageWord1.SMOOTH_NAV_OUT_2402_1HZ;
	messageOut->MessageWord1.INS_ERR_EST_2411_5HZ = messageIn.MessageWord1.INS_ERR_EST_2411_5HZ;
	messageOut->MessageWord1.INS_ERR_EST_2411_1HZ = messageIn.MessageWord1.INS_ERR_EST_2411_1HZ;
	messageOut->MessageWord1.KF_NAV_MEAS_ST_2421_5HZ = messageIn.MessageWord1.KF_NAV_MEAS_ST_2421_5HZ;
	messageOut->MessageWord1.KF_NAV_MEAS_ST_2421_1HZ = messageIn.MessageWord1.KF_NAV_MEAS_ST_2421_1HZ;
	messageOut->MessageWord1.KF_NAV_SOL_ST_DEV_2422_5HZ = messageIn.MessageWord1.KF_NAV_SOL_ST_DEV_2422_5HZ;
	messageOut->MessageWord1.KF_NAV_SOL_ST_DEV_2422_1HZ = messageIn.MessageWord1.KF_NAV_SOL_ST_DEV_2422_1HZ;
	messageOut->MessageWord1.GPS_AID_MEAS_ERR_EST_2424_5HZ = messageIn.MessageWord1.GPS_AID_MEAS_ERR_EST_2424_5HZ;
	messageOut->MessageWord1.GPS_AID_MEAS_ERR_EST_2424_1HZ = messageIn.MessageWord1.GPS_AID_MEAS_ERR_EST_2424_1HZ;
	messageOut->MessageWord1.TR_ALIGN_AID_MEAS_ERR_EST_2425_5HZ = messageIn.MessageWord1.TR_ALIGN_AID_MEAS_ERR_EST_2425_5HZ;
	messageOut->MessageWord1.TR_ALIGN_AID_MEAS_ERR_EST_2425_1HZ = messageIn.MessageWord1.TR_ALIGN_AID_MEAS_ERR_EST_2425_1HZ;

	messageOut->MessageWord2.GPS_CONFIG_2002 = messageIn.MessageWord2.GPS_CONFIG_2002;
	messageOut->MessageWord2.MOTION_DETECT_6111 = messageIn.MessageWord2.MOTION_DETECT_6111;
	messageOut->MessageWord2.MAG_AID_MEAS_ERR_EST_2427_5HZ = messageIn.MessageWord2.MAG_AID_MEAS_ERR_EST_2427_5HZ;
	messageOut->MessageWord2.MAG_AID_MEAS_ERR_EST_2427_1HZ = messageIn.MessageWord2.MAG_AID_MEAS_ERR_EST_2427_1HZ;
	messageOut->MessageWord2.BAROMETRIC_AID_MEAS_ERR_EST_2426_5HZ = messageIn.MessageWord2.BAROMETRIC_AID_MEAS_ERR_EST_2426_5HZ;
	messageOut->MessageWord2.BAROMETRIC_AID_MEAS_ERR_EST_2426_1HZ = messageIn.MessageWord2.BAROMETRIC_AID_MEAS_ERR_EST_2426_1HZ;
	messageOut->MessageWord2.INERTIAL_SENSOR_ERR_STDV_2423_5HZ = messageIn.MessageWord2.INERTIAL_SENSOR_ERR_STDV_2423_5HZ;
	messageOut->MessageWord2.INERTIAL_SENSOR_ERR_STDV_2423_1HZ = messageIn.MessageWord2.INERTIAL_SENSOR_ERR_STDV_2423_1HZ;
	messageOut->MessageWord2.GPS_CHANNEL_STATUS_2501_SET = messageIn.MessageWord2.GPS_CHANNEL_STATUS_2501_SET;
	messageOut->MessageWord2.GPS_ALMANAC_2511_SET = messageIn.MessageWord2.GPS_ALMANAC_2511_SET;
	messageOut->MessageWord2.GPS_EPHEMERIS_2512_SET = messageIn.MessageWord2.GPS_EPHEMERIS_2512_SET;
	messageOut->MessageWord2.GPS_ONAV_2513_SET = messageIn.MessageWord2.GPS_ONAV_2513_SET;
	messageOut->MessageWord2.GPS_IONO_2514_SET = messageIn.MessageWord2.GPS_IONO_2514_SET;
	messageOut->MessageWord2.GNSS_STATES_ERR_EST_6424_5HZ = messageIn.MessageWord2.GNSS_STATES_ERR_EST_6424_5HZ;
	messageOut->MessageWord2.GNSS_NORM_MEAS_RESID_6428_5HZ = messageIn.MessageWord2.GNSS_NORM_MEAS_RESID_6428_5HZ;
	messageOut->MessageWord2.ODO_NORM_MEAS_STATES_6438_5HZ = messageIn.MessageWord2.ODO_NORM_MEAS_STATES_6438_5HZ;
	messageOut->MessageWord2.IBIT_RESULTS_2601 = messageIn.MessageWord2.IBIT_RESULTS_2601;
	messageOut->MessageWord2.BIT_HISTORY_2611 = messageIn.MessageWord2.BIT_HISTORY_2611;
	messageOut->MessageWord2.GNSS_PVT_6108 = messageIn.MessageWord2.GNSS_PVT_6108;
	messageOut->MessageWord2.GEODETIC_POSITION_6403 = messageIn.MessageWord2.GEODETIC_POSITION_6403;
	messageOut->MessageWord2.EULER_ATTITUDE_6405 = messageIn.MessageWord2.EULER_ATTITUDE_6405;
	messageOut->MessageWord2.NED_VELOCITY_6504 = messageIn.MessageWord2.NED_VELOCITY_6504;
	messageOut->MessageWord2.GNSS_ATT_6109 = messageIn.MessageWord2.GNSS_ATT_6109;
	messageOut->MessageWord2.ODO_DPOS_6110 = messageIn.MessageWord2.ODO_DPOS_6110;
	messageOut->MessageWord2.APP_SP_OUTPUT_1_2901 = messageIn.MessageWord2.APP_SP_OUTPUT_1_2901;
	messageOut->MessageWord2.GNSS_SINGLE_ANT_ATT_6112 = messageIn.MessageWord2.GNSS_SINGLE_ANT_ATT_6112;
	messageOut->MessageWord2.VEHICLE_BODY_RATES_AND_ACCELS_6406_100HZ = messageIn.MessageWord2.VEHICLE_BODY_RATES_AND_ACCELS_6406_100HZ;
	messageOut->MessageWord2.GPS_NORM_MEAS_RESID_2428_5HZ = messageIn.MessageWord2.GPS_NORM_MEAS_RESID_2428_5HZ;
	messageOut->MessageWord2.GPS_NORM_MEAS_RESID_2428_1HZ = messageIn.MessageWord2.GPS_NORM_MEAS_RESID_2428_1HZ;
	messageOut->MessageWord2.TR_NORM_MEAS_RESID_2429_5HZ = messageIn.MessageWord2.TR_NORM_MEAS_RESID_2429_5HZ;
	messageOut->MessageWord2.TR_NORM_MEAS_RESID_2429_1HZ = messageIn.MessageWord2.TR_NORM_MEAS_RESID_2429_1HZ;
	messageOut->MessageWord2.LOG_MESSAGE_9900 = messageIn.MessageWord2.LOG_MESSAGE_9900;

	messageOut->MessageWord3.LED_STATUS_MESSAGE_6601 = messageIn.MessageWord3.LED_STATUS_MESSAGE_6601;
	messageOut->MessageWord3.HGUIDE_INSTALL_6003 = messageIn.MessageWord3.HGUIDE_INSTALL_6003;
	messageOut->MessageWord3.ANTENNA_CONNECTED_6508 = messageIn.MessageWord3.ANTENNA_CONNECTED_6508;
	messageOut->MessageWord3.DEVICE_IDENTIFICATION_6001 = messageIn.MessageWord3.DEVICE_IDENTIFICATION_6001;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_4 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_4;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_5 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_5;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_6 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_6;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_7 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_7;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_8 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_8;
	messageOut->MessageWord3.REAL_TIME_GOOD_TO_GO_6651 = messageIn.MessageWord3.REAL_TIME_GOOD_TO_GO_6651;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_10 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_10;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_11 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_11;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_12 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_12;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_13 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_13;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_14 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_14;
	messageOut->MessageWord3.NORTEK_DVL_BOTTOM_TRACK_6721 = messageIn.MessageWord3.NORTEK_DVL_BOTTOM_TRACK_6721;
	messageOut->MessageWord3.NORTEK_DVL_WATER_TRACK_6722 = messageIn.MessageWord3.NORTEK_DVL_WATER_TRACK_6722;
	messageOut->MessageWord3.NORTEK_DVL_CURRENT_PROFILE_6723 = messageIn.MessageWord3.NORTEK_DVL_CURRENT_PROFILE_6723;
	messageOut->MessageWord3.NORTEK_DVL_CURRENT_PROFILE_VELOCITY_DATA_6724 = messageIn.MessageWord3.NORTEK_DVL_CURRENT_PROFILE_VELOCITY_DATA_6724;
	messageOut->MessageWord3.NORTEK_DVL_CURRENT_PROFILE_AMPLITUDE_DATA_6725 = messageIn.MessageWord3.NORTEK_DVL_CURRENT_PROFILE_AMPLITUDE_DATA_6725;
	messageOut->MessageWord3.NORTEK_DVL_CURRENT_PROFILE_CORRELATION_DATA_6726 = messageIn.MessageWord3.NORTEK_DVL_CURRENT_PROFILE_CORRELATION_DATA_6726;
	messageOut->MessageWord3.NORTEK_DVL_NORM_MEAS_RESID_STATES_ERR_EST_6738 = messageIn.MessageWord3.NORTEK_DVL_NORM_MEAS_RESID_STATES_ERR_EST_6738;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_22 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_22;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_23 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_23;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_24 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_24;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_25 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_25;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_26 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_26;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_27 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_27;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_28 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_28;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_29 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_29;
	messageOut->MessageWord3.MESSAGE_WORD_3_BIT_30 = messageIn.MessageWord3.MESSAGE_WORD_3_BIT_30;
	messageOut->MessageWord3.PROCESS_LOAD_9910 = messageIn.MessageWord3.PROCESS_LOAD_9910;

	messageOut->MessageWord4.GNSS_RECEIVER_VERSION_5001 = messageIn.MessageWord4.GNSS_RECEIVER_VERSION_5001;
	messageOut->MessageWord4.MESSAGE_WORD_4_BIT_2 = messageIn.MessageWord4.MESSAGE_WORD_4_BIT_2;
	messageOut->MessageWord4.MESSAGE_WORD_4_BIT_3 = messageIn.MessageWord4.MESSAGE_WORD_4_BIT_3;
	messageOut->MessageWord4.GNSS_RECEIVER_STATUS_5012 = messageIn.MessageWord4.GNSS_RECEIVER_STATUS_5012;
	messageOut->MessageWord4.MESSAGE_WORD_4_BIT_4 = messageIn.MessageWord4.MESSAGE_WORD_4_BIT_4;
	messageOut->MessageWord4.GNSS_RANGE_5101_1Hz = messageIn.MessageWord4.GNSS_RANGE_5101_1Hz;
	messageOut->MessageWord4.GNSS_RANGE_5101_5Hz = messageIn.MessageWord4.GNSS_RANGE_5101_5Hz;
	messageOut->MessageWord4.GNSS_RANGE_5101_10Hz = messageIn.MessageWord4.GNSS_RANGE_5101_10Hz;
	messageOut->MessageWord4.MESSAGE_WORD_4_BIT_8 = messageIn.MessageWord4.MESSAGE_WORD_4_BIT_8;
	messageOut->MessageWord4.GNSS_SATELLITE_POSITION_5102_1Hz = messageIn.MessageWord4.GNSS_SATELLITE_POSITION_5102_1Hz;
	messageOut->MessageWord4.GNSS_SATELLITE_POSITION_5102_5Hz = messageIn.MessageWord4.GNSS_SATELLITE_POSITION_5102_5Hz;
	messageOut->MessageWord4.GNSS_SATELLITE_POSITION_5102_10Hz = messageIn.MessageWord4.GNSS_SATELLITE_POSITION_5102_10Hz;
	messageOut->MessageWord4.MESSAGE_WORD_4_BIT_12 = messageIn.MessageWord4.MESSAGE_WORD_4_BIT_12;
	messageOut->MessageWord4.GPS_EPHEMERIS_5103 = messageIn.MessageWord4.GPS_EPHEMERIS_5103;
	messageOut->MessageWord4.GLONASS_EPHEMERIS_5104 = messageIn.MessageWord4.GLONASS_EPHEMERIS_5104;
	messageOut->MessageWord4.GALILEO_EPHEMERIS_5105 = messageIn.MessageWord4.GALILEO_EPHEMERIS_5105;
	messageOut->MessageWord4.BEIDOU_EPHEMERIS_5106 = messageIn.MessageWord4.BEIDOU_EPHEMERIS_5106;
	messageOut->MessageWord4.MESSAGE_WORD_4_BIT_17 = messageIn.MessageWord4.MESSAGE_WORD_4_BIT_17;
	messageOut->MessageWord4.MESSAGE_WORD_4_BIT_18 = messageIn.MessageWord4.MESSAGE_WORD_4_BIT_18;
	messageOut->MessageWord4.GNSS_PVT_MEASUREMENT_5108_1Hz = messageIn.MessageWord4.GNSS_PVT_MEASUREMENT_5108_1Hz;
	messageOut->MessageWord4.GNSS_PVT_MEASUREMENT_5108_5Hz = messageIn.MessageWord4.GNSS_PVT_MEASUREMENT_5108_5Hz;
	messageOut->MessageWord4.GNSS_PVT_MEASUREMENT_5108_10Hz = messageIn.MessageWord4.GNSS_PVT_MEASUREMENT_5108_10Hz;
	messageOut->MessageWord4.GNSS_PVT_MEASUREMENT_5108_20Hz = messageIn.MessageWord4.GNSS_PVT_MEASUREMENT_5108_20Hz;
	messageOut->MessageWord4.MESSAGE_WORD_4_BIT_23 = messageIn.MessageWord4.MESSAGE_WORD_4_BIT_23;
	messageOut->MessageWord4.GNSS_ATTITUDE_MEASUREMENT_5109_1Hz = messageIn.MessageWord4.GNSS_ATTITUDE_MEASUREMENT_5109_1Hz;
	messageOut->MessageWord4.GNSS_ATTITUDE_MEASUREMENT_5109_5Hz = messageIn.MessageWord4.GNSS_ATTITUDE_MEASUREMENT_5109_5Hz;
	messageOut->MessageWord4.GNSS_ATTITUDE_MEASUREMENT_5109_10Hz = messageIn.MessageWord4.GNSS_ATTITUDE_MEASUREMENT_5109_10Hz;
	messageOut->MessageWord4.GNSS_ATTITUDE_MEASUREMENT_5109_20Hz = messageIn.MessageWord4.GNSS_ATTITUDE_MEASUREMENT_5109_20Hz;
	messageOut->MessageWord4.MESSAGE_WORD_4_BIT_28 = messageIn.MessageWord4.MESSAGE_WORD_4_BIT_28;
	messageOut->MessageWord4.MESSAGE_WORD_4_BIT_29 = messageIn.MessageWord4.MESSAGE_WORD_4_BIT_29;
	messageOut->MessageWord4.GNSS_PPS_TIME_5201 = messageIn.MessageWord4.GNSS_PPS_TIME_5201;
	messageOut->MessageWord4.MESSAGE_WORD_4_BIT_31 = messageIn.MessageWord4.MESSAGE_WORD_4_BIT_31;
}

void Msg_1001_sub_callback(const hg_nav_node::Msg_1001::ConstPtr& Message)
{
	int sentBytes = 0;
	int status = 0;
	Msg_1001 message;

	convert(*Message,&message);

	status = message.Serialize(getTxBuffer(),getBufSize());

	if (status==0) {ROS_WARN("Message 0x1001 serialization failed!"); return;}
	sentBytes = write(getSerialHandle(), getTxBuffer(), message.MessageLength*4);

	ROS_INFO("0x1001 Sent %d/%d:",sentBytes,message.MessageLength*4);
	for (int i = 0; i < sentBytes; i = i + 4)
		ROS_DEBUG("%d:\t%.8x",i/4, *(uint32_t*)(TxBuffer + i));
	return;
}
