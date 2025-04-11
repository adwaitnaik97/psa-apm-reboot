#include <include/HGuideAPI.h>
#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "message_support.h"

// Include custom message
#include <hg_nav_node/Msg_6505.h>
#include <hg_nav_node/ins_mode_table_t.h>
#include <hg_nav_node/gps_mode_table_t.h>
#include <hg_nav_node/constellation_enum_t.h>
#include <hg_nav_node/ins_gnss_summary_t.h>
hg_nav_node::Msg_6505 msgStruct_6505;

bool Msg_6505_pub_initialized = false;

ros::Publisher Msg_6505_pub;
void init_6505(ros::NodeHandle * n){
	Msg_6505_pub = n->advertise<hg_nav_node::Msg_6505>(MSG_6505_PATH, 5);
	Msg_6505_pub_initialized = true;
	ROS_INFO("Starting pub %s",MSG_6505_PATH);
	return;
}

void stop_6505(void){
	Msg_6505_pub.shutdown();
	Msg_6505_pub_initialized = false;
	ROS_INFO("0x6505 stopped");
	return;
}

// Msg_6505 to Topic
void convert(Msg_6505 messageIn, hg_nav_node::Msg_6505 * messageOut)
{
	messageOut->AddressId = messageIn.AddressId;
	messageOut->MessageId = messageIn.MessageId;
	messageOut->MessageLength = messageIn.MessageLength;
	messageOut->Checksum = messageIn.Checksum;
	messageOut->Constellation.value = static_cast<uint8_t>(messageIn.Constellation);
	messageOut->Used_in_soln = messageIn.Used_in_soln;
	messageOut->Final_msg_in_set = messageIn.Final_msg_in_set;
	messageOut->prn_slot = messageIn.prn_slot;
	messageOut->elevation = messageIn.elevation;
	messageOut->azimuth = messageIn.azimuth;
	messageOut->freq_band_1 = messageIn.freq_band_1;
	messageOut->channel_1 = messageIn.channel_1;
	messageOut->carrier_noise_1 = messageIn.carrier_noise_1;
	messageOut->freq_band_2 = messageIn.freq_band_2;
	messageOut->channel_2 = messageIn.channel_2;
	messageOut->carrier_noise_2 = messageIn.carrier_noise_2;
	messageOut->freq_band_3 = messageIn.freq_band_3;
	messageOut->channel_3 = messageIn.channel_3;
	messageOut->carrier_noise_3 = messageIn.carrier_noise_3;
	messageOut->freq_band_4 = messageIn.freq_band_4;
	messageOut->channel_4 = messageIn.channel_4;
	messageOut->carrier_noise_4 = messageIn.carrier_noise_4;
	messageOut->freq_band_5 = messageIn.freq_band_5;
	messageOut->channel_5 = messageIn.channel_5;
	messageOut->carrier_noise_5 = messageIn.carrier_noise_5;
	messageOut->freq_band_6 = messageIn.freq_band_6;
	messageOut->channel_6 = messageIn.channel_6;
	messageOut->carrier_noise_6 = messageIn.carrier_noise_6;

	messageOut->InsGnssSummary.INSMode.value = static_cast<uint8_t>(messageIn.InsGnssSummary.INSMode);
	messageOut->InsGnssSummary.INSStatus = messageIn.InsGnssSummary.INSStatus;
	messageOut->InsGnssSummary.IMUStatus = messageIn.InsGnssSummary.IMUStatus;
	messageOut->InsGnssSummary.GNSSStatus = messageIn.InsGnssSummary.GNSSStatus;
	messageOut->InsGnssSummary.MotionDetectActive = messageIn.InsGnssSummary.MotionDetectActive;
	messageOut->InsGnssSummary.StationaryMeasurementsOn = messageIn.InsGnssSummary.StationaryMeasurementsOn;
	messageOut->InsGnssSummary.MDT1RotationRate = messageIn.InsGnssSummary.MDT1RotationRate;
	messageOut->InsGnssSummary.MDT2SpeedSTDV = messageIn.InsGnssSummary.MDT2SpeedSTDV;
	messageOut->InsGnssSummary.MDT3AngularRateInstantBit = messageIn.InsGnssSummary.MDT3AngularRateInstantBit;
	messageOut->InsGnssSummary.MDT4LinearAccelerationBit = messageIn.InsGnssSummary.MDT4LinearAccelerationBit;
	messageOut->InsGnssSummary.MDT5OdometerBit = messageIn.InsGnssSummary.MDT5OdometerBit;
	messageOut->InsGnssSummary.MDNavigationMode = messageIn.InsGnssSummary.MDNavigationMode;
	messageOut->InsGnssSummary.SnapbackStatus = messageIn.InsGnssSummary.SnapbackStatus;
	messageOut->InsGnssSummary.GPSMode.value = static_cast<uint8_t>(messageIn.InsGnssSummary.GPSMode);
	messageOut->gps_week = messageIn.gps_week;
	messageOut->gpsTov = messageIn.gpsTov;
}

// Topic to Msg_6505
void convert(hg_nav_node::Msg_6505 messageIn, Msg_6505 * messageOut)
{
	messageOut->Checksum = messageIn.Checksum;
	messageOut->Constellation = static_cast<constellation_enum_t>(messageIn.Constellation.value);
	messageOut->Used_in_soln = messageIn.Used_in_soln;
	messageOut->Final_msg_in_set = messageIn.Final_msg_in_set;
	messageOut->prn_slot = messageIn.prn_slot;
	messageOut->elevation = messageIn.elevation;
	messageOut->azimuth = messageIn.azimuth;
	messageOut->freq_band_1 = messageIn.freq_band_1;
	messageOut->channel_1 = messageIn.channel_1;
	messageOut->carrier_noise_1 = messageIn.carrier_noise_1;
	messageOut->freq_band_2 = messageIn.freq_band_2;
	messageOut->channel_2 = messageIn.channel_2;
	messageOut->carrier_noise_2 = messageIn.carrier_noise_2;
	messageOut->freq_band_3 = messageIn.freq_band_3;
	messageOut->channel_3 = messageIn.channel_3;
	messageOut->carrier_noise_3 = messageIn.carrier_noise_3;
	messageOut->freq_band_4 = messageIn.freq_band_4;
	messageOut->channel_4 = messageIn.channel_4;
	messageOut->carrier_noise_4 = messageIn.carrier_noise_4;
	messageOut->freq_band_5 = messageIn.freq_band_5;
	messageOut->channel_5 = messageIn.channel_5;
	messageOut->carrier_noise_5 = messageIn.carrier_noise_5;
	messageOut->freq_band_6 = messageIn.freq_band_6;
	messageOut->channel_6 = messageIn.channel_6;
	messageOut->carrier_noise_6 = messageIn.carrier_noise_6;

	messageOut->InsGnssSummary.INSMode = static_cast<ins_mode_table_t>(messageIn.InsGnssSummary.INSMode.value);
	messageOut->InsGnssSummary.INSStatus = messageIn.InsGnssSummary.INSStatus;
	messageOut->InsGnssSummary.IMUStatus = messageIn.InsGnssSummary.IMUStatus;
	messageOut->InsGnssSummary.GNSSStatus = messageIn.InsGnssSummary.GNSSStatus;
	messageOut->InsGnssSummary.MotionDetectActive = messageIn.InsGnssSummary.MotionDetectActive;
	messageOut->InsGnssSummary.StationaryMeasurementsOn = messageIn.InsGnssSummary.StationaryMeasurementsOn;
	messageOut->InsGnssSummary.MDT1RotationRate = messageIn.InsGnssSummary.MDT1RotationRate;
	messageOut->InsGnssSummary.MDT2SpeedSTDV = messageIn.InsGnssSummary.MDT2SpeedSTDV;
	messageOut->InsGnssSummary.MDT3AngularRateInstantBit = messageIn.InsGnssSummary.MDT3AngularRateInstantBit;
	messageOut->InsGnssSummary.MDT4LinearAccelerationBit = messageIn.InsGnssSummary.MDT4LinearAccelerationBit;
	messageOut->InsGnssSummary.MDT5OdometerBit = messageIn.InsGnssSummary.MDT5OdometerBit;
	messageOut->InsGnssSummary.MDNavigationMode = messageIn.InsGnssSummary.MDNavigationMode;
	messageOut->InsGnssSummary.SnapbackStatus = messageIn.InsGnssSummary.SnapbackStatus;
	messageOut->InsGnssSummary.GPSMode = static_cast<gps_mode_table_t>(messageIn.InsGnssSummary.GPSMode.value);
	messageOut->gps_week = messageIn.gps_week;
	messageOut->gpsTov = messageIn.gpsTov;
}

void Msg_6505_pub_callback(uint8_t * buffer)
{
	Msg_6505 Message;

	int status = 0;
	status = Message.Deserialize(buffer,getBufSize());

	 if (status != 0) {ROS_WARN("Message 0x6505 deserialization failed! %d returned",status); return;}
	convert(Message, &msgStruct_6505);
	ROS_DEBUG("Message 0x6505 Received");

	// Initialize Publisher if not initialized yet
	if (Msg_6505_pub_initialized == false){
		init_6505(getRosHandle());}
	// Publish the message
	Msg_6505_pub.publish(msgStruct_6505);
	return;
}
