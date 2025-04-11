#ifndef __HGuideAPI_Msg_FA_h__
#define __HGuideAPI_Msg_FA_h__
#pragma once

#include <cstdint>

#include <include/sensor_axes_t.h>


// 0xFA : Status Message - Sensor Health
class HGUIDE_DLL Msg_FA
{
public:
	Msg_FA();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 12;}

public:
	static const uint8_t SyncByte = 0x0E; // IMU Address
	static const uint8_t MessageID = 0xFA; // Message ID
	sensor_axes_t active_sensor_axes; // Active Sensor Axes
	sensor_axes_t saturated_sensor_axes; // Saturated Sensor Axes
	sensor_axes_t sensor_stat_fail; // Sensor statistics test failure flag
	sensor_axes_t sensor_temp_fail; // Sensor temperature test failure flag
	uint16_t Checksum; // Uint16 Checksum. Complement = False
};

#endif // __HGuideAPI_Msg_FA_h__
