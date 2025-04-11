#ifndef __HGuideAPI_Msg_B0_h__
#define __HGuideAPI_Msg_B0_h__
#pragma once

#include <cstdint>

#include <include/filter_config_t.h>
#include <include/sensor_axes_t.h>


// 0xB0 : 1Hz Status Message
class HGUIDE_DLL Msg_B0
{
public:
	Msg_B0();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 82;}

public:
	static const uint8_t SyncByte = 0x0E; // IMU Address
	static const uint8_t MessageID = 0xB0; // Message ID
	uint8_t fw_major; // Firmware Major Version
	uint8_t fw_minor; // Firmware Minor Version
	uint8_t device_type[4]; // Device Type
	uint32_t part_number; // Device Part Number
	uint32_t hardware_version; // Hardware Version
	uint32_t device_config; // Device Performance Grade
	uint8_t serial_number[8]; // Serial Number
	filter_config_t gyro_filter_config; // Gyro filter config
	filter_config_t accel_filter_config; // Gyro v Filter Numerator
	float gyro_range; // Gyro Range
	float accel_range; // Accel Range
	uint32_t hours_of_operation; // Hours of Operation
	uint32_t boot_count; // Boot Count
	uint32_t baud_rate; // Baud Rate
	uint32_t sampling_frequency; // Sampling Frequency
	uint32_t bytes_transmitted; // Bytes Transmitted since last 0xB0 transmission
	float percent_fr; // percent frame rate
	float percent_ffr; // percent filter frame rate
	float percent_cr; // percent control rate
	float percent_gr; // percent guidance rate
	float percent_100_fr; // percent 100Hz frame rate
	float percent_10_fr; // percent 10Hz frame rate
	float percent_1_fr; // percent 1Hz frame rate
	float device_current; // Current Consumption
	float device_voltage; // Input Voltage
	sensor_axes_t active_sensor_axes; // Active Sensor Axes
	sensor_axes_t saturated_sensor_axes; // Saturated Sensor Axes
	uint16_t Checksum; // Checksum. Complement = False
};

#endif // __HGuideAPI_Msg_B0_h__
