#ifndef __HGuideAPI_Msg_1003_h__
#define __HGuideAPI_Msg_1003_h__
#pragma once

#include <cstdint>


// 0x1003 : GPS Initialization and Control
class HGUIDE_DLL Msg_1003
{
public:
	Msg_1003();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 176;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x1003; // Message ID
	static const uint32_t MessageLength = 44; // Message Length
	uint32_t Checksum; // Checksum
	uint32_t anti_jam_selection;
	uint32_t gps_antenna_spin;
	uint32_t constellation_type;
	uint32_t acquisition_state_selection;
	uint32_t ionosphere_corrections;
	uint32_t post_launch_auto_transition_to_trk;
	uint32_t set_back_shock;
	float position_uncertainty; // Position Uncertainty [m]
	float velocity_uncertainty; // Velocity Uncertainty [m/s]
	float time_uncertainty; // Time Uncertainty [s]
	uint32_t number_of_acquisition_attempts;
	float gps_oscillator_calibration_jitter; // [ns]
	uint32_t gps_oscillator_calibration;
	uint32_t gps_code_select;
	uint32_t gps_antenna_frequency_cmd;
	uint32_t gps_svid_deselect_1_4;
	uint32_t gps_svid_deselect_1_4_2;
	float pre_launch_temperature_ramp; // [deg C / min]
	float post_launch_temperature_ramp; // [deg C / min]
	float post_launch_power_on_time_delay; // [s]
	float post_launch_power_on_time_delay_uncertainty; // [ms]
};

#endif // __HGuideAPI_Msg_1003_h__
