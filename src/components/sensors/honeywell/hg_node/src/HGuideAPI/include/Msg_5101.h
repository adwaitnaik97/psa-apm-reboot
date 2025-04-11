#ifndef __HGuideAPI_Msg_5101_h__
#define __HGuideAPI_Msg_5101_h__
#pragma once

#include <cstdint>

#include <include/constellation_enum_t.h>
#include <include/gps_mode_table_t.h>

// Single Observation used in the GNSS Range message
struct observation_t
{
	uint16_t prn_number; // Satellite PRN number of range measurement
	uint16_t glofreq; // (GLONASS Frequency + 7) (see GLONASS Slot and Frequency Numbers section of this manual)
	double pseudorange; // Pseudorange measurement (m)
	float pseudorange_stdv; // Pseudorange measurement standard deviation (m)
	double carrier_phase; // Carrier phase, in cycles (accumulated Doppler range)
	float carrier_phase_stdv; // [RESERVED] Estimated carrier phase standard deviation (cycles)
	float doppler_freq; // Instantaneous carrier Doppler frequency (Hz)
	float carrier_to_noise; // Carrier to noise density ratio C/No = 10[log10(S/N0)] (dB-Hz)
	float locktime; // Number of seconds of continuous tracking (no cycle slipping)
	
	// Tracking State:
 	//0 = Initial
 	//1 = Sky search
 	//2 = Finished pull-in
 	//4 = Phase Locked
 	//7 = Carrier Locked
 	//9 = Tracking with all locks
 	//10 = Code locked
	uint8_t tracking_state;
	uint8_t sv_channel_number; // (n-1) (0 = first, n = last) n depends on the receiver
	bool phase_lock; // 0 = Not locked, 1 = Locked
	bool parity_known; // 0 = Not known, 1 = Known
	bool code_lock; // 0 = Not locked, 1 = Locked
	
	// Device type:
 	//2 = g080 GNSS
	uint8_t correlator_type;
	constellation_enum_t system; // Satellite system
	bool grouping; // 0 = Not grouped, 1 = Grouped
	uint8_t signal_type; // (Dependent on satellite system above) see table in 0x5101 detailed description
	bool primary_L1_channel; // 0 = Not primary, 1 = Primary
	bool carrier_phase_meas; // 0 = Half Cycle Not Added | 1 = Half Cycle Added
	bool digital_filter; // 0 = No digital filter | 1 = Digital filter
	bool PRN_lock_flag; // 0 = PRN Not Locked Out | 1 = PRN Locked Out
	bool channel_assignment; // 0 = Automatic, 1 = Forced

	void Default()
	{
		prn_number = 0;
		glofreq = 0;
		pseudorange = 0;
		pseudorange_stdv = 0;
		carrier_phase = 0;
		carrier_phase_stdv = 0;
		doppler_freq = 0;
		carrier_to_noise = 0;
		locktime = 0;
		tracking_state = 0;
		sv_channel_number = 0;
		phase_lock = 0;
		parity_known = 0;
		code_lock = 0;
		correlator_type = 0;
		system = GPS;
		grouping = 0;
		signal_type = 0;
		primary_L1_channel = 0;
		carrier_phase_meas = 0;
		digital_filter = 0;
		PRN_lock_flag = 0;
		channel_assignment = 0;
	}
};


// 0x5101 : GNSS Range Data
//
// The 0x5101 message contains the channel measurements for the currently tracked satellites.
// The length of the message is based on the amount of observations. Check the Num_Observations field to properly decode the message.
// One should integrate data from multiple messages defined by `total_pages` and `page_index` fields. Restart the integration when encountered with `page_index == 0`.
// This message can be taken as GNSS Range input by any HGuide INS system.
// 
class HGUIDE_DLL Msg_5101
{
public:
	Msg_5101();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return (11+Num_Observations*11)*4;}
	//Set Repeated variable max size
	bool set_observation_size(uint32_t size);
	//Get current valid size of the variable (either max size or current size)
	uint32_t get_observation_size(void);

private:
	uint32_t max_observation_size = 0;

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x5101; // Message ID
	
	// Message Length [Number of 32-bit Words]
	// Make sure to recalculate based on the amount of observations
	uint32_t MessageLength;
	uint32_t Checksum; // 32-bit CRC
	double systemTov; // [sec] System Time of Validity
	double gpsTov; // [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32_t gps_week; // GPS Week number (-1 if invalid)
	gps_mode_table_t GPSMode; // GPS Mode of operation
	uint8_t Num_Observations; // Number of observations in this message - defines the size of the variable size message
	uint8_t total_pages; // Total number of pages (messages) with observations in this frame (count starts from 1)
	uint8_t page_index; // index of the current page (messages) (0 = first page)
	
	// GNSS Observations each 11 words long - Repeated block
 	// [API note] make sure to allocate memory for the observation via .set_observation_size(size) method
 	// if amount of ranges is higher than the allocated size, only allocated size shall be populated
	observation_t * observation;
};

#endif // __HGuideAPI_Msg_5101_h__
