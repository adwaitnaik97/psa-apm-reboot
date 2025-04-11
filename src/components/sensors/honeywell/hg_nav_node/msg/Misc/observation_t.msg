# Single Observation used in the GNSS Range message
# custom observation_t
	uint16 prn_number  # Satellite PRN number of range measurement
	uint16 glofreq  # (GLONASS Frequency + 7) (see GLONASS Slot and Frequency Numbers section of this manual)
	float64 pseudorange  # Pseudorange measurement (m)
	float32 pseudorange_stdv  # Pseudorange measurement standard deviation (m)
	float64 carrier_phase  # Carrier phase, in cycles (accumulated Doppler range)
	float32 carrier_phase_stdv  # Estimated carrier phase standard deviation (cycles)
	float32 doppler_freq  # Instantaneous carrier Doppler frequency (Hz)
	float32 carrier_to_noise  # Carrier to noise density ratio C/No = 10[log10(S/N0)] (dB-Hz)
	float32 locktime  # Number of seconds of continuous tracking (no cycle slipping)
	
	# Tracking State:
 	#0 = Idle
 	#1 = Sky search
 	#2 = Wide frequency band pull-in
 	#3 = Narrow frequency band pull-in
 	#4 = Phase lock loop
 	#6 = Channel steering
 	#7 = Frequency lock loop
 	#9 = Channel alignment
 	#10 = Code search
 	#11 = Aided phase lock loop
 	#23 = Side peak detection
 	#24 = FFT sky search
	uint8 tracking_state
	uint8 sv_channel_number  # (n-1) (0 = first, n = last) n depends on the receiver
	bool phase_lock  # 0 = Not locked, 1 = Locked
	bool parity_known  # 0 = Not known, 1 = Known
	bool code_lock  # 0 = Not locked, 1 = Locked
	
	# Correlator Type:
 	#0 = N/A
 	#1 = Standard correlator: spacing = 1 chip
 	#2 = Narrow correlator: spacing < 1 chip
 	#3 = Reserved
 	#4 = Pulse Aperture Correlator (PAC)
 	#5 = Narrow PAC
 	#6 = Reserved
	uint8 correlator_type
	constellation_enum_t system  # Satellite system
	bool grouping  # 0 = Not grouped, 1 = Grouped
	uint8 signal_type  # (Dependent on satellite system above) see table in 0x5101 detailed description
	bool primary_L1_channel  # 0 = Not primary, 1 = Primary
	bool carrier_phase_meas  # 0 = Half Cycle Not Added | 1 = Half Cycle Added
	bool digital_filter  # 0 = No digital filter | 1 = Digital filter
	bool PRN_lock_flag  # 0 = PRN Not Locked Out | 1 = PRN Locked Out
	bool channel_assignment  # 0 = Automatic, 1 = Forced
