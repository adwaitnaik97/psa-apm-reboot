# GPS Channel Status
#
# A custom data type which should be used to define a message which contains
# GPS Channel Status
#
# custom channel_status_t
	uint8 satellite_num  # Satellite number
	uint8 hrdw_channel_num  # Channel number
	
	# Channel State:
 	# 0 = STANDBY
 	# 1 = NORMAL
 	# 2 = Reserved
 	# 3 = CODE LOCK
 	# 4 = CARRIER LOCK
 	# 5 = CARRIER TRACK 
 	# 6 = Reserved
 	# 7 = REACQISITION
	uint8 channel_state
	uint8 channel_code  # Channel Code (0 = Not Tracking | 1 = C/A | 2 = C | 3 = P)
	bool used_in_pvt_sln  # true = Used in PVT | false = Not used in PVT
	float32 carrier_to_noise  # [dB-Hz] carrier to noise value
	uint8 channel_fqcy  # FREQ_NOT_TRACKING = 0 | L1 = 1 | L2 = 2
	uint8 delta_range_valid  # Delta Range Validity (1 = Valid | 0 = Invalid)
	uint8 pseudo_range_valid  # Pseudo Range Validity (1 = Valid | 0 = Invalid)
	float32 azimuth  # [rad] satellite azimuth
	float32 elevation  # [rad] satellite elevation
	uint8 iono_comp_data_src  # Iono compensation data source (UNKNOWN = 0 | IONO MODEL = 1 | DUAL FREQUENCY = 2 | INPUT EXTERNAL SOURCE = 3)
	uint8 sat_sel_desel  # SEL_CRIT_NOT_REC = 0 | SVID_SEL = 1 | SVID_DES = 2
	float32 pseudo_range_meas  # Pseudo Range
	float32 delta_range_meas  # Delta Range
	float32 sv_pos_x 
	float32 sv_pos_y 
	float32 sv_pos_z 
