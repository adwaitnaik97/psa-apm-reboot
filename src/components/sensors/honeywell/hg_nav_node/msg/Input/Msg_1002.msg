# 0x1002 : Command and Control Message

#
#  The 0x1002 message allows the end-user to command and control the HGuide device.
#  Read the detailed description below before usage. If unsure, consult with Honeywell.
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length [Number of 32-bit Words]
	uint32 Checksum  # Checksum
	ins_mode_control_t INS_Mode  # Word 4 : Commanded INS Mode
	gps_mode_control_t gps_mode_control  # Word 5 : RESERVED - Commanded GNSS Mode
	uint32 reset_ins_gps  # Word 6 : RESERVED - not implemented
	uint32 clear_bit_history  # Word 7 : 1 = Clear | 0 = Do nothing
	uint32 time_mark_pps_select  # Word 9 : RESERVED
	uint32 gps_inertial_aiding  # Word 10 : RESERVED
	
	# Word 11 : [s] amount of time spent in coarse leveling.
 	# Minimum of 0.01s. 2 Seconds Recommended.
	float32 coarse_level_duration
	navigation_aiding_sources_t navigation_aiding_sources  # Word 12 : Enable/Disable Navigation Aiding Sources
	float32 dyn_track_loop_start_time  # Word 13 : RESERVED
	float32 dyn_track_loop_control_ev_duration  # Word 14 : RESERVED
	
	# Word 17 : [s] from reception of message to shock event
 	# -1 disables an event if the start time has not yet been reached
	float32 inertial_shock_event_start_time
	float32 inertial_shock_event_duration  # Word 18 : [s] how long the shock event is going to last
	float32 inertial_shock_event_int_velocity_unc  # Word 19 : [m/s] how much uncertainty in nav do you accumulate
	float32 inertial_shock_event_int_attitude_unc  # Word 20 : [rad] how much uncertainty in nav do you accumulate
	
	# Word 23 : [s] from reception of message to vibration event
	# -1 disables an event if the start time has not yet been reached
	float32 inertial_vib_ev_start_time
	float32 inertial_vib_event_gyro_bias_unc  # Word 25 : [rad/h] how much uncertainty in nav do you accumulate
	float32 inertial_vib_event_accel_bias_unc  # Word 26 : [mg] how much uncertainty in nav do you accumulate
	uint32 store_input_motion_detect_zupting_to_flash  # Word 27 : 0 = Do nothing | 1 = Save the motion detect configuration to flash
	uint32 motion_detect_profile  # Word 28 :
	measurement_noise_input_indicator_t measurement_noise_input_indicator  # Word 29 :
	float32 zero_velocity_measurement_noise_stdv  # Word 30 : [m/s] Zero Velocity Update Measurement Noise Standard Deviation
	float32 zero_heading_change_measurement_noise_stdv  # Word 31 : [rad] Zero Heading Change Measurement Noise Standard Deviation
	float32 motion_detect_settling_time  # Word 32 : [sec] Zero Motion detection settling time.
	float32 motion_detect_dtheta_threshold  # Word 33 : [rad/s] Zero Motion rotation rate change threshold.
	float32 motion_detect_nominal_w_fn  # Word 34 : [Hz] Zero Motion Angular rate nominal filter
	float32 motion_detect_instant_w_fn  # Word 35 : [Hz] Zero Motion Angular rate instant filter
	float32 motion_detect_instant_threshold  # Word 36 : [rad/s] Zero Motion instant Angular rate threshold.
	float32 motion_detect_accel_threshold  # Word 37 : [m/s/s] Zero Motion instant Accelration threshold
	float32 motion_detect_speed_threshold  # Word 38 : [sigma]
	float32 motion_detect_odo_dist_threshold  # Word 39 : [m] Zero Motion odometer distance threshold
