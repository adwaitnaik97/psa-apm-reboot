# 0x1003 : GPS Initialization and Control

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	uint32 anti_jam_selection 
	uint32 gps_antenna_spin 
	uint32 constellation_type 
	uint32 acquisition_state_selection 
	uint32 ionosphere_corrections 
	uint32 post_launch_auto_transition_to_trk 
	uint32 set_back_shock 
	float32 position_uncertainty  # Position Uncertainty [m]
	float32 velocity_uncertainty  # Velocity Uncertainty [m/s]
	float32 time_uncertainty  # Time Uncertainty [s]
	uint32 number_of_acquisition_attempts 
	float32 gps_oscillator_calibration_jitter  # [ns]
	uint32 gps_oscillator_calibration 
	uint32 gps_code_select 
	uint32 gps_antenna_frequency_cmd 
	uint32 gps_svid_deselect_1_4 
	uint32 gps_svid_deselect_1_4_2 
	float32 pre_launch_temperature_ramp  # [deg C / min]
	float32 post_launch_temperature_ramp  # [deg C / min]
	float32 post_launch_power_on_time_delay  # [s]
	float32 post_launch_power_on_time_delay_uncertainty  # [ms]
