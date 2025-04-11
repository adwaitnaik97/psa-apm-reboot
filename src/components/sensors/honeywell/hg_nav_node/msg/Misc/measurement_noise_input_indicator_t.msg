# Navigation noise input indicators for the INS
#
# A custom data type defining the possible measurement noise indicators for INS data
# the Measurement Noise input indicators 32-bit word .
#
# custom measurement_noise_input_indicator_t
	bool ENABLE_ZERO_VELOCITY_NOISE_INPUT  # Bit : 0 : Use zero_velocity_measurement_noise_stdv
	bool ENABLE_ZERO_HEADING_CHANGE_NOISE_INPUT  # Bit : 1 : Use zero_heading_change_measurement_noise_stdv
	bool SAVE_MEAS_NOISE_TO_FLASH  # Bit : 2 : Save configuration settings to non-volatile memory.
	bool ENABLE_MOTION_DETECT_THRESHOLD  # Bit : 8 : Use motion detection threshoulds.
	bool SAVE_MOTION_DETECT_THRESHOLD_TO_FLASH  # Bit : 9 : Save configuration settings to non-volatile memory.
	bool SAVE_AIDING_SOURCES_TO_FLASH  # Bit : 15 : Save configuration settings to non-volatile memory.
	bool ENABLE_DISABLE_NOISE_SET_TO_INPUT  # Bit : 31 : Set true, to use measurement noise terms.
