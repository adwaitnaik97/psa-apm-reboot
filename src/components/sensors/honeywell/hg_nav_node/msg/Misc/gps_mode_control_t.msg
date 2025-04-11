#
# Enumeration of available GPS Modes to command the unit to
#
# gps_mode_control_t - enumeration
uint8 value

#Pre-Defined ENUM Constants
	uint8 GPS_COMMAND_0_NO_CHANGE     = 0 #  No change to the current GPS mode
	uint8 GPS_COMMAND_1_MODE_TO_INIT  = 1
	uint8 GPS_COMMAND_2_MODE_TO_TRACK = 2
