# Word to define which trajectory aiding sources to be used
#
# A custom data type defining the list of possible trajectory aiding source
# the trajectory aiding sources 32-bit word .
# 1 = Valid
# 0 = Invalid
#
# custom trajectory_aiding_validity_discretes_t
	bool LATITUDE  # Bit 0 Least significant Bit
	bool LONGITUDE  # Bit 1
	bool ALTITUDE  # Bit 2
	bool BARO_ALTITUDE  # Bit 3
	bool TRUE_AIR_SPEED  # Bit 4
	bool VELOCITY_NORTH  # Bit 5
	bool VELOCITY_EAST  # Bit 6
	bool VELOCITY_DOWN  # Bit 7
	bool VELOCITY_X  # Bit 8
	bool VELOCITY_Y  # Bit 9
	bool VELOCITY_Z  # Bit 10
	bool ATTITUDE_ROLL  # Bit 11
	bool ATTITUDE_PITCH  # Bit 12
	bool ATTITUDE_HEADING  # Bit 13
	bool ALTITUDE_TYPE  # Bit 24
	bool HEADING_TYPE  # Bit 25
