# List of INS modes
# defines the current INS mode of operation
# ins_mode_table_t - enumeration
uint8 value

#Pre-Defined ENUM Constants
	# INS system is waiting for position, velocity and attitude data to initialize 
	# When using single antenna send the 0x1401 with attitude to initialize
	uint8 INS_MODE_STANDBY             = 1
	uint8 INS_MODE_COARSE_LEVEL        = 2 #  INS system is computing the ROLL and PITCH attitude
	uint8 INS_MODE_UNAIDED_NAV         = 3 #  RESERVED Free Inertial navigation
	uint8 INS_MODE_AIDED_NAV           = 4 #  Aided navigation - normal operation
	uint8 INS_MODE_AHRS                = 5 #  RESERVED AHRS functionality only
	uint8 INS_MODE_PP_AIDED_FORWARD    = 6 #  RESERVED Post processed - forward solution
	uint8 INS_MODE_PP_AIDED_REVERSE    = 7 #  RESERVED Post processed - reverse solution
	uint8 INS_MODE_PP_AIDED_COMBINED   = 8 #  RESERVED Post processed - combined solution
	uint8 INS_MODE_IBIT                = 9 #  RESERVED Internal Built-in Test
	uint8 INS_MODE_SD_CARD_MAINTENANCE = 10 #  RESERVED SD card maintanence ongoing
