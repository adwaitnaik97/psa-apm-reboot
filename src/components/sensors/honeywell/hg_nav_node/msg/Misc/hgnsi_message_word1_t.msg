# Word 1 of message configuration list
#
# A custom data type defining the list of possible output messages of the INS
# the Output messages 32-bit word 1.
#
# custom hgnsi_message_word1_t
	bool INS_CONFIG_2001  # Bit 0 : Enable 0x2001 - INS Configuration Word [Rate = one shot]
	bool INS_MODE_STATUS_2011_1HZ  # Bit 1 : Enable 0x2011 - INS Mode Status Built In Test [Rate = 1 Hz]
	bool TIMEMARK_EVENT_IN_6201  # Bit 2 : Enable 0x6201 - Time of validity of EventIn
	bool INS_INIT_STATUS_2021_1HZ  # Bit 3 : Enable 0x2021 - DO NOT USE
	bool SKYMAP_DATA_6505  # Bit 4 : Enable 0x6505 - Skyplot Messages
	bool GPS_PVT_OUT_2108_1HZ  # Bit 5 : Enable 0x2018 - GPS PVT output
	bool EVENT_IN_GEODETIC_POSITION_6202  # Bit 6 : Enable 0x6202 - Vehicle Geodetic Position of EventIn
	bool EVENT_IN_NED_VELOCITY_6203  # Bit 7 : Enable 0x6203 - Vehicle North/East/Down Velocity of EventIn
	bool TIMEMARK_PPS_OUT_2201_1HZ  # Bit 8 : Enable 0x2201 - Time of validity of the PPS out
	bool TIMEMARK_BLOCK4_2211_1HZ  # Bit 9 : Enable 0x2211 - DO NOT USE
	bool EVENT_IN_EULER_ATTITUDE_6204  # Bit 10 : Enable 0x6204 - Vehicle Attitude at EventIn
	bool EVENT_IN_FULL_6205  # Bit 11 : Enable 0x6205 - Vehicle Position, Velocity, Attitude and Time at EventIn
	bool AUTOPILOT_FLT_CTRL_2301_FC  # Bit 12 : Enable 0x2301 - High Rate Angular Velocity and Linear Acceleration
	bool UNFILTERED_INS_DATA_USR_REF_6311_NC  # Bit 13 : Enable 0x6311 - Delta Angle and Delta Velocity IMU Measurements in the IMU frame
	bool UNFILTERED_INS_DATA_2311_NC  # Bit 14 : Enable 0x2311 - Delta Angle and Delta Velocity IMU Measurements in the Vehicle f= 0;
	bool SAVE_CONFIGURATION_TO_FLASH  # Bit 15 : Save message configuration to flash (1 = Save to Flash | 0 = Temporary Cha= 0;
	bool NAV_OUT_2401_100HZ  # Bit 16 : Enable 0x2401 - Legacy Navigation output message [Rate = 100 Hz]
	bool NAV_OUT_2401_50HZ  # Bit 17 : Enable 0x2401 - Legacy Navigation output message [Rate = 50 Hz]
	bool NAV_OUT_2401_1HZ  # Bit 18 : Enable 0x2401 - Legacy Navigation output message [Rate = 1 Hz]
	bool SMOOTH_NAV_OUT_2402_100HZ  # Bit 19 : Enable 0x2402 - Legacy Smoothed Navigation output message [Rate = 100 Hz]
	bool SMOOTH_NAV_OUT_2402_50HZ  # Bit 20 : Enable 0x2402 - Legacy Smoothed Navigation output message [Rate = 50 Hz]
	bool SMOOTH_NAV_OUT_2402_1HZ  # Bit 21 : Enable 0x2402 - Legacy Smoothed Navigation output message [Rate = 1 Hz]
	bool INS_ERR_EST_2411_5HZ  # Bit 22 : Enable 0x2411 - DO NOT USE [Rate = 5 Hz]
	bool INS_ERR_EST_2411_1HZ  # Bit 23 : Enable 0x2411 - DO NOT USE [Rate = 1 Hz]
	bool KF_NAV_MEAS_ST_2421_5HZ  # Bit 24 : Enable 0x2421 - DO NOT USE [Rate = 5 Hz]
	bool KF_NAV_MEAS_ST_2421_1HZ  # Bit 25 : Enable 0x2421 - DO NOT USE [Rate = 1 Hz]
	bool KF_NAV_SOL_ST_DEV_2422_5HZ  # Bit 26 : Enable 0x2422 - DO NOT USE [Rate = 5 Hz]
	bool KF_NAV_SOL_ST_DEV_2422_1HZ  # Bit 27 : Enable 0x2422 - DO NOT USE [Rate = 1 Hz]
	bool GPS_AID_MEAS_ERR_EST_2424_5HZ  # Bit 28 : Enable 0x2424 - DO NOT USE [Rate = 5 Hz]
	bool GPS_AID_MEAS_ERR_EST_2424_1HZ  # Bit 29 : Enable 0x2424 - DO NOT USE [Rate = 1 Hz]
	bool TR_ALIGN_AID_MEAS_ERR_EST_2425_5HZ  # Bit 30 : Enable 0x2425 - DO NOT USE [Rate = 5 Hz]
	bool TR_ALIGN_AID_MEAS_ERR_EST_2425_1HZ  # Bit 31 : Enable 0x2425 - DO NOT USE [Rate = 1 Hz]
