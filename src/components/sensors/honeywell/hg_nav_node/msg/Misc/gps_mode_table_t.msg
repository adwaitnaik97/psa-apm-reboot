# List of GPS Modes
# RTK modes are accessible automatically when supplied with RTCMv3+ corrections
# gps_mode_table_t - enumeration
uint8 value

#Pre-Defined ENUM Constants
	uint8 GPS_MODE_0_STANDALONE   = 0 #  Basic GNSS Signal
	uint8 GPS_MODE_1_SBAS         = 1 #  Satelite based augmentation system
	uint8 GPS_MODE_2_CODE_DGPS    = 2 #  Differential GPS
	uint8 GPS_MODE_3_RTK_FLOAT    = 3 #  RTK Float accuracy
	uint8 GPS_MODE_4_RTK_FIXED    = 4 #  RTK Fixed accuracy
	uint8 GPS_MODE_5_PPP_CONVERGE = 5 #  RESERVED - Converging PPP Solution
	uint8 GPS_MODE_6_PPP_LOCK     = 6 #  RESERVED - PPP Locked solution
	uint8 GPS_MODE_7_RTK_WIDELANE = 7 #  RESERVED - RTK Widelane accuracy
	uint8 GPS_MODE_15_INVALID     = 15 #  Invalid GNSS data
