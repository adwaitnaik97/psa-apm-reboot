# INS / GNSS Summary Word
#
# Common status describing INS and GNSS Modes along with brief information about system failures
# The data is synchronized accross all messages in a single frame.
#
# custom ins_gnss_summary_t
	ins_mode_table_t INSMode  # Current INS Mode of operation
	bool INSStatus  # INS System Status (0 = OK | 1 = Fail)
	bool IMUStatus  # IMU System Status (0 = OK | 1 = Fail)
	bool GNSSStatus  # GNSS System Status (0 = OK | 1 = Fail)
	bool MotionDetectActive  # Motion Detection Algorithm (0 = Not Active | 1 = Active)
	bool StationaryMeasurementsOn  # ZUPT received from external source (0x1002 message)
	bool MDT1RotationRate  # Angular Rate limit (0 = not met | 1 = met)
	bool MDT2SpeedSTDV  # Velocity limit (0 = not met | 1 = met)
	bool MDT3AngularRateInstantBit  # Angular Rate limit (0 = not met | 1 = met)
	bool MDT4LinearAccelerationBit  # Linear Acceleration limit (0 = not met | 1 = met)
	bool MDT5OdometerBit  # DMI ZUPT received (0 = not received | 1 = received)
	bool MDNavigationMode  # All Motion Detect Tests are set - wating for settling time (5s = default) (0 = not set | 1 = set)
	uint8 SnapbackStatus  # Snapback status enunciation; 0 = OUTPUT_SMOOTH, 1 = OUTPUT_RAW_NAV, 2 = ALWAYS_RAW_NAV
	gps_mode_table_t GPSMode  # Current GNSS Mode of operation
