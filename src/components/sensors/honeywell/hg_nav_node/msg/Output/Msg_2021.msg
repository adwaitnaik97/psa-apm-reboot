# 0x2021 : INS Initialization Info

#
#  Reporting of the INS intialization data
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length in 32-bit Words
	uint32 Checksum  # Checksum
	float64 systemTov  # Time since Power Up - Always Valid [sec]
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32 gps_week  # Number of weeks since Jan 6, 1980 GPS Week = -1 indicates GPS Week Unknown/Invalid
	uint32 initialization_status_1 
	uint32 initialization_status_2 
	uint32 number_tm_pps_pulses_rcvd 
	uint32 number_time_msgs_rcvd 
	uint32 clock_calibration_status 
	float64 oscillator_drift 
	uint32 almanac_database 
	uint32 ephemeris_database 
	uint32 iono_database 
	uint32 subframe_4_database 
	uint32 subframe_5_database 
