# 0x2421 : Kalman filter measurement status

#
#  status of all available and used Kalman filter measurements.
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length in 32-bit Words
	uint32 Checksum  # Checksum
	float64 systemTov  # Time since Power Up - Always Valid [sec]
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32 gps_week  # Number of weeks since Jan 6, 1980 GPS Week = -1 indicates GPS Week Unknown/Invalid
	gps_aiding_status_t gps_pr_aiding_status  # Pseudo range aiding status
	gps_aiding_status_t gps_dr_aiding_status  # Delta range aiding status
	ta_aiding_status_t ta_aiding_status  # Position Aiding, Velocity Aiding, Attitude Aiding
	gnss_pvt_aiding_status_t gnss_pvt_aiding_status  # GNSS Position / Velocity
	misc_aiding_status_t misc_aiding_status  # Zero Velocity , Magnetic Heading , Static Heading, Baro Altimeter
	uint32 Available_Measurements  # Number of available measurements.
	uint32 Successfully_Processed  # Number of successfully processed measurements.
	uint32 Failed_Processing  # Number of failed measurements.
