# 0x2428 : GNSS (PVT, PR/DR) Measurement Normalized Residuals

#
#  Residuals of INS PVT and Pseudorange / Deltarange measurements
# 

# Variables declaration
	uint32 AddressId  # Start of Message F
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length in
	uint32 Checksum  # Checksum
	ins_mode_table_t INSMode  # INS Mode table
	float64 systemTov  # Time since Power Up - Always Valid [sec]
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32 gps_week  # Number of weeks since Jan 6, 1980 GPS Week = -1 indicates GPS Week Unknown/Invalid
	float32 GNSS_Position_X  # [sigma]
	float32 GNSS_Position_Y  # [sigma]
	float32 GNSS_Position_Z  # [sigma]
	float32 GNSS_Velocity_X  # [sigma]
	float32 GNSS_Velocity_Y  # [sigma]
	float32 GNSS_Velocity_Z  # [sigma]
	uint32 Number_of_PR_channels 
	float32 GPS_PR_Channel_1  # [sigma]
	float32 GPS_PR_Channel_2  # [sigma]
	float32 GPS_PR_Channel_3  # [sigma]
	float32 GPS_PR_Channel_4  # [sigma]
	float32 GPS_PR_Channel_5  # [sigma]
	float32 GPS_PR_Channel_6  # [sigma]
	float32 GPS_PR_Channel_7  # [sigma]
	float32 GPS_PR_Channel_8  # [sigma]
	float32 GPS_PR_Channel_9  # [sigma]
	float32 GPS_PR_Channel_10  # [sigma]
	float32 GPS_PR_Channel_11  # [sigma]
	float32 GPS_PR_Channel_12  # [sigma]
	uint32 Number_of_DR_channels 
	float32 GPS_DR_Channel_1  # [sigma]
	float32 GPS_DR_Channel_2  # [sigma]
	float32 GPS_DR_Channel_3  # [sigma]
	float32 GPS_DR_Channel_4  # [sigma]
	float32 GPS_DR_Channel_5  # [sigma]
	float32 GPS_DR_Channel_6  # [sigma]
	float32 GPS_DR_Channel_7  # [sigma]
	float32 GPS_DR_Channel_8  # [sigma]
	float32 GPS_DR_Channel_9  # [sigma]
	float32 GPS_DR_Channel_10  # [sigma]
	float32 GPS_DR_Channel_11  # [sigma]
	float32 GPS_DR_Channel_12  # [sigma]
