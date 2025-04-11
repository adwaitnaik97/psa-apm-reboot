# 0x5109 : GNSS Receiver Attitude/Heading Measurement Output

#
#  Attitude sensed from dual antenna GNSS receiver. The attitude is refered from Main (RF1) antenna to Aux (RF2) antenna
#  This message can be taken as an heading/pitch input by any HGuide INS system.
# 

# Variables declaration
	uint32 AddressId  # Start of message flag
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message length
	uint32 Checksum  # Checksum
	float64 systemTov  # [sec] System Time of Validity
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	float32 Roll  # [rad] Roll
	float32 Pitch  # [rad] Pitch
	float32 True_Heading  # [rad] True Heading
	float32 Roll_Stdv  # [rad] Roll Stdv
	float32 Pitch_Stdv  # [rad] Pitch Stdv
	float32 True_Heading_Stdv  # [rad] True Heading Stdv
	
	# Attitude Data Source
 	# 0 = Invalid Data
 	# 1 = Dual Antenna
 	# 2 = Track over ground
	uint8 Data_source
	gps_mode_table_t GPSMode  # GPS Mode of operation
