# 0x6508 : Antenna Connected Information

#
#  Message 0x6508 is used to provide the number of satellites in track and in lock
#  on each RF channel for the system. This can be used to infer if each antenna is 
#  properly connected and working correctly. 
#  This message is used to inform the user about the functionality of each antenna
# 

# Variables declaration
	uint32 AddressId  # Start of Message Flag
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length in 32-bit word
	uint32 Checksum  # Checksum
	float64 systemTov  # [sec] System Time of Validity
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32 gps_week  # GPS week number
	ins_gnss_summary_t InsGnssSummary  # INS / GNSS summary word
	sat_block_t RF1AntSatSearch  # RF1 Antenna Tracked satellites
	sat_block_t RF1AntSatLock  # RF1 Antenna Locked satellites
	sat_block_t RF2AntSatSearch  # RF2 Antenna Tracked satellites
	sat_block_t RF2AntSatLock  # RF2 Antenna Locked satellites
