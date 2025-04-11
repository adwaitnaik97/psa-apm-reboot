# 0x6428 : GNSS Normalized Measurement Residuals

#
#  Residuals of GNSS measurements of position, velocity and attitude
# 

# Variables declaration
	uint32 AddressId  # Start of Message Flag
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length in 32-bit word
	uint32 Checksum  # Checksum
	ins_gnss_summary_t InsGnssSummary  # INS / GNSS summary word
	float64 systemTov  # [s] System Time of Validity
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32 gps_week  # [-] GPS Week no.
	float32 gps_pvt_pos_x  # Word 11 :
	float32 gps_pvt_pos_y  # Word 12 :
	float32 gps_pvt_pos_z  # Word 13 :
	float32 gps_pvt_vel_x  # Word 14 :
	float32 gps_pvt_vel_y  # Word 15 :
	float32 gps_pvt_vel_z  # Word 16 :
	float32 gnss_att_x  # Word 17 :
	float32 gnss_att_y  # Word 18 :
	float32 gnss_att_z  # Word 19 :
