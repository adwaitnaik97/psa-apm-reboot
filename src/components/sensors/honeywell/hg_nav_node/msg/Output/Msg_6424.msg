# 0x6424 : GNSS Measurement Error Estimates and Uncertainty

#
#  Standard deviations of various GNSS measurements (PVT lever arms, Antenna Boresight, GNSS position)
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
	float32 gps_pvt_lever_arm_x  # Word 22 :
	float32 gps_pvt_lever_arm_y  # Word 23 :
	float32 gps_pvt_lever_arm_z  # Word 24 :
	float32 gnss_att_brsght_state_x  # Word 25 :
	float32 gnss_att_brsght_state_y  # Word 26 :
	float32 gnss_att_brsght_state_z  # Word 27 :
	float32 gnss_att_brsght_x_stdv  # Word 28 :
	float32 gnss_att_brsght_y_stdv  # Word 29 :
	float32 gnss_att_brsght_z_stdv  # Word 30 :
	float32 gps_position_state_x_stdv  # Word 31 :
	float32 gps_position_state_y_stdv  # Word 32 :
	float32 gps_position_state_z_stdv  # Word 33 :
	float32 gps_pvt_lever_arm_x_stdv  # Word 36 :
	float32 gps_pvt_lever_arm_y_stdv  # Word 37 :
	float32 gps_pvt_lever_arm_z_stdv  # Word 38 :
	float32 gps_position_state_x  # Word 39 :
	float32 gps_position_state_y  # Word 40 :
	float32 gps_position_state_z  # Word 41 :
