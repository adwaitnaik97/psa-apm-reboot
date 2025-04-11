# 0x2424 : GPS Error Estimations

#
#  Error estimations for the GNSS system
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length in 32-bit Words
	uint32 Checksum  # Checksum
	ins_mode_table_t INSMode  # INS Mode table
	float64 systemTov  # Time since Power Up - Always Valid [sec]
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32 gps_week  # Number of weeks since Jan 6, 1980 GPS Week = -1 indicates GPS Week Unknown/Invalid
	float32 gps_clk_phase_error  # [m] GPS Clock Phase Error Estimate
	float32 gps_clk_freq_error  # [m/s/s] GPS Clock Acceleration Error Estimate
	float32 gps_clk_acc_error  # [m/s] GPS Clock Frequency Error Estimate
	float32 gps_clk_g_sensitivity_x  # [ppb/g] GPS Clock Frequency g-sensitivity X Error Estimate
	float32 gps_clk_g_sensitivity_y  # [ppb/g] GPS Clock Frequency g-sensitivity Y Error Estimate
	float32 gps_clk_g_sensitivity_z  # [ppb/g] GPS Clock Frequency g-sensitivity Z Error Estimate
	float32 min_meas_range_bias  # [m] Minimum Successful Measurement Range Bias Error Estimate
	float32 max_meas_range_bias  # [m] Maximum Successful Measurement Range Bias Error Estimate
	float32 gps_los_lever_arm_x  # [m] GPS LOS Leverarm X Error Estimate
	float32 gps_los_lever_arm_y  # [m] GPS LOS Leverarm Y Error Estimate
	float32 gps_los_lever_arm_z  # [m] GPS LOS Leverarm Z Error Estimate
	float32 gps_pvt_lever_arm_x  # [m] GPS PVT Leverarm X Error Estimate
	float32 gps_pvt_lever_arm_y  # [m] GPS PVT Leverarm Y Error Estimate
	float32 gps_pvt_lever_arm_z  # [m] GPS PVT Leverarm Z Error Estimate
	float32 gps_position_state_x  # [m] GPS PVT Position X State Estimate
	float32 gps_position_state_y  # [m] GPS PVT Position Y State Estimate
	float32 gps_position_state_z  # [m] GPS PVT Position Z State Estimate
	float32 gps_clk_g_sensitivity_x_stdv  # [ppb/g] GPS Clock Frequency g-sensitivity X STDV
	float32 gps_clk_g_sensitivity_y_stdv  # [ppb/g] GPS Clock Frequency g-sensitivity Y STDV
	float32 gps_clk_g_sensitivity_z_stdv  # [ppb/g] GPS Clock Frequency g-sensitivity Z STDV
	float32 min_meas_range_bias_stdv  # [m] Minimum Successful Measurement Range Bias STDV
	float32 max_meas_range_bias_stdv  # [m] Maximum Successful Measurement Range Bias STDV
	float32 gps_los_lever_arm_x_stdv  # [m] GPS LOS Leverarm X STDV
	float32 gps_los_lever_arm_y_stdv  # [m] GPS LOS Leverarm Y STDV
	float32 gps_los_lever_arm_z_stdv  # [m] GPS LOS Leverarm Z STDV
	float32 gps_pvt_lever_arm_x_stdv  # [m] GPS PVT Leverarm X STDV
	float32 gps_pvt_lever_arm_y_stdv  # [m] GPS PVT Leverarm Y STDV
	float32 gps_pvt_lever_arm_z_stdv  # [m] GPS PVT Leverarm Z STDV
	float32 gps_position_state_stdv_x  # [m] GPS PVT Position X State Estimate STDV
	float32 gps_position_state_stdv_y  # [m] GPS PVT Position Y State Estimate STDV
	float32 gps_position_state_stdv_z  # [m] GPS PVT Position Z State Estimate STDV
