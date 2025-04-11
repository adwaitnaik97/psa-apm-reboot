# 0x2411 : IMU Error Values

#
#  Inertial system error values
#  See 0x2423 for standard deviations
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
	float32 gyro_bias_error_x  # [deg/h]
	float32 gyro_bias_error_y  # [deg/h]
	float32 gyro_bias_error_z  # [deg/h]
	float32 gyro_bias_inrun_error_x  # [deg/h]
	float32 gyro_bias_inrun_error_y  # [deg/h]
	float32 gyro_bias_inrun_error_z  # [deg/h]
	float32 gyro_scale_factor_error_x  # [ppm]
	float32 gyro_scale_factor_error_y  # [ppm]
	float32 gyro_scale_factor_error_z  # [ppm]
	float32 gyro_nonorthogonality_error_yz  # [urad]
	float32 gyro_nonorthogonality_error_zx  # [urad]
	float32 gyro_nonorthogonality_error_xy  # [urad]
	float32 accelerometer_bias_error_x  # [ug]
	float32 accelerometer_bias_error_y  # [ug]
	float32 accelerometer_bias_error_z  # [ug]
	float32 accelerometer_bias_inrun_error_x  # [ug]
	float32 accelerometer_bias_inrun_error_y  # [ug]
	float32 accelerometer_bias_inrun_error_z  # [ug]
	float32 accelerometer_scale_factor_error_x  # [ppm]
	float32 accelerometer_scale_factor_error_y  # [ppm]
	float32 accelerometer_scale_factor_error_z  # [ppm]
	float32 accelerometer_nonorthogonality_error_yz  # [urad]
	float32 accelerometer_nonorthogonality_error_zx  # [urad]
	float32 accelerometer_nonorthogonality_error_xy  # [urad]
	float32 accelerometer_misalignment_error_x  # [urad]
	float32 accelerometer_misalignment_error_y  # [urad]
	float32 accelerometer_misalignment_error_z  # [urad]
	float32 accelerometer_scale_factor_nonlinearity_error_x  # [ppm]
	float32 accelerometer_scale_factor_nonlinearity_error_y  # [ppm]
	float32 accelerometer_scale_factor_nonlinearity_error_z  # [ppm]
