# 0x2423 : IMU Standard deviation Values

#
#  Inertial system standard deviation values
#  see 0x2411 for error values
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
	float32 gyro_bias_x_std_dev  # [deg/h]
	float32 gyro_bias_y_std_dev  # [deg/h]
	float32 gyro_bias_z_std_dev  # [deg/h]
	float32 gyro_bias_inrun_x_std_dev  # [deg/h]
	float32 gyro_bias_inrun_y_std_dev  # [deg/h]
	float32 gyro_bias_inrun_z_std_dev  # [deg/h]
	float32 gyro_scale_factor_x_std_dev  # [ppm]
	float32 gyro_scale_factor_y_std_dev  # [ppm]
	float32 gyro_scale_factor_z_std_dev  # [ppm]
	float32 gyro_nonorthogonality_yz_std_dev  # [urad]
	float32 gyro_nonorthogonality_zx_std_dev  # [urad]
	float32 gyro_nonorthogonality_xy_std_dev  # [urad]
	float32 accelerometer_bias_x_std_dev  # [ug]
	float32 accelerometer_bias_y_std_dev  # [ug]
	float32 accelerometer_bias_z_std_dev  # [ug]
	float32 accelerometer_bias_inrun_x_std_dev  # [ug]
	float32 accelerometer_bias_inrun_y_std_dev  # [ug]
	float32 accelerometer_bias_inrun_z_std_dev  # [ug]
	float32 accelerometer_scale_factor_x_std_dev  # [ppm]
	float32 accelerometer_scale_factor_y_std_dev  # [ppm]
	float32 accelerometer_scale_factor_z_std_dev  # [ppm]
	float32 accelerometer_nonorthogonality_yz_std_dev  # [urad]
	float32 accelerometer_nonorthogonality_zx_std_dev  # [urad]
	float32 accelerometer_nonorthogonality_xy_std_dev  # [urad]
	float32 accelerometer_misalignment_x_std_dev  # [urad]
	float32 accelerometer_misalignment_y_std_dev  # [urad]
	float32 accelerometer_misalignment_z_std_dev  # [urad]
	float32 accelerometer_scale_factor_nonlinearity_x_std_dev  # [ppm]
	float32 accelerometer_scale_factor_nonlinearity_y_std_dev  # [ppm]
	float32 accelerometer_scale_factor_nonlinearity_z_std_dev  # [ppm]
