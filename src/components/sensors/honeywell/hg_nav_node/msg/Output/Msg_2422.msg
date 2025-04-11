# 0x2422 : KF Navigation Error Values

#
#  Kalman Filter error values and standard deviations of Navigation solution
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
	float32 ecef_pos_x_std_dev  # [m]
	float32 ecef_pos_y_std_dev  # [m]
	float32 ecef_pos_z_std_dev  # [m]
	float32 ecef_vel_x_std_dev  # [m/s]
	float32 ecef_vel_y_std_dev  # [m/s]
	float32 ecef_vel_z_std_dev  # [m/s]
	float32 attitude_roll_std_dev  # [rad]
	float32 attitude_pitch_std_dev  # [rad]
	float32 attitude_true_heading_std_dev  # [rad]
	float32 vehicle_body_x_axis_rotational_angle_std_dev 
	float32 vehicle_body_y_axis_rotational_angle_std_dev 
	float32 vehicle_body_z_axis_rotational_angle_std_dev 
	uint32 vehicle_body_x_axis_linear_acceleration_std_dev 
	uint32 vehicle_body_y_axis_linear_acceleration_std_dev 
	uint32 vehicle_body_z_axis_linear_acceleration_std_dev 
	float32 zero_velocity_x_norm_meas_resid 
	float32 zero_velocity_y_norm_meas_resid 
	float32 zero_velocity_z_norm_meas_resid 
	float32 zero_heading_change_norm_meas_resid 
