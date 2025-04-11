# 0x6311 : Unfiltered Inertial Data User Selected Reference Frame

#
#  Inertial data output in a user defined reference frame.
# 

# Variables declaration
	uint32 AddressId  # Start of Message Flag
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length in 32-bit word
	uint32 Checksum  # Checksum
	float64 systemTov  # [s] System Time of Validity
	ins_gnss_summary_t InsGnssSummary  # INS/GPS BIT Summary
	float64 delta_theta_x  # [rad] Delta Theta X
	float64 delta_theta_y  # [rad] Delta Theta Y
	float64 delta_theta_z  # [rad] Delta Theta Z
	int32 gps_week  # GPS Week
	float64 delta_velocity_x  # [m/s] Delta Velocity X
	float64 delta_velocity_y  # [m/s] Delta Velocity Y
	float64 delta_velocity_z  # [m/s] Delta Velocity Z
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	coordinate_frame_t current_coordinate_frame  # Current Coordinate Frame
