# 0x2311 : Unfiltered Inertial Data

#
#  This is the basic source of inertial data from the INS system.
#  Inertial data is output in the IMU body reference frame. See the 0x2001 Case to Nav Lever Arms and Quaternion for details.
#  Log this message and 0x2001 along with output from GNSS receiver (COM2) for post-processing
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length in 32-bit Words
	uint32 Checksum  # Checksum
	float64 systemTov  # [s] System Time of Validity
	ins_gnss_summary_t InsGnssSummary  # INS/GPS BIT Summary
	float64 delta_theta_x  # [rad] Delta Theta X
	float64 delta_theta_y  # [rad] Delta Theta Y
	float64 delta_theta_z  # [rad] Delta Theta Z
	int32 gps_week  # [-] Number of weeks since Jan 6, 1980 GPS Week = -1 indicates GPS Week Unknown/Invalid
	float64 delta_velocity_x  # [m/s] Delta Velocity X
	float64 delta_velocity_y  # [m/s] Delta Velocity Y
	float64 delta_velocity_z  # [m/s] Delta Velocity Z
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
