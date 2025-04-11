# 0x2301 : Autopilot Flight Control Inertial Data

#
#  Inertial data output frame is defined by the  is outputted in the IMU body reference frame
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length in 32-bit Words
	uint32 Checksum  # Checksum
	float64 systemTov  # [s] System Time of Validity
	ins_gnss_summary_t InsGnssSummary  # INS/GPS BIT Summary
	float64 angular_rate_x  # [rad/s] Angular Rate X
	float64 angular_rate_y  # [rad/s] Angular Rate Y
	float64 angular_rate_z  # [rad/s] Angular Rate Z
	float64 linear_acceleration_x  # [m/s/s] Linear Acceleration X
	float64 linear_acceleration_y  # [m/s/s] Linear Acceleration Y
	float64 linear_acceleration_z  # [m/s/s] Linear Acceleration Z
	uint32 outputFrame  # 0 = IMU Body | 1 = INS Case | 2 = Vehicle Body
