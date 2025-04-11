# 0x1111 : ESpace Trajectory Aiding

#
#  Message is used to aid the GNSS receiver to acquire lock faster
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length [Number of 32-bit Words]
	uint32 Checksum  # 32-bit CRC
	trajectory_aiding_validity_discretes_t trajectory_aiding_validity  # Select which aiding values are valid
	float64 ESpaceTrajectoryTOV  # [s] Message Time of validity
	float64 Latitude  # [rad] Input Latitude
	float64 Longitude  # [rad] Input Longitude
	float64 Altitude_height_above_ellipsoid  # [m] Input Altitude above ellipsoid
	float64 Baro_altitude  # [m] Input barometer altitude
	float64 True_air_speed  # [m/s] Input True Air speed
	float64 velocity_north  # [m/s] Input North Velocity
	float64 velocity_east  # [m/s] Input East Velocity
	float64 velocity_down  # [m/s] Input Down Velocity
	float64 velocity_x  # [m/s] Input ECEF Velocity X
	float64 velocity_y  # [m/s] Input ECEF Velocity Y
	float64 velocity_z  # [m/s] Input ECEF Velocity Z
	float32 attitude_roll  # [rad] Input Roll Attitude
	float32 attitude_pitch  # [rad] Input Pitch Attitude
	float32 attitude_heading  # [rad] Input Heading Attitude
