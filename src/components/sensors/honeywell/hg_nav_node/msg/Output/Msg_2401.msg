# 0x2401 : Navigation

#
#  DO NOT USE, message deprecated, use 0x6403 for position, 0x6405 for attitude and 0x6504 for Velocity data
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	uint32 InsGnssSummary  # INS/GPS BIT Summary
	ins_mode_table_t INSMode  # INS Mode table
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	float64 systemTov  # Time since Power Up - Always Valid [sec]
	int32 gps_week  # [-] gps week number
	int32 utc_time_figure_of_merit 
	int32 gps_figure_of_merit 
	int32 ins_blended_figure_of_merit 
	float64 PositionTov  # [s] Position time of validity
	float64 Latitude  # [rad] Latitude position
	float64 Longitude  # [rad] Longitude position
	float64 AltitudeHeightAboveEllipsoid  # [m] Altitude Above Ellipsoid
	float64 AltitudeMeanSeaLevel  # [m] Altitude above Mean Sea Level
	float64 EcefPositionX  # [m] ECEF coordinates X position
	float64 EcefPositionY  # [m] ECEF coordinates Y position
	float64 EcefPositionZ  # [m] ECEF coordinates Z position
	float64 VelocityTov  # [s] Velocity time of validity
	float64 NorthVelocity  # [m/s] NED North Velocity
	float64 EastVelocity  # [m/s] NED East Velocity
	float64 DownVelocity  # [m/s] NED Down Velocity
	float64 EcefVelocityX  # [m/s] Velocity in X axis of ECEF Frame
	float64 EcefVelocityY  # [m/s] Velocity in Y axis of ECEF Frame
	float64 EcefVelocityZ  # [m/s] Velocity in Z axis of ECEF Frame
	float64 AttitudeTov  # [s] Attitude time of validity
	float32 EulerAnglesRoll  # [rad] Roll Angle of Vehicle Body
	float32 EulerAnglesPitch  # [rad] Pitch Angle of Vehicle Body
	float32 EulerAnglesTrueHeading  # [rad] Heading Angle of Vehicle Body
	float32 wander_angle 
	float64 DCM11  # DCM of Vehicle body to local body [xx]
	float64 DCM12  # DCM of Vehicle body to local body [xy]
	float64 DCM13  # DCM of Vehicle body to local body [xz]
	float64 DCM21  # DCM of Vehicle body to local body [yx]
	float64 DCM22  # DCM of Vehicle body to local body [yy]
	float64 DCM23  # DCM of Vehicle body to local body [yz]
	float64 DCM31  # DCM of Vehicle body to local body [zx]
	float64 DCM32  # DCM of Vehicle body to local body [zy]
	float64 DCM33  # DCM of Vehicle body to local body [zz]
	float32 angular_rate_x  # [rad/s] Angular Rate in Vehicle Body X
	float32 angular_rate_y  # [rad/s] Angular Rate in Vehicle Body Y
	float32 angular_rate_z  # [rad/s] Angular Rate in Vehicle Body Z
	float32 linear_acceleration_x  # [m/s/s] Linear Acceleration in Vehicle Body X
	float32 linear_acceleration_y  # [m/s/s] Linear Acceleration in Vehicle Body Y
	float32 linear_acceleration_z  # [m/s/s] Linear Acceleration in Vehicle Body Z
	int32 attitude_figure_of_merit 
	float32 q0_vehicle_body_to_ecef  # Vehicle body to ECEF quaternion [s]
	float32 q1_vehicle_body_to_ecef  # Vehicle body to ECEF quaternion [i]
	float32 q2_vehicle_body_to_ecef  # Vehicle body to ECEF quaternion [j]
	float32 q3_vehicle_body_to_ecef  # Vehicle body to ECEF quaternion [k]
