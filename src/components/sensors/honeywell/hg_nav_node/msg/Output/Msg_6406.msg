# 0x6406 : Vehicle Body Rates and Linear Accelerations

#
#  The 0x6406 message outputs the angular velocity and linear acceleration of the vehicle body frame with respect to the ECEF frame.  
#  Note : Earth rate has been removed w_IB = w_IE + w_EB
#  Note : Linear acceleration is the output and NOT specific force.  Gravity and Coriolis have been removed and what left is the best estimate of the second derivative of ECEF position with respect to time.
#  Note : the message provides zero outputs until the unit begins the process of navigating (entering Coarse Level or Aided Navigation)
# 

# Variables declaration
	uint32 AddressId  # Start of Message Flag
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length in 32-bit word
	uint32 Checksum  # Checksum
	float64 systemTov  # [s] System Time of Validity
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	float32 AngularRateX  # [rad/s] Angular Velocity in Vehicle Body with respect to the ECEF coordinatized in the Vehicle Body Frame [x component]
	float32 AngularRateY  # [rad/s] Angular Velocity in Vehicle Body with respect to the ECEF coordinatized in the Vehicle Body Frame [y component]
	float32 AngularRateZ  # [rad/s] Angular Velocity in Vehicle Body with respect to the ECEF coordinatized in the Vehicle Body Frame [z component]
	ins_gnss_summary_t InsGnssSummary  # INS / GNSS summary word
	float32 LinearAccelerationX  # [m/s/s] Linear Acceleration in the Vehicle Body [x component]
	float32 LinearAccelerationY  # [m/s/s] Linear Acceleration in the Vehicle Body [y component]
	float32 LinearAccelerationZ  # [m/s/s] Linear Acceleration in the Vehicle Body [z component]
