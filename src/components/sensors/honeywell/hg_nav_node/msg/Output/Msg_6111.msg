# 0x6111 : Results of Motion detection algorithm in kalman filter

#
#  Results of 5 tests needed for motion detection algorithm (automatic Zero Velocity and Heading Change => ZUPT).
#  Test 1 Rotation.
#  Test 2 Speed.
#  Test 3 Angular Rate.
#  Test 4 Linear Acceleration.
#  Test 5 Odometer Data.
#  Settling time is indicating how are those test passed
# 

# Variables declaration
	uint32 AddressId  # Start of message flag
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message length
	uint32 Checksum  # Checksum
	float64 systemTov  # [sec] System Time of Validity
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	ins_gnss_summary_t InsGnssSummary  # INS / GNSS summary word
	float64 Latitude  # [rad] Latitude of ZUPT detection
	float64 Longitude  # [rad] Longitude of ZUPT detection
	float32 Test1_RotationX  # [rad/s] X axis (Roll) Value of rotation
	float32 Test1_RotationY  # [rad/s] Y axis (Pitch) Value of rotation
	float32 Test1_RotationZ  # [rad/s] Z axis (Yaw) Value of rotation
	float32 Test1_RotationNormRate  # [rad/s] Actual value of sum of Angular Rates in all axes
	float32 Test1_RotationNormRateThreshold  # [rad/s] Threshold of sum of rotation in all axes
	float32 Test2_SpeedValid  # Speed Valid
	float32 Test2_Speed  # [m/s] Actual sum of speed in all axes
	float32 Test2_SpeedThreshold  # [m/s] Threshold of sum of speed in all axes
	float32 Test3_AngularRateInstantX  # [rad/s] X Angular Rate Filtered by Instant FN 3dB filter
	float32 Test3_AngularRateInstantY  # [rad/s] Y Angular Rate Filtered by Instant FN 3dB filter
	float32 Test3_AngularRateInstantZ  # [rad/s] Z Angular Rate Filtered by Instant FN 3dB filter
	float32 Test3_InstantFilterBandwidth  # [Hz] Instant Filter Bandwidth for exit of ZUPT
	float32 Test3_AngularRateNominalX  # [rad/s] X Angular Rate Filtered by Nominal FN 3dB filter
	float32 Test3_AngularRateNominalY  # [rad/s] Y Angular Rate Filtered by Nominal FN 3dB filter
	float32 Test3_AngularRateNominalZ  # [rad/s] Z Angular Rate Filtered by Nominal FN 3dB filter
	float32 Test3_NominalFilterBandwidth  # [Hz] Nominal Filter Bandwidth for entering of ZUPT
	float32 Test3_AngularRateX  # [rad/s] X Actual Angular rate
	float32 Test3_AngularRateY  # [rad/s] Y Actual Angular rate
	float32 Test3_AngularRateZ  # [rad/s] Z Actual Angular rate
	float32 Test3_AngularRateThreshold  # [rad/s] Threshold of sum of angular rate in all axes
	float32 Test4_LinearAcceleration  # [m/s2] Actual value of sum of linar accelerations in all axes
	float32 Test4_LinearAccelerationThreshold  # [m/s2] Threshold of sum of linear accelerations in all axes
	float32 Test5_OdometerDeltaDistance  # [m] Change in distance based on Odometer Input
	float32 Test5_OdometerDeltaDistanceThreshold  # [m] Threshold of change in distance based on Odometer Input
	float32 SettlingTime_Odometer  # [s] Number of seconds when Odometer sent no pulses
	float32 SettlingTime_Stationary  # [s] Amount of time stationary
	float32 SettlingTime_Threshold  # [s] Threshold of stationary times`
