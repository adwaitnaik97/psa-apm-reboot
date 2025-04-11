# 0x6438 : Kalman Filter Calibration Of Odometer Inputs

#
#  Odometer Calibration information. Notably Scale factor, lever arms and residuals
# 

# Variables declaration
	uint32 AddressId  # Start of Message Flag
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length in 32-bit word
	uint32 Checksum  # Checksum
	float64 systemTov  # [s] Time since INS power up
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32 gps_week  # [-] GPS week no.
	float32 residual_x  # [m] X axis KF measurement residual
	float32 residual_y  # [m] Y axis KF measurement residual
	float32 residual_z  # [m] Z axis KF measurement residual
	float32 scalefactor_correction  # [<0; 1>] Correction of Scalefactor (1 = 100% correction)
	float32 scalefactor_correction_pitchsens  # DO NOT USE [<0; 1>] Correction of Scalefactor (1 = 100% correction) sensed from pitch
	float32 scalefactor_correction_pitchratesens  # DO NOT USE [<0; 1>] Correction of Scalefactor (1 = 100% correction) sensed from pitch rate
	float32 boresight_yaw  # [rad] Odometer Boresight Yaw with respect to Navigation Frame
	float32 boresight_yaw_rollsens  # DO NOT USE [rad] Odometer Boresight Yaw with respect to Navigation Frame sensed from roll
	float32 boresight_yaw_rocksens  # DO NOT USE [rad] Odometer Boresight Yaw with respect to Navigation Frame sensed from rock
	float32 boresight_pitch  # [rad] Odometer Boresight Pitch with respect to Navigation Frame
	float32 boresight_pitch_rollsens  # DO NOT USE [rad] Odometer Boresight Pitch with respect to Navigation Frame as sensed from roll
	float32 boresight_pitch_rocksens  # DO NOT USE [rad] Odometer Boresight Pitch with respect to Navigation Frame as sensed from rock
	float32 lever_arm_x  # DO NOT USE [m] X odometer lever arm
	float32 lever_arm_y  # DO NOT USE [m] Y odometer lever arm
	float32 lever_arm_z  # DO NOT USE [m] Z odometer lever arm
	float32 scalefactor_correction_stdv  # [<0; 1>] Standard deviation of Scalefactor correction
	float32 scalefactor_correction_pitchsens_stdv  # DO NOT USE [<0; 1>] Standard deviation of Scalefactor correction
	float32 scalefactor_correction_pitchratesens_stdv  # DO NOT USE [<0; 1>] Standard deviation of Scalefactor correction
	float32 boresight_yaw_stdv  # [rad] Odometer Boresight Yaw standard deviation with respect to Navigation Frame
	float32 boresight_yaw_rollsens_stdv  # DO NOT USE [rad] Odometer Boresight Yaw standard deviation with respect to Navigation Frame sensed from roll
	float32 boresight_yaw_rocksens_stdv  # DO NOT USE [rad] Odometer Boresight Yaw standard deviation with respect to Navigation Frame sensed from rock
	float32 boresight_pitch_stdv  # [rad] Odometer Boresight Pitch standard deviation with respect to Navigation Frame
	float32 boresight_pitch_rollsens_stdv  # DO NOT USE [rad] Odometer Boresight Pitch standard deviation with respect to Navigation Frame sensed from roll
	float32 boresight_pitch_rocksens_stdv  # DO NOT USE [rad] Odometer Boresight Pitch standard deviation with respect to Navigation Frame sensed from rock
	float32 lever_arm_stdv_x  # [m] X standard deviations of Odometer position
	float32 lever_arm_stdv_y  # [m] Y standard deviations of Odometer position
	float32 lever_arm_stdv_z  # [m] Z standard deviations of Odometer position
	float32 stored_lever_arm_x  # [m] X Initially Stored position of Odometer with respect to Navigation Frame
	float32 stored_lever_arm_y  # [m] Y Initially Stored position of Odometer with respect to Navigation Frame
	float32 stored_lever_arm_z  # [m] Z Initially Stored position of Odometer with respect to Navigation Frame
	float32 boresight_roll  # [rad] Odometer Boresight Roll with respect to Navigation Frame
	float32 boresight_stdv_roll  # [rad] Odometer Boresight Roll standard deviation with respect to Navigation Frame
