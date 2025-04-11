# 0x4738 : DVL Configuration Initialization

#
#  Message to input DVL KF configuration and beam leverarm and rotation.
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message length in 32-bit words
	uint32 Checksum  # Checksum
	float32 measNoise  # [m] Measurement noise - stdv of noise on DVL used by KF
	float32 measRejectThreshold  # [sigma] Measurement rejection threshold - rejection threshold for DVL measurements used by KF
	float32 sfUncertainty  # [parts] Scale Factor uncertainty - estimated error from DVL scale factor (from datasheet)
	float32 sfTimeConst  # [hours] Scale Factor Time Constant - time at which the scale factor uncertainty degrades by 1 sigma
	float32 misalignInitUncertainty  # [mrad] Misalignment initial uncertainty - estimated initial error in the yaw direction of the mounting error relative to the unit
	float32 misalignTimeConst  # [hours] Misalignment Time Constant - time at which the misalignment uncertainty degrades by 1 sigma
	float32 pitchBoresightInitUncertainty  # [rad] Pitch boresight initial uncertainty - estimated initial error in the pitch direction of the mounting error relative to the unit
	float32 pitchBoresightStateNoise  # [rad/sqrt(hz)] Pitch boresight state noise - estimated in run noise in the pitch direction of the mounting error relative to the unit
	float32[3] leverarmBeam0  # [m] X, Y, Z Leverarm Beam 0 - Lever Arm from Vehicle Body to DVL Beam 0 in the vehicle frame
	float32 psiRotationBeam0  # [deg] PSI rotation beam 0 - Angular Rotation From Case Body to Beam 0
	float32[3] leverarmBeam1  # [m] X, Y, Z Leverarm Beam 1 - Lever Arm from Vehicle Body to DVL Beam 1 in the vehicle frame
	float32 psiRotationBeam1  # [deg] PSI rotation beam 1 - Angular Rotation From Case Body to Beam 1
	float32[3] leverarmBeam2  # [m] X, Y, Z Leverarm Beam 2 - Lever Arm from Vehicle Body to DVL Beam 2 in the vehicle frame
	float32 psiRotationBeam2  # [deg] PSI rotation beam 2 - Angular Rotation From Case Body to Beam 2
	float32[3] leverarmBeam3  # [m] X, Y, Z Leverarm Beam 3 - Lever Arm from Vehicle Body to DVL Beam 3 in the vehicle frame
	float32 psiRotationBeam3  # [deg] PSI rotation beam 3 - Angular Rotation From Case Body to Beam 3
	float32 psiThtRotation  # [deg] PSI THT rotation - Angular Rotation About The Case Y Axis From Case Body to Beam set
	float32 psiOffset  # [deg] PSI Offset - Angular Rotation About the Case Z Axis From Case Body to Beam set
	bool saveCfgToFlash  # Save configuration to flash (bit 0 of word)
