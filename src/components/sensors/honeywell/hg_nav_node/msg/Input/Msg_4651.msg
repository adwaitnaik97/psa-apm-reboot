# 0x4651 : Real Time Post Processing Good To Go Setup

#
#  The 0x4651 message is used to setup the Real Time Post Processing Good (0x6651)
#  The assumption is that the user is logging the data from the INS startup.
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	float32 WarmupTime  # [sec] Warmup time
	check_enable_t ThresholdCheckEnable  # Threshold check enable bit field
	float32 PosXErrorThreshold  # [m] Position X Error Threshold
	float32 PosYErrorThreshold  # [m] Position Y Error Threshold
	float32 PosZErrorThreshold  # [m] Position Z Error Threshold
	float32 VelXErrorThreshold  # [m/s] Velocity X Error Threshold
	float32 VelYErrorThreshold  # [m/s] Velocity Y Error Threshold
	float32 VelZErrorThreshold  # [m/s] Velocity Z Error Threshold
	float32 AttXErrorThreshold  # [rad] Attitude X Error Threshold
	float32 AttYErrorThreshold  # [rad] Attitude Y Error Threshold
	float32 AttZErrorThreshold  # [rad] Attitude Z Error Threshold
	uint8 NavAxesSelector  # Navigation Axes Selector [0: Pos=NED, Vel=NED, Att=Euler | 1: Pos=ECEF, Vel=ECEF, Att=Gamma]
