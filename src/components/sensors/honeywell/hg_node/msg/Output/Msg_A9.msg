# 0xA9 : Navigation Message with magnetometer flux

#
#  It contains the Delta Angle in Rad and Delta Velocity in m/s and Magnetic Flux in mGauss however no Control Data
# 

# Variables declaration
	uint8 SyncByte  # Sync Byte
	uint8 MessageID  # Message ID
	float32 DeltaAngleX  # [rad] Case Frame Delta Angle X
	float32 DeltaAngleY  # [rad] Case Frame Delta Angle Y
	float32 DeltaAngleZ  # [rad] Case Frame Delta Angle Z
	float32 DeltaVelocityX  # [m/sec] Case Frame Delta Velocity X
	float32 DeltaVelocityY  # [m/sec] Case Frame Delta Velocity Y
	float32 DeltaVelocityZ  # [m/sec] Case Frame Delta Velocity Z
	float32 mg1FluxA  # [mGauss] Case Frame Magnetic Flux X of Mangnetomer #1
	float32 mg1FluxB  # [mGauss] Case Frame Magnetic Flux Y of Mangnetomer #1
	float32 mg1FluxC  # [mGauss] Case Frame Magnetic Flux Z of Mangnetomer #1
	float32 mg1Temperature  # [deg C] Temperature of Mangnetomer #1
	float32 mg2FluxA  # [mGauss] Case Frame Magnetic Flux X of Mangnetomer #2
	float32 mg2FluxB  # [mGauss] Case Frame Magnetic Flux Y of Mangnetomer #2
	float32 mg2FluxC  # [mGauss] Case Frame Magnetic Flux Z of Mangnetomer #2
	float32 mg2Temperature  # [deg C] Temperature of Mangnetomer #2
	float32 mg3FluxA  # [mGauss] Case Frame Magnetic Flux X of Mangnetomer #3
	float32 mg3FluxB  # [mGauss] Case Frame Magnetic Flux Y of Mangnetomer #3
	float32 mg3FluxC  # [mGauss] Case Frame Magnetic Flux Z of Mangnetomer #3
	float32 mg3Temperature  # [deg C] Temperature of Mangnetomer #3
	uint16 Checksum  # 16 bit checksum
