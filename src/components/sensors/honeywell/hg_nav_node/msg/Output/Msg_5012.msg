# 0x5012 : GNSS Receiver Hardware Status Information

#
#  Detailed status of the GNSS Receiver hardware.
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length in 32-bit Words
	uint32 Checksum  # Checksum
	float32 systemTov  # GNSS receiver system time
	float32 Temperature  # [deg C]
	float32 RF1_SupplyVoltage  # [VDC]
	float32 RF1_SupplyCurrent  # [Amperes]
	float32 RF2_SupplyVoltage  # [VDC]
	float32 RF2_SupplyCurrent  # [Amperes]
	float32 Voltage3V3  # [VDC] Actual Voltage on the 3V3 rail
	float32 Voltage1V8  # [VDC] Actual Voltage on the 1V8 rail
	float32 Voltage1V2  # [VDC] Actual Voltage on the 1V2 rail
	float32 Voltage5V0  # [VDC] Actual Voltage on the 5V0 rail
	uint8 ptc  # GNSS receiver Percent Throughput - Current [Scaled to represent 0-100%]
	uint8 ptm  # GNSS receiver Percent Throughput - Maximum [Scaled to represent 0-100%]
	uint8 memUsage  # GNSS receiver RAM Usage [Scaled to represent 0-100%]
	uint32 mode  # GPS Receiver Mode: 0 = Test | 1 = Init | 2 = Track | 3 = Powerup
	int32 power_cycle_count  # Power Cycle Count
	float64 eti  # [sec] Total Device Elapsed Time Indicator (ETI)
