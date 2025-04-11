# 0x6504 : INS NED Velocity

#
#  This is the basic source of velocity from the INS system.
#  Contains North East Down Velocity with standard deviations
#  The velocity is referenced from Vehicle Frame
# 

# Variables declaration
	uint32 AddressId  # Start of Message Flag
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length in 32-bit word
	uint32 Checksum  # Checksum
	float64 systemTov  # System Time of Validity [sec]
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	float32 NorthVelocity  # North Velocity [m/s]
	float32 EastVelocity  # East Velocity [m/s]
	float32 DownVelocity  # Down Velocity [m/s]
	ins_gnss_summary_t InsGnssSummary  # INS / GNSS Mode Summary
	float32 NorthVelocitySTDV  # Estimated North Velocity Error Standard Deviation (1-sigma) [m/s]
	float32 EastVelocitySTDV  # Estimated East Velocity Error Standard Deviation (1-sigma) [m/s]
	float32 DownVelocitySTDV  # Estimated Down Velocity Error Standard Deviation (1-sigma) [m/s]
