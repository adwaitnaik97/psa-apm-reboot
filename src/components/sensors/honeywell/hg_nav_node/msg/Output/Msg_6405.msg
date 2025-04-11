# 0x6405 : INS Euler Attitudes

#
#  This is the basic source of attitude from the INS system.
#  Message contains Euler sequence (heading, pitch, roll) attitudes with associated stadard deviations
#  The attitude outputted with respect to NED frame and True North. The rotation convention is from NED Frame to the Vehicle Frame.
# 

# Variables declaration
	uint32 AddressId  # Start of Message Flag
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length in 32-bit word
	uint32 Checksum  # Checksum
	float64 systemTov  # [s] System Time of Validity
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	float32 Roll  # [rad] Roll
	float32 Pitch  # [rad] Pitch
	float32 Heading  # [rad] True Heading
	ins_gnss_summary_t InsGnssSummary  # INS / GNSS Summary Word
	float32 EulerAnglesSTDVRoll  # [rad] Estimated Roll Error Standard Deviation
	float32 EulerAnglesSTDVPitch  # [rad] Estimated Pitch Error Standard Deviation
	float32 EulerAnglesSTDVHeading  # [rad] Estimated True Heading Error Standard Deviation
