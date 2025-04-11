# 0x6204 : Vehicle Euler Attitudes of Event-In

#
#  The 0x6204 message provides the attitude estimate of the vehicle at the time of the Event-In signal occurred.
#  When multiple event in sources are enabled, sources are differentiated by the markport field.
# 

# Variables declaration
	uint32 AddressId  # Start of Message Flag
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length in 32-bit word
	uint32 Checksum  # Checksum
	event_in_mark_t markPort  # Event port the message pertains to
	float64 system_time_of_event_in  # [s] System Time of Validity Event In
	float64 gps_time_of_event_in  # [s] GPS Time of Week Event In
	float32 Roll  # [rad] Roll
	float32 Pitch  # [rad] Pitch
	float32 Heading  # [rad] Heading
	ins_gnss_summary_t InsGnssSummary  # INS / GNSS Summary Word
	float32 EulerAnglesSTDVRoll  # [rad] Estimated Roll Error Standard Deviation
	float32 EulerAnglesSTDVPitch  # [rad] Estimated Pitch Error Standard Deviation
	float32 EulerAnglesSTDVHeading  # [rad] Estimated Heading Error Standard Deviation
