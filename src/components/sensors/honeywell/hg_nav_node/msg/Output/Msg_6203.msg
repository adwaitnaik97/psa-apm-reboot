# 0x6203 : Vehicle NED Velocity of Event-In

#
#  The 0x6203 message provides the North,East,Down velocity estimate of the vehicle at the time of the Event-In signal occurred.
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
	float32 NorthVelocity  # [m/s] North Velocity
	float32 EastVelocity  # [m/s] East Velocity
	float32 DownVelocity  # [m/s] Down Velocity
	ins_gnss_summary_t InsGnssSummary  # INS / GNSS Mode Summary
	float32 NorthVelocitySTDV  # [m/s] Estimated North Velocity Error Standard Deviation (1-sigma)
	float32 EastVelocitySTDV  # [m/s] Estimated East Velocity Error Standard Deviation (1-sigma)
	float32 DownVelocitySTDV  # [m/s] Estimated Down Velocity Error Standard Deviation (1-sigma)
