# 0x6205 : Vehicle All-In-One Pos, Vel, Att of Event-In

#
#  The 0x6205 message provides the Geodectic position, velocity and attitude estimates of the vehicle at the time of the Event-In signal occurred.
#  When multiple event in sources are enabled, sources are differentiated by the markport field.
# 

# Variables declaration
	uint32 AddressId  # Start of Message Flag
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length in 32-bit word
	uint32 Checksum  # Checksum
	event_in_mark_t markPort  # Event port the message pertains to
	float64 system_time_of_event_in  # [s] EventIn System Time of Validity
	float64 gps_time_of_event_in  # [s] EventIn GPS Time of Week
	float64 Latitude  # [rad] WGS-84 Latitude
	float64 Longitude  # [rad] WGS-84 Longitude
	float64 AltitudeHeightAboveEllipsoid  # [m] Altitude : Height Above Ellipsoid
	ins_gnss_summary_t InsGnssSummary  # INS / GNSS Summary Word
	float32 LatitudeSTDV  # [rad] Estimated Latitude Error Standard Deviation (1-sigma)
	float32 LongitudeSTDV  # [rad] Estimated Longitude Error Standard Deviation (1-sigma)
	float32 AltitudeHeightAboveEllipsoidSTDV  # [m] Estimated Altitude Error Standard Deviation (1-sigma)
	float32 NorthVelocity  # [m/s] North Velocity
	float32 EastVelocity  # [m/s] East Velocity
	float32 DownVelocity  # [m/s] Down Velocity
	float32 NorthVelocitySTDV  # [m/s] Estimated North Velocity Error Standard Deviation (1-sigma)
	float32 EastVelocitySTDV  # [m/s] Estimated East Velocity Error Standard Deviation (1-sigma)
	float32 DownVelocitySTDV  # [m/s] Estimated Down Velocity Error Standard Deviation (1-sigma)
	float32 Roll  # [rad] Roll
	float32 Pitch  # [rad] Pitch
	float32 Heading  # [rad] Heading
	float32 EulerAnglesSTDVRoll  # [rad] Estimated Roll Error Standard Deviation
	float32 EulerAnglesSTDVPitch  # [rad] Estimated Pitch Error Standard Deviation
	float32 EulerAnglesSTDVHeading  # [rad] Estimated Heading Error Standard Deviation
