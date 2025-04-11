# 0x6202 : Vehicle Geodetic Position of Event-In

#
#  The 0x6202 message provides the geodectic position estimate of the vehicle at the time of the Event-In signal occurred.
#  When multiple event in sources are enabled, sources are differentiated by the markport field.
# 

# Variables declaration
	uint32 AddressId  # Start of Message Flag
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length in 32-bit word
	uint32 Checksum  # Checksum
	event_in_mark_t markPort  # Event port the message pertains to
	float64 system_time_of_event_in  # [s] System Time of Validity of EventIn
	float64 gps_time_of_event_in  # [s] GPS Time of Week of EventIn
	float64 Latitude  # [rad] WGS-84 Latitude
	float64 Longitude  # [rad] WGS-84 Longitude
	float64 AltitudeHeightAboveEllipsoid  # [m] Altitude : Height Above Ellipsoid
	ins_gnss_summary_t InsGnssSummary  # INS / GNSS Summary Word
	float32 LatitudeSTDV  # [m] Estimated Latitude Error Standard Deviation (1-sigma)
	float32 LongitudeSTDV  # [m] Estimated Longitude Error Standard Deviation (1-sigma)
	float32 AltitudeHeightAboveEllipsoidSTDV  # [m] Estimated Altitude Error Standard Deviation (1-sigma)
