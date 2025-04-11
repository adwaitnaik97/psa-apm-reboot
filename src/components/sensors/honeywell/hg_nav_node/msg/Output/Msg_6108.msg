# 0x6108 : GNSS Receiver PVT Measurement Output

#
#  RAW position from GNSS receiver. The position is refered to Main (RF1) antenna
#  This message is being outputted only when GNSS Signal is avaiable
# 

# Variables declaration
	uint32 AddressId  # Start of message flag
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message length
	uint32 Checksum  # Checksum
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32 gps_week  # GPS Week
	ins_gnss_summary_t InsGnssSummary  # INS / GNSS summary word
	float64 Latitude  # [rad] WGS-84 Latitude
	float64 Longitude  # [rad] WGS-84 Longitude
	float64 AltitudeHeightAboveEllipsoid  # [m] Altitude : Height Above Ellipsoid
	float32 NorthVelocity  # [m/s] North Velocity
	float32 EastVelocity  # [m/s] East Velocity
	float32 DownVelocity  # [m/s] Down Velocity
	float32 RxClkBias  # RESERVED - [m] Receiver Clock Bias
	
	# Altitude type:
 	# 0 = Undefined
 	# 1 = Geoid Altitude
 	# 2 = Geoid Height
 	# 3 = Ellipsoid WGS-84
	uint8 Datum
	uint8 TimeReference  # RESERVED - Time Reference
	uint8 NumberOfSvs  # Number of SVs
	uint8 RTKfixProg  # RESERVED
	uint32 CorrInfo  # RESERVED
	uint32 SignalInfo  # RESERVED
	uint32 PPPInfo  # RESERVED
	float32 LatitudeSTDV  # [m] Estimated Latitude Error Standard Deviation (1-sigma)
	float32 LongitudeSTDV  # [m] Estimated Longitude Error Standard Deviation (1-sigma)
	float32 AltitudeHeightAboveEllipsoidSTDV  # [m] Estimated Altitude Error Standard Deviation (1-sigma)
	float32 NorthVelocitySTDV  # [m/s] Estimated North Velocity Error Standard Deviation (1-sigma)
	float32 EastVelocitySTDV  # [m/s] Estimated East Velocity Error Standard Deviation (1-sigma)
	float32 DownVelocitySTDV  # [m/s] Estimated Down Velocity Error Standard Deviation (1-sigma)
	float64 systemTov  # [sec] System Time of Validity
	float32 PDOP  # RESERVED - Position dilution of precision (PDOP) [ DOP from 0.0 to 10.2]
	float32 HDOP  # RESERVED - Horizontal Position dilution of precision (HDOP) [ DOP from 0.0 to 10.2]
	float32 VDOP  # RESERVED - Vertical Position dilution of precision (VDOP) [ DOP from 0.0 to 10.2]
	float32 TDOP  # RESERVED - Time dilution of precision (TDOP) [ DOP from 0.0 to 10.2]
	float32 Undulation  # [m] Undulation - the difference between the geoid and the ellipsoid
