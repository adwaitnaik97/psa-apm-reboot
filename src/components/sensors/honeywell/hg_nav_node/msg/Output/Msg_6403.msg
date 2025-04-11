# 0x6403 : INS Geodetic Position

#
#  This is the basic source of position from the INS system.
#  Message contains Geodetic Position (Latitude, Longitude, Altitude) with associated Standard deviations
#  The position is referenced from Vehicle Frame
# 

# Variables declaration
	uint32 AddressId  # Start of Message Flag
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length in 32-bit word
	uint32 Checksum  # Checksum
	float64 systemTov  # [s] System Time of Validity
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	float64 Latitude  # [rad] WGS-84 Latitude
	float64 Longitude  # [rad] WGS-84 Longitude
	float64 AltitudeHeightAboveEllipsoid  # [m] Altitude : Height Above Ellipsoid
	ins_gnss_summary_t InsGnssSummary  # INS / GNSS Summary Word
	float32 LatitudeSTDV  # [m] Estimated Latitude Error Standard Deviation (1-sigma)
	float32 LongitudeSTDV  # [m] Estimated Longitude Error Standard Deviation (1-sigma)
	float32 AltitudeHeightAboveEllipsoidSTDV  # [m] Estimated Altitude Error Standard Deviation (1-sigma)
