# 0x1201 : Time Mark PPS Input

#
#  This is used by the Navigation device to associate a GPS and UTC time with an
#  Incoming hardware PPS.  The message must be received <0.25 seconds after generation
#  of a 1 Hz rising edge positive polarity pulse on the PPS hardware line.
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	bool gpsTovValid  # is gps time valid
	bool gpsToUTCofsetValid  # is offset between gps and utc time valid
	float64 gps_time_to_utc_offset  # utc_time = Fmod(gps_time + gps_time_to_utc_offset,86400);
	float64 PPSgpsTov  # [sec] GPS time within current week when the rising edge of the PPS occurs
	int32 gps_week  # Number of weeks since Jan 6, 1980 GPS Week = -1 indicates GPS Week Unknown/Invalid
