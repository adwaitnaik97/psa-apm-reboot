# 0x6211 : Time Mark of Event-Out

#
#  The 0x6211 message displays the time of validity of the Event-Out signal.
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length [Number of 32-bit Words]
	uint32 Checksum  # 32-bit CRC
	event_out_mark_t markPort  # Event Out port the message pertains to
	float64 system_time_of_event_in  # [sec] INS system time of EventIn [seconds]
	float64 gps_time_of_event_in  # [sec] GPS Time of EventIn in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32 gps_week_of_event_in  # GPS Week of EventIn [#]
