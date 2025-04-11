# 0x2201 : Time Mark PPS

#
#  DO NOT USE for Event IN timing - use 0x620x messages
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	time_validity_bits_t time_validity_bits  # validity of time information and PPS mode
	
	# 0 = External Source
	# 1 = accuracy <= 1ns
	# 2 = accuracy <= 10ns
	# 3 = accuracy <= 100ns
	# 4 = accuracy <= 1us
	# 5 = accuracy <= 10us
	# 6 = accuracy <= 100us
	# 7 = accuracy <= 1ms
	# 8 = accuracy <= 10ms
	# 9 = Unknown/Fault
	int32 utc_time_figure_of_merit
	float64 systemTov  # [sec] Time since Power Up - Always Valid
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32 gps_week  # Number of weeks since Jan 6, 1980 GPS Week = -1 indicates GPS Week Unknown/Invalid
	uint32 utc_day_month_year_date 
	uint8 utc_day_of_week  # day of week (0 = Sunday | ... | 6 = Saturday)
	uint16 utc_day_of_year  # day of year (1 - 366 | 0 = Invalid)
	uint8 utc_day_of_month  # day in month (1 - 31 | 0 = Invalid)
	uint8 utc_month  # month number (1 - 12 | 0 = Invalid)
	uint16 utc_year  # year number (0 = Invalid | >= 1980)
	uint32 utc_hour_min_sec 
	uint8 utc_second  # 0 - 60 (60 is used when leap second is added )
	uint8 utc_minute  # 0 - 59
	uint8 utc_hour  # 0 - 23
	float64 utc_time  # UTC Time within day
	float64 ins_gps_internal_system_time_pps_in  # [s] INS time of event input
	float64 gps_time_pps_in  # [s] GPS Time of event input
	uint32 pps_in_count  # [-] Event in count
