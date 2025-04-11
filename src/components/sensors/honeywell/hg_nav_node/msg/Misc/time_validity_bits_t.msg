# Custom type to define time validity and PPS mode
# custom time_validity_bits_t
	bool gpsTovValid  # GPS Time Validity (0 = Invalid | 1 = Valid)
	bool utcTimeValid  # UTC Time Validity (0 = Invalid | 1 = Valid)
	uint8 PPS_Mode  # PPS Mode of operation (0 = Disabled | 1 = Connected to Event IN | 4 = Connected to System Time)
