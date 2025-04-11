# GPS board status
# The GPS Receiver has 96 fault words, each 16 bits wide. This message can output 3 unique GPS fault words per message. If there are more than 3 GPS faults words, multiple occurrences of this message are required - using the 8 MSB as an identifier.
# custom gps_bit_status_t
	bool gpssw_GPS_Function  # GPS Function
	bool gpssw_GPS_Communication  # GPS Communication
	bool gpssw_GPS_Time_Mark  # GPS Time Mark
	bool gpssw_GPS_T20_Synchronization  # GPS T20 Synchronization
