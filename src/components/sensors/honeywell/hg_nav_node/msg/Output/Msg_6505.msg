# 0x6505 : Skymap Information

#
#  Individual message per satellite, holds skymap data including elevation, azimuth, and signal C/No values.
#  Message is being sent in batches. One should condense the information until 'Final_msg_in_set=1'
#  Only satellites in view are reported - not the whole ephemeris
# 

# Variables declaration
	uint32 AddressId  # Start of Message Flag
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length in 32-bit word
	uint32 Checksum  # Checksum
	constellation_enum_t Constellation  # Constellation that the satellite being described belongs to.
	bool Used_in_soln  # Used in the solution: 0=not used in GNSS solution | 1=used in GNSS solution
	bool Final_msg_in_set  # Final message to be sent: 0=not final message in set | 1= final message int set
	uint8 prn_slot  # Prn/slot of the satellite
	float32 elevation  # Elevation angle of the satellite in degrees. (Horizon = 0 degrees)
	float32 azimuth  # Azimuth angle of the satellite in degrees. (North = 0 degrees, increasing towards East)
	uint8 freq_band_1  # Frequency band of this signal
	uint8 channel_1  # Channel number for this signal (GLONASS specific)
	float32 carrier_noise_1  # Carrier/noise for this signal, 0 if not tracked
	uint8 freq_band_2  # Frequency band of this signal
	uint8 channel_2  # Channel number for this signal (GLONASS specific)
	float32 carrier_noise_2  # Carrier/noise for this signal, 0 if not tracked
	uint8 freq_band_3  # Frequency band of this signal
	uint8 channel_3  # Channel number for this signal (GLONASS specific)
	float32 carrier_noise_3  # Carrier/noise for this signal, 0 if not tracked
	uint8 freq_band_4  # Frequency band of this signal
	uint8 channel_4  # Channel number for this signal (GLONASS specific)
	float32 carrier_noise_4  # Carrier/noise for this signal, 0 if not tracked
	uint8 freq_band_5  # Frequency band of this signal
	uint8 channel_5  # Channel number for this signal (GLONASS specific)
	float32 carrier_noise_5  # Carrier/noise for this signal, 0 if not tracked
	uint8 freq_band_6  # Frequency band of this signal
	uint8 channel_6  # Channel number for this signal (GLONASS specific)
	float32 carrier_noise_6  # Carrier/noise for this signal, 0 if not tracked
	ins_gnss_summary_t InsGnssSummary  # INS / GNSS Mode Summary
	int32 gps_week  # GPS Week
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
