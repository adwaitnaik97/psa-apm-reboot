# Structure defining where the information originated
# custom init_sources_t
	uint8 Position  # 0 = Unknown, 1 = External Input, 2 = GPS
	uint8 Velocity  # 0 = Unknown, 1 = External Input, 2 = GPS, 3 = Zero Velocity
	uint8 Attitude  # 0 = Unknown, 1 = External Input, 2 = GPS, 3 = Coarse Leveling
	uint8 Heading  # 0 = Unknown, 1 = External Input, 2 = GPS, 3 = Magnetometer
