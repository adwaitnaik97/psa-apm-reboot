# Single Observation used in the GNSS Range message
# custom satellite_pos_t
	constellation_enum_t system  # Satellite system
	uint32 satelliteID  # Satellite ID Number
	float64 pos_x  # Satellite X co-ordinates (ECEF,m)
	float64 pos_y  # Satellite Y co-ordinates (ECEF,m)
	float64 pos_z  # Satellite Z co-ordinates (ECEF,m)
	float64 clk_corr  # Satellite clock correction (m)
	float64 iono_delay  # Ionosphere delay (m)
	float64 tropo_delay  # Troposphere delay (m)
