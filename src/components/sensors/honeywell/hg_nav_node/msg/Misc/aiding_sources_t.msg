# List of currently utilized aiding sources
# note: this list is updated based on ZUPT detection (a switch between Dynamic and Static aiding sources is visible here)
# custom aiding_sources_t
	bool zero_velocity  # 0 = Not Enabled | 1 = Enabled
	bool GNSS_PVT_P  # 0 = Not Enabled | 1 = Enabled
	bool GNSS_PVT_V  # 0 = Not Enabled | 1 = Enabled
	bool GNSS_ATT_A  # 0 = Not Enabled | 1 = Enabled
	bool GNSS_pseudo_range  # 0 = Not Enabled | 1 = Enabled
	bool GNSS_delta_range  # 0 = Not Enabled | 1 = Enabled
	bool baro_altitude  # 0 = Not Enabled | 1 = Enabled
	bool position_match  # 0 = Not Enabled | 1 = Enabled
	bool velocity_match  # 0 = Not Enabled | 1 = Enabled
	bool attitude_match  # 0 = Not Enabled | 1 = Enabled
	bool mag_heading  # 0 = Not Enabled | 1 = Enabled
	bool static_heading  # 0 = Not Enabled | 1 = Enabled
	bool motion_detect  # 0 = Not Enabled | 1 = Enabled
	bool odometer  # 0 = Not Enabled | 1 = Enabled
	bool land_constraints  # 0 = Not Enabled | 1 = Enabled
