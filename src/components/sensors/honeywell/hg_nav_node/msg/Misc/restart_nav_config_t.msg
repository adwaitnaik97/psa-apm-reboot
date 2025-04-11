# Navigation Config on Restart
# custom restart_nav_config_t
	bool gyro_bias  # RESERVED
	bool accel_bias  # RESERVED
	bool gyro_sf  # RESERVED
	bool accel_sf  # RESERVED
	bool main_ant_lev  # Store the current, calculated lever arm for the main antenna
	bool aux_ant_lev  # Store the current, calculated lever arm for the aux antenna
	bool nav_init  # Store the current POSE for nav to start with on next cycle
