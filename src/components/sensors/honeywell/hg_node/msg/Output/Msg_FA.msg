# 0xFA : Status Message - Sensor Health

# Variables declaration
	uint8 SyncByte  # IMU Address
	uint8 MessageID  # Message ID
	sensor_axes_t active_sensor_axes  # Active Sensor Axes
	sensor_axes_t saturated_sensor_axes  # Saturated Sensor Axes
	sensor_axes_t sensor_stat_fail  # Sensor statistics test failure flag
	sensor_axes_t sensor_temp_fail  # Sensor temperature test failure flag
	uint16 Checksum  # Uint16 Checksum. Complement = False
