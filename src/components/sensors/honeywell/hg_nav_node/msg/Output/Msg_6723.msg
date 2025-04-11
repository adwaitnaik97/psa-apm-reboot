# 0x6723 : Nortek Current Profile Data

#
#  Non-variable length current profile data.
#  Reference Nortek DVL binary data format DF23.
# 

# Variables declaration
	uint32 AddressId  # Start of message flag
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message length in 32-bit word
	uint32 Checksum  # Checksum
	float64 systemTov  # [s] Time of validity - trigger/ping time of message ensemble
	uint8 version  # DVL Version
	uint32 serialNumber  # DVL Serial Number
	uint8 offsetOfData  # NOT APPLICABLE to this message. Number of bytes from start of data from Nortek DVL msg format.
	record_configuration_t profileConfiguration  # Current profile record configuration bit field
	uint8 year  # Years since 1900
	uint8 month  # Month, zero based (Jan = 0, Feb = 1, etc.)
	uint8 day  # Day
	uint8 hour  # [hr] Hour
	uint8 minute  # [min] Minute
	uint8 seconds  # [sec] Seconds
	uint16 microseconds  # [usec] Microseconds (raw value in 100 usec)
	float32 speedOfSounds  # [m/s] Speed of sound
	float32 temperature  # [deg C] Temperature
	float32 pressure  # [dBar] Pressure
	float32 heading  # [deg] Heading
	float32 pitch  # [deg] Pitch
	float32 roll  # [deg] Roll
	uint16 numCells  # number of cells
	
	# coordinate system type
 	# (1) b01=XYZ
 	# (2) b10=beam
 	# (3) b11=not defined
	uint8 coorinateSystem
	uint8 numBeams  # number of beams
	uint16 cellSize  # [mm] Cell size
	uint16 blanking  # [mm] Blanking
	uint8 nomCorrelation  # Nominal correlation for configured combination of cell size and velocity range in %
	float32 tempPressureSensor  # Temperature pressure sensor in degree Celsius; t = (val/5) - 4.0
	float32 batteryVoltage  # Battery voltage in volts
	int16 magX  # Magnetometer Raw X Axis
	int16 magY  # Magnetometer Raw Y Axis
	int16 magZ  # Magnetometer Raw Z Axis
	int16 accelX  # Accel X Axis (16384 = 1.0)
	int16 accelY  # Accel Y Axis (16384 = 1.0)
	int16 accelZ  # Accel Z Axis (16384 = 1.0)
	uint16 ambiguityVelocity  # Ambiguity velocity, corrected for sound velocity (10^ velocity scaling) m/s.
	uint8 dataSet1_beam  # physical beam used for 1st data set
	uint8 dataSet2_beam  # physical beam used for 2nd data set
	uint8 dataSet3_beam  # physical beam used for 3rd data set
	uint8 dataSet4_beam  # physical beam used for 4th data set
	uint16 transmitEnergy  # Transmit Energy
	int8 velocityScaling  # Velocity scaling
	int8 powerLevel  # [dB] Power level
	int16 tempMag  # Temperature of magnetometer (not calibrated)
	float32 tempRtc  # Temperature real time clock in degree Celsius
	dvl_error_t error  # Error bitfield
	
	# value indicates excesive DVL processor load
	# (7) 111 < 3% dvl processing capicity left
	# (6) 110 < 6% dvl processing capicity left
	# (4) 100 < 12% dvl processing capicity left
	uint8 cpu_load
	bool status0_used  # (1 = status0 fields should be processed | 0 = status0 bitfiled not used)
	dvl_status_t status  # DVL Status
	uint32 ensembleCounter  # Counts the number of ensembles in both averaged and burst data
