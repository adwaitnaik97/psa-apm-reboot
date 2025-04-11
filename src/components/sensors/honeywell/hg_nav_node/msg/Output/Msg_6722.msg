# 0x6722 : Nortek DVL Water Track Data

#
#  Output of Water Track Data from Nortek DVL.
#  Reference Nortek DVL binary data format DF22.
# 

# Variables declaration
	uint32 AddressId  # Start of message flag
	uint32 MessageId  # Message ID
	uint32 Checksum  # Checksum
	uint32 MessageLength  # Message length in 32-bit word
	float64 systemTov  # [s] Time of validity - trigger/ping time of message ensemble
	uint32 serialNumber  # DVL Serial Number
	uint8 version  # DVL Version
	uint8 offsetOfData  # NOT APPLICABLE to this message. Number of bytes from start of data from Nortek DVL msg format.
	uint8 year  # Years since 1900
	uint8 month  # Month, zero based (Jan = 0, Feb = 1, etc.)
	uint8 day  # Day
	uint8 hour  # [hr] Hour
	uint8 minute  # [min] Minute
	uint8 seconds  # [sec] Seconds
	uint16 microseconds  # [usec] Microseconds (raw value in 100 usec)
	uint16 numOfBeams  # Number of beams used [0 - 4 beams used]
	uint32 error  # DVL Error number
	dvl_track_status_t status  # DVL Status
	float32 speedOfSound  # [m/s] Speed of sound
	float32 temperature  # [deg C] Water Temperature
	float32 pressure  # [Bar] Pressure
	float32 velBeam0  # Velocity of beam 0 in m/s [invalid set to -32.768 m/s]
	float32 velBeam1  # Velocity of beam 1 in m/s [invalid set to -32.768 m/s]
	float32 velBeam2  # Velocity of beam 2 in m/s [invalid set to -32.768 m/s]
	float32 velBeam3  # Velocity of beam 3 in m/s [invalid set to -32.768 m/s]
	float32 distBeam0  # Vertical distance beam 0 in meters [invalid set to 0.0 m]
	float32 distBeam1  # Vertical distance beam 1 in meters [invalid set to 0.0 m]
	float32 distBeam2  # Vertical distance beam 2 in meters [invalid set to 0.0 m]
	float32 distBeam3  # Vertical distance beam 3 in meters [invalid set to 0.0 m]
	float32 fomBeam0  # Figure of merit beam 0 in m/s [invalid set to 10.0 m/s]
	float32 fomBeam1  # Figure of merit beam 1 in m/s [invalid set to 10.0 m/s]
	float32 fomBeam2  # Figure of merit beam 2 in m/s [invalid set to 10.0 m/s]
	float32 fomBeam3  # Figure of merit beam 3 in m/s [invalid set to 10.0 m/s]
	float32 dt1Beam0  # Time diff 1 beam 0 in seconds
	float32 dt1Beam1  # Time diff 1 beam 1 in seconds
	float32 dt1Beam2  # Time diff 1 beam 2 in seconds
	float32 dt1Beam3  # Time diff 1 beam 3 in seconds
	float32 dt2Beam0  # Time diff 2 beam 0 in seconds
	float32 dt2Beam1  # Time diff 2 beam 1 in seconds
	float32 dt2Beam2  # Time diff 2 beam 2 in seconds
	float32 dt2Beam3  # Time diff 2 beam 3 in seconds
	float32 timeVelEstBeam0  # Duration of velocity estimate for beam 0 in seconds
	float32 timeVelEstBeam1  # Duration of velocity estimate for beam 1 in seconds
	float32 timeVelEstBeam2  # Duration of velocity estimate for beam 2 in seconds
	float32 timeVelEstBeam3  # Duration of velocity estimate for beam 3 in seconds
	float32 velX  # Velocity X in m/s [invalid set to -32.768 m/s]
	float32 velY  # Velocity Y in m/s [invalid set to -32.768 m/s]
	float32 velZ1  # Velocity Z1 in m/s [invalid set to -32.768 m/s]
	float32 velZ2  # Velocity Z2 in m/s [invalid set to -32.768 m/s]
	float32 fomX  # Figure of merit in m/s [invalid set to 10.0 m/s]
	float32 fomY  # Figure of merit in m/s [invalid set to 10.0 m/s]
	float32 fomZ1  # Figure of merit in m/s [invalid set to 10.0 m/s]
	float32 fomZ2  # Figure of merit in m/s [invalid set to 10.0 m/s]
	float32 dt1X  # Time diff 1 in X in seconds
	float32 dt1Y  # Time diff 1 in Y in seconds
	float32 dt1Z1  # Time diff 1 in Z1 in seconds
	float32 dt1Z2  # Time diff 1 in Z2 in seconds
	float32 dt2X  # Time diff 2 in X in seconds
	float32 dt2Y  # Time diff 2 in Y in seconds
	float32 dt2Z1  # Time diff 2 in Z1 in seconds
	float32 dt2Z2  # Time diff 2 in Z2 in seconds
	float32 timeVelEstX  # Duration of velocity estimate for each component in seconds
	float32 timeVelEstY  # Duration of velocity estimate for each component in seconds
	float32 timeVelEstZ1  # Duration of velocity estimate for each component in seconds
	float32 timeVelEstZ2  # Duration of velocity estimate for each component in seconds
