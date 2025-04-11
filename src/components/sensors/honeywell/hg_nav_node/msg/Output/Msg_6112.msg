# 0x6112 : Velocity Heading Output

#
#  Pitch, roll, and heading as determined by velocity 
#  Outputted only when using a single antenna
# 

# Variables declaration
	uint32 AddressId  # Start of message flag
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message length, 19 32 bit words
	uint32 Checksum  # Checksum
	float64 systemTov  # [s] System Time of Validity
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	float32 Roll  # Spare - [rad] Roll as calculated from velocity
	float32 Pitch  # [rad] Pitch as calculated from velocity
	float32 Heading  # [rad] Yaw/True Heading as calculated from velocity
	float32 Roll_stdv  # Spare - [rad] Roll STDV
	float32 Pitch_stdv  # [rad] Pitch STDV
	float32 Heading_stdv  # [rad] Yaw/True Heading STDV
	init_sources_t Initialization_Sources  # Position, Velocity, Attitude and Heading Initialization_Sources
	ins_gnss_summary_t InsGnssSummary  # INS / GNSS summary word
	uint32 Num_antennas  # Number of antennae used during Navigation Init
