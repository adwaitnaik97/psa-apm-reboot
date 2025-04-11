# 0x4012 : Command Power Cycle Nav Initialization

#
#  Send position, attitude, and velocity (assumed zero) information to unit.  
#  Information used on next power cycle only to initialize the unit into navigation.  
#  Read the detailed description below before usage. If unsure, consult with Honeywell.
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	restart_nav_config_t nav_init_config 
	uint32 Sigma_Latitude  # RESERVED
	uint32 Sigma_Longitude  # RESERVED
	uint32 Sigma_Altitude  # RESERVED
	uint32 Sigma_Heading  # RESERVED
