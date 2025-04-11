# List of Interface protocols
# interface_protocol_t - enumeration
uint8 value

#Pre-Defined ENUM Constants
	uint8 protocol_No_Change                     = 0 #  No Change to interface
	uint8 protocol_HGNSI_DICD_Formatted_Messages = 1 #  HGNSI Interface
	uint8 protocol_Legacy_IMU_Formatted_Messages = 2 #  IMU Message Types
	uint8 protocol_NMEA                          = 3 #  NMEA Messages
	uint8 protocol_GNSSR_Navstorm                = 4 #  Spare
	uint8 protocol_GNSSR_Navfire                 = 5 #  Spare
	uint8 protocol_GNSSR_AsteRx_4                = 6 #  Spare
	uint8 protocol_GNSSR_AsteRx_M2A              = 7 #  Spare
	uint8 protocol_GNSSR_NovAtel_OEM7            = 8 #  Spare
	uint8 protocol_GNSSR_Trimble_C1216           = 9 #  Spare
	uint8 protocol_GNSSR_uBlox_NEO               = 10 #  Spare
	# [IMU] CAN bus with 11-bit ID 
	# [INS] Reserved protocol_GNSSR_Swift
	uint8 protocol_CAN_11_bit                    = 11
	uint8 protocol_HGNSI_CONSOLE                 = 12 #  HGuide Console Commands
	uint8 protocol_CAN_29_bit                    = 29 #  [IMU] CAN bus with 29-bit ID
