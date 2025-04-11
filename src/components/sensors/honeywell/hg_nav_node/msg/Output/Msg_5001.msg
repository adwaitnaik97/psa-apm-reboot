# 0x5001 : GNSS Receiver Version Information

#
#  Information GNSS Receiver and its various components and versions
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	uint8[16] SerialNumber  # GNSS Receiver Serial Number
	uint8[16] PartNumber  # GNSS Receiver Part Number
	uint8[16] Model  # GNSS Receiver Model (Indicates Functionality)
	uint8[16] GNSSHardwareVersion  # GNSS Receiver Hardware Version
	uint8[16] FirmwareVersion  # GNSS Receiver Firmware Version
	uint8[16] FirmwareBuildDate  # GNSS Receiver Firmware Build Date
	uint8[16] FirmwareBuildTime  # GNSS Receiver Firmware Build Time of Day
	uint8[16] Component1Type  # GNSS Receiver Component Type
	uint8[16] Component1Version  # GNSS Receiver Component Version
	uint8[16] Component2Type  # GNSS Receiver Component Type
	uint8[16] Component2Version  # GNSS Receiver Component Version
	uint8[16] Component3Type  # GNSS Receiver Component Type
	uint8[16] Component3Version  # GNSS Receiver Component Version
