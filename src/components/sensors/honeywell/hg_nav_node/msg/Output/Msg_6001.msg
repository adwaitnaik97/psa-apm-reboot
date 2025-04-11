# 0x6001 : INS Configuration

#
#  Information about various device serial numbers and sw versions
#  Use the 0x1001 message to leave this on at 0.1 Hz (enabled; default), or to have this only produced and sent once (disabled).
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	uint8[16] DeviceSerialNumber  # Device Serial Number
	uint8[16] DevicePartNumber  # Device Part Number (End-Item)
	uint8[16] SensorAssyPartNumber  # "Sensor Assembly" Part Number
	uint8[16] NavSoftwareVersion  # Navigation Software (ECTOS) Version
	uint8[16] NavSoftwareBuildDate  # Navigation Software Build Date
	uint8[16] IMUSerialNumber  # IMU Serial Number
	uint8[16] IMUPartNumber  # IMU Part Number
	uint8[16] IMUSoftwareVersion  # IMU Software Version
	uint8[16] GNSSReceiverSerialNumber  # GNSS Receiver Serial Number
	uint8[16] GNSSReceiverPartNumber  # GNSS Receiver Part Number
	uint8[16] GNSSReceiverFirmwareVersion  # GNSS Receiver Firmware Version
	uint8[32] ProcessorHWIdentifier  # Processor/Rigid Flex PN
	uint8[32] InterconnectHWIdentifier  # Interconnect Flex PN
