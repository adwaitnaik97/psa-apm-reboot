# 0x2501 : GPS Channel Status Output

#
#  Reporting of all 25 GPS channels. Each instance contains 6 Channels, message information has to be acumulated over 4 instances 
#  (repeat the cycle each time the ChannelIdentifier = 1)
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	float64 systemTov  # Time since Power Up - Always Valid [sec]
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	int32 gps_week  # Number of weeks since Jan 6, 1980 GPS Week = -1 indicates GPS Week Unknown/Invalid
	uint32 ChannelIdentifier  # Start Channel ID number (1, 7, 13, 19)
	channel_status_t channel_status_1  # Status of channel # = ChannelIdentifier+0
	channel_status_t channel_status_2  # Status of channel # = ChannelIdentifier+1
	channel_status_t channel_status_3  # Status of channel # = ChannelIdentifier+2
	channel_status_t channel_status_4  # Status of channel # = ChannelIdentifier+3
	channel_status_t channel_status_5  # Status of channel # = ChannelIdentifier+4
	channel_status_t channel_status_6  # Status of channel # = ChannelIdentifier+5
