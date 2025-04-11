# 0x6601 : Virtual Status LED message

#
#  Message used to report the physical LED status as well as simulate LEDs on devices without LEDs
#  blue/green/red => RGB value
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	led_bitfield_t PWR_LED_State  # Power LED State
	led_bitfield_t POS_LED_State  # Position Status LED State
	led_bitfield_t SYS_LED_State  # System Status LED State
	led_bitfield_t LOG_LED_State  # Data logging Status LED State
