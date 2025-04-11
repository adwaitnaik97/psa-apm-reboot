# 0x4202 : Event-Out Control Input Message

#
#  The 0x4202 message is used to enable and disable output Events as well as configure their parameters
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	event_out_mark_t MarkPort  # Event-Out Port Number
	
	# Edge Trigger:
	# 0 = Rising Edge (Active High)
	# 1 = Falling Edge (Active Low)
	# 2 = Both rising and falling Edge (Alternating High-Low)
	uint8 Polarity
	bool Enable_6211_Time  # Output 0x6211 Time Stamp + GPS Week
	bool ChangeEventOutSetUp  # 0 = Temporary | 1 = Flash (Saves Over Power Cyle)
	
	# Event-Out Trigger
 	# 0 = Disabled
 	# 1 = Frequency (100Hz Max)
 	# 2 = Distance Travelled
 	# 3 = Velocity Change (Reserved)
 	# 4 = Attitude Change (Reserved)
 	# 5 = Position Change (Reserved)
 	# 6 = Acceleration Threshold (Reserved)
 	# 7 = Angular Rate Threshold (Reserved)
	uint8 Trigger
	
	# Value to trigger Event-Out. The unit is different for each trigger source.
 	# Consult the Trigger table to learn more
	float32 Trigger_Value
	float32 time_offset  # Time of Event Output = Trigger of event + time_offset [seconds]
	
	# Event Out pulse width
 	# in case of Polarity = 2 - Minimum time to elapse before allowing another switch
	float32 pulse_width
