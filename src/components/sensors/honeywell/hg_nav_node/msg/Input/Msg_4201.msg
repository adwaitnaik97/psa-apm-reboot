# 0x4201 : Event-In Control Input Message

#
#  The 0x4201 message is used to enable and disable output (0x62xx) messages in response to
#  to an analog Event In 1 or 2 signal or to an internal PPS signal. 
#       a) The 0x4201 message is output every 10 seconds if any Mark Events are enabled. 
#       b) If there is no GNSS - the Internal GNSS Mark Port 8 will not generate events.
#       c) 100 Hz is the maximum output frequency for each set of 62xx messages.
#       d) The 0x4201 message only effects the commmunication port connected.   If one wishes
#          to affect the internal logger (if available), one must hook up to the port designated
#          for logging (typically COM3 is the default)
#       e) Each event in generates its own set of messages distinguished by the Mark Port Number.
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	event_in_mark_t MarkPort  # Event-In Port Number
	bool Enable_6201_Time  # Output 0x6201 Time Stamp + GPS Week
	bool Enable_6202_Position  # Output 0x6202 Position and Time Stamp
	bool Enable_6203_Velocity  # Output 0x6203 Velocity and Time Stamp
	bool Enable_6204_Attitude  # Output 0x6204 Attitude and Time Stamp
	bool Enable_6205_Full_State  # Output 0x6205 Position, Velocity, Attitude and Time Stamp
	bool ChangeEventInSetUp  # 0 = Temporary | 1 = Flash (Saves Over Power Cyle)
	
	# Edge Trigger:
	# 0 = Rising Edge
	# 1 = Falling Edge
	# 2 = Both rising and falling Edge
	uint8 Polarity
	float32 time_offset  # Time of Validity of Event = System Time of event + time_offset [seconds]
	
	# Minimum time to elapse before allowing another input [seconds]
	# It is recommended to use the time_guard value if you expect noise in your trigger signal.
	# This circumvents the probability of unwanted triggers.
	float32 time_guard
