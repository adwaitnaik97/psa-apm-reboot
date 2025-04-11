# 0x9910 : Processor Loading

#
#  Contains the processor loading for the INS SW threads.
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	uint8 own_usage_rate_0  # Rate 0 thread's own usage [%]
	uint8 own_usage_rate_1  # Rate 1 thread's own usage [%]
	uint8 own_usage_rate_2  # Rate 2 thread's own usage [%]
	uint8 own_usage_rate_3  # Rate 3 thread's own usage [%]
	uint8 own_usage_rate_4  # Rate 4 thread's own usage [%]
	uint8 own_usage_rate_5  # Rate 5 thread's own usage [%]
	uint8 own_usage_rate_6  # Rate 6 thread's own usage [%]
	uint8 own_usage_rate_7  # Rate 7 thread's own usage [%]
	uint8 total_usage_rate_0  # Rate 0 thread's total usage [%]
	uint8 total_usage_rate_1  # Rate 1 thread's total usage [%]
	uint8 total_usage_rate_2  # Rate 2 thread's total usage [%]
	uint8 total_usage_rate_3  # Rate 3 thread's total usage [%]
	uint8 total_usage_rate_4  # Rate 4 thread's total usage [%]
	uint8 total_usage_rate_5  # Rate 5 thread's total usage [%]
	uint8 total_usage_rate_6  # Rate 6 thread's total usage [%]
	uint8 total_usage_rate_7  # Rate 7 thread's total usage [%]
	uint8 max_usage_rate_0  # Rate 0 thread's max usage [%]
	uint8 max_usage_rate_1  # Rate 1 thread's max usage [%]
	uint8 max_usage_rate_2  # Rate 2 thread's max usage [%]
	uint8 max_usage_rate_3  # Rate 3 thread's max usage [%]
	uint8 max_usage_rate_4  # Rate 4 thread's max usage [%]
	uint8 max_usage_rate_5  # Rate 5 thread's max usage [%]
	uint8 max_usage_rate_6  # Rate 6 thread's max usage [%]
	uint8 max_usage_rate_7  # Rate 7 thread's max usage [%]
	uint8 total_usage  # Total usage accross all threads [%]
	uint8 max_usage  # Maximum usage accross all threads [%]
