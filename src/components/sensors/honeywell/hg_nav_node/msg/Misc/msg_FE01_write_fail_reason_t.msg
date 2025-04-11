# Parameter_2 is used to convey the reason why the Xfer failed
# msg_FE01_write_fail_reason_t - enumeration
uint8 value

#Pre-Defined ENUM Constants
	uint8 WRITE_FAILED_CRC_ERROR          = 1 #  CRC of received data doesn't match the one presented
	uint8 WRITE_FAILED_BLOCK_XFER_TIMEOUT = 2 #  Timeout in block transfer
	uint8 WRITE_FAILED_MISSED_BLOCK       = 4 #  Block missed in transfer - parameter 4 contains the amount of missed blocks
	uint8 WRITE_FAILED_DATA_NOT_NEW       = 8
	uint8 WRITE_FAILED_UNEXPECTED_TYPE    = 16
	uint8 WRITE_FAILED_DUPLICATE_BLOCK    = 32 #  Same block sent multiple times
