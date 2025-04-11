# 0x6110 : Odometer Measurement Output

#
#  Sent out as a reply to the 0x4110 DMI message
# 

# Variables declaration
	uint32 AddressId  # Start of message flag
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message length
	uint32 Checksum  # Checksum
	float64 systemTov  # [sec] System Time of Validity
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	float32 Odo_X_Body_Distance  # [m] Distance travelled X (from DMI input)
	float32 Odo_Y_Body_Distance  # [m] Distance travelled Y (from DMI input)
	float32 Odo_Z_Body_Distance  # [m] Distance travelled Z (from DMI input)
	float32 Odo_X_Body_Distance_Compensated  # RESERVED [m] Compensated distance travelled X
	float32 Odo_Y_Body_Distance_Compensated  # RESERVED [m] Compensated distance travelled Y
	float32 Odo_Z_Body_Distance_Compensated  # RESERVED [m] Compensated distance travelled Z
	odo_body_vel_status_t Odo_status  # Mirrored 0x4110 input status
	float32 uart_latency  # [s] The UART Latency value added by the INS SW
	uint32 customer_latency  # [s] The value of the TimeOfValidityAdjustment in the 0x4110 message.
	ins_gnss_summary_t InsGnssSummary  # INS / GNSS summary word
