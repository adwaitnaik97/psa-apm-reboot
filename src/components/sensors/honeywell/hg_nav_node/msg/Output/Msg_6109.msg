# 0x6109 : GNSS Receiver Attitude/Heading Measurement Output

#
#  RAW attitude from GNSS receiver. The attitude is refered from Main (RF1) antenna to Aux (RF2) antenna
#  This message is being outputted only when GNSS Signal is avaiable and both RF1 and RF2 are connected and separated by sufficient distance
# 

# Variables declaration
	uint32 AddressId  # Start of message flag
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message length
	uint32 Checksum  # Checksum
	float64 systemTov  # [sec] System Time of Validity
	float64 gpsTov  # [sec] GPS Time of Validity in current week (Sunday 00:00 UTC). When invalid, GPS time = INS/GPS Internal System Time
	float32 Roll  # [rad] Roll
	float32 Pitch  # [rad] Pitch
	float32 True_Heading  # [rad] True Heading
	float32 Roll_Stdv  # [rad] Roll Stdv
	float32 Pitch_Stdv  # [rad] Pitch Stdv
	float32 True_Heading_Stdv  # [rad] True Heading Stdv
	bool Data_Valid  # Data Valid
	ins_gnss_summary_t InsGnssSummary  # INS / GNSS summary word
