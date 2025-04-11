# 0x6738 : DVL Kalman Filter Data

#
#  DVL velocity residuals, error estimates and standard deviations
#  values provided by the INS kalman filter
# 

# Variables declaration
	uint32 AddressId  # Start of message flag
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message length in 32-bit word
	uint32 Checksum  # Checksum
	float64 systemTov  # System Time of Validity (seconds)
	float32 velNormResBeam0BT  # Velocity normalized residual Beam 0 BT
	float32 velNormResBeam0WT  # Velocity normalized residual Beam 0 WT
	float32 velNormResBeam1BT  # Velocity normalized residual Beam 1 BT
	float32 velNormResBeam1WT  # Velocity normalized residual Beam 1 WT
	float32 velNormResBeam2BT  # Velocity normalized residual Beam 2 BT
	float32 velNormResBeam2WT  # Velocity normalized residual Beam 2 WT
	float32 velNormResBeam3BT  # Velocity normalized residual Beam 3 BT
	float32 velNormResBeam3WT  # Velocity normalized residual Beam 3 WT
	float32 sfErrorEstBeam0  # Scale Factor Error Estimate Beam 0
	float32 sfErrorEstBeam1  # Scale Factor Error Estimate Beam 1
	float32 sfErrorEstBeam2  # Scale Factor Error Estimate Beam 2
	float32 sfErrorEstBeam3  # Scale Factor Error Estimate Beam 3
	float32 psiYErrorEstBeam0  # PSI Y Error Estimate Beam 0
	float32 psiYErrorEstBeam1  # PSI Y Error Estimate Beam 1
	float32 psiYErrorEstBeam2  # PSI Y Error Estimate Beam 2
	float32 psiYErrorEstBeam3  # PSI Y Error Estimate Beam 3
	float32 psiZErrorEstBeam0  # PSI Z Error Estimate Beam 0
	float32 psiZErrorEstBeam1  # PSI Z Error Estimate Beam 1
	float32 psiZErrorEstBeam2  # PSI Z Error Estimate Beam 2
	float32 psiZErrorEstBeam3  # PSI Z Error Estimate Beam 3
	float32 oceanCurErrorEstBeam0  # Ocean Current Error Estimate Beam 0
	float32 oceanCurErrorEstBeam1  # Ocean Current Error Estimate Beam 1
	float32 oceanCurErrorEstBeam2  # Ocean Current Error Estimate Beam 2
	float32 oceanCurErrorEstBeam3  # Ocean Current Error Estimate Beam 3
	float32 sfStdvBeam0  # Scale Factor Standard Deviation Beam 0
	float32 sfStdvBeam1  # Scale Factor Standard Deviation Beam 1
	float32 sfStdvBeam2  # Scale Factor Standard Deviation Beam 2
	float32 sfStdvBeam3  # Scale Factor Standard Deviation Beam 3
	float32 psiYStdvBeam0  # PSI Y Standard Deviation Beam 0
	float32 psiYStdvBeam1  # PSI Y Standard Deviation Beam 1
	float32 psiYStdvBeam2  # PSI Y Standard Deviation Beam 2
	float32 psiYStdvBeam3  # PSI Y Standard Deviation Beam 3
	float32 psiZStdvBeam0  # PSI Z Standard Deviation Beam 0
	float32 psiZStdvBeam1  # PSI Z Standard Deviation Beam 1
	float32 psiZStdvBeam2  # PSI Z Standard Deviation Beam 2
	float32 psiZStdvBeam3  # PSI Z Standard Deviation Beam 3
	float32 oceanCurStdvBeam0  # Ocean Current Standard Deviation Beam 0
	float32 oceanCurStdvBeam1  # Ocean Current Standard Deviation Beam 1
	float32 oceanCurStdvBeam2  # Ocean Current Standard Deviation Beam 2
	float32 oceanCurStdvBeam3  # Ocean Current Standard Deviation Beam 3
