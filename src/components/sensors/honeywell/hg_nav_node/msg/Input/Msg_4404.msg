# 0x4404 : Case to Vehicle Frame Set Up

#
#  The message 0x4404 is used to change the output frame for the HGuide INS.
#  The device always outputs in the vehicle frame and when the vehicle frame
#  linear and attitude translations are zero (default setting), the case and vehicle frames are coincident.
# 

# Variables declaration
	uint32 AddressId  # Start of Message
	uint32 MessageId  # Message ID
	uint32 MessageLength  # Message Length
	uint32 Checksum  # Checksum
	float32 CasetoVehicleX  # [m] X Case to Vehicle Frame Offset (in the Case Frame)
	float32 CasetoVehicleY  # [m] Y Case to Vehicle Frame Offset (in the Case Frame)
	float32 CasetoVehicleZ  # [m] Z Case to Vehicle Frame Offset (in the Case Frame)
	float32 CasetoVehicleRoll  # [rad] X Intermediate Frame Roll: Third Rotation
	float32 CasetoVehiclePitch  # [rad] Y Intermediate Frame Pitch: Second Rotation
	float32 CasetoVehicleYaw  # [rad] Z Case Frame Yaw: First Rotation
	bool ChangeVehicleFrameAttitude  # 0 = Temporary | 1 = Flash (Saves Over Power Cycle)
	bool ChangeVehicleFrameOffset  # 0 = Temporary | 1 = Flash (Saves Over Power Cycle)
