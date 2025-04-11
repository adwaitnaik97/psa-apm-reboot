#ifndef __HGuideAPI_Msg_4404_h__
#define __HGuideAPI_Msg_4404_h__
#pragma once

#include <cstdint>


// 0x4404 : Case to Vehicle Frame Set Up
//
// The message 0x4404 is used to change the output frame for the HGuide INS.
// The device always outputs in the vehicle frame and when the vehicle frame
// linear and attitude translations are zero (default setting), the case and vehicle frames are coincident.
// 
class HGUIDE_DLL Msg_4404
{
public:
	Msg_4404();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 52;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of Message
	static const uint32_t MessageId = 0x4404; // Message ID
	static const uint32_t MessageLength = 13; // Message Length
	uint32_t Checksum; // Checksum
	float CasetoVehicleX; // [m] X Case to Vehicle Frame Offset (in the Case Frame)
	float CasetoVehicleY; // [m] Y Case to Vehicle Frame Offset (in the Case Frame)
	float CasetoVehicleZ; // [m] Z Case to Vehicle Frame Offset (in the Case Frame)
	float CasetoVehicleRoll; // [rad] X Intermediate Frame Roll: Third Rotation
	float CasetoVehiclePitch; // [rad] Y Intermediate Frame Pitch: Second Rotation
	float CasetoVehicleYaw; // [rad] Z Case Frame Yaw: First Rotation
	bool ChangeVehicleFrameAttitude; // 0 = Temporary | 1 = Flash (Saves Over Power Cycle)
	bool ChangeVehicleFrameOffset; // 0 = Temporary | 1 = Flash (Saves Over Power Cycle)
};

#endif // __HGuideAPI_Msg_4404_h__
