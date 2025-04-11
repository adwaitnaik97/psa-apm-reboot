#ifndef __HGuideAPI_Msg_6738_h__
#define __HGuideAPI_Msg_6738_h__
#pragma once

#include <cstdint>


// 0x6738 : DVL Kalman Filter Data
//
// DVL velocity residuals, error estimates and standard deviations
// values provided by the INS kalman filter
// 
class HGUIDE_DLL Msg_6738
{
public:
	Msg_6738();

	void Default();
	bool Serialize(unsigned char * buffer, const unsigned int bufferSize);
	int Deserialize(unsigned char * buffer, const unsigned int bufferSize);

	//Get Message Length as a number of uint8 bytes
	int getMessageLength(void) {return 192;}

public:
	static const uint32_t AddressId = 0xA5C381FF; // Start of message flag
	static const uint32_t MessageId = 0x6738; // Message ID
	static const uint32_t MessageLength = 48; // Message length in 32-bit word
	uint32_t Checksum; // Checksum
	double systemTov; // System Time of Validity (seconds)
	float velNormResBeam0BT; // Velocity normalized residual Beam 0 BT
	float velNormResBeam0WT; // Velocity normalized residual Beam 0 WT
	float velNormResBeam1BT; // Velocity normalized residual Beam 1 BT
	float velNormResBeam1WT; // Velocity normalized residual Beam 1 WT
	float velNormResBeam2BT; // Velocity normalized residual Beam 2 BT
	float velNormResBeam2WT; // Velocity normalized residual Beam 2 WT
	float velNormResBeam3BT; // Velocity normalized residual Beam 3 BT
	float velNormResBeam3WT; // Velocity normalized residual Beam 3 WT
	float sfErrorEstBeam0; // Scale Factor Error Estimate Beam 0
	float sfErrorEstBeam1; // Scale Factor Error Estimate Beam 1
	float sfErrorEstBeam2; // Scale Factor Error Estimate Beam 2
	float sfErrorEstBeam3; // Scale Factor Error Estimate Beam 3
	float psiYErrorEstBeam0; // PSI Y Error Estimate Beam 0
	float psiYErrorEstBeam1; // PSI Y Error Estimate Beam 1
	float psiYErrorEstBeam2; // PSI Y Error Estimate Beam 2
	float psiYErrorEstBeam3; // PSI Y Error Estimate Beam 3
	float psiZErrorEstBeam0; // PSI Z Error Estimate Beam 0
	float psiZErrorEstBeam1; // PSI Z Error Estimate Beam 1
	float psiZErrorEstBeam2; // PSI Z Error Estimate Beam 2
	float psiZErrorEstBeam3; // PSI Z Error Estimate Beam 3
	float oceanCurErrorEstBeam0; // Ocean Current Error Estimate Beam 0
	float oceanCurErrorEstBeam1; // Ocean Current Error Estimate Beam 1
	float oceanCurErrorEstBeam2; // Ocean Current Error Estimate Beam 2
	float oceanCurErrorEstBeam3; // Ocean Current Error Estimate Beam 3
	float sfStdvBeam0; // Scale Factor Standard Deviation Beam 0
	float sfStdvBeam1; // Scale Factor Standard Deviation Beam 1
	float sfStdvBeam2; // Scale Factor Standard Deviation Beam 2
	float sfStdvBeam3; // Scale Factor Standard Deviation Beam 3
	float psiYStdvBeam0; // PSI Y Standard Deviation Beam 0
	float psiYStdvBeam1; // PSI Y Standard Deviation Beam 1
	float psiYStdvBeam2; // PSI Y Standard Deviation Beam 2
	float psiYStdvBeam3; // PSI Y Standard Deviation Beam 3
	float psiZStdvBeam0; // PSI Z Standard Deviation Beam 0
	float psiZStdvBeam1; // PSI Z Standard Deviation Beam 1
	float psiZStdvBeam2; // PSI Z Standard Deviation Beam 2
	float psiZStdvBeam3; // PSI Z Standard Deviation Beam 3
	float oceanCurStdvBeam0; // Ocean Current Standard Deviation Beam 0
	float oceanCurStdvBeam1; // Ocean Current Standard Deviation Beam 1
	float oceanCurStdvBeam2; // Ocean Current Standard Deviation Beam 2
	float oceanCurStdvBeam3; // Ocean Current Standard Deviation Beam 3
};

#endif // __HGuideAPI_Msg_6738_h__
