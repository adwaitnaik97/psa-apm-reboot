#pragma once

#include <cstdint>

HGUIDE_DLL uint32_t computeChecksum(uint32_t* buffer, unsigned int size);
HGUIDE_DLL uint16_t computeChecksum16Bit(uint8_t *buffer, unsigned int size);
HGUIDE_DLL unsigned long Calculate32BitCRC(const unsigned char * data, unsigned long length);
HGUIDE_DLL unsigned long computeNovatelCRC32(unsigned char *ucBuffer, unsigned long ulCount);