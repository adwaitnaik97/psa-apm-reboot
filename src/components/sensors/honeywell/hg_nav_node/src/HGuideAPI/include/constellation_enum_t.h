#ifndef __HGuideAPI_constellation_enum_t_h__
#define __HGuideAPI_constellation_enum_t_h__
#pragma once

// List of available Satellite constellations
enum constellation_enum_t
{
	GPS     = 0,
	GLONASS = 1,
	Galileo = 2,
	SBAS    = 3,
	QZSS    = 4,
	BeiDou  = 5,
	IRNSS   = 6,
	Other   = 7
};

#endif // __HGuideAPI_constellation_enum_t_h__
