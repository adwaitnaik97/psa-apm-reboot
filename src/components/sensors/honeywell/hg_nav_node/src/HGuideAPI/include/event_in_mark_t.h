#ifndef __HGuideAPI_event_in_mark_t_h__
#define __HGuideAPI_event_in_mark_t_h__
#pragma once

//
// Event-In port the message pertains to:
//
enum event_in_mark_t
{
	EVENTIN1      = 0, //  [n580, n380, g080]
	EVENTIN2      = 1, //  [n380, g080]
	EVENTIN3      = 2, //  [g080]
	EVENTIN4      = 3, //  [g080]
	EVENTIN5      = 4, //  [reserved]
	EVENTIN6      = 5, //  [reserved]
	EVENTIN7      = 6, //  [reserved]
	EVENTIN8      = 7, //  [reserved]
	INTERNAL_GNSS = 31 //  Internal GNSS Receiver PPS  [n580, n380]
};

#endif // __HGuideAPI_event_in_mark_t_h__
