#ifndef __HGuideAPI_event_in_mark_t_h__
#define __HGuideAPI_event_in_mark_t_h__
#pragma once

//
// Event-In port the message pertains to:
//
enum event_in_mark_t
{
	EVENTIN1      = 0,
	EVENTIN2      = 1,
	EVENTIN3      = 2,
	EVENTIN4      = 3,
	EVENTIN5      = 4, //  [reserved]
	EVENTIN6      = 5, //  [reserved]
	EVENTIN7      = 6, //  [reserved]
	EVENTIN8      = 7, //  [reserved]
	INTERNAL_GNSS = 8 //  Internal GNSS Receiver PPS
};

#endif // __HGuideAPI_event_in_mark_t_h__
