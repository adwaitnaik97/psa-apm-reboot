#ifndef __HGuideAPI_event_out_mark_t_h__
#define __HGuideAPI_event_out_mark_t_h__
#pragma once

//
// Event-Out port the message pertains to:
//
enum event_out_mark_t
{
	EVENTOUT1    = 0, //  [g080]
	EVENTOUT2    = 1, //  [g080]
	EVENTOUT3    = 2, //  [g080]
	EVENTOUT4    = 3, //  [g080]
	EVENTOUT5    = 4, //  [reserved]
	EVENTOUT6    = 5, //  [reserved]
	EVENTOUT7    = 6, //  [reserved]
	EVENTOUT8    = 7, //  [reserved]
	EVENTOUT_PPS = 8 //  GNSS Receiver PPS  [g080]
};

#endif // __HGuideAPI_event_out_mark_t_h__
