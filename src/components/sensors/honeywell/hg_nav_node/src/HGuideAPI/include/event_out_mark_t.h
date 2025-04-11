#ifndef __HGuideAPI_event_out_mark_t_h__
#define __HGuideAPI_event_out_mark_t_h__
#pragma once

//
// Event-Out port the message pertains to:
//
enum event_out_mark_t
{
	EVENTOUT1    = 1, //  [g080]
	EVENTOUT2    = 2, //  [g080]
	EVENTOUT3    = 3, //  [g080]
	EVENTOUT4    = 4, //  [g080]
	EVENTOUT5    = 5, //  [reserved]
	EVENTOUT6    = 6, //  [reserved]
	EVENTOUT7    = 7, //  [reserved]
	EVENTOUT8    = 8, //  [reserved]
	EVENTOUT_PPS = 31 //  GNSS Receiver PPS  [g080]
};

#endif // __HGuideAPI_event_out_mark_t_h__
