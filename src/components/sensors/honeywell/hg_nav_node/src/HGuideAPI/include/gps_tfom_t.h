#ifndef __HGuideAPI_gps_tfom_t_h__
#define __HGuideAPI_gps_tfom_t_h__
#pragma once

//
// GPS Time Figure Of Merit (TFOM)
//
enum gps_tfom_t
{
	TFOM_0  = 0, //  Not valid 
	TFOM_1  = 1, //  Estimated time error (ETE) <=  1 nanosecond
	TFOM_2  = 2, //       1 nanosecond    < ETE <=   10 nanosecond
	TFOM_3  = 3, //      10 nanosecond    < ETE <=  100 nanosecond
	TFOM_4  = 4, //     100 nanosecond    < ETE <=    1 microsecond 
	TFOM_5  = 5, //       1 microsecond   < ETE <=   10 microsecond 
	TFOM_6  = 6, //      10 microsecond   < ETE <=  100 microsecond 
	TFOM_7  = 7, //     100 microsecond   < ETE <=    1 millisecond
	TFOM_8  = 8, //       1 millisecond   < ETE <=   10 millisecond
	TFOM_9  = 9, //                         ETE >    10 millisecond
	TFOM_10 = 10 //  Not valid
};

#endif // __HGuideAPI_gps_tfom_t_h__
