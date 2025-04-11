#ifndef ECTOS_CONSTANTS_package
#define ECTOS_CONSTANTS_package

/*
* SOFTWARE RIGHTS NOTICE:
*
*                           HONEYWELL PROPRIETARY
*
*                   Copyright Honeywell International Inc.
*
*                      Unpublished--All rights reserved
*
*  This work comprises software developed by Honeywell International Inc.
*  under internal Independent Research and Development funds.  This
*  software and all information and expression are the property of
*  Honeywell International Inc., contain trade secrets and may not, in
*  whole or in part, be licensed, used, duplicated, or disclosed for any
*  purpose without prior written permission of Honeywell International
*  Inc.
*
*  To the extent that any rights to use, modify, reproduce, release,
*  perform, display, or disclose this software have been granted, those
*  rights have been specified in the contract_rights.h file included in
*  this work.  Any reproduction or derivatives of this work, or portions
*  thereof, marked with this legend must also reproduce this marking and
*  be distributed with the contract_rights.h file. This notice and the
*  contract_rights.h file may not be removed or altered for any source
*  code distribution.
*
*  This document is controlled by the U.S. government and authorized 
*  for export only to the country of ultimate destination for use by 
*  the ultimate consignee or end-user(s) herein identified. They may 
*  not be resold, transferred, or otherwise disposed of, to any other 
*  country or to any person other than the authorized ultimate consignee 
*  or end-user(s), either in their original form or after being 
*  incorporated into other items, without first obtaining approval from 
*  the U.S. government or as otherwise authorized by U.S. law and 
*  regulations.
*  
*  Export Classification:               EAR99
*  
*  Country of Ultimate Origin:          United States
*  
*  Country of Ultimate Destination:     Czech Republic
*  
*  End-User:                            
*                                       Honeywell Aerospace - United States
*                                       Honeywell Aerospace - Czech Republic
*  
*  License:                             No License Required
*  
*/


#include <include/base_types.h>
#include <include/utils/math/math_interface.h>


namespace ECTOS
{
///    The CONSTANTS package contains the specification of all the constants 
///    defined for system operations purposes.  These include basic
///    numerical constants, and unit conversion factors.  
///
///    Name of constants do tend to be lenghthy, which is unfortunate.
///    However, it was decided that explicit clarity was very important
///    for these values as abreviated names tend to be somewhat
///    ambiguous.
///    (ex.  "nm" is this "nautical miles" or "nano-meters" or 
///           "Newton-meters" ??)
///
///    To use the conversion factors:
///       Multiply a given variable by the conversion factor to go
///       from "units A" to "units B".  
///       Example.  variable_of_units B := variable_of_unitsA * 
///                                           CONSTANTS::unitsA_to_unitsB
   namespace CONSTANTS
   {

      const   LONG_FLOAT_TYPE   INVALID_TIME               = -1.0;

// --------------------------------------------------------------------------
// Constant "Powers of 2"
// --------------------------------------------------------------------------

      const   LONG_FLOAT_TYPE   pow_2_neg_40    = 0.0000000000009094947017729282379150390625;/* 2^-40 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_39    = 0.000000000001818989403545856475830078125; /* 2^-39 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_38    = 0.00000000000363797880709171295166015625; /* 2^-38 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_37    = 0.0000000000072759576141834259033203125;  /* 2^-37 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_36    = 0.000000000014551915228366851806640625;   /* 2^-36 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_35    = 0.00000000002910383045673370361328125;    /* 2^-35 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_34    = 0.0000000000582076609134674072265625;     /* 2^-34 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_33    = 0.000000000116415321826934814453125;      /* 2^-33 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_32    = 0.00000000023283064365386962890625;       /* 2^-32 for scaling of output */   
      const   LONG_FLOAT_TYPE   pow_2_neg_31    = 0.0000000004656612873077392578125;        /* 2^-31 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_30    = 0.000000000931322574615478515625;         /* 2^-30 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_29    = 0.00000000186264514923095703125;          /* 2^-29 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_28    = 0.0000000037252902984619140625;           /* 2^-28 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_27    = 0.000000007450580596923828125;            /* 2^-27 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_26    = 0.00000001490116119384765625;             /* 2^-26 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_25    = 0.0000000298023223876953125;              /* 2^-25 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_24    = 0.000000059604644775390625;               /* 2^-24 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_23    = 0.00000011920928955078125;                /* 2^-23 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_22    = 0.0000002384185791015625;                 /* 2^-22 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_21    = 0.000000476837158203125;                  /* 2^-21 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_20    = 0.00000095367431640625;                   /* 2^-20 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_19    = 0.0000019073486328125;                    /* 2^-19 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_18    = 0.000003814697265625;                     /* 2^-18 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_17    = 0.00000762939453125;                      /* 2^-17 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_16    = 0.0000152587890625;                       /* 2^-16 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_15    = 0.000030517578125;                        /* 2^-15 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_14    = 0.00006103515625;                         /* 2^-14 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_13    = 0.0001220703125;                          /* 2^-13 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_12    = 0.000244140625;                           /* 2^-12 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_11    = 0.00048828125;                            /* 2^-11 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_10    = 0.0009765625;                             /* 2^-10 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_9     = 0.001953125;                              /* 2^-9 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_8     = 0.00390625;                               /* 2^-8 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_7     = 0.0078125;                                /* 2^-7 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_6     = 0.015625;                                 /* 2^-6 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_5     = 0.03125;                                  /* 2^-5 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_4     = 0.0625;                                   /* 2^-4 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_3     = 0.125;                                    /* 2^-3 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_2     = 0.25;                                     /* 2^-2 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_neg_1     = 0.5;                                      /* 2^-1 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_0         = 1.0;                                      /* 2^0 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_1         = 2.0;                                      /* 2^1 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_2         = 4.0;                                      /* 2^2 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_3         = 8.0;                                      /* 2^3 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_4         = 16.0;                                     /* 2^4 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_5         = 32.0;                                     /* 2^5 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_6         = 64.0;                                     /* 2^6 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_7         = 128.0;                                    /* 2^7 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_8         = 256.0;                                    /* 2^8 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_9         = 512.0;                                    /* 2^9 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_10        = 1024.0;                                   /* 2^10 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_11        = 2048.0;                                   /* 2^11 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_12        = 4096.0;                                   /* 2^12 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_13        = 8192.0;                                   /* 2^13 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_14        = 16384.0;                                  /* 2^14 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_15        = 32768.0;                                  /* 2^15 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_16        = 65536.0;                                  /* 2^16 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_17        = 131072.0;                                 /* 2^17 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_18        = 262144.0;                                 /* 2^18 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_19        = 524288.0;                                 /* 2^19 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_20        = 1048576.0;                                /* 2^20 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_21        = 2097152.0;                                /* 2^21 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_22        = 4194304.0;                                /* 2^22 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_23        = 8388608.0;                                /* 2^23 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_24        = 16777216.0;                               /* 2^24 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_25        = 33554432.0;                               /* 2^25 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_26        = 67108864.0;                               /* 2^26 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_27        = 134217728.0;                              /* 2^27 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_28        = 268435456.0;                              /* 2^28 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_29        = 536870912.0;                              /* 2^29 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_30        = 1073741824.0;                             /* 2^30 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_31        = 2147483648.0;                             /* 2^31 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_32        = 4294967296.0;                             /* 2^32 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_33        = 8589934592.0;                             /* 2^33 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_34        = 17179869184.0;                            /* 2^34 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_35        = 34359738368.0;                            /* 2^35 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_36        = 68719476736.0;                            /* 2^36 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_37        = 137438953472.0;                           /* 2^37 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_38        = 274877906944.0;                           /* 2^38 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_39        = 549755813888.0;                           /* 2^39 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_40        = 1099511627776.0;                          /* 2^40 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_43        = 8796093022208.0;                          /* 2^43 for scaling of output */
      const   LONG_FLOAT_TYPE   pow_2_55        = 36028797018963968.0;                      /* 2^55 for scaling of output */

// --------------------------------------------------------------------------
// Define number constants
// --------------------------------------------------------------------------

      const LONG_FLOAT_TYPE    hundred = 100.0;
      const LONG_FLOAT_TYPE    million = 1000000.0;
      const LONG_FLOAT_TYPE    two_hundred_sqrd =  200.0 * 200.0;
      const LONG_FLOAT_TYPE    four_hundred_sqrd =  400.0 * 400.0;

// --------------------------------------------------------------------------
// Define time constants
// --------------------------------------------------------------------------

      const LONG_FLOAT_TYPE    seconds_per_minute = 60.0;
      const LONG_FLOAT_TYPE    seconds_per_hour = 3600.0;
      const LONG_FLOAT_TYPE    seconds_per_day = 86400.0;
      const LONG_FLOAT_TYPE    seconds_per_week = 86400.0 * 7.0;
      const LONG_FLOAT_TYPE    seconds_per_year = 86400.0 * 365.0;
      
// --------------------------------------------------------------------------
// Define special numerical constants
// --------------------------------------------------------------------------

      const LONG_FLOAT_TYPE    pi               = 3.14159265358979323846264338327950288;
      const LONG_FLOAT_TYPE    half_pi          = 1.57079632679489661923132169163975114;
      const LONG_FLOAT_TYPE    inverse_pi       = 0.31830988618379067153776752674502872;
      const LONG_FLOAT_TYPE    pi_squared       = 9.86960440108935861883449099987615114;
      const LONG_FLOAT_TYPE    two_pi           = 6.28318530717958647692528676655900576;
      const LONG_FLOAT_TYPE    base_e           = 2.71828182845904523536028747135266250;
      const LONG_FLOAT_TYPE    e_squared        = 7.38905609893065022723042746057500781;
      const LONG_FLOAT_TYPE    inverse_e        = 0.36787944117144232159552377016146087;

// --------------------------------------------------------------------------
// Define some miscellaneous constants
// --------------------------------------------------------------------------

      // Speed of light in vacuum (meters/sec)
      const LONG_FLOAT_TYPE    speed_of_light = 299792458.0;
      // A unit conversion constant
      //const LONG_FLOAT_TYPE   fine_delta = MATH_INTERFACE::Pow(2.0,-31.0);
      // Pitch angle deviation from 90 degrees where heading and roll are computed
      // (radians)
      const LONG_FLOAT_TYPE   roll_heading_valid = 1e-3;

      const LONG_FLOAT_TYPE   std_pressure_at_sea_level = 14.69072;      

// --------------------------------------------------------------------------
//               Theoretical WGS-84 Gravity Model Parameters
// --------------------------------------------------------------------------

      // Description : Mean Value of Theoretical (Normal) Gravity
      // Units       : (meters)/(second**2)
      const LONG_FLOAT_TYPE   gravity_mean = 9.7976446561;

// --------------------------------------------------------------------------
// Chip lengths and wavelengths
// --------------------------------------------------------------------------

      const LONG_FLOAT_TYPE   CAChipRate          = 1023000;      // Hz
      const LONG_FLOAT_TYPE   PyChipRate          = 10230000;      // Hz
      const LONG_FLOAT_TYPE   L1Frequency         = 1575420000;   // Hz
      const LONG_FLOAT_TYPE   L2Frequency         = 1227600000;   // Hz
      const LONG_FLOAT_TYPE   CAChipLength        = speed_of_light / CAChipRate;   // meters
      const LONG_FLOAT_TYPE   PyChipLength        = speed_of_light / PyChipRate;   // meters
      const LONG_FLOAT_TYPE   L1CarrierWavelength = speed_of_light / L1Frequency;   // meters
      const LONG_FLOAT_TYPE   L2CarrierWavelength = speed_of_light / L2Frequency;   // meters


// --------------------------------------------------------------------------
// Angle Conversion
// --------------------------------------------------------------------------

      // Description : Converts arc seconds to radians.
      // Units       : (radians)/(arc seconds)
      const LONG_FLOAT_TYPE    arcseconds_to_radians = (pi / 180.0) / 3600.0;
      // Description : Converts degrees to radians.
      // Units       : (radians)/(degrees)
      const LONG_FLOAT_TYPE    degrees_to_radians = pi / 180.0;
      // Description : Converts pi fractions to radians.
      // Units       : (radians)/(pi fractions)
      const LONG_FLOAT_TYPE    pi_fractions_to_radians = pi * (1/pow_2_31);
      // Description : Converts radians to arc seconds.
      // Units       : (arc seconds)/(radians)
      const LONG_FLOAT_TYPE    radians_to_arcseconds = 3600.0 * (180.0 / pi);
      // Description : Converts radians to degrees.
      // Units       : (degrees)/(radians)
      const LONG_FLOAT_TYPE    radians_to_degrees = 180.0 / pi;
      // Description : Converts radians to milli-radians.
      // Units       : (milli-radians)/(radians)
      const LONG_FLOAT_TYPE    radians_to_milliradians = 1000.0;
      // Description : Converts radians to micro-radians.
      // Units       : (micro-radians)/(radians)
      const LONG_FLOAT_TYPE    radians_to_microradians = 1000000.0;
      // Description : Converts milli-radians to radians.
      // Units       : (radians)/(milli-radians)
      const LONG_FLOAT_TYPE    milliradians_to_radians = 0.001;
      // Description : Converts micro-radians to radians.
      // Units       : (radians)/(micro-radians)
      const LONG_FLOAT_TYPE    microradians_to_radians = 0.000001;
      // Description : Converts radians to pi fractions.
      // Units       : (pi fractions)/(radians)
      const LONG_FLOAT_TYPE    radians_to_pi_fractions = 1.0 / pi_fractions_to_radians;
      // Description : Converts minutes to degrees.
      // Units       : (degrees)/(minutes)
      const LONG_FLOAT_TYPE    minutes_to_degrees = 1.0 / 60.0;
      
// --------------------------------------------------------------------------
// Length Conversion
// --------------------------------------------------------------------------

      // Description : Converts English feet to SI meters.
      // Units       : (meters)/(feet)
      const LONG_FLOAT_TYPE    feet_to_meters = 0.3048;
      // Description : Converts SI meters to English feet.
      // Units       : (feet)/(meters)
      const LONG_FLOAT_TYPE    meters_to_feet = 1.0 / feet_to_meters;
       // Description : Converts nautical miles to English feet.
      // Units       : (feet)/(nautical mile)
      const LONG_FLOAT_TYPE    nautical_miles_to_feet = 6076.12;
      // Description : Converts nautical miles to SI meters.
      // Units       : (meters)/(nautical mile)
      const LONG_FLOAT_TYPE    nautical_miles_to_meters = 1852.001376;
      // Description : Converts English feet to nautical miles.
      // Units       : (nautical mile)/(feet)
      const LONG_FLOAT_TYPE    feet_to_nautical_miles = 1.0 / nautical_miles_to_feet;
      // Description : Converts SI meters to nautical miles.
      // Units       : (nautical mile)/(meters)
      const LONG_FLOAT_TYPE    meters_to_nautical_miles = 1.0 / nautical_miles_to_meters;
      // Description : Converts meters to millimeters
      // Units       : (millimeter)/(meter)
      const LONG_FLOAT_TYPE    meter_to_millimeter = 1000.0;
      // Description : Converts millimeters to meters
      // Units       : (meter)/(millimeter)
      const LONG_FLOAT_TYPE    millimeter_to_meter = 1.0 / meter_to_millimeter;

// --------------------------------------------------------------------------
// Time Conversion
// --------------------------------------------------------------------------

      // Description : Converts hours to seconds.
      // Units       : (seconds)/(hours)
      const LONG_FLOAT_TYPE    hours_to_seconds = 3600.0;
      // Description : Converts nanoseconds to seconds.
      // Units       : (seconds)/(nanoseconds)
      const LONG_FLOAT_TYPE    nanoseconds_to_seconds = 1.0e-9;
      // Description : Converts microseconds to seconds.
      // Units       : (seconds)/(microseconds)
      const LONG_FLOAT_TYPE    microseconds_to_seconds = 1.0 / 1000000.0;
      // Description : Converts milliseconds to seconds.
      // Units       : (seconds)/(milliseconds)
      const LONG_FLOAT_TYPE    milliseconds_to_seconds = 1.0 / 1000.0;
      // Description : Converts minutes to seconds.
      // Units       : (seconds)/(minutes)
      const LONG_FLOAT_TYPE    minutes_to_seconds = 60.0;
      // Description : Converts seconds to hours.
      // Units       : (hour)/(seconds)
      const LONG_FLOAT_TYPE    seconds_to_hours = 1.0 / 3600.0;
      // Description : Converts seconds to nanoseconds.
      // Units       : (nanoseconds)/(seconds)
      const LONG_FLOAT_TYPE    seconds_to_nanoseconds = 1.0e9;
      // Description : Converts seconds to microseconds.
      // Units       : (microseconds)/(seconds)
      const LONG_FLOAT_TYPE    seconds_to_microseconds = 1000000.0;
      // Description : Converts seconds to milliseconds.
      // Units       : (milliseconds)/(seconds)
      const LONG_FLOAT_TYPE    seconds_to_milliseconds = 1000.0;
      // Description : Converts seconds to minutes.
      // Units       : (minutes)/(seconds)
      const LONG_FLOAT_TYPE    seconds_to_minutes = 1.0 / 60.0;
      // Description : Converts (1/square-root hours) to (1/square-root seconds)
      // Units       : sqrt(hours)/sqrt(seconds)
      const LONG_FLOAT_TYPE    per_root_hour_to_per_root_second = 1.0 / 60.0;
      // Description : Converts (1/square-root minutes) to (1/square-root seconds)
      // Units       : sqrt(minutes)/sqrt(seconds)
      //const LONG_FLOAT_TYPE    per_root_minute_to_per_root_second = 1.0 / MATH_INTERFACE::Sqrt(60.0);

// --------------------------------------------------------------------------
// Velocity Conversion
// --------------------------------------------------------------------------

      // Description : Converts feet/second to nautical miles/hour.
      // Units       : (nautical miles/hour)/(feet/second)
      const LONG_FLOAT_TYPE    feet_per_second_to_knots = 3600.0 / 6076.0;
      // Description : Converts nautical miles/hour to feet/second.
      // Units       : (feet/second)/(nautical miles/hour)
      const LONG_FLOAT_TYPE    knots_to_feet_per_second = 6076.0 / 3600.0;
      // Description : Converts micro-g's to g's.
      // Units       : (micro-g)/(g)
      const LONG_FLOAT_TYPE    micro_g_to_g = 0.000001;
      // Description : Converts g's to micro-g's.
      // Units       : (g)/(micro-g)
      const LONG_FLOAT_TYPE    g_to_micro_g = 1000000.0;
      // Description : Converts g's to meters/sec/sec
      // Units       : (m/s^2)/ (g)
      const LONG_FLOAT_TYPE    g_to_meters_per_sec2 = gravity_mean;
      // Description : Converts meters/sec/sec to g's
      // Units       : (g)/(m/s^2)
      const LONG_FLOAT_TYPE    meters_per_sec2_to_g = 1.0/gravity_mean;
      // Description : Converts meters/second to millimeters/second
      // Units       : (mm/s)/(m/s)
      const LONG_FLOAT_TYPE    meter_per_sec_to_millimeter_per_sec = 1000.0;
      // Description : Converts millimeters/second to meters/second
      // Units       : (m/s)/(mm/s)
      const LONG_FLOAT_TYPE    millimeter_per_sec_to_meter_per_sec = 1.0 / meter_per_sec_to_millimeter_per_sec;

// --------------------------------------------------------------------------
// Angular Rate Conversion
// --------------------------------------------------------------------------

      // Description : Converts degrees/hour to radians/second.
      // Units       : (radians/second)/(degrees/hour)
      const LONG_FLOAT_TYPE    deg_per_hr_to_rad_per_sec = 
         degrees_to_radians / hours_to_seconds;
      // Description : Converts radians/second to degrees/hour.
      // Units       : (degrees/hour)/(radians/second)
      const LONG_FLOAT_TYPE    rad_per_sec_to_deg_per_hr = 
         radians_to_degrees / seconds_to_hours;
      // Description : Converts degrees/sqrt(hour) to radians/sqrt(second).
      // Units       : (radians/sqrt(second))/(degrees/sqrt(hour))
      //const LONG_FLOAT_TYPE    deg_per_sqrthr_to_rad_per_sqrtsec =
      //   degrees_to_radians / MATH_INTERFACE::Sqrt(hours_to_seconds);
      // Description : Converts radians/sqrt(second) to degrees/sqrt(hour).
      // Units       : (degrees/sqrt(hour))/(radians/sqrt(second))
      //const LONG_FLOAT_TYPE    rad_per_sqrtsec_to_deg_per_sqrthr =
      //   radians_to_degrees / MATH_INTERFACE::Sqrt(seconds_to_hours);

// --------------------------------------------------------------------------
// Scaling Conversion
// --------------------------------------------------------------------------

      // Description : Converts parts per million to part
      // Units       : (part/ppm)
      const LONG_FLOAT_TYPE    ppm_to_part = 0.000001;
      
      const LONG_FLOAT_TYPE    ppb_to_part = 0.000000001;

      // Description : Converts parts per million to part
      // Units       : (ppm/part)
      const LONG_FLOAT_TYPE    part_to_ppm = 1000000.0;
      const LONG_FLOAT_TYPE    percent_to_part = 0.01;
      
   }
}

#endif // #ifndef ECTOS_CONSTANTS_package
