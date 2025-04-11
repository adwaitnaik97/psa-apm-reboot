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

#ifndef MATH_INTERFACE_package
#define MATH_INTERFACE_package
// *****************************************************************************
// UNIT: MATH_INTERFACE
//
//       Package Specification
//
// FILES: math_interface.h
//
// RELATED FILES: math_interface.cpp
//
// DESCRIPTION:
//
//    This package has been created to interface between the created
//    BASE_TYPES data types and the intrinsic math functions available in
//    a given compiler environment.
//
//    Because various compiler math function/packages have been written
//    specifically for FLOAT, SHORT FLOAT, and INTEGER, or are declared
//    generic, in order to use the data types presented in BASE_TYPES this
//    interface package must be utilized to type define the various math
//    functions.
//
//    This package aids with the portability of the Ada code since new
//    compilers can link directly to their own math routines through this
//    package, and all code using this MATH_INTERFACE package can remain
//    the same.
//
//    Math functions have been created in the following formats:
//      y = f(x); output "y" is some function of input parameter "x",
//    or
//      z = f(x,y); output "z" is some function of input parameters "x" and "y".
//
//
// USAGE: with MATH_INTERFACE; use MATH_INTERFACE;
//
// REFERENCE:
//    Honeywell IFMU project math package (1993)
//
// INITIAL VERSION:
//
//    Suneel Sheikh, NAV/SRC
//    June 7, 1993
//

#include <include/base_types.h>

namespace ECTOS
{

namespace MATH_INTERFACE
{
   // **************************************************************************
   // **************************************************************************
   //                     PACKAGE SPECIFICATION TYPES
   // **************************************************************************
   // **************************************************************************
   // --------------------------------------------------------------------------
   // --------------------------------------------------------------------------
   // Magnitude and Sign Functions
   // --------------------------------------------------------------------------
   // --------------------------------------------------------------------------
   // --------------------------------------------------------------------------
   // --------------------------------------------------------------------------
   // Error value return functions
   // --------------------------------------------------------------------------
   // --------------------------------------------------------------------------
   // LONG_FLOAT_TYPE Function Specifications
   // Fabs(x)
   // returns floating point absolute value
   LONG_FLOAT_TYPE
      Fabs(LONG_FLOAT_TYPE x);
   // Fmod(x,y)
   // returns x modulo y
   LONG_FLOAT_TYPE
      Fmod(LONG_FLOAT_TYPE x,
      LONG_FLOAT_TYPE y);
   // Sign(x)
   // Returns a -1.0 for negative numbers and +1.0 for positive numbers.
   // Assumes that 0.0 is positive.
   LONG_FLOAT_TYPE
      Sign(LONG_FLOAT_TYPE x);
   // Sign is inlined using pragma from INTEGER
   // Floor(x)
   // Returns the floor, or the lower bound of a float value x,.
   LONG_FLOAT_TYPE
      Floor(LONG_FLOAT_TYPE x);
   /* Pragma INLINE(Floor)*/
   // Ceil(x)
   // Returns the ceiling, or the upper bound of a float value x.
   LONG_FLOAT_TYPE
      Ceil(LONG_FLOAT_TYPE x);
   //      /* Pragma INLINE(Ceil)*/
   // Min(x, y)
   // Returns the smaller value of x and y.
   LONG_FLOAT_TYPE
      Min(LONG_FLOAT_TYPE x,
      LONG_FLOAT_TYPE y);
   // Min is inlined using pragma from INTEGER
   // Max(x, y)
   // Returns the larger value of x and y.
   LONG_FLOAT_TYPE
      Max(LONG_FLOAT_TYPE x,
      LONG_FLOAT_TYPE y);
   // Max is inlined using pragma from INTEGER

   // Truncate(x)
   // Truncates the decimal portion of x.
   // Rounds to nearest integer closest to but not greater in magnitude than x.
   // Negative numbers round toward 0.
   // Invariant is x = Truncate(x) + Fractional(x)
   LONG_FLOAT_TYPE
      Truncate(LONG_FLOAT_TYPE x);
   /* Pragma INLINE(Truncate)*/
   // Fractional(x)
   // Returns the factional part of x as a float value.
   LONG_FLOAT_TYPE
      Fractional(LONG_FLOAT_TYPE x);
   /* Pragma INLINE(Fractional)*/
   // Round(x)
   // Returns the result of rounding x to the nearest integer.
   LONG_FLOAT_TYPE
      Round(LONG_FLOAT_TYPE x);
   /* Pragma INLINE(Round)*/
   // INTEGER_TYPE Function Specifications
   // Sign (+ or -) of x
   // Returns a -1 for negative numbers and +1 for positive numbers.
   // Assumes that 0 is positive.
   INTEGER_TYPE
      Sign(INTEGER_TYPE x);
   /* Pragma INLINE(Sign)*/
   // Min(x, y)
   // Returns the smaller value of x and y.
   INTEGER_TYPE
      Min(INTEGER_TYPE x,
      INTEGER_TYPE y);
   /* Pragma INLINE(Min)*/
   // Max(x, y)
   // Returns the larger value of x and y.
   INTEGER_TYPE
      Max(INTEGER_TYPE x,
      INTEGER_TYPE y);
   /* Pragma INLINE(Max)*/

   // --------------------------------------------------------------------------
   // NOTE: The following functions checks and corrects for trunction errors
   //       caused by casting floats to ints.
   // --------------------------------------------------------------------------
   // Ifloor(x)
   // Returns the floor of float x as an int
   INTEGER_TYPE
      Ifloor( LONG_FLOAT_TYPE x );

   // Iceil(x)
   // Returns the ceil of float x as an int
   INTEGER_TYPE
      Iceil( LONG_FLOAT_TYPE x );

   // Iround(x)
   // Returns the float x rounded to the nearest int as an int
   INTEGER_TYPE
      Iround( LONG_FLOAT_TYPE x );
   // --------------------------------------------------------------------------
   // --------------------------------------------------------------------------

   // --------------------------------------------------------------------------
   // --------------------------------------------------------------------------
   // Power and Root Functions
   // --------------------------------------------------------------------------
   // --------------------------------------------------------------------------
   // LONG_FLOAT_TYPE Function Specifications
   // Pow(x,y)
   // Raises x to the power of y.
   LONG_FLOAT_TYPE
      Pow(LONG_FLOAT_TYPE x,
      LONG_FLOAT_TYPE y);
   // Pow is inlined using pragma from INTEGER
   // Sqrt(x)
   // Returns the square root of float value x.
   LONG_FLOAT_TYPE
      Sqrt(LONG_FLOAT_TYPE x);
   // Sqrt is inlined using pragma from INTEGER
   // INTEGER_TYPE Function Specifications
   // Pow(x,y)
   // Raises x to the power of y.
   INTEGER_TYPE
      Pow(INTEGER_TYPE x,
      INTEGER_TYPE y);
   /* Pragma INLINE(Pow)*/
   // Sqrt(x)
   // Returns the square root of integer value x.
   INTEGER_TYPE
      Sqrt(INTEGER_TYPE x);
   /* Pragma INLINE(SQRT)*/

   // --------------------------------------------------------------------------
   // --------------------------------------------------------------------------
   // Exponential and Logarithmic Functions
   // --------------------------------------------------------------------------
   // --------------------------------------------------------------------------
   // LONG_FLOAT_TYPE Function Specifications
   // Exp(x)
   // Returns e, the Euler or transcendental number, raised to
   // the power of x (e**x).
   LONG_FLOAT_TYPE
      Exp(LONG_FLOAT_TYPE x);
   /* Pragma INLINE(Exp)*/
   // Log base e (x)
   LONG_FLOAT_TYPE
      Nat_Log(LONG_FLOAT_TYPE x);
   /* Pragma INLINE(Nat_Log)*/
   LONG_FLOAT_TYPE /* Renames Nat_Log */
      Ln(LONG_FLOAT_TYPE x);
   /* Pragma INLINE(Ln)*/
   // Log base 10 (x)
   LONG_FLOAT_TYPE
      Log10(LONG_FLOAT_TYPE x);
   /* Pragma INLINE(Log10)*/
   // Log base 2 (x)
   LONG_FLOAT_TYPE
      Log2(LONG_FLOAT_TYPE x);
   /* Pragma INLINE(Log2)*/

   // --------------------------------------------------------------------------
   // --------------------------------------------------------------------------
   // Trigonometric Functions
   // (Input parameters must be radians)
   // (Output parameters are unitless ratios)
   // --------------------------------------------------------------------------
   // --------------------------------------------------------------------------
   // LONG_FLOAT_TYPE Function Specifications
   // Sin(x)
   // Sine of float value x.
   LONG_FLOAT_TYPE
      Sin(LONG_FLOAT_TYPE x);
   /* Pragma INLINE(Sin)*/
   // Cos(x)
   // Cosine of float value x.
   LONG_FLOAT_TYPE
      Cos(LONG_FLOAT_TYPE x);
   /* Pragma INLINE(Cos)*/
   // Tan(x)
   // Tangent of float value x.
   LONG_FLOAT_TYPE
      Tan(LONG_FLOAT_TYPE x);
   /* Pragma INLINE(Tan)*/
   // Cot(x)
   // Cotangent of float value x.
   LONG_FLOAT_TYPE
      Cot(LONG_FLOAT_TYPE x);
   /* Pragma INLINE(Cot)*/

   // --------------------------------------------------------------------------
   // --------------------------------------------------------------------------
   // Inverse Trigonometric Functions
   // (Input parameters must be unitless ratios)
   // (Output parameters are in radians)
   // --------------------------------------------------------------------------
   // --------------------------------------------------------------------------
   // LONG_FLOAT_TYPE Function Specifications
   // ArcSin(x)
   // Arcsine of float value x.
   LONG_FLOAT_TYPE
      ArcSin(LONG_FLOAT_TYPE x);
   /* Pragma INLINE(ARCSIN)*/
   // ArcCos(x)
   // Arccosine of float value x.
   LONG_FLOAT_TYPE
      ArcCos(LONG_FLOAT_TYPE x);
   /* Pragma INLINE(ARCCOS)*/
   // ArcTan(x)
   // Arctangent of float value x.
   LONG_FLOAT_TYPE
      ArcTan(LONG_FLOAT_TYPE x);
   /* Pragma INLINE(ArcTan)*/
   // ArcCot(x)
   // Arccotangent of float value x.
   LONG_FLOAT_TYPE
      ArcCot(LONG_FLOAT_TYPE x);
   /* Pragma INLINE(ArcCot)*/
   // ArcTan2(y,x)
   // Arctangent of y / x.
   LONG_FLOAT_TYPE
      ArcTan2(LONG_FLOAT_TYPE y,
      LONG_FLOAT_TYPE x);
   /* Pragma INLINE(ARCTAN2)*/
   // ArcCot2(y,x)
   // Arccotangent of y / x.
   LONG_FLOAT_TYPE
      ArcCot2(LONG_FLOAT_TYPE y,
      LONG_FLOAT_TYPE x);
   /* Pragma INLINE(ARCCOT2)*/

   // --------------------------------------------------------------------------
   // --------------------------------------------------------------------------
   // Hyperbolic Functions
   // --------------------------------------------------------------------------
   // --------------------------------------------------------------------------
   // LONG_FLOAT_TYPE Function Specifications
   // Sinh(x)
   // The hyperbolic sine of float value x.
   LONG_FLOAT_TYPE
      Sinh(LONG_FLOAT_TYPE x);
   /* Pragma INLINE(Sinh)*/
   // Cosh(x)
   // The hyperbolic cosine of float value x.
   LONG_FLOAT_TYPE
      Cosh(LONG_FLOAT_TYPE x);
   /* Pragma INLINE(Cosh)*/
   // Tanh(x)
   // The hyperbolic tangent of float value x.
   LONG_FLOAT_TYPE
      Tanh(LONG_FLOAT_TYPE x);
   /* Pragma INLINE(Tanh)*/
   // Coth(x)
   // The hyperbolic cotangent of float value x.
   LONG_FLOAT_TYPE
      Coth(LONG_FLOAT_TYPE x);
   /* Pragma INLINE(Coth)*/

   // --------------------------------------------------------------------------
   // --------------------------------------------------------------------------
   // Inverse Hyperbolic Functions
   // --------------------------------------------------------------------------
   // --------------------------------------------------------------------------
   // LONG_FLOAT_TYPE Function Specifications
   // ArcSinh(x)
   // The inverse hyperbolic sine of float value x.
   LONG_FLOAT_TYPE
      ArcSinh(LONG_FLOAT_TYPE x);
   /* Pragma INLINE(ArcSinh)*/
   // ArcCosh(x)
   // The inverse hyperbolic cosine of float value x.
   LONG_FLOAT_TYPE
      ArcCosh(LONG_FLOAT_TYPE x);
   /* Pragma INLINE(ArcCosh)*/
   // ArcTanh(x)
   // The inverse hyperbolic tangent of float value x.
   LONG_FLOAT_TYPE
      ArcTanh(LONG_FLOAT_TYPE x);
   /* Pragma INLINE(ArcTanh)*/
   // ArcCoth(x)
   // The inverse hyperbolic cotangent of float value x.
   LONG_FLOAT_TYPE
      ArcCoth(LONG_FLOAT_TYPE x);
   /* Pragma INLINE(ArcCoth)*/

   // --------------------------------------------------------------------------
   // Seed_Random_Number_Generator(UNSIGNED_INTEGER_TYPE seed)
   //
   // Use the inputted number to seed the random number generator
   // --------------------------------------------------------------------------
   void Seed_Random_Number_Generator(UNSIGNED_INTEGER_TYPE seed);

   // --------------------------------------------------------------------------
   // Uniform_Random()
   //
   // Returns a uniformally distributed random number between 0 and 1 (inclusive)
   // --------------------------------------------------------------------------
   LONG_FLOAT_TYPE Uniform_Random(void);

   // --------------------------------------------------------------------------
   // Gauss_Random(LONG_FLOAT_TYPE &rand1, LONG_FLOAT_TYPE &rand2)
   //
   // Returns (via input parameters) two normally distrubed random numbers
   // with 0 mean and a standard deviation of 1
   // --------------------------------------------------------------------------
   void Gauss_Random(LONG_FLOAT_TYPE &rand1, LONG_FLOAT_TYPE &rand2);

   // --------------------------------------------------------------------------
   // RSS(UNSIGNED_INTEGER_TYPE i, ...)
   //
   // Returns the square root of the sum of the squares for a variable number of 
   // inputs.
   // --------------------------------------------------------------------------
   LONG_FLOAT_TYPE RSS(UNSIGNED_INTEGER_TYPE i, ... );

   // --------------------------------------------------------------------------
   // RSS_Array(UNSIGNED_INTEGER_TYPE i, LONG_FLOAT_TYPE* input_array)
   //
   // Returns the square root of the sum of the squares for an array with a  
   // variable length.  Requires an input of the array size.
   // --------------------------------------------------------------------------
   LONG_FLOAT_TYPE RSS_Array(UNSIGNED_INTEGER_TYPE i, LONG_FLOAT_TYPE* input_array);
}

} // namespace ECTOS


#endif
