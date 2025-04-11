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

#ifndef BASE_TYPES_package
#define BASE_TYPES_package

#include <limits.h>
#include <float.h>

namespace ECTOS
{

// **************************************************************************
// **************************************************************************
//             Declare Major General Purpose Numeric Data Types
//
//   This section declares the major numeric data types for general purpose
//   usage.  They are designed to NOT be tied specifically to a certain bit
//   length, since the length may change for different target processors and
//   compilers.  These should be used for all floating point and integer
//   math operations that do not require a specific bit length.
// **************************************************************************
// **************************************************************************

// Normally, NULL is defined by a standard library. This definition ensures NULL is defined.
// The __cplusplus symbol is defined when the compiler accepts C++

/* Define NULL pointer value */
#ifndef NULL
#ifdef __cplusplus
#define NULL    0
#else
#define NULL    ((void *)0)
#endif
#endif

// INT_TYPE is used for the generic "int"
      typedef int INT_TYPE;

// INTEGER_TYPE defines the standard integer type for this compiler.
// In this case, it is a derived 32-bit integer type having the
// range -2,147,483,648 to +2,147,483,647. (same as INTEGER32)
      typedef long   INTEGER_TYPE;
      const long MAX_INTEGER_TYPE = LONG_MAX;
      const long MIN_INTEGER_TYPE = LONG_MIN;

// UNSIGNED_INTEGER_TYPE defines the standard unsigned integer type for
// this compiler.  In this case, it is a derived 32-bit integer type having
// the range 0 to 4294967295. (same as UNSIGNED_INTEGER32)
      //(MANUALLY_FIX)
      typedef  unsigned long UNSIGNED_INTEGER_TYPE;
      const unsigned long MAX_UNSIGNED_INTEGER_TYPE = ULONG_MAX;
      const unsigned long MIN_UNSIGNED_INTEGER_TYPE = 0;

// LONG_FLOAT_TYPE defines the standard floating point type.  In this case,
// it is a derived 64-bit floating-point type. (same as FLOAT64)
#ifdef USE_ONLY_SINGLE_PRECISION_FLOAT
      typedef float    LONG_FLOAT_TYPE;
      const double MAX_LONG_FLOAT_TYPE = FLT_MAX;
      const double MIN_LONG_FLOAT_TYPE = FLT_MIN;
#else
      typedef double    LONG_FLOAT_TYPE;
      const double MAX_LONG_FLOAT_TYPE = DBL_MAX;
      const double MIN_LONG_FLOAT_TYPE = DBL_MIN;
#endif


// **************************************************************************
// **************************************************************************
//                  Declare Specific Data Type Lengths
//
//   This section declares data types which are tied specifically to
//   a certain word length.  These should be used when an exact number of
//   bits is needed, as is typically the case for input/output.
// **************************************************************************
// **************************************************************************

// --------------------------------------------------------------------------
// Define Integer Types
// --------------------------------------------------------------------------

// INTEGER8_TYPE defines a derived 8-bit integer type having the
// range -128 to +127.
      typedef char   INTEGER8_TYPE;
      const long MAX_INTEGER8_TYPE = CHAR_MAX;
      const long MIN_INTEGER8_TYPE = CHAR_MIN;

// INTEGER16_TYPE defines a derived 16-bit integer type having the
// range -32,768 to +32,767.
      typedef short  INTEGER16_TYPE;
      const long MAX_INTEGER16_TYPE = SHRT_MAX;
      const long MIN_INTEGER16_TYPE = SHRT_MIN;

// INTEGER32_TYPE defines a derived 32-bit integer type having the
// range -2,147,483,648 to +2,147,483,647.
      typedef long   INTEGER32_TYPE;
      const long MAX_INTEGER32_TYPE = LONG_MAX;
      const long MIN_INTEGER32_TYPE = LONG_MIN;

// INTEGER64_TYPE defines a derived 64-bit integer type having the
// range -??? to +???.
   // it has been created to assist in making conversions between 64 bit DEC floating point numbers and 64 bit IEEE numbers.
      typedef long long INTEGER64_TYPE;

// --------------------------------------------------------------------------
// Define Unsigned Integer Types
// --------------------------------------------------------------------------

// UNSIGNED_INTEGER8_TYPE defines a derived 8-bit integer type having the
// range 0 to 255.
      //(MANUALLY_FIXED WS 5/8/00)
      typedef  unsigned char UNSIGNED_INTEGER8_TYPE;
      const long MAX_UNSIGNED_INTEGER8_TYPE = CHAR_MAX;
      const long MIN_UNSIGNED_INTEGER8_TYPE = 0;

// UNSIGNED_INTEGER16_TYPE defines a derived 16-bit integer type having the
// range 0 to 65535.
      //(MANUALLY_FIXED WS 5/8/00)
      typedef  unsigned short UNSIGNED_INTEGER16_TYPE;
      const long MAX_UNSIGNED_INTEGER16_TYPE = USHRT_MAX;
      const long MIN_UNSIGNED_INTEGER16_TYPE = 0;

// UNSIGNED_INTEGER32_TYPE defines a derived 32-bit integer type having the
// range 0 to 4294967295.
      //(MANUALLY_FIXED WS 5/8/00)
      typedef  unsigned long UNSIGNED_INTEGER32_TYPE;
      const unsigned long MAX_UNSIGNED_INTEGER32_TYPE = ULONG_MAX;
      const unsigned long MIN_UNSIGNED_INTEGER32_TYPE = 0;

// UNSIGNED_INTEGER64_TYPE defines a derived 64-bit integer type having the
// range 0 to ?????.
      typedef unsigned long long       UNSIGNED_INTEGER64_TYPE;

// --------------------------------------------------------------------------
// Define Float Types
// --------------------------------------------------------------------------

// FLOAT32_TYPE defines a derived 32-bit floating-point type.
      typedef float  FLOAT32_TYPE;
      const double MAX_FLOAT32_TYPE = FLT_MAX;
      const double MIN_FLOAT32_TYPE = FLT_MIN;

// FLOAT64_TYPE defines a derived 64-bit floating-point type.
#ifdef USE_ONLY_SINGLE_PRECISION_FLOAT
      typedef float    FLOAT64_TYPE;
      const double MAX_FLOAT64_TYPE = FLT_MAX;
      const double MIN_FLOAT64_TYPE = FLT_MIN;
#else
      typedef double    FLOAT64_TYPE;
      const double MAX_FLOAT64_TYPE = DBL_MAX;
      const double MIN_FLOAT64_TYPE = DBL_MAX;
#endif


// **************************************************************************
// **************************************************************************
//                  Declare Other Data Types
//
//   This section declares other data types, including those derived from
//   the already defined types.
// **************************************************************************
// **************************************************************************

// --------------------------------------------------------------------------
// Define Boolean Types
// --------------------------------------------------------------------------
      typedef bool   BOOLEAN_TYPE;

// --------------------------------------------------------------------------
// Define Character and String Types
// --------------------------------------------------------------------------

// CHARACTER_TYPE defines character data.
      typedef char CHARACTER_TYPE;

// STRING_TYPE defines an unconstrained array of characters.
      typedef char *STRING_TYPE;

// STRING_TYPE defines const ptr to an unconstrained array of characters.
      typedef const char *CONST_STRING_TYPE;

// STRING_TYPE defines const ptr to an unconstrained array of characters.
      typedef const char * const CONST_STRING_CONST_TYPE;

// --------------------------------------------------------------------------
// Define types for accessing hardware addresses
// --------------------------------------------------------------------------
      typedef long  *ADDRESS_TYPE;
      typedef short *SHORT_ADDRESS_TYPE;


// --------------------------------------------------------------------------
// Create Packed Bit Types
//
// The following types are for bit packed words. Use Ada one dimensional
// array operations (AND, OR, XOR, NOT) for manipulation of these types.
//
// Bit numbering is right-to-left, i.e. bit 0 is LSB.
// --------------------------------------------------------------------------

       typedef  INTEGER8_TYPE PACKED_FOUR_INTEGER8_TYPE[4];
//       /* Pragma PACK(PACKED_FOUR_INTEGER8_TYPE)*/
//
       typedef  INTEGER8_TYPE PACKED_EIGHT_INTEGER8_TYPE[8];
//       /* Pragma PACK(PACKED_EIGHT_INTEGER8_TYPE)*/
//
       typedef  INTEGER32_TYPE PACKED_TWO_INTEGER32_TYPE[2];
//       /* Pragma PACK(PACKED_TWO_INTEGER32_TYPE)*/
//
// // --------------------------------------------------------------------------
// // Create Integer Record Types
// //
// // "ms" refers to the "most significant" bits.
// // "ls" refers to the "least significant" bits.
// // --------------------------------------------------------------------------
//
// Integer8 record types
       typedef struct t_TWO_INTEGER8_RECORD_TYPE
          {
             INTEGER8_TYPE    ls_8_bits;
             INTEGER8_TYPE    ms_8_bits;
          } TWO_INTEGER8_RECORD_TYPE;

//       // FOR ... USE clause for TWO_INTEGER8_RECORD_TYPE ignored.
//
       typedef struct t_FOUR_INTEGER8_RECORD_TYPE
          {
             INTEGER8_TYPE    ls_8_bits;
             INTEGER8_TYPE    third_ms_8_bits;
             INTEGER8_TYPE    second_ms_8_bits;
             INTEGER8_TYPE    ms_8_bits;
          } FOUR_INTEGER8_RECORD_TYPE;

//       // FOR ... USE clause for FOUR_INTEGER8_RECORD_TYPE ignored.
//
// // Integer16 record types
       typedef struct t_TWO_INTEGER16_RECORD_TYPE
          {
             INTEGER16_TYPE   ls_16_bits;
             INTEGER16_TYPE   ms_16_bits;
          } TWO_INTEGER16_RECORD_TYPE;

//       // FOR ... USE clause for TWO_INTEGER16_RECORD_TYPE ignored.
//

#ifdef __cplusplus
       typedef struct t_CONSTRUCTOR
       {
          //
          // This enumeration type is used to create class constructors that
          // deliberately leave a large data structure uninitialized. Objects
          // instantiated by utilizing this method often have large memory
          // regions contained within the class that would be inefficient to
          // initialize on each construction in every case. This construct
          // provides a way to initialize these specific types of classes
          // without initializing the instance memory. This is useful when
          // an object assumes a known state just after construction and
          // can be utilized to avoid unnecessary bus transactions.
          //
          typedef enum t_TRAITS{ UNDEFINED_INSTANCE } TRAITS;
       } CONSTRUCTOR;
#endif

} // namespace ECTOS

#endif



