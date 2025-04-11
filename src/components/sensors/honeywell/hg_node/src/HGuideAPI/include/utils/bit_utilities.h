#ifndef BIT_UTILITIES_package
#define BIT_UTILITIES_package

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

#include <string.h>

#include <include/base_types.h>

namespace ECTOS
{
   namespace BIT_UTILITIES
   {
      /// Returns true if "bitLocation" is 1 in "data", otherwise it returns false.
      /// Bit 0 is the right-most bit in the word
      ///
      inline
      BOOLEAN_TYPE Get_Boolean(UNSIGNED_INTEGER8_TYPE data, INTEGER_TYPE bitLocation)
      {
         UNSIGNED_INTEGER8_TYPE   mask      = 0x01;

         return((data & (mask << bitLocation))!=0);
      }   

      /// Sets "bitLocation" to 1 if data is true, otherwise it sets 
      /// "bitLocation" to 0.
      /// Bit 0 is the right-most bit in the word
      ///
      inline
      void Set_Boolean(UNSIGNED_INTEGER8_TYPE *in_out, BOOLEAN_TYPE data, INTEGER_TYPE bitLocation)
      {
         UNSIGNED_INTEGER8_TYPE   mask      = 0x01;

         mask = mask << bitLocation;
         if (data)
         {
             //set the bit
            *in_out = (*in_out | mask);
         }
         else
         {
               //clear the bit
            *in_out = (*in_out & ~mask);
         }

      }

      /// Sets the specified memory block to the value specified.
      ///
      void Set_To(void *in_out, UNSIGNED_INTEGER8_TYPE   value, INTEGER_TYPE size_in_integer8s);
      
      
      /// Copies the specified memory blocks and returns number of characters copied
      ///
      INTEGER_TYPE Copy_Str(STRING_TYPE out, const STRING_TYPE in, INTEGER_TYPE max_size_in_integer8s);

      /// Takes in an array of bytes (unsigned integer 8's) and outputs a single
      /// variable (via an input parameter) whose size is the same as the total
      /// size of the array. The inputted array must have the LSB at index 0.
      /// **************************************************************************
      /// ********** NOTE: Both "in" and "out" must be UNSIGNED variables **********
      /// **************************************************************************
      ///
      inline
      void Convert_Array_To_Single_Variable(UNSIGNED_INTEGER16_TYPE *out, UNSIGNED_INTEGER8_TYPE *in)
      {
         *out = ((UNSIGNED_INTEGER16_TYPE)in[1] << 8) | 
                 (UNSIGNED_INTEGER16_TYPE)in[0];
      }

      inline
      void Convert_Array_To_Single_Variable(UNSIGNED_INTEGER32_TYPE *out, UNSIGNED_INTEGER8_TYPE *in)
      {
         *out = ((UNSIGNED_INTEGER32_TYPE)in[3] << 24) |
                ((UNSIGNED_INTEGER32_TYPE)in[2] << 16) |
                ((UNSIGNED_INTEGER32_TYPE)in[1] <<  8) |
                 (UNSIGNED_INTEGER32_TYPE)in[0];
      }

      inline
      void Convert_Array_To_Single_Variable(UNSIGNED_INTEGER64_TYPE *out, UNSIGNED_INTEGER8_TYPE *in)
      {
         *out =   ((UNSIGNED_INTEGER64_TYPE)in[7]) << 56 |
                  ((UNSIGNED_INTEGER64_TYPE)in[6]) << 48 |
                  ((UNSIGNED_INTEGER64_TYPE)in[5]) << 40 |
                  ((UNSIGNED_INTEGER64_TYPE)in[4]) << 32 |
                  ((UNSIGNED_INTEGER64_TYPE)in[3]) << 24 |
                  ((UNSIGNED_INTEGER64_TYPE)in[2]) << 16 |
                  ((UNSIGNED_INTEGER64_TYPE)in[1]) <<  8 |
                  ((UNSIGNED_INTEGER64_TYPE)in[0]);
      }

      inline
      void Convert_Array_To_Single_Variable(INTEGER16_TYPE *out, UNSIGNED_INTEGER8_TYPE *in)
      {
         *out = (INTEGER16_TYPE)
                ((UNSIGNED_INTEGER16_TYPE)in[1] << 8) | 
                 (UNSIGNED_INTEGER16_TYPE)in[0];
      }

      inline
      void Convert_Array_To_Single_Variable(INTEGER32_TYPE *out, UNSIGNED_INTEGER8_TYPE *in)
      {
         *out = (INTEGER32_TYPE)
                ((UNSIGNED_INTEGER32_TYPE)in[3] << 24) |
                ((UNSIGNED_INTEGER32_TYPE)in[2] << 16) |
                ((UNSIGNED_INTEGER32_TYPE)in[1] <<  8) |
                 (UNSIGNED_INTEGER32_TYPE)in[0];
      }

      inline
      void Convert_Array_To_Single_Variable(INTEGER64_TYPE *out, UNSIGNED_INTEGER8_TYPE *in)
      {
         *out = (INTEGER64_TYPE)
                ((UNSIGNED_INTEGER64_TYPE)in[7]) << 56 |
                ((UNSIGNED_INTEGER64_TYPE)in[6]) << 48 |
                ((UNSIGNED_INTEGER64_TYPE)in[5]) << 40 |
                ((UNSIGNED_INTEGER64_TYPE)in[4]) << 32 |
                ((UNSIGNED_INTEGER64_TYPE)in[3]) << 24 |
                ((UNSIGNED_INTEGER64_TYPE)in[2]) << 16 |
                ((UNSIGNED_INTEGER64_TYPE)in[1]) <<  8 |
                ((UNSIGNED_INTEGER64_TYPE)in[0]);
      }

/*
      template <class OUTPUT>
      void Convert_Array_To_Single_Variable(OUTPUT *out, UNSIGNED_INTEGER8_TYPE *in)
      {
         INTEGER_TYPE   i;
         INTEGER_TYPE   loops;

         *out = 0;

         loops = sizeof(*out);

         for (i=0; i<loops; i++)
         {
            *out = *out | (((OUTPUT)in[i]) << (8*i));
         }
      }
*/

      /// This routine packs the input "bits" into the input "in" and returns the result.
      /// The (stop-start) lower bits are used from "bits".  (Any higher order bits are ignored.)
      /// The lower order bits are put into the "start" throught "stop" bits of "in".
      /// "status_ok" contains a flag indicating success or failure \n
      /// Bit numbering starts at 0.\n
      /// example \n
      /// in = 0x00000000 \n
      /// bits = 0x07 (3 lowest bits set) \n
      /// start = 0  stop = 2  result = 0x0007 \n
      /// start = 4  stop = 6  result = 0x0070 \n
      /// start = 5  stop = 7  result = 0x00E0 \n
      /// start = 4  stop = 4  result = 0x0010 \n
      /// start = 4  stop = 5  result = 0x0030 \n
      /// start = 29 stop = 31 result = 0xE0000000 \n
      /// Bit 0 is the right-most bit in the word \n
      ///
      template <class TYPE1,class TYPE2>
      TYPE1 Pack(TYPE1 in, INTEGER_TYPE start, INTEGER_TYPE stop, TYPE2 bits, BOOLEAN_TYPE &status_ok)
      {
         INTEGER32_TYPE   last_bit;
         TYPE1            working1 = 0;
         TYPE1            working2 = 0;
         TYPE1            mask = 0;
         INTEGER32_TYPE   i;
         INTEGER32_TYPE   right_shift;
         INTEGER32_TYPE   width;

         // Find the position of the msb
         last_bit = 8*sizeof(TYPE1) - 1;

         // Make sure "start" isn't too small
         if (start < 0)
         {
            status_ok = false;
            return 0;
         }

         // Make sure "stop" isn't too big
         if (stop > last_bit)
         {
            status_ok = false;
            return 0;
         }

         // Make sure "start" isn't bigger than "stop"
         if (start > stop)
         {
            status_ok = false;
            return 0;
         }

         // Make sure that TYPE2 is not bigger than TYPE1
         if (sizeof(TYPE2) > sizeof(TYPE1))
         {
            status_ok = false;
            return 0;
         }

         // first clear the appropriate bits from in by grabbing the upper and lower
         // portions to be saved

         // build the mask to get the lower part of in
         mask = 0;
         for(i=0;i<start;i++){
            mask = (mask << 1) | 0x01;
         }
         working1 = in & mask;

         // get the upper part by shifting right and then back
         // if the upper part is empty because we are setting the highest order bit then just set it to zero.
         right_shift = (stop + 1);

         if (right_shift == 8*sizeof(TYPE1)){
            working2 = 0;
         }else{
            working2 = in >> right_shift;
            working2 = working2 << right_shift;
         }

         working1 = working1 | working2;

         // clear the upper (un-used) bits from bits with a mask
         width = stop-start+1;

         // build the mask for bits
         mask = 0;
         for(i=0;i<width;i++){
            mask = (mask << 1) | 0x01;
         }
         working2 = bits & mask;

         working2 = working2 << start;

         working1 = working1 | working2;

         status_ok = true;

         return working1;
      }

      /// This routine packs the input "data" bool into the bit location
      /// specified by "bit_num" in "in" and returns the result.
      /// "status_ok" contains a flag indicating success or failure
      /// Bit 0 is the right-most bit in the word
      ///
      template <class TYPE>
      TYPE PackBool(TYPE in, INTEGER_TYPE bit_num, BOOLEAN_TYPE data, BOOLEAN_TYPE &status_ok)
      {
         if (data == true)
         {
            return Pack(in, bit_num, bit_num, (TYPE) 1, status_ok);
         }
         else
         {
            return Pack(in, bit_num, bit_num, (TYPE) 0, status_ok);
         }
      }

      /// Returns the bits from "in" between (inclusive) "start" and "stop"
      /// "status_ok" contains a flag indicating success or failure
      /// Bit 0 is the right-most bit in the word
      ///
      inline
      UNSIGNED_INTEGER8_TYPE UnPack (UNSIGNED_INTEGER8_TYPE in, INTEGER_TYPE start, INTEGER_TYPE stop, BOOLEAN_TYPE &status_ok)
      {
         static UNSIGNED_INTEGER8_TYPE mask_for_length [8] = {0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F, 0xFF};
         INTEGER_TYPE length_index = stop - start; // length is +1 of this, but mask array starts at 0 rather than 1.
         UNSIGNED_INTEGER8_TYPE mask_for_start_to_stop = mask_for_length [length_index] << start;
         status_ok = true; //unneeded parameter must be set for compatibility with previous implementation
         return (((in & mask_for_start_to_stop) >> start) & 0xFF);
         
      }

      /// Returns the bits from "in" between (inclusive) "start" and "stop"
      /// "status_ok" contains a flag indicating success or failure
      /// Bit 0 is the right-most bit in the word
      ///
      inline
      UNSIGNED_INTEGER16_TYPE UnPack (UNSIGNED_INTEGER16_TYPE in, INTEGER_TYPE start, INTEGER_TYPE stop, BOOLEAN_TYPE &status_ok)
      {
         static UNSIGNED_INTEGER16_TYPE mask_for_length [16] = {0x0001, 0x0003, 0x0007, 0x000F, 0x001F, 0x003F, 0x007F, 0x00FF,
                                                               0x01FF, 0x03FF, 0x07FF, 0x0FFF, 0x1FFF, 0x3FFF, 0x7FFF, 0xFFFF};
         INTEGER_TYPE length_index = stop - start; // length is +1 of this, but mask array starts at 0 rather than 1.
         UNSIGNED_INTEGER8_TYPE mask_for_start_to_stop = mask_for_length [length_index] << start;
         status_ok = true; //unneeded parameter must be set for compatibility with previous implementation
         return (((in & mask_for_start_to_stop) >> start) & 0xFFFF);
         
      }

      /// Returns the bits from "in" between (inclusive) "start" and "stop"
      /// "status_ok" contains a flag indicating success or failure
      /// Bit 0 is the right-most bit in the word
      ///
      inline
      UNSIGNED_INTEGER32_TYPE UnPack (UNSIGNED_INTEGER32_TYPE in, INTEGER_TYPE start, INTEGER_TYPE stop, BOOLEAN_TYPE &status_ok)
      {
         static UNSIGNED_INTEGER32_TYPE mask_for_length [32] = 
             {
              0x00000001, 0x00000003, 0x00000007, 0x0000000F, 
              0x0000001F, 0x0000003F, 0x0000007F, 0x000000FF,
              0x000001FF, 0x000003FF, 0x000007FF, 0x00000FFF,
              0x00001FFF, 0x00003FFF, 0x00007FFF, 0x0000FFFF,
              0x0001FFFF, 0x0003FFFF, 0x0007FFFF, 0x000FFFFF,
              0x001FFFFF, 0x003FFFFF, 0x007FFFFF, 0x00FFFFFF,
              0x01FFFFFF, 0x03FFFFFF, 0x07FFFFFF, 0x0FFFFFFF,
              0x1FFFFFFF, 0x3FFFFFFF, 0x7FFFFFFF, 0xFFFFFFFF};
         INTEGER_TYPE length_index = stop - start; // length is +1 of this, but mask array starts at 0 rather than 1.
         UNSIGNED_INTEGER32_TYPE mask_for_start_to_stop = mask_for_length [length_index] << start;
         status_ok = true; //unneeded parameter must be set for compatibility with previous implementation
         return (((in & mask_for_start_to_stop) >> start)&0xFFFFFFFF);
         
      }


      template <class TYPE>
      TYPE UnPack(TYPE in, INTEGER_TYPE start, INTEGER_TYPE stop, BOOLEAN_TYPE &status_ok)
      {
         INTEGER32_TYPE   last_bit;
         TYPE            mask;

         // Find the position of the msb
         last_bit = 8*sizeof(TYPE) - 1;

         // Make sure "start" isn't too small
         if (start < 0)
         {
            status_ok = false;
            return 0;
         }

         // Make sure "stop" isn't too big
         if (stop > last_bit)
         {
            status_ok = false;
            return 0;
         }

         // Make sure "start" isn't bigger than "stop"
         if (start > stop)
         {
            status_ok = false;
            return 0;
         }

         // Make sure all of "in" isn't wanted
         if ((last_bit-stop+start) == 0)
         {
            status_ok = true;
            return in;
         }

         // Get rid of msb's that aren't wanted
         in<<=last_bit-stop;

         // Shift right one and then make high bit zero
         // If this isn't done and the high bit is a 1,
         // each new msb will be a 1
         in>>=1;
         mask = 1;
         mask<<=last_bit;
         mask = ~mask;
         in = in & mask;

         // Get rid of the lsb's that aren't wanted.
         in>>=last_bit-stop+start-1;

         status_ok = true;
         return in;
      }


      /// Returns true if the bit in "in" at bit location "bit_num" is a 1.
      /// Returns false if the bit in "in" at bit location "bit_num" is a 0.
      /// "status_ok" contains a flag indicating success or failure
      /// Bit 0 is the right-most bit in the word
      ///
      template <class TYPE>
      BOOLEAN_TYPE UnPackBool(TYPE in, INTEGER_TYPE bit_num, BOOLEAN_TYPE &status_ok)
      {

          const TYPE bit_mask = 1;
          status_ok = true; //unneeded parameter must be set for compatibility with previous implementation

          return (BOOLEAN_TYPE) ((in & (bit_mask << bit_num)) != 0);

      }

      // Returns the LSB of "in"
      // "status_ok" contains a flag indicating success or failure
      // Bit 0 is the right-most bit in the word
      // efficient code for most uses of this functionality

      /// Returns the LSB of "in"
      /// "status_ok" contains a flag indicating success or failure
      /// Bit 0 is the right-most bit in the word
      ///
      inline
      UNSIGNED_INTEGER8_TYPE UnPackLSB(UNSIGNED_INTEGER16_TYPE in, BOOLEAN_TYPE &status_ok)
      {
         status_ok=true;
         return ( (UNSIGNED_INTEGER8_TYPE)(in & 0x00FF) );
      }

      /// Returns the LSB of "in"
      /// "status_ok" contains a flag indicating success or failure
      /// Bit 0 is the right-most bit in the word
      ///
      inline
      UNSIGNED_INTEGER8_TYPE UnPackLSB(INTEGER16_TYPE in, BOOLEAN_TYPE &status_ok)
      {
         status_ok=true;
         return ( (UNSIGNED_INTEGER8_TYPE)(in & 0x00FF) );
      }

      /// Returns the LSB of "in"
      /// "status_ok" contains a flag indicating success or failure
      /// Bit 0 is the right-most bit in the word
      ///
      inline
      UNSIGNED_INTEGER8_TYPE UnPackLSB(UNSIGNED_INTEGER32_TYPE in, BOOLEAN_TYPE &status_ok)
      {
         status_ok=true;
         return ( (UNSIGNED_INTEGER8_TYPE)(in & 0x000000FF) );
      }

      /// Returns the LSB of "in"
      /// "status_ok" contains a flag indicating success or failure
      /// Bit 0 is the right-most bit in the word
      ///
      inline
      UNSIGNED_INTEGER8_TYPE UnPackLSB(INTEGER32_TYPE in, BOOLEAN_TYPE &status_ok)
      {
         status_ok=true;
         return ( (UNSIGNED_INTEGER8_TYPE)(in & 0x000000FF) );
      }
      
       //for any other types that might come in to here...
/*
      template <class TYPE>
      UNSIGNED_INTEGER8_TYPE UnPackLSB(TYPE in, BOOLEAN_TYPE &status_ok)
      {
         UNSIGNED_INTEGER8_TYPE temp;

         temp = (UNSIGNED_INTEGER8_TYPE) UnPack(in, 0, 7, status_ok);

         return (temp);
      }
*/

      // Returns the 2nd LSB of "in"
      // "status_ok" contains a flag indicating success or failure
      // Bit 0 is the right-most bit in the word

      //efficient code for most uses of this functionality


      /// Returns the 2nd LSB of "in"
      /// "status_ok" contains a flag indicating success or failure
      /// Bit 0 is the right-most bit in the word
      ///
      inline
      UNSIGNED_INTEGER8_TYPE UnPack2ndLSB(UNSIGNED_INTEGER16_TYPE in, BOOLEAN_TYPE &status_ok)
      {
         // 2nd LSB on a 16bit integer makes little sense
         status_ok=true;
         return ( (UNSIGNED_INTEGER8_TYPE)((in & 0xFF00) >>8) );
      }

      /// Returns the 2nd LSB of "in"
      /// "status_ok" contains a flag indicating success or failure
      /// Bit 0 is the right-most bit in the word
      ///
      inline
      UNSIGNED_INTEGER8_TYPE UnPack2ndLSB(INTEGER16_TYPE in, BOOLEAN_TYPE &status_ok)
      {
         status_ok=true;
         return ( (UNSIGNED_INTEGER8_TYPE)((in & 0xFF00) >>8) );
      }

      /// Returns the 2nd LSB of "in"
      /// "status_ok" contains a flag indicating success or failure
      /// Bit 0 is the right-most bit in the word
      ///
      inline
      UNSIGNED_INTEGER8_TYPE UnPack2ndLSB(UNSIGNED_INTEGER32_TYPE in, BOOLEAN_TYPE &status_ok)
      {
         status_ok=true;
         return ( (UNSIGNED_INTEGER8_TYPE)((in & 0x0000FF00) >>8) );
      }

      /// Returns the 2nd LSB of "in"
      /// "status_ok" contains a flag indicating success or failure
      /// Bit 0 is the right-most bit in the word
      ///
      inline
      UNSIGNED_INTEGER8_TYPE UnPack2ndLSB(INTEGER32_TYPE in, BOOLEAN_TYPE &status_ok)
      {
         status_ok=true;
         return ( (UNSIGNED_INTEGER8_TYPE)((in & 0x0000FF00) >>8) );
      }
      
       //for any other types that might come in to here...
/*
      template <class TYPE>
      UNSIGNED_INTEGER8_TYPE UnPack2ndLSB(TYPE in, BOOLEAN_TYPE &status_ok)
      {
         UNSIGNED_INTEGER8_TYPE temp;

         // Make sure in is at least 16 bits
         if (sizeof(in) < 2)
         {
            status_ok = false;
            return 0;
         }

         temp = (UNSIGNED_INTEGER8_TYPE) UnPack(in, 8, 15, status_ok);

         return (temp);
      }
*/

      /// Returns the MSB of "in"
      /// "status_ok" contains a flag indicating success or failure
      /// Bit 0 is the right-most bit in the word

      //efficient code for most uses of this functionality
      
      
      /// Returns the MSB of "in"
      /// "status_ok" contains a flag indicating success or failure
      /// Bit 0 is the right-most bit in the word
      ///
      inline
      UNSIGNED_INTEGER8_TYPE UnPack2ndMSB(UNSIGNED_INTEGER16_TYPE in, BOOLEAN_TYPE &status_ok)
      {
         //2nd MSB on a 16bit integer makes little sense
         status_ok=true;
         return ( (UNSIGNED_INTEGER8_TYPE)(in & 0x00FF) );
      }

      /// Returns the MSB of "in"
      /// "status_ok" contains a flag indicating success or failure
      /// Bit 0 is the right-most bit in the word
      ///
      inline
      UNSIGNED_INTEGER8_TYPE UnPack2ndMSB(INTEGER16_TYPE in, BOOLEAN_TYPE &status_ok)
      {
         status_ok=true;
         return ( (UNSIGNED_INTEGER8_TYPE)(in & 0x00FF) );
      }

      /// Returns the MSB of "in"
      /// "status_ok" contains a flag indicating success or failure
      /// Bit 0 is the right-most bit in the word
      ///
      inline
      UNSIGNED_INTEGER8_TYPE UnPack2ndMSB(UNSIGNED_INTEGER32_TYPE in, BOOLEAN_TYPE &status_ok)
      {
         status_ok=true;
         return ( (UNSIGNED_INTEGER8_TYPE)((in & 0x00FF0000) >>16) );
      }

      /// Returns the MSB of "in"
      /// "status_ok" contains a flag indicating success or failure
      /// Bit 0 is the right-most bit in the word
      ///
      inline 
      UNSIGNED_INTEGER8_TYPE UnPack2ndMSB(INTEGER32_TYPE in, BOOLEAN_TYPE &status_ok)
      {
         status_ok=true;
         return ( (UNSIGNED_INTEGER8_TYPE)((in & 0x00FF0000) >>16) );
      }

       //for any other types that might come in to here...
/*
      template <class TYPE>
      UNSIGNED_INTEGER8_TYPE UnPack2ndMSB(TYPE in, BOOLEAN_TYPE &status_ok)
      {
         UNSIGNED_INTEGER8_TYPE   temp;
         INTEGER_TYPE            last_bit;

         // Make sure in is at least 16 bits
         if (sizeof(in) < 2)
         {
            status_ok = false;
            return 0;
         }

         last_bit = (sizeof(in) * 8) - 1;

         temp = (UNSIGNED_INTEGER8_TYPE) UnPack(in, last_bit-15, last_bit-8, status_ok);

         return (temp);
      }
*/

      // Returns the MSB of "in"
      // "status_ok" contains a flag indicating success or failure
      // Bit 0 is the right-most bit in the word

      //efficient code for most uses of this functionality

      /// Returns the MSB of "in"
      /// "status_ok" contains a flag indicating success or failure
      /// Bit 0 is the right-most bit in the word
      ///
      inline
      UNSIGNED_INTEGER8_TYPE UnPackMSB(UNSIGNED_INTEGER16_TYPE in, BOOLEAN_TYPE &status_ok)
      {
         status_ok=true;
         return ( (UNSIGNED_INTEGER8_TYPE)((in & 0xFF00) >>8) );
      }

      /// Returns the MSB of "in"
      /// "status_ok" contains a flag indicating success or failure
      /// Bit 0 is the right-most bit in the word
      ///
      inline
      UNSIGNED_INTEGER8_TYPE UnPackMSB(INTEGER16_TYPE in, BOOLEAN_TYPE &status_ok)
      {
         status_ok=true;
         return ( (UNSIGNED_INTEGER8_TYPE)((in & 0xFF00) >>8) );
      }

      /// Returns the MSB of "in"
      /// "status_ok" contains a flag indicating success or failure
      /// Bit 0 is the right-most bit in the word
      ///
      inline
      UNSIGNED_INTEGER8_TYPE UnPackMSB(UNSIGNED_INTEGER32_TYPE in, BOOLEAN_TYPE &status_ok)
      {
         status_ok=true;
         return ( (UNSIGNED_INTEGER8_TYPE)((in & 0xFF000000) >>24) );
      }

      /// Returns the MSB of "in"
      /// "status_ok" contains a flag indicating success or failure
      /// Bit 0 is the right-most bit in the word
      ///
      inline
      UNSIGNED_INTEGER8_TYPE UnPackMSB(INTEGER32_TYPE in, BOOLEAN_TYPE &status_ok)
      {
         status_ok=true;
         return ( (UNSIGNED_INTEGER8_TYPE)((in & 0xFF000000) >>24) );
      }

      //for any other types that might come in to here...
/*
      template <class TYPE>
      UNSIGNED_INTEGER8_TYPE UnPackMSB(TYPE in, BOOLEAN_TYPE &status_ok)
      {
         UNSIGNED_INTEGER8_TYPE   temp;
         INTEGER_TYPE            last_bit;

         last_bit = (sizeof(in) * 8) - 1;

         temp = (UNSIGNED_INTEGER8_TYPE) UnPack(in, last_bit-7, last_bit, status_ok);

         return (temp);
      }
*/
   } // namespace BIT_UTILITIES
} // namespace ECTOS

#endif
