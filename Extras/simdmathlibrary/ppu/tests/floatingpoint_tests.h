/* Header file for common parts of the testsuite
   Copyright (C) 2006, 2007 Sony Computer Entertainment Inc.
   All rights reserved.

   Redistribution and use in source and binary forms,
   with or without modification, are permitted provided that the
   following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Sony Computer Entertainment Inc nor the names
      of its contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef _FLOATINGPOINT_TESTS_H_
#define _FLOATINGPOINT_TESTS_H_

#if defined(__PPC__)
  #include <altivec.h>
  #define vec_uchar16             vector unsigned char
  #define vec_char16              vector   signed char
  #define vec_ushort8             vector unsigned short
  #define vec_short8              vector   signed short
  #define vec_uint4               vector unsigned int
  #define vec_int4                vector   signed int
  #define vec_ullong2             vector unsigned long long
  #define vec_llong2              vector   signed long long
  #define vec_float4              vector          float
  #define vec_double2             vector          double
#else
  #if __SPU__
    #include <spu_intrinsics.h>
  #endif
#endif

// To avoid type punning warnings (for printing in hex notation, doing bit-diff etc)
typedef union {
  double d;
  unsigned char uc[8];
  unsigned int ui[2];
  unsigned long long int ull;
} sce_math_alt_double;

typedef union {
  float f;
  unsigned char uc[4];
  unsigned int ui;
} sce_math_alt_float;

#if (__PPC__ || __SPU__)
typedef union {
  vec_int4 vsi;
  int si[4];
} sce_math_alt_vec_int4;

typedef union {
  vec_uint4 vui;
  int ui[4];
} sce_math_alt_vec_uint4;

typedef union {
  vec_float4 vf;
  float sf[4];
  unsigned int ui[4];
} sce_math_alt_vec_float4;
#endif
#if __SPU__
  typedef union {
    double sd[2];
    vec_double2 vd;
    unsigned long long int ui[2];
  } sce_math_alt_vec_double2;
#endif

#if __PPC__
  inline vec_int4 bitdiff4(vec_float4 ref, vec_float4 vals) {
    vec_int4 refi  = (vec_int4)ref;
    vec_int4 valsi = (vec_int4)vals;
    vec_int4 diff  = vec_sub(refi, valsi);
    vec_int4 negdiff = vec_sub(((vec_int4){0,0,0,0}), diff);
    
    return vec_sel(negdiff, diff, vec_cmpgt(diff, ((vec_int4){0,0,0,0}) ));
  }  
  inline int bitdiff(float ref, float val) {
    sce_math_alt_float aref, aval;
    aref.f = ref;
    aval.f = val;
    int diff = aref.ui - aval.ui;
    return (diff>0)?diff:-diff;
  }
  inline vec_int4 bitmatch4(vec_float4 ref, vec_float4 vals) {
    vec_int4 refi  = (vec_int4)ref;
    vec_int4 valsi = (vec_int4)vals;
    vec_int4 diff  = vec_sub(refi, valsi);
    vec_int4 negdiff = vec_sub(((vec_int4){0,0,0,0}), diff);
    
    diff = vec_sel(negdiff, diff, vec_cmpgt(diff, ((vec_int4){0,0,0,0}) ));
    vec_float4 logdiff = vec_loge(vec_ctf(diff,0));
    return vec_sub(((vec_int4){32,32,32,32}), vec_cts(vec_ceil(logdiff),0));
  }
  inline int bitmatch(float ref, float val) {
    sce_math_alt_vec_float4 aref, aval;
    sce_math_alt_vec_int4 adiff;
    aref.sf[0] = ref;
    aval.sf[0] = val;
    adiff.vsi = bitmatch4(aref.vf, aval.vf);
    return adiff.si[0];
  }
  inline float extractFloat(vec_float4 vf, int index)
  {
      sce_math_alt_vec_float4 vec;
      vec.vf = vf;
      return vec.sf[index];
  }
  inline int extractInt(vec_int4 vi, int index)
  {
      sce_math_alt_vec_int4 vec;
      vec.vsi = vi;
      return vec.si[index];
  }
  inline int extractUInt(vec_uint4 vi, int index)
  {
      sce_math_alt_vec_uint4 vec;
      vec.vui = vi;
      return vec.ui[index];
  }   
#else
  #if __SPU__
    inline vec_int4 bitdiff4(vec_float4 ref, vec_float4 vals) {
      vec_int4 refi  = (vec_int4)ref;
      vec_int4 valsi = (vec_int4)vals;
      vec_int4 diff  = spu_sub(refi, valsi);
      vec_int4 negdiff = spu_sub(spu_splats((int)0), diff);
      
      return spu_sel(negdiff, diff, (vec_uchar16)spu_cmpgt(diff, 0));
    }  
    inline int bitdiff(float ref, float val) {
      return spu_extract(bitdiff4(spu_promote(ref,0), spu_promote(val,0)), 0);
    }
    inline vec_int4 bitmatch4(vec_float4 ref, vec_float4 vals) {
      vec_int4 refi  = (vec_int4)ref;
      vec_int4 valsi = (vec_int4)vals;
      vec_int4 diff  = spu_sub(refi, valsi);
      vec_int4 negdiff = spu_sub(spu_splats((int)0), diff);
      
      return (vec_int4)spu_cntlz(spu_sel(negdiff, diff, (vec_uchar16)spu_cmpgt(diff, 0)));
    } 
    inline int bitmatch(float ref, float val) {
      return spu_extract(bitmatch4(spu_promote(ref,0), spu_promote(val,0)), 0);
    }

  #else
    inline int bitdiff(sce_math_alt_float ref, sce_math_alt_float val) {
      int diff = ref.ui - val.ui;
      return((diff>0)?diff:-diff);
    }
    inline int bitmatch(sce_math_alt_float ref, sce_math_alt_float val) {
      int diff, i;
      unsigned int udiff;
      diff = ref.ui - val.ui;
      udiff = (diff>0) ? diff : -diff;
      i = 32;
      while(udiff != 0) {
	i = i-1;
	udiff = udiff >> 1;
      }
      return udiff;
    }
  #endif // __SPU__
#endif // __PPC__


#endif // _FLOATINGPOINT_TESTS_H_
