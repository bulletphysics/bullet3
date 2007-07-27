/* Common part of testsuite for SPU SIMD Math library
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


#include <math.h>
#include <spu_intrinsics.h>
#include <simdmath.h>
#include "testutils.h"

typedef union {
   unsigned int ui;
   float f;
} conv4_t;

typedef union {
   unsigned long long ull;
   double df;
} conv8_t;

unsigned int 
hide_uint( unsigned int x )
{
   return x;
}

int 
hide_int( int x )
{
   return x;
}

float
hide_float( float x )
{
   return x;
}

double
hide_double( double x )
{
   return x;
}

float 
make_float( unsigned int x )
{
   conv4_t val;
   val.ui = x;
   return val.f;
}

unsigned int 
make_uint( float x )
{
   conv4_t val;
   val.f = x;
   return val.ui;
}

double
make_double( unsigned long long x )
{
   conv8_t val;
   val.ull = x;
   return val.df;
}

unsigned long long
make_ulonglong( double x )
{
   conv8_t val;
   val.df = x;
   return val.ull;
}

vec_uint4 bitDiff_f4(vec_float4 ref, vec_float4 vals) {
  vec_int4 refi  = (vec_int4)ref;
  vec_int4 valsi = (vec_int4)vals;
  vec_int4 diff  = spu_sub(refi, valsi);
  vec_int4 negdiff = spu_sub(spu_splats((int)0), diff);

  return spu_sub((vec_uint4)spu_splats(32), spu_cntlz(spu_sel(negdiff, diff, spu_cmpgt(diff, 0))));
}

unsigned int bitDiff_f(float ref, float val) {
  return spu_extract(bitDiff_f4(spu_promote(ref,0), spu_promote(val,0)), 0);
}

vec_ullong2 bitDiff_d2(vec_double2 ref, vec_double2 vals) {
   double ref0, ref1, vals0, vals1;
   long long refi0, refi1, valsi0, valsi1, diff0, diff1;
   vec_ullong2 bits;

   ref0 = spu_extract(ref,0);
   ref1 = spu_extract(ref,1);
   vals0 = spu_extract(vals,0);
   vals1 = spu_extract(vals,1);

   refi0 = make_ulonglong(ref0);
   refi1 = make_ulonglong(ref1);
   valsi0 = make_ulonglong(vals0);
   valsi1 = make_ulonglong(vals1);

   diff0 = refi0 - valsi0;
   diff1 = refi1 - valsi1;

   if ( diff0 < 0 )
   {
      diff0 = valsi0 - refi0;
   }

   if ( diff1 < 0 )
   {
      diff1 = valsi1 - refi1;
   }

   bits = spu_promote( (unsigned long long)ceil(log2((double)diff0)), 0 );
   bits = spu_insert( (unsigned long long)ceil(log2((double)diff1)), bits, 1 );

   return bits;
}

unsigned long long bitDiff_d(double ref, double val) {
  return spu_extract(bitDiff_d2(spu_promote(ref,0), spu_promote(val,0)), 0);
}

vec_uint4 ulpDiff_f4(vec_float4 ref, vec_float4 vals) {
  vec_int4 refi  = (vec_int4)ref;
  vec_int4 valsi = (vec_int4)vals;
  vec_int4 diff  = spu_sub(refi, valsi);
  vec_int4 negdiff = spu_sub(spu_splats((int)0), diff);

  return (vec_uint4)(spu_sel(negdiff, diff, spu_cmpgt(diff, 0)));
}

unsigned int ulpDiff_f(float ref, float val) {
  return spu_extract(ulpDiff_f4(spu_promote(ref,0), spu_promote(val,0)), 0);
}

vec_ullong2 ulpDiff_d2(vec_double2 ref, vec_double2 vals) {
   double ref0, ref1, vals0, vals1;
   long long refi0, refi1, valsi0, valsi1, diff0, diff1;
   vec_ullong2 ulps;

   ref0 = spu_extract(ref,0);
   ref1 = spu_extract(ref,1);
   vals0 = spu_extract(vals,0);
   vals1 = spu_extract(vals,1);

   refi0 = make_ulonglong(ref0);
   refi1 = make_ulonglong(ref1);
   valsi0 = make_ulonglong(vals0);
   valsi1 = make_ulonglong(vals1);

   diff0 = refi0 - valsi0;
   diff1 = refi1 - valsi1;

   if ( diff0 < 0 )
   {
      diff0 = valsi0 - refi0;
   }

   if ( diff1 < 0 )
   {
      diff1 = valsi1 - refi1;
   }

   ulps = spu_promote( (unsigned long long)diff0, 0 );
   ulps = spu_insert( (unsigned long long)diff1, ulps, 1 );

   return ulps;
}

unsigned long long ulpDiff_d(double ref, double val) {
  return spu_extract(ulpDiff_d2(spu_promote(ref,0), spu_promote(val,0)), 0);
}

vec_ullong2 cmpposzerod2( vec_double2 x )
{
   vec_ullong2 cmp;
   vec_uchar16 even = (vec_uchar16)(vec_uint4){ 0x00010203, 0x00010203, 0x08090a0b, 0x08090a0b };
   vec_uchar16 odd = (vec_uchar16)(vec_uint4){ 0x04050607, 0x04050607, 0x0c0d0e0f, 0x0c0d0e0f };
 
   cmp = (vec_ullong2)spu_cmpeq( (vec_int4)x, spu_splats(0) );
   cmp = spu_and( spu_shuffle( cmp, cmp, even ), spu_shuffle( cmp, cmp, odd ) );
 
   return cmp;
}

vec_ullong2 cmpnegzerod2( vec_double2 x )
{
   vec_ullong2 cmp;
   vec_uchar16 even = (vec_uchar16)(vec_uint4){ 0x00010203, 0x00010203, 0x08090a0b, 0x08090a0b };
   vec_uchar16 odd = (vec_uchar16)(vec_uint4){ 0x04050607, 0x04050607, 0x0c0d0e0f, 0x0c0d0e0f };
 
   cmp = (vec_ullong2)spu_cmpeq( (vec_int4)x, (vec_int4)spu_splats(0x8000000000000000ull) );
   cmp = spu_and( spu_shuffle( cmp, cmp, even ), spu_shuffle( cmp, cmp, odd ) );
 
   return cmp;
}

int allequal_int4( vec_int4 x, vec_int4 y )
{
   return ( spu_extract( spu_gather( spu_cmpeq( x, y ) ), 0 ) == 0xf );
}
int allequal_llong2( vec_llong2 x, vec_llong2 y )
{
   return spu_extract( spu_gather( spu_cmpeq ((vec_int4)(x - y), spu_splats((int)0) )), 0) == 0xF;
}

int allequal_float4( vec_float4 x, vec_float4 y )
{
   return ( spu_extract( spu_gather( (vec_uint4)spu_cmpeq( x, y ) ), 0 ) == 0xf );
}

int allequal_double2( vec_double2 x, vec_double2 y )
{
   return ( spu_extract(x,0) == spu_extract(y,0) && spu_extract(x,1) == spu_extract(y,1) );
}

int allequal_llroundf4( llroundf4_t x, llroundf4_t y )
{
   return ( spu_extract(x.vll[0],0) == spu_extract(y.vll[0],0) &&
            spu_extract(x.vll[0],1) == spu_extract(y.vll[0],1) &&
            spu_extract(x.vll[1],0) == spu_extract(y.vll[1],0) &&
            spu_extract(x.vll[1],1) == spu_extract(y.vll[1],1)   );
}

int allequal_ulps_float4( vec_float4 x, vec_float4 y, int tolerance )
{
   vec_uint4 ulps = ulpDiff_f4( x, y );
   return ( (int)spu_extract(ulps,0) <= tolerance && 
            (int)spu_extract(ulps,1) <= tolerance &&
            (int)spu_extract(ulps,2) <= tolerance &&
            (int)spu_extract(ulps,3) <= tolerance );
}

int allequal_ulps_double2( vec_double2 x, vec_double2 y, int tolerance )
{
   vec_ullong2 ulps = ulpDiff_d2( x, y );
   return ( (int)spu_extract(ulps,0) <= tolerance && (int)spu_extract(ulps,1) <= tolerance );
}

int allequal_bits_float4( vec_float4 x, vec_float4 y, int tolerance )
{
   vec_uint4 bits = bitDiff_f4( x, y );
   return ( (int)spu_extract(bits,0) <= tolerance && 
            (int)spu_extract(bits,1) <= tolerance &&
            (int)spu_extract(bits,2) <= tolerance &&
            (int)spu_extract(bits,3) <= tolerance );
}

int allequal_bits_double2( vec_double2 x, vec_double2 y, int tolerance )
{
   vec_ullong2 bits = bitDiff_d2( x, y );
   return ( (int)spu_extract(bits,0) <= tolerance && (int)spu_extract(bits,1) <= tolerance );
}

int allposinf_double2( vec_double2 x )
{
   vec_ullong2 posinf = spu_andc( isinfd2 ( x ), signbitd2 ( x ) );
   return ( spu_extract(posinf,0) != 0 && spu_extract(posinf,1) != 0 );
}

int allneginf_double2( vec_double2 x )
{
   vec_ullong2 neginf = spu_and( isinfd2 ( x ), signbitd2 ( x ) );
   return ( spu_extract(neginf,0) != 0 && spu_extract(neginf,1) != 0 );
}

int allzerodenorm_double2( vec_double2 x )
{
   vec_ullong2 zero = is0denormd2 ( x );
   return ( spu_extract(zero,0) != 0 && spu_extract(zero,1) != 0 );
}

int allposzero_double2( vec_double2 x )
{
   vec_ullong2 poszero = cmpposzerod2( x );
   return ( spu_extract(poszero,0) != 0 && spu_extract(poszero,1) != 0 );
}

int allnegzero_double2( vec_double2 x )
{
   vec_ullong2 negzero = cmpnegzerod2( x );
   return ( spu_extract(negzero,0) != 0 && spu_extract(negzero,1) != 0 );
}

int allnan_double2( vec_double2 x )
{
   vec_ullong2 nan = isnand2 ( x );
   return ( spu_extract(nan,0) != 0 && spu_extract(nan,1) != 0 );
}


