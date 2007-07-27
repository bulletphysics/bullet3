/* Common constants for Sin/Cos/Tan
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

#ifndef ___SIMD_MATH__SINCOS_H___
#define ___SIMD_MATH__SINCOS_H___

//
// Common constants used to evaluate sind2/cosd2/tand2
//
#define __SINCOSD_CC0  0.00000000206374484196
#define __SINCOSD_CC1 -0.00000027555365134677
#define __SINCOSD_CC2  0.00002480157946764225
#define __SINCOSD_CC3 -0.00138888888730525966
#define __SINCOSD_CC4  0.04166666666651986722
#define __SINCOSD_CC5 -0.49999999999999547304

#define __SINCOSD_SC0  0.00000000015893606014
#define __SINCOSD_SC1 -0.00000002505069049138
#define __SINCOSD_SC2  0.00000275573131527032
#define __SINCOSD_SC3 -0.00019841269827816117
#define __SINCOSD_SC4  0.00833333333331908278
#define __SINCOSD_SC5 -0.16666666666666612594

#define __SINCOSD_KC1  (13176794.0 / 8388608.0)
#define __SINCOSD_KC2  7.5497899548918821691639751442098584e-8


//
// Common constants used to evaluate sinf4/cosf4/tanf4
//
#define __SINCOSF_CC0  -0.0013602249f
#define __SINCOSF_CC1   0.0416566950f
#define __SINCOSF_CC2  -0.4999990225f
#define __SINCOSF_SC0  -0.0001950727f
#define __SINCOSF_SC1   0.0083320758f
#define __SINCOSF_SC2  -0.1666665247f

#define __SINCOSF_KC1  1.57079625129f
#define __SINCOSF_KC2  7.54978995489e-8f

//
// Common constants used to evaluate sinf4est/cosf4est
//
#define __SINCOSF_R1 -0.1666665668f
#define __SINCOSF_R2  0.8333025139e-2f
#define __SINCOSF_R3 -0.1980741872e-3f
#define __SINCOSF_R4  0.2601903036e-5f

#define __SINCOSF_C1  (201.0f/64.0f)
#define __SINCOSF_C2  9.67653589793e-4f

#endif 
