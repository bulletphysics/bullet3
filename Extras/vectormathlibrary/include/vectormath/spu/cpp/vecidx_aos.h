/*
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

#ifndef _VECTORMATH_VECIDX_AOS_H
#define _VECTORMATH_VECIDX_AOS_H

#include <spu_intrinsics.h>

namespace Vectormath {
namespace Aos {

//-----------------------------------------------------------------------------
// VecIdx 
// Used in setting elements of Vector3, Vector4, Point3, or Quat with the 
// subscripting operator.
//

class VecIdx
{
private:
    typedef vec_float4 vec_float4_t;
    vec_float4_t &ref __attribute__ ((aligned(16)));
    int i __attribute__ ((aligned(16)));
public:
    inline VecIdx( vec_float4& vec, int idx ): ref(vec) { i = idx; }
    inline operator float() const;
    inline float operator =( float scalar );
    inline float operator =( const VecIdx& scalar );
    inline float operator *=( float scalar );
    inline float operator /=( float scalar );
    inline float operator +=( float scalar );
    inline float operator -=( float scalar );
};

} // namespace Aos
} // namespace Vectormath

#endif
