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

#ifndef _FLOATINVEC_H
#define _FLOATINVEC_H

#include <math.h>
#include <spu_intrinsics.h>
#include <simdmath.h>
#undef bool

namespace Vectormath {

class boolInVec;

//--------------------------------------------------------------------------------------------------
// floatInVec class
//

class floatInVec
{
    private:
        vec_float4 mData;

        inline floatInVec(vec_float4 vec);
    public:
        inline floatInVec() {}

        // matches standard type conversions
        //
        inline floatInVec(boolInVec vec);

        // construct from a slot of vec_float4
        //
        inline floatInVec(vec_float4 vec, int slot);
        
        // explicit cast from float
        //
        explicit inline floatInVec(float scalar);

#ifdef _VECTORMATH_NO_SCALAR_CAST
        // explicit cast to float
        // 
        inline float getAsFloat() const;
#else
        // implicit cast to float
        //
        inline operator float() const;
#endif

        // get vector data
        // float value is in 0 word slot of vector
        //
        inline vec_float4 get128() const;

        // operators
        // 
        inline const floatInVec operator ++ (int);
        inline const floatInVec operator -- (int);
        inline floatInVec& operator ++ ();
        inline floatInVec& operator -- ();
        inline const floatInVec operator - () const;
        inline floatInVec& operator = (floatInVec vec);
        inline floatInVec& operator *= (floatInVec vec);
        inline floatInVec& operator /= (floatInVec vec);
        inline floatInVec& operator += (floatInVec vec);
        inline floatInVec& operator -= (floatInVec vec);

        // friend functions
        //
        friend inline const floatInVec operator * (floatInVec vec0, floatInVec vec1);
        friend inline const floatInVec operator / (floatInVec vec0, floatInVec vec1);
        friend inline const floatInVec operator + (floatInVec vec0, floatInVec vec1);
        friend inline const floatInVec operator - (floatInVec vec0, floatInVec vec1);
        friend inline const floatInVec select(floatInVec vec0, floatInVec vec1, boolInVec select_vec1);
};

//--------------------------------------------------------------------------------------------------
// floatInVec functions
//

// operators
// 
inline const floatInVec operator * (floatInVec vec0, floatInVec vec1);
inline const floatInVec operator / (floatInVec vec0, floatInVec vec1);
inline const floatInVec operator + (floatInVec vec0, floatInVec vec1);
inline const floatInVec operator - (floatInVec vec0, floatInVec vec1);
inline const boolInVec operator < (floatInVec vec0, floatInVec vec1);
inline const boolInVec operator <= (floatInVec vec0, floatInVec vec1);
inline const boolInVec operator > (floatInVec vec0, floatInVec vec1);
inline const boolInVec operator >= (floatInVec vec0, floatInVec vec1);
inline const boolInVec operator == (floatInVec vec0, floatInVec vec1);
inline const boolInVec operator != (floatInVec vec0, floatInVec vec1);

// select between vec0 and vec1 using boolInVec.
// false selects vec0, true selects vec1
//
inline const floatInVec select(floatInVec vec0, floatInVec vec1, boolInVec select_vec1);

} // namespace Vectormath

//--------------------------------------------------------------------------------------------------
// floatInVec implementation
//

#include "boolInVec.h"

namespace Vectormath {

inline
floatInVec::floatInVec(vec_float4 vec)
{
    mData = vec;
}

inline
floatInVec::floatInVec(boolInVec vec)
{
    mData = spu_sel(spu_splats(0.0f), spu_splats(1.0f), vec.get128());
}

inline
floatInVec::floatInVec(vec_float4 vec, int slot)
{
    mData = spu_promote(spu_extract(vec, slot), 0);
}

inline
floatInVec::floatInVec(float scalar)
{
    mData = spu_promote(scalar, 0);
}

#ifdef _VECTORMATH_NO_SCALAR_CAST
inline
float
floatInVec::getAsFloat() const
#else
inline
floatInVec::operator float() const
#endif
{
    return spu_extract(mData,0);
}

inline
vec_float4
floatInVec::get128() const
{
    return mData;
}

inline
const floatInVec
floatInVec::operator ++ (int)
{
    vec_float4 olddata = mData;
    operator ++();
    return floatInVec(olddata);
}

inline
const floatInVec
floatInVec::operator -- (int)
{
    vec_float4 olddata = mData;
    operator --();
    return floatInVec(olddata);
}

inline
floatInVec&
floatInVec::operator ++ ()
{
    *this += floatInVec(1.0f);
    return *this;
}

inline
floatInVec&
floatInVec::operator -- ()
{
    *this -= floatInVec(1.0f);
    return *this;
}

inline
const floatInVec
floatInVec::operator - () const
{
    return floatInVec((vec_float4)spu_xor((vec_uint4)mData, spu_splats(0x80000000)));
}

inline
floatInVec&
floatInVec::operator = (floatInVec vec)
{
    mData = vec.mData;
    return *this;
}

inline
floatInVec&
floatInVec::operator *= (floatInVec vec)
{
    *this = *this * vec;
    return *this;
}

inline
floatInVec&
floatInVec::operator /= (floatInVec vec)
{
    *this = *this / vec;
    return *this;
}

inline
floatInVec&
floatInVec::operator += (floatInVec vec)
{
    *this = *this + vec;
    return *this;
}

inline
floatInVec&
floatInVec::operator -= (floatInVec vec)
{
    *this = *this - vec;
    return *this;
}

inline
const floatInVec
operator * (floatInVec vec0, floatInVec vec1)
{
    return floatInVec(spu_mul(vec0.get128(), vec1.get128()));
}

inline
const floatInVec
operator / (floatInVec num, floatInVec den)
{
    return floatInVec(divf4(num.get128(), den.get128()));
}

inline
const floatInVec
operator + (floatInVec vec0, floatInVec vec1)
{
    return floatInVec(spu_add(vec0.get128(), vec1.get128()));
}

inline
const floatInVec
operator - (floatInVec vec0, floatInVec vec1)
{
    return floatInVec(spu_sub(vec0.get128(), vec1.get128()));
}

inline
const boolInVec
operator < (floatInVec vec0, floatInVec vec1)
{
    return boolInVec(spu_cmpgt(vec1.get128(), vec0.get128()));
}

inline
const boolInVec
operator <= (floatInVec vec0, floatInVec vec1)
{
    return !(vec0 > vec1);
}

inline
const boolInVec
operator > (floatInVec vec0, floatInVec vec1)
{
    return boolInVec(spu_cmpgt(vec0.get128(), vec1.get128()));
}

inline
const boolInVec
operator >= (floatInVec vec0, floatInVec vec1)
{
    return !(vec0 < vec1);
}

inline
const boolInVec
operator == (floatInVec vec0, floatInVec vec1)
{
    return boolInVec(spu_cmpeq(vec0.get128(), vec1.get128()));
}

inline
const boolInVec
operator != (floatInVec vec0, floatInVec vec1)
{
    return !(vec0 == vec1);
}
    
inline
const floatInVec
select(floatInVec vec0, floatInVec vec1, boolInVec select_vec1)
{
    return floatInVec(spu_sel(vec0.get128(), vec1.get128(), select_vec1.get128()));
}

} // namespace Vectormath

#endif // floatInVec_h
