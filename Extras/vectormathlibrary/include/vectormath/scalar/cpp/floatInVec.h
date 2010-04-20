/*
   Copyright (C) 2006-2010 Sony Computer Entertainment Inc.
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

#ifndef _FLOATINVEC__SCALAR_H
#define _FLOATINVEC__SCALAR_H

#include <math.h>
namespace Vectormath {

class boolInVec;

//--------------------------------------------------------------------------------------------------
// floatInVec class
//

// A class representing a scalar float value contained in a vector register
// This class does not support fastmath
class floatInVec
{
private:
    float mData;

public:
    // Default constructor; does no initialization
    //
    inline floatInVec( ) { };

    // Construct from a value converted from bool
    //
    inline floatInVec(boolInVec vec);

    // Explicit cast from float
    //
    explicit inline floatInVec(float scalar);

    // Explicit cast to float
    //
    inline float getAsFloat() const;

#ifndef _VECTORMATH_NO_SCALAR_CAST
    // Implicit cast to float
    //
    inline operator float() const;
#endif

    // Post increment (add 1.0f)
    //
    inline const floatInVec operator ++ (int);

    // Post decrement (subtract 1.0f)
    //
    inline const floatInVec operator -- (int);

    // Pre increment (add 1.0f)
    //
    inline floatInVec& operator ++ ();

    // Pre decrement (subtract 1.0f)
    //
    inline floatInVec& operator -- ();

    // Negation operator
    //
    inline const floatInVec operator - () const;

    // Assignment operator
    //
    inline floatInVec& operator = (floatInVec vec);

    // Multiplication assignment operator
    //
    inline floatInVec& operator *= (floatInVec vec);

    // Division assignment operator
    //
    inline floatInVec& operator /= (floatInVec vec);

    // Addition assignment operator
    //
    inline floatInVec& operator += (floatInVec vec);

    // Subtraction assignment operator
    //
    inline floatInVec& operator -= (floatInVec vec);

};

// Multiplication operator
//
inline const floatInVec operator * (floatInVec vec0, floatInVec vec1);

// Division operator
//
inline const floatInVec operator / (floatInVec vec0, floatInVec vec1);

// Addition operator
//
inline const floatInVec operator + (floatInVec vec0, floatInVec vec1);

// Subtraction operator
//
inline const floatInVec operator - (floatInVec vec0, floatInVec vec1);

// Less than operator
//
inline const boolInVec operator < (floatInVec vec0, floatInVec vec1);

// Less than or equal operator
//
inline const boolInVec operator <= (floatInVec vec0, floatInVec vec1);

// Greater than operator
//
inline const boolInVec operator > (floatInVec vec0, floatInVec vec1);

// Greater than or equal operator
//
inline const boolInVec operator >= (floatInVec vec0, floatInVec vec1);

// Equal operator
//
inline const boolInVec operator == (floatInVec vec0, floatInVec vec1);

// Not equal operator
//
inline const boolInVec operator != (floatInVec vec0, floatInVec vec1);

// Conditionally select between two values
//
inline const floatInVec select(floatInVec vec0, floatInVec vec1, boolInVec select_vec1);


} // namespace Vectormath


//--------------------------------------------------------------------------------------------------
// floatInVec implementation
//

#include "boolInVec.h"

namespace Vectormath {

inline
floatInVec::floatInVec(boolInVec vec)
{
    mData = float(vec.getAsBool());
}

inline
floatInVec::floatInVec(float scalar)
{
    mData = scalar;
}

inline
float
floatInVec::getAsFloat() const
{
    return mData;
}

#ifndef _VECTORMATH_NO_SCALAR_CAST
inline
floatInVec::operator float() const
{
    return getAsFloat();
}
#endif

inline
const floatInVec
floatInVec::operator ++ (int)
{
    float olddata = mData;
    operator ++();
    return floatInVec(olddata);
}

inline
const floatInVec
floatInVec::operator -- (int)
{
    float olddata = mData;
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
    return floatInVec(-mData);
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
    return floatInVec(vec0.getAsFloat() * vec1.getAsFloat());
}

inline
const floatInVec
operator / (floatInVec num, floatInVec den)
{
    return floatInVec(num.getAsFloat() / den.getAsFloat());
}

inline
const floatInVec
operator + (floatInVec vec0, floatInVec vec1)
{
    return floatInVec(vec0.getAsFloat() + vec1.getAsFloat());
}

inline
const floatInVec
operator - (floatInVec vec0, floatInVec vec1)
{
    return floatInVec(vec0.getAsFloat() - vec1.getAsFloat());
}

inline
const boolInVec
operator < (floatInVec vec0, floatInVec vec1)
{
    return boolInVec(vec0.getAsFloat() < vec1.getAsFloat());
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
    return boolInVec(vec0.getAsFloat() > vec1.getAsFloat());
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
    return boolInVec(vec0.getAsFloat() == vec1.getAsFloat());
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
    return (select_vec1.getAsBool() == 0) ? vec0 : vec1;
}

} // namespace Vectormath

#endif // floatInVec_h
