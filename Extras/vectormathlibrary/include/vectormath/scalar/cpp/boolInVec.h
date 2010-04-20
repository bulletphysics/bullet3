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

#ifndef _BOOLINVEC_SCALAR_H
#define _BOOLINVEC_SCALAR_H

#include <math.h>
namespace Vectormath {

class floatInVec;

//--------------------------------------------------------------------------------------------------
// boolInVec class
//

class boolInVec
{
private:
    unsigned int mData;

public:
    // Default constructor; does no initialization
    //
    inline boolInVec( ) { };

    // Construct from a value converted from float
    //
    inline boolInVec(floatInVec vec);

    // Explicit cast from bool
    //
    explicit inline boolInVec(bool scalar);

    // Explicit cast to bool
    //
    inline bool getAsBool() const;

#ifndef _VECTORMATH_NO_SCALAR_CAST
    // Implicit cast to bool
    //
    inline operator bool() const;
#endif

    // Boolean negation operator
    //
    inline const boolInVec operator ! () const;

    // Assignment operator
    //
    inline boolInVec& operator = (boolInVec vec);

    // Boolean and assignment operator
    //
    inline boolInVec& operator &= (boolInVec vec);

    // Boolean exclusive or assignment operator
    //
    inline boolInVec& operator ^= (boolInVec vec);

    // Boolean or assignment operator
    //
    inline boolInVec& operator |= (boolInVec vec);

};

// Equal operator
//
inline const boolInVec operator == (boolInVec vec0, boolInVec vec1);

// Not equal operator
//
inline const boolInVec operator != (boolInVec vec0, boolInVec vec1);

// And operator
//
inline const boolInVec operator & (boolInVec vec0, boolInVec vec1);

// Exclusive or operator
//
inline const boolInVec operator ^ (boolInVec vec0, boolInVec vec1);

// Or operator
//
inline const boolInVec operator | (boolInVec vec0, boolInVec vec1);

// Conditionally select between two values
//
inline const boolInVec select(boolInVec vec0, boolInVec vec1, boolInVec select_vec1);


} // namespace Vectormath


//--------------------------------------------------------------------------------------------------
// boolInVec implementation
//

#include "floatInVec.h"

namespace Vectormath {

inline
boolInVec::boolInVec(floatInVec vec)
{
    *this = (vec != floatInVec(0.0f));
}

inline
boolInVec::boolInVec(bool scalar)
{
    mData = -(int)scalar;
}

inline
bool
boolInVec::getAsBool() const
{
    return (mData > 0);
}

#ifndef _VECTORMATH_NO_SCALAR_CAST
inline
boolInVec::operator bool() const
{
    return getAsBool();
}
#endif

inline
const boolInVec
boolInVec::operator ! () const
{
    return boolInVec(!mData);
}

inline
boolInVec&
boolInVec::operator = (boolInVec vec)
{
    mData = vec.mData;
    return *this;
}

inline
boolInVec&
boolInVec::operator &= (boolInVec vec)
{
    *this = *this & vec;
    return *this;
}

inline
boolInVec&
boolInVec::operator ^= (boolInVec vec)
{
    *this = *this ^ vec;
    return *this;
}

inline
boolInVec&
boolInVec::operator |= (boolInVec vec)
{
    *this = *this | vec;
    return *this;
}

inline
const boolInVec
operator == (boolInVec vec0, boolInVec vec1)
{
    return boolInVec(vec0.getAsBool() == vec1.getAsBool());
}

inline
const boolInVec
operator != (boolInVec vec0, boolInVec vec1)
{
    return !(vec0 == vec1);
}

inline
const boolInVec
operator & (boolInVec vec0, boolInVec vec1)
{
    return boolInVec(vec0.getAsBool() & vec1.getAsBool());
}

inline
const boolInVec
operator | (boolInVec vec0, boolInVec vec1)
{
    return boolInVec(vec0.getAsBool() | vec1.getAsBool());
}

inline
const boolInVec
operator ^ (boolInVec vec0, boolInVec vec1)
{
    return boolInVec(vec0.getAsBool() ^ vec1.getAsBool());
}

inline
const boolInVec
select(boolInVec vec0, boolInVec vec1, boolInVec select_vec1)
{
    return (select_vec1.getAsBool() == 0) ? vec0 : vec1;
}

} // namespace Vectormath

#endif // boolInVec_h
