//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#ifndef PSFOUNDATION_PSBASICTEMPLATES_H
#define PSFOUNDATION_PSBASICTEMPLATES_H

#include "Ps.h"

namespace physx
{
namespace shdfnd
{
template <typename A>
struct Equal
{
	bool operator()(const A& a, const A& b) const
	{
		return a == b;
	}
};

template <typename A>
struct Less
{
	bool operator()(const A& a, const A& b) const
	{
		return a < b;
	}
};

template <typename A>
struct Greater
{
	bool operator()(const A& a, const A& b) const
	{
		return a > b;
	}
};

template <class F, class S>
class Pair
{
  public:
	F first;
	S second;
	Pair() : first(F()), second(S())
	{
	}
	Pair(const F& f, const S& s) : first(f), second(s)
	{
	}
	Pair(const Pair& p) : first(p.first), second(p.second)
	{
	}
	// CN - fix for /.../PsBasicTemplates.h(61) : warning C4512: 'physx::shdfnd::Pair<F,S>' : assignment operator could
	// not be generated
	Pair& operator=(const Pair& p)
	{
		first = p.first;
		second = p.second;
		return *this;
	}
	bool operator==(const Pair& p) const
	{
		return first == p.first && second == p.second;
	}
	bool operator<(const Pair& p) const
	{
		if(first < p.first)
			return true;
		else
			return !(p.first < first) && (second < p.second);
	}
};

template <unsigned int A>
struct LogTwo
{
	static const unsigned int value = LogTwo<(A >> 1)>::value + 1;
};
template <>
struct LogTwo<1>
{
	static const unsigned int value = 0;
};

template <typename T>
struct UnConst
{
	typedef T Type;
};
template <typename T>
struct UnConst<const T>
{
	typedef T Type;
};

template <typename T>
T pointerOffset(void* p, ptrdiff_t offset)
{
	return reinterpret_cast<T>(reinterpret_cast<char*>(p) + offset);
}
template <typename T>
T pointerOffset(const void* p, ptrdiff_t offset)
{
	return reinterpret_cast<T>(reinterpret_cast<const char*>(p) + offset);
}

template <class T>
PX_CUDA_CALLABLE PX_INLINE void swap(T& x, T& y)
{
	const T tmp = x;
	x = y;
	y = tmp;
}

} // namespace shdfnd
} // namespace physx

#endif // #ifndef PSFOUNDATION_PSBASICTEMPLATES_H
