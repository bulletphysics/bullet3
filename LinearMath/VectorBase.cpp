// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
//
// VectorBase.cpp
//
// Copyright (c) 2006 Simon Hobbs
//
// This software is provided 'as-is', without any express or implied warranty. In no event will the authors be held liable for any damages arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose, including commercial applications, and to alter it and redistribute it freely, subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source distribution.
#ifdef WIN32
#if _MSC_VER >= 1310

#include "VectorBase.h"

const __m128 Vector4Base::Consts::kZero		= _mm_set1_ps(0.0f);
const __m128 Vector4Base::Consts::kHalf		= _mm_set1_ps(0.5f);
const __m128 Vector4Base::Consts::kThree	= _mm_set1_ps(3.0f);

const __m128 Vector4Base::Consts::k1000		= _mm_setr_ps(1.0f, 0.0f, 0.0f, 0.0f);
const __m128 Vector4Base::Consts::k0100		= _mm_setr_ps(0.0f, 1.0f, 0.0f, 0.0f);
const __m128 Vector4Base::Consts::k0010		= _mm_setr_ps(0.0f, 0.0f, 1.0f, 0.0f);
const __m128 Vector4Base::Consts::k0001		= _mm_setr_ps(0.0f, 0.0f, 0.0f, 1.0f);

const __m128 Vector4Base::Consts::kNeg1000	= _mm_setr_ps(-1.0f, 0.0f, 0.0f, 0.0f);
const __m128 Vector4Base::Consts::kNeg0100	= _mm_setr_ps(0.0f, -1.0f, 0.0f, 0.0f);
const __m128 Vector4Base::Consts::kNeg0010	= _mm_setr_ps(0.0f, 0.0f, -1.0f, 0.0f);
const __m128 Vector4Base::Consts::kNeg0001	= _mm_setr_ps(0.0f, 0.0f, 0.0f, -1.0f);

const __m128 Vector4Base::Consts::kNeg111_1	= _mm_setr_ps(-1.0f, -1.0f, -1.0f, 1.0f);
const __m128 Vector4Base::Consts::k1110		= _mm_setr_ps(1.0f, 1.0f, 1.0f, 0.0f);

const unsigned int Vector4Base::Consts::maskAbs = 0x7fffffff;
const __m128 Vector4Base::Consts::kMaskAbs = _mm_load1_ps((float*)&Vector4Base::Consts::maskAbs);

const unsigned int Vector4Base::Consts::mask1110[4] = {0xffffffff, 0xffffffff, 0xffffffff, 0x00000000};
const unsigned int Vector4Base::Consts::mask0001[4] = {0x00000000, 0x00000000, 0x00000000, 0xffffffff};

const __m128 Vector4Base::Consts::kMask1110 = _mm_loadu_ps((float*)Vector4Base::Consts::mask1110);
const __m128 Vector4Base::Consts::kMask0001 = _mm_loadu_ps((float*)Vector4Base::Consts::mask0001);

#endif 
#endif //WIN32
