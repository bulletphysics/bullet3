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

#ifndef GU_BV4_SLABS_H
#define GU_BV4_SLABS_H

#include "PsFPU.h"
#include "GuBV4_Common.h"

#ifdef GU_BV4_USE_SLABS

	// PT: contains code for tree-traversal using the swizzled format.
	// PT: ray traversal based on Kay & Kajiya's slab intersection code, but using SIMD to do 4 ray-vs-AABB tests at a time.
	// PT: other (ordered or unordered) traversals just process one node at a time, similar to the non-swizzled format.

	#define BV4_SLABS_FIX
	#define BV4_SLABS_SORT

	#define PNS_BLOCK3(a, b, c, d)	{										\
		if(code2 & (1<<a))	{ stack[nb++] = tn->getChildData(a);	}		\
		if(code2 & (1<<b))	{ stack[nb++] = tn->getChildData(b);	}		\
		if(code2 & (1<<c))	{ stack[nb++] = tn->getChildData(c);	}		\
		if(code2 & (1<<d))	{ stack[nb++] = tn->getChildData(d);	}	}	\

	#define OPC_SLABS_GET_MIN_MAX(i)																	\
		const __m128i minVi = _mm_set_epi32(0, node->mZ[i].mMin, node->mY[i].mMin, node->mX[i].mMin);	\
		const Vec4V minCoeffV = V4LoadA_Safe(&params->mCenterOrMinCoeff_PaddedAligned.x);				\
		Vec4V minV = V4Mul(_mm_cvtepi32_ps(minVi), minCoeffV);											\
		const __m128i maxVi = _mm_set_epi32(0, node->mZ[i].mMax, node->mY[i].mMax, node->mX[i].mMax);	\
		const Vec4V maxCoeffV = V4LoadA_Safe(&params->mExtentsOrMaxCoeff_PaddedAligned.x);				\
		Vec4V maxV = V4Mul(_mm_cvtepi32_ps(maxVi), maxCoeffV);											\

	#define OPC_SLABS_GET_CEQ(i)									\
		OPC_SLABS_GET_MIN_MAX(i)									\
		const FloatV HalfV = FLoad(0.5f);							\
		const Vec4V centerV = V4Scale(V4Add(maxV, minV), HalfV);	\
		const Vec4V extentsV = V4Scale(V4Sub(maxV, minV), HalfV);

	#define OPC_SLABS_GET_CE2Q(i)					\
		OPC_SLABS_GET_MIN_MAX(i)					\
		const Vec4V centerV = V4Add(maxV, minV);	\
		const Vec4V extentsV = V4Sub(maxV, minV);

	#define OPC_SLABS_GET_CENQ(i)																\
		const FloatV HalfV = FLoad(0.5f);														\
		const Vec4V minV = _mm_set_ps(0.0f, node->mMinZ[i], node->mMinY[i], node->mMinX[i]);	\
		const Vec4V maxV = _mm_set_ps(0.0f, node->mMaxZ[i], node->mMaxY[i], node->mMaxX[i]);	\
		const Vec4V centerV = V4Scale(V4Add(maxV, minV), HalfV);								\
		const Vec4V extentsV = V4Scale(V4Sub(maxV, minV), HalfV);

	#define OPC_SLABS_GET_CE2NQ(i)																\
		const Vec4V minV = _mm_set_ps(0.0f, node->mMinZ[i], node->mMinY[i], node->mMinX[i]);	\
		const Vec4V maxV = _mm_set_ps(0.0f, node->mMaxZ[i], node->mMaxY[i], node->mMaxX[i]);	\
		const Vec4V centerV = V4Add(maxV, minV);												\
		const Vec4V extentsV = V4Sub(maxV, minV);

#if PX_PS4
	// PT: TODO: for some reason using the intrinsics directly produces a compile error on PS4. TODO: find a better fix.
	PX_FORCE_INLINE __m128i my_mm_srai_epi32(__m128i a, int count)
	{
		return _mm_srai_epi32(a, count);
	}

	PX_FORCE_INLINE __m128i my_mm_slli_epi32(__m128i a, int count)
	{
		return _mm_slli_epi32(a, count);
	}
#else
	#define my_mm_srai_epi32	_mm_srai_epi32
	#define my_mm_slli_epi32	_mm_slli_epi32
#endif

#define OPC_DEQ4(part2xV, part1xV, mMember, minCoeff, maxCoeff)												\
{																											\
	part2xV = V4LoadA(reinterpret_cast<const float*>(tn->mMember));											\
	part1xV = _mm_castsi128_ps(_mm_and_si128(_mm_castps_si128(part2xV), _mm_set1_epi32(0x0000ffff)));		\
		part1xV = _mm_castsi128_ps(my_mm_srai_epi32(my_mm_slli_epi32(_mm_castps_si128(part1xV), 16), 16));	\
	part1xV = V4Mul(_mm_cvtepi32_ps(_mm_castps_si128(part1xV)), minCoeff);									\
	part2xV = _mm_castsi128_ps(my_mm_srai_epi32(_mm_castps_si128(part2xV), 16));							\
	part2xV = V4Mul(_mm_cvtepi32_ps(_mm_castps_si128(part2xV)), maxCoeff);									\
}

#define SLABS_INIT\
	Vec4V maxT4 = V4Load(params->mStabbedFace.mDistance);\
	const Vec4V rayP = V4LoadU_Safe(&params->mOrigin_Padded.x);\
	Vec4V rayD = V4LoadU_Safe(&params->mLocalDir_Padded.x);\
	const VecU32V raySign = V4U32and(VecU32V_ReinterpretFrom_Vec4V(rayD), signMask);\
	const Vec4V rayDAbs = V4Abs(rayD);\
	Vec4V rayInvD = Vec4V_ReinterpretFrom_VecU32V(V4U32or(raySign, VecU32V_ReinterpretFrom_Vec4V(V4Max(rayDAbs, epsFloat4))));\
	rayD = rayInvD;\
	rayInvD = V4RecipFast(rayInvD);\
	rayInvD = V4Mul(rayInvD, V4NegMulSub(rayD, rayInvD, twos));\
	const Vec4V rayPinvD = V4NegMulSub(rayInvD, rayP, zeroes);\
	const Vec4V rayInvDsplatX = V4SplatElement<0>(rayInvD);\
	const Vec4V rayInvDsplatY = V4SplatElement<1>(rayInvD);\
	const Vec4V rayInvDsplatZ = V4SplatElement<2>(rayInvD);\
	const Vec4V rayPinvDsplatX = V4SplatElement<0>(rayPinvD);\
	const Vec4V rayPinvDsplatY = V4SplatElement<1>(rayPinvD);\
	const Vec4V rayPinvDsplatZ = V4SplatElement<2>(rayPinvD);

#define SLABS_TEST\
	const Vec4V tminxa0 = V4MulAdd(minx4a, rayInvDsplatX, rayPinvDsplatX);\
	const Vec4V tminya0 = V4MulAdd(miny4a, rayInvDsplatY, rayPinvDsplatY);\
	const Vec4V tminza0 = V4MulAdd(minz4a, rayInvDsplatZ, rayPinvDsplatZ);\
	const Vec4V tmaxxa0 = V4MulAdd(maxx4a, rayInvDsplatX, rayPinvDsplatX);\
	const Vec4V tmaxya0 = V4MulAdd(maxy4a, rayInvDsplatY, rayPinvDsplatY);\
	const Vec4V tmaxza0 = V4MulAdd(maxz4a, rayInvDsplatZ, rayPinvDsplatZ);\
	const Vec4V tminxa = V4Min(tminxa0, tmaxxa0);\
	const Vec4V tmaxxa = V4Max(tminxa0, tmaxxa0);\
	const Vec4V tminya = V4Min(tminya0, tmaxya0);\
	const Vec4V tmaxya = V4Max(tminya0, tmaxya0);\
	const Vec4V tminza = V4Min(tminza0, tmaxza0);\
	const Vec4V tmaxza = V4Max(tminza0, tmaxza0);\
	const Vec4V maxOfNeasa = V4Max(V4Max(tminxa, tminya), tminza);\
	const Vec4V minOfFarsa = V4Min(V4Min(tmaxxa, tmaxya), tmaxza);\

	#define SLABS_TEST2\
		__m128 ignore4a = _mm_cmpgt_ps(epsFloat4, minOfFarsa);  /* if tfar is negative, ignore since its a ray, not a line */\
		ignore4a = _mm_or_ps(ignore4a, _mm_cmpgt_ps(maxOfNeasa, maxT4));  /* if tnear is over maxT, ignore this result */\
		__m128 resa4 = _mm_cmpgt_ps(maxOfNeasa, minOfFarsa); /* if 1 => fail */\
		resa4 = _mm_or_ps(resa4, ignore4a);\
		const int code = _mm_movemask_ps(resa4);\
		if(code==15)\
			continue;

#define SLABS_PNS										\
	if(code2)											\
	{													\
		if(tn->decodePNSNoShift(0) & dirMask)			\
		{												\
			if(tn->decodePNSNoShift(1) & dirMask)		\
			{											\
				if(tn->decodePNSNoShift(2) & dirMask)	\
					PNS_BLOCK3(3,2,1,0)					\
				else									\
					PNS_BLOCK3(2,3,1,0)					\
			}											\
			else										\
			{											\
				if(tn->decodePNSNoShift(2) & dirMask)	\
					PNS_BLOCK3(3,2,0,1)					\
				else									\
					PNS_BLOCK3(2,3,0,1)					\
			}											\
		}												\
		else											\
		{												\
			if(tn->decodePNSNoShift(1) & dirMask)		\
			{											\
				if(tn->decodePNSNoShift(2) & dirMask)	\
					PNS_BLOCK3(1,0,3,2)					\
				else									\
					PNS_BLOCK3(1,0,2,3)					\
			}											\
			else										\
			{											\
				if(tn->decodePNSNoShift(2) & dirMask)	\
					PNS_BLOCK3(0,1,3,2)					\
				else									\
					PNS_BLOCK3(0,1,2,3)					\
			}											\
		}												\
	}

#endif	// GU_BV4_USE_SLABS

#endif // GU_BV4_SLABS_H
