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
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef GU_BV4_BOX_BOX_OVERLAP_TEST_H
#define GU_BV4_BOX_BOX_OVERLAP_TEST_H

#if PX_INTEL_FAMILY	
#ifndef GU_BV4_USE_SLABS		
	PX_FORCE_INLINE Ps::IntBool BV4_BoxBoxOverlap(const PxVec3& extents, const PxVec3& center, const OBBTestParams* PX_RESTRICT params)
	{
		const PxU32 maskI = 0x7fffffff;

		const Vec4V extentsV = V4LoadU(&extents.x);

		const Vec4V TV = V4Sub(V4LoadA_Safe(&params->mTBoxToModel_PaddedAligned.x), V4LoadU(&center.x));
		{
			__m128 absTV = _mm_and_ps(TV, _mm_load1_ps((float*)&maskI));
			absTV = _mm_cmpgt_ps(absTV, V4Add(extentsV, V4LoadA_Safe(&params->mBB_PaddedAligned.x)));
			const PxU32 test = (PxU32)_mm_movemask_ps(absTV);
			if(test&7)
				return 0;
		}

		__m128 tV;
		{
			const __m128 T_YZX_V = _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(TV), _MM_SHUFFLE(3,0,2,1)));
			const __m128 T_ZXY_V = _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(TV), _MM_SHUFFLE(3,1,0,2)));

			tV = V4Mul(TV, V4LoadA_Safe(&params->mPreca0_PaddedAligned.x));
			tV = V4Add(tV, V4Mul(T_YZX_V, V4LoadA_Safe(&params->mPreca1_PaddedAligned.x)));
			tV = V4Add(tV, V4Mul(T_ZXY_V, V4LoadA_Safe(&params->mPreca2_PaddedAligned.x)));
		}

		__m128 t2V;
		{
			const __m128 extents_YZX_V = _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(extentsV), _MM_SHUFFLE(3,0,2,1)));
			const __m128 extents_ZXY_V = _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(extentsV), _MM_SHUFFLE(3,1,0,2)));

			t2V = V4Mul(extentsV, V4LoadA_Safe(&params->mPreca0b_PaddedAligned.x));
			t2V = V4Add(t2V, V4Mul(extents_YZX_V, V4LoadA_Safe(&params->mPreca1b_PaddedAligned.x)));
			t2V = V4Add(t2V, V4Mul(extents_ZXY_V, V4LoadA_Safe(&params->mPreca2b_PaddedAligned.x)));
			t2V = V4Add(t2V, V4LoadA_Safe(&params->mBoxExtents_PaddedAligned.x));
		}

		{
			__m128 abstV = _mm_and_ps(tV, _mm_load1_ps((float*)&maskI));
			abstV = _mm_cmpgt_ps(abstV, t2V);
			const PxU32 test = (PxU32)_mm_movemask_ps(abstV);
			if(test&7)
				return 0;
		}
		return 1;
	}

#ifdef GU_BV4_QUANTIZED_TREE	
	template<class T>
	PX_FORCE_INLINE Ps::IntBool BV4_BoxBoxOverlap(const T* PX_RESTRICT node, const OBBTestParams* PX_RESTRICT params)
	{ 
#define NEW_VERSION
#ifdef NEW_VERSION
	SSE_CONST4(maskV,	0x7fffffff);
	SSE_CONST4(maskQV,	0x0000ffff);
#else
	const PxU32 maskI = 0x7fffffff;
#endif

		Vec4V centerV = V4LoadA((float*)node->mAABB.mData);
#ifdef NEW_VERSION
		__m128 extentsV = _mm_castsi128_ps(_mm_and_si128(_mm_castps_si128(centerV), SSE_CONST(maskQV)));
#else
		__m128 extentsV = _mm_castsi128_ps(_mm_and_si128(_mm_castps_si128(centerV), _mm_set1_epi32(0x0000ffff)));
#endif
		extentsV = V4Mul(_mm_cvtepi32_ps(_mm_castps_si128(extentsV)), V4LoadA_Safe(&params->mExtentsOrMaxCoeff_PaddedAligned.x));
		centerV = _mm_castsi128_ps(_mm_srai_epi32(_mm_castps_si128(centerV), 16));
		centerV = V4Mul(_mm_cvtepi32_ps(_mm_castps_si128(centerV)), V4LoadA_Safe(&params->mCenterOrMinCoeff_PaddedAligned.x));

		const Vec4V TV = V4Sub(V4LoadA_Safe(&params->mTBoxToModel_PaddedAligned.x), centerV);
		{
#ifdef NEW_VERSION
			__m128 absTV = _mm_and_ps(TV, SSE_CONSTF(maskV));
#else
			__m128 absTV = _mm_and_ps(TV, _mm_load1_ps((float*)&maskI));
#endif

			absTV = _mm_cmpgt_ps(absTV, V4Add(extentsV, V4LoadA_Safe(&params->mBB_PaddedAligned.x)));
			const PxU32 test = (PxU32)_mm_movemask_ps(absTV);
			if(test&7)
				return 0;
		}

		__m128 tV;
		{
			const __m128 T_YZX_V = _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(TV), _MM_SHUFFLE(3,0,2,1)));
			const __m128 T_ZXY_V = _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(TV), _MM_SHUFFLE(3,1,0,2)));

			tV = V4Mul(TV, V4LoadA_Safe(&params->mPreca0_PaddedAligned.x));
			tV = V4Add(tV, V4Mul(T_YZX_V, V4LoadA_Safe(&params->mPreca1_PaddedAligned.x)));
			tV = V4Add(tV, V4Mul(T_ZXY_V, V4LoadA_Safe(&params->mPreca2_PaddedAligned.x)));
		}

		__m128 t2V;
		{
			const __m128 extents_YZX_V = _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(extentsV), _MM_SHUFFLE(3,0,2,1)));
			const __m128 extents_ZXY_V = _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(extentsV), _MM_SHUFFLE(3,1,0,2)));

			t2V = V4Mul(extentsV, V4LoadA_Safe(&params->mPreca0b_PaddedAligned.x));
			t2V = V4Add(t2V, V4Mul(extents_YZX_V, V4LoadA_Safe(&params->mPreca1b_PaddedAligned.x)));
			t2V = V4Add(t2V, V4Mul(extents_ZXY_V, V4LoadA_Safe(&params->mPreca2b_PaddedAligned.x)));
			t2V = V4Add(t2V, V4LoadA_Safe(&params->mBoxExtents_PaddedAligned.x));
		}

		{
#ifdef NEW_VERSION
			__m128 abstV = _mm_and_ps(tV, SSE_CONSTF(maskV));
#else
			__m128 abstV = _mm_and_ps(tV, _mm_load1_ps((float*)&maskI));
#endif
			abstV = _mm_cmpgt_ps(abstV, t2V);
			const PxU32 test = (PxU32)_mm_movemask_ps(abstV);
			if(test&7)
				return 0;
		}
		return 1;
	}
#endif	// GU_BV4_QUANTIZED_TREE
#endif	// GU_BV4_USE_SLABS

#ifdef GU_BV4_USE_SLABS	
	PX_FORCE_INLINE Ps::IntBool BV4_BoxBoxOverlap(const __m128 boxCenter, const __m128 extentsV, const OBBTestParams* PX_RESTRICT params)
	{
		const PxU32 maskI = 0x7fffffff;

		const Vec4V TV = V4Sub(V4LoadA_Safe(&params->mTBoxToModel_PaddedAligned.x), boxCenter);
		{
			__m128 absTV = _mm_and_ps(TV, _mm_load1_ps(reinterpret_cast<const float*>(&maskI)));
			absTV = _mm_cmpgt_ps(absTV, V4Add(extentsV, V4LoadA_Safe(&params->mBB_PaddedAligned.x)));
			const PxU32 test = PxU32(_mm_movemask_ps(absTV));
			if(test&7)
				return 0;
		}

		__m128 tV;
		{
			const __m128 T_YZX_V = _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(TV), _MM_SHUFFLE(3,0,2,1)));
			const __m128 T_ZXY_V = _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(TV), _MM_SHUFFLE(3,1,0,2)));

			tV = V4Mul(TV, V4LoadA_Safe(&params->mPreca0_PaddedAligned.x));
			tV = V4Add(tV, V4Mul(T_YZX_V, V4LoadA_Safe(&params->mPreca1_PaddedAligned.x)));
			tV = V4Add(tV, V4Mul(T_ZXY_V, V4LoadA_Safe(&params->mPreca2_PaddedAligned.x)));
		}

		__m128 t2V;
		{
			const __m128 extents_YZX_V = _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(extentsV), _MM_SHUFFLE(3,0,2,1)));
			const __m128 extents_ZXY_V = _mm_castsi128_ps(_mm_shuffle_epi32(_mm_castps_si128(extentsV), _MM_SHUFFLE(3,1,0,2)));

			t2V = V4Mul(extentsV, V4LoadA_Safe(&params->mPreca0b_PaddedAligned.x));
			t2V = V4Add(t2V, V4Mul(extents_YZX_V, V4LoadA_Safe(&params->mPreca1b_PaddedAligned.x)));
			t2V = V4Add(t2V, V4Mul(extents_ZXY_V, V4LoadA_Safe(&params->mPreca2b_PaddedAligned.x)));
			t2V = V4Add(t2V, V4LoadA_Safe(&params->mBoxExtents_PaddedAligned.x));
		}

		{
			__m128 abstV = _mm_and_ps(tV, _mm_load1_ps(reinterpret_cast<const float*>(&maskI)));
			abstV = _mm_cmpgt_ps(abstV, t2V);
			const PxU32 test = PxU32(_mm_movemask_ps(abstV));
			if(test&7)
				return 0;
		}
		return 1;
	}
#endif	// GU_BV4_USE_SLABS
#endif	// PX_INTEL_FAMILY

#endif	// GU_BV4_BOX_BOX_OVERLAP_TEST_H
