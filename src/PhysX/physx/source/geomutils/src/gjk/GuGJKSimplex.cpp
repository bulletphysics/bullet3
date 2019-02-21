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

#include "GuGJKSimplex.h"

namespace physx
{
namespace Gu
{

	using namespace Ps::aos;

	static Vec3V getClosestPtPointTriangle(Vec3V* PX_RESTRICT Q, const BoolVArg bIsOutside4, PxU32* indices, PxU32& size)
	{
		FloatV bestSqDist = FMax();
		
		PxU32 _indices[3] = {0, 1, 2};

		Vec3V closestPt = V3Zero();
	
		if(BAllEqTTTT(BGetX(bIsOutside4)))
		{
			//use the original indices, size, v and w
			bestSqDist = closestPtPointTriangleBaryCentric(Q[0], Q[1], Q[2], indices, size, closestPt);
		}

		if(BAllEqTTTT(BGetY(bIsOutside4)))
		{

			PxU32 _size = 3;
			_indices[0] = 0; _indices[1] = 2; _indices[2] = 3; 
			Vec3V tClosestPt;
			const FloatV sqDist = closestPtPointTriangleBaryCentric(Q[0], Q[2], Q[3], _indices, _size, tClosestPt);

			const BoolV con = FIsGrtr(bestSqDist, sqDist);
			if(BAllEqTTTT(con))
			{
				closestPt = tClosestPt;
				bestSqDist = sqDist;

				indices[0] = _indices[0];
				indices[1] = _indices[1];
				indices[2] = _indices[2];

				size = _size;
			}
		}

		if(BAllEqTTTT(BGetZ(bIsOutside4)))
		{
			PxU32 _size = 3;
			
			_indices[0] = 0; _indices[1] = 3; _indices[2] = 1; 

			Vec3V tClosestPt;
			const FloatV sqDist = closestPtPointTriangleBaryCentric(Q[0], Q[3], Q[1], _indices, _size, tClosestPt);
			
			const BoolV con = FIsGrtr(bestSqDist, sqDist);
			if(BAllEqTTTT(con))
			{
				closestPt = tClosestPt;
				bestSqDist = sqDist;

				indices[0] = _indices[0];
				indices[1] = _indices[1];
				indices[2] = _indices[2];

				size = _size;
			}

		}

		if(BAllEqTTTT(BGetW(bIsOutside4)))
		{
	

			PxU32 _size = 3;
			_indices[0] = 1; _indices[1] = 3; _indices[2] = 2; 
			Vec3V tClosestPt;
			const FloatV sqDist = closestPtPointTriangleBaryCentric(Q[1], Q[3], Q[2], _indices, _size, tClosestPt);

			const BoolV con = FIsGrtr(bestSqDist, sqDist);

			if(BAllEqTTTT(con))
			{
				closestPt = tClosestPt;
				bestSqDist = sqDist;

				indices[0] = _indices[0];
				indices[1] = _indices[1];
				indices[2] = _indices[2];

				size = _size;
			}
		}

		return closestPt;
	}

	PX_NOALIAS Vec3V closestPtPointTetrahedron(Vec3V* PX_RESTRICT Q, Vec3V* PX_RESTRICT A, Vec3V* PX_RESTRICT B, PxU32& size)
	{
		
		const FloatV eps = FLoad(1e-4f);
		const Vec3V a = Q[0];
		const Vec3V b = Q[1];
		const Vec3V c = Q[2];  
		const Vec3V d = Q[3];

		//degenerated
		const Vec3V ab = V3Sub(b, a);
		const Vec3V ac = V3Sub(c, a);
		const Vec3V n = V3Normalize(V3Cross(ab, ac));
		const FloatV signDist = V3Dot(n, V3Sub(d, a));
		if(FAllGrtr(eps, FAbs(signDist)))
		{
			size = 3;
			return closestPtPointTriangle(Q, A, B, size);
		}

		const BoolV bIsOutside4 = PointOutsideOfPlane4(a, b, c, d);

		if(BAllEqFFFF(bIsOutside4))
		{
			//All inside
			return V3Zero();
		}

		PxU32 indices[3] = {0, 1, 2};
		
		const Vec3V closest = getClosestPtPointTriangle(Q, bIsOutside4, indices, size);

		const Vec3V q0 = Q[indices[0]]; const Vec3V q1 = Q[indices[1]]; const Vec3V q2 = Q[indices[2]];
		const Vec3V a0 = A[indices[0]]; const Vec3V a1 = A[indices[1]]; const Vec3V a2 = A[indices[2]];
		const Vec3V b0 = B[indices[0]]; const Vec3V b1 = B[indices[1]]; const Vec3V b2 = B[indices[2]];
		Q[0] = q0; Q[1] = q1; Q[2] = q2;
		A[0] = a0; A[1] = a1; A[2] = a2;
		B[0] = b0; B[1] = b1; B[2] = b2; 

		return closest;
	}

	PX_NOALIAS Vec3V closestPtPointTetrahedron(Vec3V* PX_RESTRICT Q, Vec3V* PX_RESTRICT A, Vec3V* PX_RESTRICT B, PxI32* PX_RESTRICT aInd,  PxI32* PX_RESTRICT bInd, PxU32& size)
	{
		
		const FloatV eps = FLoad(1e-4f);
		const Vec3V zeroV = V3Zero();
		
		const Vec3V a = Q[0];
		const Vec3V b = Q[1];
		const Vec3V c = Q[2];
		const Vec3V d = Q[3];

		//degenerated
		const Vec3V ab = V3Sub(b, a);
		const Vec3V ac = V3Sub(c, a);
		const Vec3V n = V3Normalize(V3Cross(ab, ac));
		const FloatV signDist = V3Dot(n, V3Sub(d, a));
		if(FAllGrtr(eps, FAbs(signDist)))
		{
			size = 3;
			return closestPtPointTriangle(Q, A, B, aInd, bInd, size);
		}

		const BoolV bIsOutside4 = PointOutsideOfPlane4(a, b, c, d);

		if(BAllEqFFFF(bIsOutside4))
		{
			//All inside
			return zeroV;
		}

		PxU32 indices[3] = {0, 1, 2};
		const Vec3V closest = getClosestPtPointTriangle(Q, bIsOutside4, indices, size);

		const Vec3V q0 = Q[indices[0]]; const Vec3V q1 = Q[indices[1]]; const Vec3V q2 = Q[indices[2]];
		const Vec3V a0 = A[indices[0]]; const Vec3V a1 = A[indices[1]]; const Vec3V a2 = A[indices[2]];
		const Vec3V b0 = B[indices[0]]; const Vec3V b1 = B[indices[1]]; const Vec3V b2 = B[indices[2]];
		const PxI32 _aInd0 = aInd[indices[0]]; const PxI32 _aInd1 = aInd[indices[1]]; const PxI32 _aInd2 = aInd[indices[2]];
		const PxI32 _bInd0 = bInd[indices[0]]; const PxI32 _bInd1 = bInd[indices[1]]; const PxI32 _bInd2 = bInd[indices[2]];
		Q[0] = q0; Q[1] = q1; Q[2] = q2;
		A[0] = a0; A[1] = a1; A[2] = a2;
		B[0] = b0; B[1] = b1; B[2] = b2; 
		aInd[0] = _aInd0; aInd[1] = _aInd1; aInd[2] = _aInd2;
		bInd[0] = _bInd0; bInd[1] = _bInd1; bInd[2] = _bInd2;

		return closest;
	}
}

}
