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

#ifndef GU_CUBE_INDEX_H
#define GU_CUBE_INDEX_H

#include "foundation/PxVec3.h"
#include "CmPhysXCommon.h"
#include "PsFPU.h"

namespace physx
{

	enum CubeIndex
	{
		CUBE_RIGHT,
		CUBE_LEFT,
		CUBE_TOP,
		CUBE_BOTTOM,
		CUBE_FRONT,
		CUBE_BACK,

		CUBE_FORCE_DWORD	= 0x7fffffff
	};

	/*
		It's pretty straightforwards in concept (though the execution in hardware is
		a bit crufty and complex). You use a 3D texture coord to look up a texel in
		a cube map. First you find which of the axis has the largest value (i.e.
		X,Y,Z), and then the sign of that axis decides which face you are going to
		use. Which is why the faces are called +X, -X, +Y, -Y, +Z, -Z - after their
		principle axis. Then you scale the vector so that the largest value is +/-1.
		Then use the other two as 2D coords to look up your texel (with a 0.5 scale
		& offset).

		For example, vector (0.4, -0.2, -0.5). Largest value is the Z axis, and it's
		-ve, so we're reading from the -Z map. Scale so that this Z axis is +/-1,
		and you get the vector (0.8, -0.4, -1.0). So now use the other two values to
		look up your texel. So we look up texel (0.8, -0.4). The scale & offset move
		the -1->+1 range into the usual 0->1 UV range, so we actually look up texel
		(0.9, 0.3). The filtering is extremely complex, especially where three maps
		meet, but that's a hardware problem :-)
	*/

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	 *	Cubemap lookup function.
	 *
	 *	To transform returned uvs into mapping coordinates :
	 *	u += 1.0f;	u *= 0.5f;
	 *	v += 1.0f;	v *= 0.5f;
	 *
	 *	\fn			CubemapLookup(const PxVec3& direction, float& u, float& v)
	 *	\param		direction	[in] a direction vector
	 *	\param		u			[out] impact coordinate on the unit cube, in [-1,1]
	 *	\param		v			[out] impact coordinate on the unit cube, in [-1,1]
	 *	\return		cubemap texture index
	 */
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	PX_INLINE CubeIndex		CubemapLookup(const PxVec3& direction, float& u, float& v);

	PX_INLINE PxU32 ComputeCubemapOffset(const PxVec3& dir, PxU32 subdiv)
	{
		float u,v;
		const CubeIndex CI = CubemapLookup(dir, u, v);

		// Remap to [0, subdiv[
		const float Coeff = 0.5f * float(subdiv-1);
		u += 1.0f;	u *= Coeff;
		v += 1.0f;	v *= Coeff;

		// Compute offset
		return PxU32(CI)*(subdiv*subdiv) + PxU32(u)*subdiv + PxU32(v);
	}


	PX_INLINE PxU32 ComputeCubemapNearestOffset(const PxVec3& dir, PxU32 subdiv)
	{
		float u,v;
		const CubeIndex CI = CubemapLookup(dir, u, v);

		// Remap to [0, subdiv]
		const float Coeff = 0.5f * float(subdiv-1);
		u += 1.0f;	u *= Coeff;
		v += 1.0f;	v *= Coeff;

		// Compute offset
		return PxU32(CI)*(subdiv*subdiv) + PxU32(u + 0.5f)*subdiv + PxU32(v + 0.5f);
	}


	PX_INLINE CubeIndex CubemapLookup(const PxVec3& direction, float& u, float& v)
	{
		const PxU32* binary = reinterpret_cast<const PxU32*>(&direction.x);

		const PxU32 absPx = binary[0] & ~PX_SIGN_BITMASK;
		const PxU32 absNy = binary[1] & ~PX_SIGN_BITMASK;
		const PxU32 absNz = binary[2] & ~PX_SIGN_BITMASK;

		PxU32 Index1 = 0;	//x biggest axis
		PxU32 Index2 = 1;
		PxU32 Index3 = 2;
		if( (absNy > absPx) & (absNy > absNz))
		{
			//y biggest
			Index2 = 2;
			Index3 = 0;
			Index1 = 1;
		}
		else if(absNz > absPx)
		{
			//z biggest
			Index2 = 0;
			Index3 = 1;
			Index1 = 2;
		}

		const PxF32* data = &direction.x;
		const float Coeff = 1.0f / fabsf(data[Index1]);
		u = data[Index2] * Coeff;
		v = data[Index3] * Coeff;

		const PxU32 Sign = binary[Index1]>>31;
		return CubeIndex(Sign|(Index1+Index1));
	}

}

#endif
