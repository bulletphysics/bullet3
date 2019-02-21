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

#include "MeshBuilder.h"
#include "GuInternal.h"

namespace physx
{
	//#define PROFILE_BOUNDS
#ifdef PROFILE_BOUNDS
#include <windows.h>
#pragma comment(lib, "winmm.lib")
#endif

	void MeshBulider::computeLocalBounds(Gu::MeshDataBase&  meshData)
	{
#ifdef PROFILE_BOUNDS
		int time = timeGetTime();
#endif

		PxBounds3& localBounds = meshData.mAABB;
		Gu::computeBoundsAroundVertices(localBounds, meshData.mNbVertices, meshData.mVertices);

		// Derive a good geometric epsilon from local bounds. We must do this before bounds extrusion for heightfields.
		//
		// From Charles Bloom:
		// "Epsilon must be big enough so that the consistency condition abs(D(Hit))
		// <= Epsilon is satisfied for all queries. You want the smallest epsilon
		// you can have that meets that constraint. Normal floats have a 24 bit
		// mantissa. When you do any float addition, you may have round-off error
		// that makes the result off by roughly 2^-24 * result. Our result is
		// scaled by the position values. If our world is strictly required to be
		// in a box of world size W (each coordinate in -W to W), then the maximum
		// error is 2^-24 * W. Thus Epsilon must be at least >= 2^-24 * W. If
		// you're doing coordinate transforms, that may scale your error up by some
		// amount, so you'll need a bigger epsilon. In general something like
		// 2^-22*W is reasonable. If you allow scaled transforms, it needs to be
		// something like 2^-22*W*MAX_SCALE."
		// PT: TODO: runtime checkings for this
		PxReal geomEpsilon = 0.0f;
		for (PxU32 i = 0; i < 3; i++)
			geomEpsilon = PxMax(geomEpsilon, PxMax(PxAbs(localBounds.maximum[i]), PxAbs(localBounds.minimum[i])));
		geomEpsilon *= powf(2.0f, -22.0f);
		meshData.mGeomEpsilon = geomEpsilon;

#ifdef PROFILE_BOUNDS
		int deltaTime = timeGetTime() - time;
		printf("Bounds time: %f\n", float(deltaTime)*0.001f);
#endif
	}

}
