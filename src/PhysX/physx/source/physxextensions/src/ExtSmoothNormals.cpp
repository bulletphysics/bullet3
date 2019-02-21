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


#include "foundation/PxMemory.h"
#include "PxSmoothNormals.h"
#include "PsMathUtils.h"
#include "PsUserAllocated.h"
#include "PsUtilities.h"
#include "CmPhysXCommon.h"

using namespace physx;

static PxReal computeAngle(const PxVec3* verts, const PxU32* refs, PxU32 vref)
{
	PxU32 e0=0,e2=0;
	if(vref==refs[0])
	{
		e0 = 2;
		e2 = 1;
	}
	else if(vref==refs[1])
	{
		e0 = 2;
		e2 = 0;
	}
	else if(vref==refs[2])
	{
		e0 = 0;
		e2 = 1;
	}
	else
	{
		PX_ASSERT(0);
	}
	const PxVec3 edge0 = verts[refs[e0]] - verts[vref];
	const PxVec3 edge1 = verts[refs[e2]] - verts[vref];

	return Ps::angle(edge0, edge1);
}

bool PxBuildSmoothNormals(PxU32 nbTris, PxU32 nbVerts, const PxVec3* verts, const PxU32* dFaces, const PxU16* wFaces, PxVec3* normals, bool flip)
{
	if(!verts || !normals || !nbTris || !nbVerts)	
		return false;

	// Get correct destination buffers
	// - if available, write directly to user-provided buffers
	// - else get some ram and keep track of it
	PxVec3* FNormals = reinterpret_cast<PxVec3*>(PX_ALLOC_TEMP(sizeof(PxVec3)*nbTris, "PxVec3"));
	if(!FNormals) return false;

	// Compute face normals
	const PxU32 c = PxU32(flip!=0);
	for(PxU32 i=0; i<nbTris; i++)
	{
		// compute indices outside of array index to workaround
		// SNC bug which was generating incorrect addresses
		const PxU32 i0 = i*3+0;
		const PxU32 i1 = i*3+1+c;
		const PxU32 i2 = i*3+2-c;

		const PxU32 Ref0 = dFaces ? dFaces[i0] : wFaces ? wFaces[i0] : 0;
		const PxU32 Ref1 = dFaces ? dFaces[i1] : wFaces ? wFaces[i1] : 1;
		const PxU32 Ref2 = dFaces ? dFaces[i2] : wFaces ? wFaces[i2] : 2;

		FNormals[i] = (verts[Ref2]-verts[Ref0]).cross(verts[Ref1] - verts[Ref0]);
		PX_ASSERT(!FNormals[i].isZero());
		FNormals[i].normalize();
	}

	// Compute vertex normals
	PxMemSet(normals, 0, nbVerts*sizeof(PxVec3));

	// TTP 3751
	PxVec3* TmpNormals = reinterpret_cast<PxVec3*>(PX_ALLOC_TEMP(sizeof(PxVec3)*nbVerts, "PxVec3"));
	PxMemSet(TmpNormals, 0, nbVerts*sizeof(PxVec3));
	for(PxU32 i=0;i<nbTris;i++)
	{
		PxU32 Ref[3];
		Ref[0] = dFaces ? dFaces[i*3+0] : wFaces ? wFaces[i*3+0] : 0;
		Ref[1] = dFaces ? dFaces[i*3+1] : wFaces ? wFaces[i*3+1] : 1;
		Ref[2] = dFaces ? dFaces[i*3+2] : wFaces ? wFaces[i*3+2] : 2;

		for(PxU32 j=0;j<3;j++)
		{
			if(TmpNormals[Ref[j]].isZero())
				TmpNormals[Ref[j]] = FNormals[i];
		}
	}
	//~TTP 3751

	for(PxU32 i=0;i<nbTris;i++)
	{
		PxU32 Ref[3];
		Ref[0] = dFaces ? dFaces[i*3+0] : wFaces ? wFaces[i*3+0] : 0;
		Ref[1] = dFaces ? dFaces[i*3+1] : wFaces ? wFaces[i*3+1] : 1;
		Ref[2] = dFaces ? dFaces[i*3+2] : wFaces ? wFaces[i*3+2] : 2;

		normals[Ref[0]] += FNormals[i] * computeAngle(verts, Ref, Ref[0]);
		normals[Ref[1]] += FNormals[i] * computeAngle(verts, Ref, Ref[1]);
		normals[Ref[2]] += FNormals[i] * computeAngle(verts, Ref, Ref[2]);
	}

	// Normalize vertex normals
	for(PxU32 i=0;i<nbVerts;i++)
	{
		if(normals[i].isZero())
			normals[i] = TmpNormals[i];
//		PX_ASSERT(!normals[i].isZero());
		normals[i].normalize();
	}

	PX_FREE_AND_RESET(TmpNormals);	// TTP 3751
	PX_FREE_AND_RESET(FNormals);

	return true;
}

