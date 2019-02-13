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

#ifndef GU_TRIANGLE_VERTEX_POINTERS_H
#define GU_TRIANGLE_VERTEX_POINTERS_H

#include "PxTriangleMesh.h"
#include "GuTriangleMesh.h"

namespace physx {
	namespace Gu {

	// PT: TODO: replace with Gu::TriangleMesh::getLocalTriangle(...)
	struct TriangleVertexPointers
	{
		static void PX_FORCE_INLINE getTriangleVerts(const TriangleMesh* mesh, PxU32 triangleIndex, PxVec3& v0, PxVec3& v1, PxVec3& v2)
		{
			const PxVec3* verts = mesh->getVerticesFast();
			if(mesh->has16BitIndices())
			{
				const PxU16* tris = reinterpret_cast<const PxU16*>(mesh->getTrianglesFast());
				const PxU16* inds = tris+triangleIndex*3;
				v0 = verts[inds[0]];
				v1 = verts[inds[1]];
				v2 = verts[inds[2]];
			} 
			else 
			{ 
				const PxU32* tris = reinterpret_cast<const PxU32*>(mesh->getTrianglesFast());
				const PxU32* inds = tris+triangleIndex*3;
				v0 = verts[inds[0]];
				v1 = verts[inds[1]];
				v2 = verts[inds[2]];
			} 
		}
	};
} } // physx, Gu

#endif
