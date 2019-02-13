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


#include "PxConvexMeshExt.h"
#include "PxConvexMeshGeometry.h"
#include "PxConvexMesh.h"
#include "foundation/PxTransform.h"

using namespace physx;

static const PxReal gEpsilon = .01f;

PxU32 physx::PxFindFaceIndex(const PxConvexMeshGeometry& convexGeom, const PxTransform& pose, 
	const PxVec3& impactPos, const PxVec3& unitDir)
{
	PX_ASSERT(unitDir.isFinite());
	PX_ASSERT(unitDir.isNormalized());
	PX_ASSERT(impactPos.isFinite());
	PX_ASSERT(pose.isFinite());

	const PxVec3 impact = impactPos - unitDir * gEpsilon;

	const PxVec3 localPoint = pose.transformInv(impact);
	const PxVec3 localDir = pose.rotateInv(unitDir);

	// Create shape to vertex scale transformation matrix
	const PxMeshScale& meshScale = convexGeom.scale;
	const PxMat33 rot(meshScale.rotation);
	PxMat33 shape2VertexSkew = rot.getTranspose();
	const PxMat33 diagonal = PxMat33::createDiagonal(PxVec3(1.0f / meshScale.scale.x, 1.0f / meshScale.scale.y, 1.0f / meshScale.scale.z));
	shape2VertexSkew = shape2VertexSkew * diagonal;
	shape2VertexSkew = shape2VertexSkew * rot;

	const PxU32 nbPolys = convexGeom.convexMesh->getNbPolygons();
	PxU32 minIndex = 0;
	PxReal minD = PX_MAX_REAL;
	for (PxU32 j = 0; j < nbPolys; j++)
	{
		PxHullPolygon hullPolygon;
		convexGeom.convexMesh->getPolygonData(j, hullPolygon);
		
		// transform hull plane into shape space
		PxPlane plane;
		const PxVec3 tmp = shape2VertexSkew.transformTranspose(PxVec3(hullPolygon.mPlane[0],hullPolygon.mPlane[1],hullPolygon.mPlane[2]));
		const PxReal denom = 1.0f / tmp.magnitude();
		plane.n = tmp * denom;
		plane.d = hullPolygon.mPlane[3] * denom;

		PxReal d = plane.distance(localPoint);
		if (d < 0.0f)
			continue;

		const PxReal tweak = plane.n.dot(localDir) * gEpsilon;
		d += tweak;

		if (d < minD)
		{
			minIndex = j;
			minD = d;
		}
	}
	return minIndex;
}

