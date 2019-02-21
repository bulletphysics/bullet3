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

#include "GuConvexMeshData.h"
#include "GuContactMethodImpl.h"
#include "GuContactBuffer.h"
#include "GuGeometryUnion.h"
#include "CmScaling.h"


namespace physx
{
namespace Gu
{
bool contactPlaneConvex(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);
	PX_UNUSED(cache);
	PX_UNUSED(shape0);

	// Get actual shape data
	//const PxPlaneGeometry& shapePlane = shape.get<const PxPlaneGeometry>();
	const PxConvexMeshGeometryLL& shapeConvex = shape1.get<const PxConvexMeshGeometryLL>();

	const PxVec3* PX_RESTRICT hullVertices = shapeConvex.hullData->getHullVertices();
	PxU32 numHullVertices = shapeConvex.hullData->mNbHullVertices;
//	Ps::prefetch128(hullVertices);

	// Plane is implicitly <1,0,0> 0 in localspace
	Cm::Matrix34 convexToPlane (transform0.transformInv(transform1));
	PxMat33 convexToPlane_rot(convexToPlane[0], convexToPlane[1], convexToPlane[2] );

	bool idtScale = shapeConvex.scale.isIdentity();
	Cm::FastVertex2ShapeScaling convexScaling;	// PT: TODO: remove default ctor
	if(!idtScale)
		convexScaling.init(shapeConvex.scale);

	convexToPlane = Cm::Matrix34( convexToPlane_rot * convexScaling.getVertex2ShapeSkew(), convexToPlane[3] );

	//convexToPlane = context.mVertex2ShapeSkew[1].getVertex2WorldSkew(convexToPlane);

	const Cm::Matrix34 planeToW (transform0);

	// This is rather brute-force
	
	bool status = false;

	const PxVec3 contactNormal = -planeToW.m.column0;

	while(numHullVertices--)
	{
		const PxVec3& vertex = *hullVertices++;
//		if(numHullVertices)
//			Ps::prefetch128(hullVertices);

		const PxVec3 pointInPlane = convexToPlane.transform(vertex);		//TODO: this multiply could be factored out!
		if(pointInPlane.x <= params.mContactDistance)
		{
//			const PxVec3 pointInW = planeToW.transform(pointInPlane);
//			contactBuffer.contact(pointInW, -planeToW.m.column0, pointInPlane.x);
			status = true;
			Gu::ContactPoint* PX_RESTRICT pt = contactBuffer.contact();
			if(pt)
			{
				pt->normal				= contactNormal;
				pt->point				= planeToW.transform(pointInPlane);
				pt->separation			= pointInPlane.x;
				pt->internalFaceIndex1	= PXC_CONTACT_NO_FACE_INDEX;
			}
		}
	}
	return status;
}
}//Gu
}//physx
