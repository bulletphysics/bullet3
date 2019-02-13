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

#include "GuContactMethodImpl.h"
#include "GuContactBuffer.h"
#include "GuGeometryUnion.h"
#include "CmMatrix34.h"

#include "PsUtilities.h"
#include "foundation/PxUnionCast.h"

namespace physx
{
namespace Gu
{
bool contactPlaneBox(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);
	PX_UNUSED(cache);
	PX_UNUSED(shape0);

	// Get actual shape data      
	//const PxPlaneGeometry& shapePlane = shape.get<const PxPlaneGeometry>();
	const PxBoxGeometry& shapeBox = shape1.get<const PxBoxGeometry>();
	
	const PxVec3 negPlaneNormal = -transform0.q.getBasisVector0();
	
	//Make sure we have a normalized plane
	//PX_ASSERT(PxAbs(shape0.mNormal.magnitudeSquared() - 1.0f) < 0.000001f);

	Cm::Matrix34 boxMatrix(transform1);
	Cm::Matrix34 boxToPlane(transform0.transformInv(transform1));

	PxVec3 point;

	PX_ASSERT(contactBuffer.count==0);

/*	for(int vx=-1; vx<=1; vx+=2)
		for(int vy=-1; vy<=1; vy+=2)
			for(int vz=-1; vz<=1; vz+=2)
			{				
				//point = boxToPlane.transform(PxVec3(shapeBox.halfExtents.x*vx, shapeBox.halfExtents.y*vy, shapeBox.halfExtents.z*vz));	
				//PxReal planeEq = point.x;
				//Optimized a bit
				point.set(shapeBox.halfExtents.x*vx, shapeBox.halfExtents.y*vy, shapeBox.halfExtents.z*vz);
				const PxReal planeEq = boxToPlane.m.column0.x*point.x + boxToPlane.m.column1.x*point.y + boxToPlane.m.column2.x*point.z + boxToPlane.p.x;

				if(planeEq <= contactDistance)
				{
					contactBuffer.contact(boxMatrix.transform(point), negPlaneNormal, planeEq);
					
					//no point in making more than 4 contacts.
					if (contactBuffer.count >= 6) //was: 4)	actually, with strong interpenetration more than just the bottom surface goes through,
						//and we want to find the *deepest* 4 vertices, really.
						return true;
				}
			}*/

	// PT: the above code is shock full of LHS/FCMPs. And there's no point in limiting the number of contacts to 6 when the max possible is 8.

	const PxReal limit = params.mContactDistance - boxToPlane.p.x;
	const PxReal dx = shapeBox.halfExtents.x;
	const PxReal dy = shapeBox.halfExtents.y;
	const PxReal dz = shapeBox.halfExtents.z;
	const PxReal bxdx = boxToPlane.m.column0.x * dx;
	const PxReal bxdy = boxToPlane.m.column1.x * dy;
	const PxReal bxdz = boxToPlane.m.column2.x * dz;

	PxReal depths[8];
	depths[0] =   bxdx + bxdy + bxdz - limit;
	depths[1] =   bxdx + bxdy - bxdz - limit;
	depths[2] =   bxdx - bxdy + bxdz - limit;
	depths[3] =   bxdx - bxdy - bxdz - limit;
	depths[4] = - bxdx + bxdy + bxdz - limit;
	depths[5] = - bxdx + bxdy - bxdz - limit;
	depths[6] = - bxdx - bxdy + bxdz - limit;
	depths[7] = - bxdx - bxdy - bxdz - limit;

	//const PxU32* binary = reinterpret_cast<const PxU32*>(depths);
	const PxU32* binary = PxUnionCast<PxU32*, PxF32*>(depths);

	if(binary[0] & PX_SIGN_BITMASK)
		contactBuffer.contact(boxMatrix.transform(PxVec3(dx, dy, dz)), negPlaneNormal, depths[0] + params.mContactDistance);
	if(binary[1] & PX_SIGN_BITMASK)
		contactBuffer.contact(boxMatrix.transform(PxVec3(dx, dy, -dz)), negPlaneNormal, depths[1] + params.mContactDistance);
	if(binary[2] & PX_SIGN_BITMASK)
		contactBuffer.contact(boxMatrix.transform(PxVec3(dx, -dy, dz)), negPlaneNormal, depths[2] + params.mContactDistance);
	if(binary[3] & PX_SIGN_BITMASK)
		contactBuffer.contact(boxMatrix.transform(PxVec3(dx, -dy, -dz)), negPlaneNormal, depths[3] + params.mContactDistance);
	if(binary[4] & PX_SIGN_BITMASK)
		contactBuffer.contact(boxMatrix.transform(PxVec3(-dx, dy, dz)), negPlaneNormal, depths[4] + params.mContactDistance);
	if(binary[5] & PX_SIGN_BITMASK)
		contactBuffer.contact(boxMatrix.transform(PxVec3(-dx, dy, -dz)), negPlaneNormal, depths[5] + params.mContactDistance);
	if(binary[6] & PX_SIGN_BITMASK)
		contactBuffer.contact(boxMatrix.transform(PxVec3(-dx, -dy, dz)), negPlaneNormal, depths[6] + params.mContactDistance);
	if(binary[7] & PX_SIGN_BITMASK)
		contactBuffer.contact(boxMatrix.transform(PxVec3(-dx, -dy, -dz)), negPlaneNormal, depths[7] + params.mContactDistance);

	return contactBuffer.count > 0;
}
}//Gu
}//physx
