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

#include "GuIntersectionSphereBox.h"
#include "GuSphere.h"
#include "GuBox.h"

using namespace physx;

bool Gu::intersectSphereBox(const Sphere& sphere, const Box& box)
{
	const PxVec3 delta = sphere.center - box.center;
	PxVec3 dRot = box.rot.transformTranspose(delta);	//transform delta into OBB body coords. (use method call!)

	//check if delta is outside AABB - and clip the vector to the AABB.
	bool outside = false;

	if(dRot.x < -box.extents.x)
	{ 
		outside = true; 
		dRot.x = -box.extents.x;
	}
	else if(dRot.x >  box.extents.x)
	{ 
		outside = true; 
		dRot.x = box.extents.x;
	}

	if(dRot.y < -box.extents.y)
	{ 
		outside = true; 
		dRot.y = -box.extents.y;
	}
	else if(dRot.y >  box.extents.y)
	{ 
		outside = true; 
		dRot.y = box.extents.y;
	}

	if(dRot.z < -box.extents.z)
	{ 
		outside = true; 
		dRot.z = -box.extents.z;
	}
	else if(dRot.z >  box.extents.z)
	{ 
		outside = true; 
		dRot.z = box.extents.z;
	}

	if(outside)	//if clipping was done, sphere center is outside of box.
	{
		const PxVec3 clippedDelta = box.rot.transform(dRot);	//get clipped delta back in world coords.

		const PxVec3 clippedVec = delta - clippedDelta;			  //what we clipped away.	
		const PxReal lenSquared = clippedVec.magnitudeSquared();
		const PxReal radius = sphere.radius;
		if(lenSquared > radius * radius)	// PT: objects are defined as closed, so we return 'true' in case of equality
			return false;	//disjoint
	}
	return true;
}
