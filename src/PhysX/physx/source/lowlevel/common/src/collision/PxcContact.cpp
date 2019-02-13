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

#include "PxcContactMethodImpl.h"

namespace physx
{

////////////////////////////////////////////////////////////////////////////////////////////////////////
//non-pcm sphere function
bool PxcContactSphereSphere(GU_CONTACT_METHOD_ARGS)
{
	return contactSphereSphere(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcContactSphereCapsule(GU_CONTACT_METHOD_ARGS)
{
	return contactSphereCapsule(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcContactSphereBox(GU_CONTACT_METHOD_ARGS)
{
	return contactSphereBox(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcContactSpherePlane(GU_CONTACT_METHOD_ARGS)
{
	return contactSpherePlane(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcContactSphereConvex(GU_CONTACT_METHOD_ARGS)
{
	return contactCapsuleConvex(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcContactSphereHeightField(GU_CONTACT_METHOD_ARGS)
{
	return contactSphereHeightfield(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcContactSphereMesh(GU_CONTACT_METHOD_ARGS)
{
	return contactSphereMesh(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

//non-pcm plane functions
bool PxcContactPlaneBox(GU_CONTACT_METHOD_ARGS)
{
	return contactPlaneBox(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcContactPlaneCapsule(GU_CONTACT_METHOD_ARGS)
{
	return contactPlaneCapsule(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcContactPlaneConvex(GU_CONTACT_METHOD_ARGS)
{
	return contactPlaneConvex(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

//non-pcm capsule funtions
bool PxcContactCapsuleCapsule(GU_CONTACT_METHOD_ARGS)
{
	return contactCapsuleCapsule(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcContactCapsuleBox(GU_CONTACT_METHOD_ARGS)
{
	return contactCapsuleBox(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcContactCapsuleConvex(GU_CONTACT_METHOD_ARGS)
{
	return contactCapsuleConvex(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcContactCapsuleHeightField(GU_CONTACT_METHOD_ARGS)
{
	return contactCapsuleHeightfield(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcContactCapsuleMesh(GU_CONTACT_METHOD_ARGS)
{
	return contactCapsuleMesh(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

//non-pcm box functions
bool PxcContactBoxBox(GU_CONTACT_METHOD_ARGS)
{
	return contactBoxBox(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcContactBoxConvex(GU_CONTACT_METHOD_ARGS)
{
	return contactBoxConvex(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcContactBoxHeightField(GU_CONTACT_METHOD_ARGS)
{
	return contactBoxHeightfield(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcContactBoxMesh(GU_CONTACT_METHOD_ARGS)
{
	return contactBoxMesh(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

//non-pcm convex functions
bool PxcContactConvexConvex(GU_CONTACT_METHOD_ARGS)
{
	return contactConvexConvex(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcContactConvexHeightField(GU_CONTACT_METHOD_ARGS)
{
	return contactConvexHeightfield(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcContactConvexMesh(GU_CONTACT_METHOD_ARGS)
{
	return contactConvexMesh(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//pcm sphere functions
bool PxcPCMContactSphereSphere(GU_CONTACT_METHOD_ARGS)
{
	return pcmContactSphereSphere(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcPCMContactSpherePlane(GU_CONTACT_METHOD_ARGS)
{
	return pcmContactSpherePlane(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcPCMContactSphereCapsule(GU_CONTACT_METHOD_ARGS)
{
	return pcmContactSphereCapsule(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcPCMContactSphereBox(GU_CONTACT_METHOD_ARGS)
{
	return pcmContactSphereBox(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcPCMContactSphereConvex(GU_CONTACT_METHOD_ARGS)
{
	return pcmContactSphereConvex(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcPCMContactSphereHeightField(GU_CONTACT_METHOD_ARGS)
{
	return pcmContactSphereHeightField(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcPCMContactSphereMesh(GU_CONTACT_METHOD_ARGS)
{
	return pcmContactSphereMesh(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

//pcm plane functions
bool PxcPCMContactPlaneBox(GU_CONTACT_METHOD_ARGS)
{
	return pcmContactPlaneBox(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcPCMContactPlaneCapsule(GU_CONTACT_METHOD_ARGS)
{
	return pcmContactPlaneCapsule(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcPCMContactPlaneConvex(GU_CONTACT_METHOD_ARGS)
{
	return pcmContactPlaneConvex(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

//pcm capsule functions
bool PxcPCMContactCapsuleCapsule(GU_CONTACT_METHOD_ARGS)
{
	return pcmContactCapsuleCapsule(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcPCMContactCapsuleBox(GU_CONTACT_METHOD_ARGS)
{
	return pcmContactCapsuleBox(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcPCMContactCapsuleConvex(GU_CONTACT_METHOD_ARGS)
{
	return pcmContactCapsuleConvex(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcPCMContactCapsuleHeightField(GU_CONTACT_METHOD_ARGS)
{
	return pcmContactCapsuleHeightField(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcPCMContactCapsuleMesh(GU_CONTACT_METHOD_ARGS)
{
	return pcmContactCapsuleMesh(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

//pcm box functions
bool PxcPCMContactBoxBox(GU_CONTACT_METHOD_ARGS)
{
	return pcmContactBoxBox(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcPCMContactBoxConvex(GU_CONTACT_METHOD_ARGS)
{
	return pcmContactBoxConvex(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcPCMContactBoxHeightField(GU_CONTACT_METHOD_ARGS)
{
	return pcmContactBoxHeightField(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcPCMContactBoxMesh(GU_CONTACT_METHOD_ARGS)
{
	return pcmContactBoxMesh(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

//pcm convex functions
bool PxcPCMContactConvexConvex(GU_CONTACT_METHOD_ARGS)
{
	return pcmContactConvexConvex(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcPCMContactConvexHeightField(GU_CONTACT_METHOD_ARGS)
{
	return pcmContactConvexHeightField(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

bool PxcPCMContactConvexMesh(GU_CONTACT_METHOD_ARGS)
{
	return pcmContactConvexMesh(shape0, shape1, transform0, transform1, params, cache, contactBuffer, renderOutput);
}

}
