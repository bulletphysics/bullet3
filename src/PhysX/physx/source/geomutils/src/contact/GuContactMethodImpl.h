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

#ifndef GU_CONTACTMETHODIMPL_H
#define GU_CONTACTMETHODIMPL_H

#include "foundation/PxAssert.h"
#include "PxPhysXCommonConfig.h"
#include "CmPhysXCommon.h"
#include "collision/PxCollisionDefs.h"

namespace physx
{
namespace Cm
{
	class RenderOutput;
}

namespace Gu
{
	class GeometryUnion;
	class ContactBuffer;
	struct NarrowPhaseParams;
	class PersistentContactManifold;
	class MultiplePersistentContactManifold;

	enum ManifoldFlags
	{
		IS_MANIFOLD			= (1<<0),
		IS_MULTI_MANIFOLD	= (1<<1)
	};

	struct Cache : public PxCache
	{
		Cache()
		{
		}

		PX_FORCE_INLINE void setManifold(void* manifold)
		{
			PX_ASSERT((size_t(manifold) & 0xF) == 0);
			mCachedData = reinterpret_cast<PxU8*>(manifold);
			mManifoldFlags |= IS_MANIFOLD;
		}

		PX_FORCE_INLINE void setMultiManifold(void* manifold)
		{
			PX_ASSERT((size_t(manifold) & 0xF) == 0);
			mCachedData = reinterpret_cast<PxU8*>(manifold);
			mManifoldFlags |= IS_MANIFOLD|IS_MULTI_MANIFOLD;
		}

		PX_FORCE_INLINE PxU8 isManifold()	const
		{
			return PxU8(mManifoldFlags & IS_MANIFOLD);
		}

		PX_FORCE_INLINE PxU8 isMultiManifold()	const
		{
			return PxU8(mManifoldFlags & IS_MULTI_MANIFOLD);
		}

		PX_FORCE_INLINE PersistentContactManifold& getManifold()
		{
			PX_ASSERT(isManifold());
			PX_ASSERT(!isMultiManifold());
			PX_ASSERT((uintptr_t(mCachedData) & 0xf) == 0);
			return *reinterpret_cast<PersistentContactManifold*>(mCachedData);
		}

		PX_FORCE_INLINE MultiplePersistentContactManifold& getMultipleManifold()
		{
			PX_ASSERT(isManifold());
			PX_ASSERT(isMultiManifold());
			PX_ASSERT((uintptr_t(mCachedData) & 0xf) == 0);
			return *reinterpret_cast<MultiplePersistentContactManifold*>(mCachedData);
		}
	};
}

#define GU_CONTACT_METHOD_ARGS				\
	const Gu::GeometryUnion& shape0,		\
	const Gu::GeometryUnion& shape1,		\
	const PxTransform& transform0,			\
	const PxTransform& transform1,			\
	const Gu::NarrowPhaseParams& params,	\
	Gu::Cache& cache,						\
	Gu::ContactBuffer& contactBuffer,		\
	Cm::RenderOutput* renderOutput

namespace Gu
{
	PX_PHYSX_COMMON_API bool contactSphereSphere(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool contactSphereCapsule(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool contactSphereBox(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool contactCapsuleCapsule(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool contactCapsuleBox(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool contactCapsuleConvex(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool contactBoxBox(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool contactBoxConvex(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool contactConvexConvex(GU_CONTACT_METHOD_ARGS);
	
	PX_PHYSX_COMMON_API bool contactSphereMesh(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool contactCapsuleMesh(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool contactBoxMesh(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool contactConvexMesh(GU_CONTACT_METHOD_ARGS);

	PX_PHYSX_COMMON_API bool contactSphereHeightfield(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool contactCapsuleHeightfield(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool contactBoxHeightfield(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool contactConvexHeightfield(GU_CONTACT_METHOD_ARGS);
	
	PX_PHYSX_COMMON_API bool contactSpherePlane(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool contactPlaneBox(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool contactPlaneCapsule(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool contactPlaneConvex(GU_CONTACT_METHOD_ARGS);

	PX_PHYSX_COMMON_API bool pcmContactSphereMesh(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool pcmContactCapsuleMesh(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool pcmContactBoxMesh(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool pcmContactConvexMesh(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool pcmContactSphereHeightField(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool pcmContactCapsuleHeightField(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool pcmContactBoxHeightField(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool pcmContactConvexHeightField(GU_CONTACT_METHOD_ARGS);

	PX_PHYSX_COMMON_API bool pcmContactPlaneCapsule(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool pcmContactPlaneBox(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool pcmContactPlaneConvex(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool pcmContactSphereSphere(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool pcmContactSpherePlane(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool pcmContactSphereCapsule(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool pcmContactSphereBox(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool pcmContactSphereConvex(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool pcmContactCapsuleCapsule(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool pcmContactCapsuleBox(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool pcmContactCapsuleConvex(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool pcmContactBoxBox(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool pcmContactBoxConvex(GU_CONTACT_METHOD_ARGS);
	PX_PHYSX_COMMON_API bool pcmContactConvexConvex(GU_CONTACT_METHOD_ARGS);
}
}

#endif
