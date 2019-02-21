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


#ifndef PX_PHYSICS_SC_PHYSICS
#define PX_PHYSICS_SC_PHYSICS

#include "PxPhysics.h"
#include "PxScene.h"
#include "PsUserAllocated.h"
#include "CmPhysXCommon.h"
#include "PsBasicTemplates.h"
#include "PxActor.h"

namespace physx
{

class PxMaterial;
class PxTolerancesScale;
struct PxvOffsetTable;

#if PX_SUPPORT_GPU_PHYSX
class PxPhysXGpu;
#endif

namespace Sc
{
	class Scene;
	class StaticCore;
	class RigidCore;
	class BodyCore;
	class ArticulationCore;
	class ConstraintCore;
	class ShapeCore;

	struct OffsetTable
	{
		PX_FORCE_INLINE OffsetTable() {}

		PX_FORCE_INLINE PxActor*				convertScRigidStatic2PxActor(StaticCore* sc)					const	{ return Ps::pointerOffset<PxActor*>(sc, scRigidStatic2PxActor);					}
		PX_FORCE_INLINE PxActor*				convertScRigidDynamic2PxActor(BodyCore* sc)						const	{ return Ps::pointerOffset<PxActor*>(sc, scRigidDynamic2PxActor);					}
		PX_FORCE_INLINE PxActor*				convertScArticulationLink2PxActor(BodyCore* sc)					const	{ return Ps::pointerOffset<PxActor*>(sc, scArticulationLink2PxActor);				}

		PX_FORCE_INLINE PxShape*				convertScShape2Px(ShapeCore* sc)								const	{ return Ps::pointerOffset<PxShape*>(sc, scShape2Px);								}
		PX_FORCE_INLINE const PxShape*			convertScShape2Px(const ShapeCore* sc)							const	{ return Ps::pointerOffset<const PxShape*>(sc, scShape2Px);							}

		PX_FORCE_INLINE PxArticulation*			convertScArticulation2Px(ArticulationCore* sc)					const	{ return Ps::pointerOffset<PxArticulation*>(sc, scArticulation2Px);					}
		PX_FORCE_INLINE const PxArticulation*	convertScArticulation2Px(const ArticulationCore* sc)			const	{ return Ps::pointerOffset<const PxArticulation*>(sc, scArticulation2Px);			}

		PX_FORCE_INLINE PxConstraint*			convertScConstraint2Px(ConstraintCore* sc)						const	{ return Ps::pointerOffset<PxConstraint*>(sc, scConstraint2Px);						}
		PX_FORCE_INLINE const PxConstraint*		convertScConstraint2Px(const ConstraintCore* sc)				const	{ return Ps::pointerOffset<const PxConstraint*>(sc, scConstraint2Px);				}

		ptrdiff_t	scRigidStatic2PxActor;
		ptrdiff_t 	scRigidDynamic2PxActor;
		ptrdiff_t 	scArticulationLink2PxActor;
		ptrdiff_t 	scShape2Px;
		ptrdiff_t 	scArticulation2Px;
		ptrdiff_t 	scSoftBody2Px;
		ptrdiff_t 	scConstraint2Px;

		ptrdiff_t	scCore2PxActor[PxActorType::eACTOR_COUNT];
	};
	extern OffsetTable gOffsetTable;

	class Physics : public Ps::UserAllocated
	{
	public:
		PX_FORCE_INLINE static Physics&				getInstance()						{ return *mInstance; }

													Physics(const PxTolerancesScale&, const PxvOffsetTable& pxvOffsetTable);
													~Physics(); // use release() instead
	public:
						void						release();

		PX_FORCE_INLINE	const PxTolerancesScale&	getTolerancesScale()		const	{ return mScale;	}

	private:
						PxTolerancesScale			mScale;
		static			Physics*					mInstance;

	public:
		static			const PxReal				sWakeCounterOnCreation;
	};

} // namespace Sc

}

#endif
