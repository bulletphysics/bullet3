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


#ifndef PX_PHYSICS_SCP_SHAPESIM
#define PX_PHYSICS_SCP_SHAPESIM

#include "ScElementSim.h"
#include "ScShapeCore.h"
#include "CmPtrTable.h"
#include "ScRigidSim.h"
#include "PxsShapeSim.h"

namespace physx
{

	class PxsTransformCache;
namespace Gu
{
	class TriangleMesh;
	class HeightField;
}

/** Simulation object corresponding to a shape core object. This object is created when
    a ShapeCore object is added to the simulation, and destroyed when it is removed
*/

struct PxsRigidCore;

namespace Sc
{

	class RigidSim;
	class ShapeCore;
	class Scene;
	class BodySim;
	class StaticSim;

	class ShapeSim : public ElementSim
	{
		ShapeSim &operator=(const ShapeSim &);
	public:

		// passing in a pointer for the shape to output its bounds allows us not to have to compute them twice.
		// A neater way to do this would be to ask the AABB Manager for the bounds after the shape has been 
		// constructed, but there is currently no spec for what the AABBMgr is allowed to do with the bounds, 
		// hence better not to assume anything.

														ShapeSim(RigidSim&, const ShapeCore& core);
														~ShapeSim();

						void							reinsertBroadPhase();

		PX_FORCE_INLINE	const ShapeCore&				getCore()				const	{ return mCore; }

						// TODO: compile time coupling

		PX_INLINE		PxGeometryType::Enum			getGeometryType()		const	{ return mCore.getGeometryType();	}

		// This is just for getting a reference for the user, so we cast away const-ness

		PX_INLINE		PxShape*						getPxShape()			const	{ return const_cast<PxShape*>(mCore.getPxShape());	}
		
		PX_FORCE_INLINE	PxReal							getRestOffset()			const	{ return mCore.getRestOffset();		}
		PX_FORCE_INLINE	PxReal							getTorsionalPatchRadius() const { return mCore.getTorsionalPatchRadius(); }
		PX_FORCE_INLINE	PxReal							getMinTorsionalPatchRadius() const { return mCore.getMinTorsionalPatchRadius(); }
		PX_FORCE_INLINE	PxU32							getFlags()				const	{ return mCore.getFlags();			}
		PX_FORCE_INLINE	PxReal							getContactOffset()		const	{ return mCore.getContactOffset();	}

		PX_FORCE_INLINE	RigidSim&						getRbSim()				const	{ return static_cast<RigidSim&>(getActor());	}
						BodySim*						getBodySim()			const;

						PxsRigidCore&					getPxsRigidCore()		const;

						void							onFilterDataChange();
						void							onRestOffsetChange();
						void							onFlagChange(PxShapeFlags oldFlags);
						void							onResetFiltering();
						void							onVolumeOrTransformChange(bool forceBoundsUpdate);
						void							onMaterialChange();  // remove when material properties are gone from PxcNpWorkUnit
						void							onContactOffsetChange();
						void							markBoundsForUpdate(bool forceBoundsUpdate);

						void							getAbsPoseAligned(PxTransform* PX_RESTRICT globalPose)	const;

		PX_FORCE_INLINE	PxU32							getID()								const	{ return mId;		}
		PX_FORCE_INLINE	PxU32							getTransformCacheID()				const	{ return getElementID();	}

						void							removeFromBroadPhase(bool wakeOnLostTouch);

						void							createSqBounds();
						void							destroySqBounds();

		PX_FORCE_INLINE PxU32							getSqBoundsId()						const	{ return mSqBoundsId; }
		PX_FORCE_INLINE void							setSqBoundsId(PxU32 id)						{ mSqBoundsId = id; }

						void							updateCached(PxU32 transformCacheFlags, Cm::BitMapPinned* shapeChangedMap);
						void							updateCached(PxsTransformCache& transformCache, Bp::BoundsArray& boundsArray);
						void							updateContactDistance(PxReal* contactDistance, const PxReal inflation, const PxVec3 angVel, const PxReal dt, Bp::BoundsArray& boundsArray);
						Ps::IntBool						updateSweptBounds();
						void							updateBPGroup();

		PX_FORCE_INLINE PxsShapeSim&					getLLShapeSim() 							{ return mLLShape; }
	private:
						PxsShapeSim						mLLShape;
						const ShapeCore&				mCore;
						PxU32							mId;
						PxU32							mSqBoundsId;

		PX_FORCE_INLINE	void							internalAddToBroadPhase();
		PX_FORCE_INLINE	void							internalRemoveFromBroadPhase(bool wakeOnLostTouch=true);
						void							initSubsystemsDependingOnElementID();
						Bp::FilterGroup::Enum			getBPGroup()	const;
	};

#if !PX_P64_FAMILY
//	PX_COMPILE_TIME_ASSERT(32==sizeof(Sc::ShapeSim)); // after removing bounds from shapes
//	PX_COMPILE_TIME_ASSERT((sizeof(Sc::ShapeSim) % 16) == 0); // aligned mem bounds are better for prefetching
#endif

} // namespace Sc

}

#endif
