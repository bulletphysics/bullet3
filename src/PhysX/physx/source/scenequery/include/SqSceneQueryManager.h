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


#ifndef PX_PHYSICS_SCENEQUERYMANAGER
#define PX_PHYSICS_SCENEQUERYMANAGER
/** \addtogroup physics 
@{ */

#include "PxSceneDesc.h"
#include "CmBitMap.h"
#include "PsArray.h"
#include "SqPruner.h"
#include "PsMutex.h"
#include "PxActor.h" // needed for offset table
#include "ScScene.h"
// threading
#include "PsSync.h"

namespace physx
{
namespace Scb
{
	class Scene;
	class Shape;
	class Actor;
}

namespace Gu
{
	class BVHStructure;
}

namespace Sq
{
	typedef size_t	PrunerData;
	#define SQ_INVALID_PRUNER_DATA	0xffffffff

	struct PrunerPayload;
	class Pruner;
	class CompoundPruner;

	// PT: extended pruner structure. We might want to move the additional data to the pruner itself later.
	struct PrunerExt
	{
														PrunerExt();
														~PrunerExt();

						void							init(PxPruningStructureType::Enum type, PxU64 contextID, PxU32 sceneLimit);
						void							flushMemory();
						void							preallocate(PxU32 nbShapes);
						void							flushShapes(PxU32 index);

						void							addToDirtyList(PrunerHandle handle);
						Ps::IntBool						isDirty(PrunerHandle handle)	const;
						void							removeFromDirtyList(PrunerHandle handle);
						void							growDirtyList(PrunerHandle handle);

		PX_FORCE_INLINE	PxPruningStructureType::Enum	type()			const	{ return mPrunerType;	}
		PX_FORCE_INLINE	const Pruner*					pruner()		const	{ return mPruner;		}
		PX_FORCE_INLINE	Pruner*							pruner()				{ return mPruner;		}
		PX_FORCE_INLINE PxU32							timestamp()		const	{ return mTimestamp;	}
		PX_FORCE_INLINE void							invalidateTimestamp()	{ mTimestamp++;			}

		private:
						Pruner*							mPruner;
						Cm::BitMap						mDirtyMap;
						Ps::Array<PrunerHandle>			mDirtyList;
						PxPruningStructureType::Enum	mPrunerType;
						PxU32							mTimestamp;

						PX_NOCOPY(PrunerExt)

		friend class SceneQueryManager;
	};

	typedef Ps::Pair<PrunerCompoundId, PrunerHandle>	CompoundPair;
	typedef Ps::CoalescedHashSet<CompoundPair >			CompoundPrunerSet;
	// AB: extended compoud pruner structure, buffers compound shape changes and flushes them.
	struct CompoundPrunerExt
	{
														CompoundPrunerExt();
														~CompoundPrunerExt();

						void							flushMemory();
						void							preallocate(PxU32 nbShapes);
						void							flushShapes();

						void							addToDirtyList(PrunerCompoundId compoundId, PrunerHandle handle);
						Ps::IntBool						isDirty(PrunerCompoundId compoundId, PrunerHandle handle)	const;
						void							removeFromDirtyList(PrunerCompoundId compoundId, PrunerHandle handle);						

		PX_FORCE_INLINE	const CompoundPruner*			pruner()		const	{ return mPruner;		}
		PX_FORCE_INLINE	CompoundPruner*					pruner()				{ return mPruner;		}

		private:
						CompoundPruner*					mPruner;						
						CompoundPrunerSet				mDirtyList;

						PX_NOCOPY(CompoundPrunerExt)

		friend class SceneQueryManager;
	};


	struct DynamicBoundsSync : public Sc::SqBoundsSync
	{
		virtual void sync(const PrunerHandle* handles, const PxU32* indices, const PxBounds3* bounds, PxU32 count, const Cm::BitMap& dirtyShapeSimMap);
		
		Pruner*	mPruner;
		PxU32*	mTimestamp;
	};

	class SceneQueryManager : public Ps::UserAllocated
	{
		PX_NOCOPY(SceneQueryManager)
	public:
														SceneQueryManager(Scb::Scene& scene, PxPruningStructureType::Enum staticStructure, 
															PxPruningStructureType::Enum dynamicStructure, PxU32 dynamicTreeRebuildRateHint,
															const PxSceneLimits& limits);
														~SceneQueryManager();

						PrunerData						addPrunerShape(const Scb::Shape& scbShape, const Scb::Actor& scbActor, bool dynamic, PrunerCompoundId compoundId, const PxBounds3* bounds=NULL, bool hasPrunerStructure = false);
						void							removePrunerShape(PrunerCompoundId compoundId, PrunerData shapeData);
						const PrunerPayload&			getPayload(PrunerCompoundId compoundId, PrunerData shapeData) const;

						void							addPruningStructure(const Sq::PruningStructure& ps);
						void							addCompoundShape(const Gu::BVHStructure& bvhStructure, PrunerCompoundId compoundId, const PxTransform& compoundTransform, PrunerData* prunerData, const Scb::Shape** scbShapes, const Scb::Actor& scbActor);

	public:
		PX_FORCE_INLINE	Scb::Scene&						getScene()						const	{ return mScene;			}
		PX_FORCE_INLINE	PxU32							getDynamicTreeRebuildRateHint()	const	{ return mRebuildRateHint;	}

		PX_FORCE_INLINE	const PrunerExt&				get(PruningIndex::Enum index)	const	{ return mPrunerExt[index];	}
		PX_FORCE_INLINE	PrunerExt&						get(PruningIndex::Enum index)			{ return mPrunerExt[index];	}

		PX_FORCE_INLINE	const CompoundPrunerExt&		getCompoundPruner() const { return mCompoundPrunerExt; }

						void							preallocate(PxU32 staticShapes, PxU32 dynamicShapes);
						void							markForUpdate(PrunerCompoundId compoundId, PrunerData s);
						void							setDynamicTreeRebuildRateHint(PxU32 dynTreeRebuildRateHint);
						
						void							flushUpdates();
						void							forceDynamicTreeRebuild(bool rebuildStaticStructure, bool rebuildDynamicStructure);
						void							sceneQueryBuildStep(PruningIndex::Enum index);

						void							updateCompoundActors(Sc::BodyCore*const* bodies, PxU32 numBodies);
						void							updateCompoundActor(PrunerCompoundId compoundId, const PxTransform& compoundTransform, bool dynamic);						
						void							removeCompoundActor(PrunerCompoundId compoundId, bool dynamic);

						DynamicBoundsSync&				getDynamicBoundsSync()					{ return mDynamicBoundsSync; }

						bool							prepareSceneQueriesUpdate(PruningIndex::Enum index);

		// Force a rebuild of the aabb/loose octree etc to allow raycasting on multiple threads.
						void							afterSync(PxSceneQueryUpdateMode::Enum updateMode);
						void							shiftOrigin(const PxVec3& shift);

						void							flushMemory();
	private:
						PrunerExt						mPrunerExt[PruningIndex::eCOUNT];
						CompoundPrunerExt				mCompoundPrunerExt;										

						PxU32							mRebuildRateHint;

						Scb::Scene&						mScene;

						// threading
						shdfnd::Mutex					mSceneQueryLock;  // to make sure only one query updates the dirty pruner structure if multiple queries run in parallel

						DynamicBoundsSync				mDynamicBoundsSync;

						volatile bool					mPrunerNeedsUpdating;

						void							flushShapes();
	};

	///////////////////////////////////////////////////////////////////////////////

	// PT: TODO: replace PrunerData with just PxU32 to save memory on Win64. Breaks binary compatibility though.
	// PT: was previously called 'ActorShape' but does not contain an actor or shape pointer, contrary to the Np-level struct with the same name.
	// PT: it only contains a pruner index (0 or 1) and a pruner handle. Hence the new name.
	PX_FORCE_INLINE PrunerData createPrunerData(PxU32 index, PrunerHandle h)	{ return PrunerData((h << 1) | index);	}
	PX_FORCE_INLINE PxU32 getPrunerIndex(PrunerData data)						{ return PxU32(data & 1);				}
	PX_FORCE_INLINE PrunerHandle getPrunerHandle(PrunerData data)				{ return PrunerHandle(data >> 1);		}

	///////////////////////////////////////////////////////////////////////////////


} // namespace Sq

}

/** @} */
#endif
