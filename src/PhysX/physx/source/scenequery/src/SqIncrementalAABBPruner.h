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

#ifndef SQ_INCREMENTAL_AABB_PRUNER_H
#define SQ_INCREMENTAL_AABB_PRUNER_H

#include "SqPruner.h"
#include "SqPruningPool.h"
#include "SqIncrementalAABBTree.h"
#include "SqAABBTreeUpdateMap.h"

namespace physx
{

namespace Sq
{
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	class IncrementalAABBPruner : public IncrementalPruner
	{
		public:
												IncrementalAABBPruner(PxU32 sceneLimit, PxU64 contextID);
		virtual									~IncrementalAABBPruner();

		// Pruner
		virtual			bool					addObjects(PrunerHandle* results, const PxBounds3* bounds, const PrunerPayload* userData, PxU32 count, bool hasPruningStructure);
		virtual			void					removeObjects(const PrunerHandle* handles, PxU32 count);
		virtual			void					updateObjectsAfterManualBoundsUpdates(const PrunerHandle* handles, PxU32 count);
		virtual			void					updateObjectsAndInflateBounds(const PrunerHandle* handles, const PxU32* indices, const PxBounds3* newBounds, PxU32 count);
		virtual			void					commit();
		virtual			PxAgain					raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal& inOutDistance, PrunerCallback&)	const;
		virtual			PxAgain					overlap(const Gu::ShapeData& queryVolume, PrunerCallback&)	const;
		virtual			PxAgain					sweep(const Gu::ShapeData& queryVolume, const PxVec3& unitDir, PxReal& inOutDistance, PrunerCallback&)	const;
		virtual			const PrunerPayload&	getPayload(PrunerHandle handle)						const	{ return mPool.getPayload(handle);			}
		virtual			const PrunerPayload&	getPayload(PrunerHandle handle, PxBounds3*& bounds)	const	{ return mPool.getPayload(handle, bounds);	}
		virtual			void					preallocate(PxU32 entries)									{ mPool.preallocate(entries);				}
		virtual			void					shiftOrigin(const PxVec3& shift);
		virtual			void					visualize(Cm::RenderOutput& out, PxU32 color) const;		
		virtual			void					merge(const void* mergeParams);
		//~Pruner
		
		// IncrementalPruner
		virtual			void					purge();		// gets rid of internal accel struct
		virtual			void					setRebuildRateHint(PxU32 nbStepsForRebuild);	// Besides the actual rebuild steps, 3 additional steps are needed.
		virtual			bool					buildStep(bool );	// returns true if finished
		virtual			bool					prepareBuild();
		//~IncrementalPruner

		// direct access for test code
		PX_FORCE_INLINE	const IncrementalAABBTree*		getAABBTree()		const		{ return mAABBTree;	}
				
		// local functions
		private:
						void					release();
						void					fullRebuildAABBTree();
						void					test();
						void					updateMapping(const PoolIndex poolIndex, IncrementalAABBTreeNode* node);

		private:
						IncrementalAABBTree*	mAABBTree; 						

						PruningPool				mPool; // Pool of AABBs

						Ps::Array<IncrementalAABBTreeNode*>	mMapping;

						PxU64					mContextID;
						NodeList				mChangedLeaves;
	};

} // namespace Sq

} // namespace physx
#endif
