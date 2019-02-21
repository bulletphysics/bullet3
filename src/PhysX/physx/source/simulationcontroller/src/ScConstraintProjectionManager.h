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


#ifndef PX_PHYSICS_SCP_CONSTRAINT_PROJECTION_MANAGER
#define PX_PHYSICS_SCP_CONSTRAINT_PROJECTION_MANAGER

#include "PsPool.h"
#include "PsHashSet.h"
#include "ScConstraintGroupNode.h"

namespace physx
{
	class PxcScratchAllocator;

namespace Sc
{
	class ConstraintSim;
	class BodySim;
	template<typename T, const PxU32 elementsPerBlock = 64> class ScratchAllocatorList;

	class ConstraintProjectionManager : public Ps::UserAllocated
	{
	public:
		ConstraintProjectionManager();
		~ConstraintProjectionManager() {}

		void addToPendingGroupUpdates(ConstraintSim& s);
		void removeFromPendingGroupUpdates(ConstraintSim& s);

		void addToPendingTreeUpdates(ConstraintGroupNode& n);
		void removeFromPendingTreeUpdates(ConstraintGroupNode& n);

		void processPendingUpdates(PxcScratchAllocator&);
		void invalidateGroup(ConstraintGroupNode& node, ConstraintSim* constraintDeleted);

	private:
		PX_INLINE Sc::ConstraintGroupNode* createGroupNode(BodySim& b);

		void addToGroup(BodySim& b, BodySim* other, ConstraintSim& c);
		void groupUnion(ConstraintGroupNode& root0, ConstraintGroupNode& root1);
		void markConnectedConstraintsForUpdate(BodySim& b, ConstraintSim* c);
		PX_FORCE_INLINE void processConstraintForGroupBuilding(ConstraintSim* c, ScratchAllocatorList<ConstraintSim*>&);


	private:
		Ps::Pool<ConstraintGroupNode>				mNodePool;
		Ps::CoalescedHashSet<ConstraintSim*>		mPendingGroupUpdates; //list of constraints for which constraint projection groups need to be generated/updated
		Ps::CoalescedHashSet<ConstraintGroupNode*>	mPendingTreeUpdates;	//list of constraint groups that need their projection trees rebuilt. Note: non of the
																			//constraints in those groups are allowed to be in mPendingGroupUpdates at the same time
																			//because a group update will automatically trigger tree rebuilds.
	};

} // namespace Sc

}

#endif
