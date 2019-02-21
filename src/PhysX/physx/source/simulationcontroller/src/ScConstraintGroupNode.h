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


#ifndef PX_PHYSICS_SCP_CONSTRAINT_GROUP_NODE
#define PX_PHYSICS_SCP_CONSTRAINT_GROUP_NODE

#include "ScConstraintProjectionTree.h"
#include "PsUtilities.h"  // for Ps::to8()

namespace physx
{
namespace Sc
{
	class ConstraintSim;
	class BodySim;
	class ConstraintProjectionManager;

	// A 'simulation island' of constraints. Created by a union-find algorithm every time a new constraint is added to any of the involved bodies.
	struct ConstraintGroupNode : public Ps::UserAllocated
	{
		enum StateFlags
		{
			eDISCOVERED						= 1 << 0,	// Used during projection tree generation to mark processed nodes.
			eIN_PROJECTION_PASS_LIST		= 1 << 1,	// Temporarily used to avoid duplicate entries in the list of nodes that should project the pose after the solver
			ePENDING_TREE_UPDATE			= 1 << 2,	// Marks the constraint groups that need their projection trees updated. Must only be set on the root group node.
			eNEXT_FREE_SHIFT				= 3,
			eNEXT_FREE						= 1 << eNEXT_FREE_SHIFT
		};

		// these flags should give a rough hint how many projecting constraints to expect in the constraint group. This will be used for
		// load balancing when running projection in parallel. The intervals were chosen somewhat arbitrarily but the general motivation was
		// to cover very simple constraint setups, simple ragdolls, complex ragdolls and very complex projection setups. Note that the load
		// balancing is not waterproof since at the end it is the projection shader from the external constraint implementer (for example, a joint)
		// which decides based on some thresholds whether projection runs or not.
		enum ProjectionCountHintFlags
		{
			e1_TO_4							= eNEXT_FREE,
			e5_TO_16						= eNEXT_FREE << 1,
			e17_TO_64						= eNEXT_FREE << 2,
			e65_TO_INF						= eNEXT_FREE << 3,
			eCLEAR_MASK						= ~(0xffffffff << eNEXT_FREE_SHIFT)
		};

		ConstraintGroupNode(BodySim& b);
		~ConstraintGroupNode()
		{
			PX_ASSERT(!readFlag(ePENDING_TREE_UPDATE));
			PX_ASSERT(projectionFirstRoot == NULL);
		}

		PX_FORCE_INLINE		void					raiseFlag(StateFlags f) { flags |= f; }
		PX_FORCE_INLINE		void					clearFlag(StateFlags f) { flags &= ~f; }
		PX_FORCE_INLINE		bool					readFlag(StateFlags f) const { return (flags & f) != 0; }
		PX_FORCE_INLINE		PxU32					getProjectionCountHint() const;
		PX_FORCE_INLINE		void					setProjectionCountHint(PxU32 constraintsToProjectCount);

							ConstraintGroupNode&	getRoot();

		PX_FORCE_INLINE		void					buildProjectionTrees(); //build the projection trees for a constraint group.
							void					markForProjectionTreeRebuild(ConstraintProjectionManager&);
		PX_FORCE_INLINE		void					purgeProjectionTrees();
		PX_FORCE_INLINE		bool					hasProjectionTreeRoot() { return projectionFirstRoot != NULL; }
		PX_FORCE_INLINE		void					setProjectionTreeRoot(ConstraintGroupNode* root) { projectionFirstRoot = root; }

							void					initProjectionData(ConstraintGroupNode* parent, ConstraintSim* c);
							void					clearProjectionData();

		static				void					projectPose(ConstraintGroupNode& root, Ps::Array<BodySim*>& projectedBodies);


		BodySim*					body;		//the owner body of this node

		//tree for union/find:
		ConstraintGroupNode*		parent;
		ConstraintGroupNode*		tail;		//only valid if this is root of group, points to LList tail node.
		PxU32						rank;		//rank counter for union/find. Initially zero. Is number of hops from root to furthest leaf in tree. This is just a hint to create more balanced trees.
		
		//linked list for traversal:
		ConstraintGroupNode*		next;		//next in list, NULL at tail.

		//projection tree information
		ConstraintGroupNode*		projectionFirstRoot;	//pointer to first projection tree root node. Only set for constraint group roots
		ConstraintGroupNode*		projectionNextRoot;		//pointer to next projection root node. Only set for constraint group roots
															//a constraint group can consist of multiple projection trees if kinematics are involved! Because a kinematic doesn't split
															//the constraint group as a static anchor does.
		ConstraintGroupNode*		projectionParent;		//node to project to
		ConstraintGroupNode*		projectionFirstChild;	//first node which gets projected to this one
		ConstraintGroupNode*		projectionNextSibling;	//the next sibling which gets projected to the same node as this one. NULL if projectionParent is NULL.
		ConstraintSim*				projectionConstraint;	//the constraint to project (constraint to projection parent)

	private:
		PxU8						flags;
	};

} // namespace Sc


PX_FORCE_INLINE PxU32 Sc::ConstraintGroupNode::getProjectionCountHint() const
{
	// return the mean of the upper and lower bound

	if (flags & ConstraintGroupNode::e65_TO_INF)
		return 128;
	else if (flags & ConstraintGroupNode::e17_TO_64)
		return 40;
	else if (flags & ConstraintGroupNode::e5_TO_16)
		return 10;
	else if (flags & ConstraintGroupNode::e1_TO_4)
		return 2;

	return 0;
}


PX_FORCE_INLINE void Sc::ConstraintGroupNode::setProjectionCountHint(PxU32 constraintsToProjectCount)
{
	PxU8 tmpFlags = flags;
	tmpFlags &= PxU8(ConstraintGroupNode::eCLEAR_MASK);

	if (constraintsToProjectCount >= 65)
		tmpFlags |= ConstraintGroupNode::e65_TO_INF;
	else if (constraintsToProjectCount >= 17)
		tmpFlags |= ConstraintGroupNode::e17_TO_64;
	else if (constraintsToProjectCount >= 5)
		tmpFlags |= ConstraintGroupNode::e5_TO_16;
	else if (constraintsToProjectCount >= 1)
		tmpFlags |= ConstraintGroupNode::e1_TO_4;

	flags = tmpFlags;
}


PX_FORCE_INLINE void Sc::ConstraintGroupNode::buildProjectionTrees()
{
	PX_ASSERT(this == parent);  // Only call for group roots
	PX_ASSERT(!hasProjectionTreeRoot());

	ConstraintProjectionTree::buildProjectionTrees(*this);
}


PX_FORCE_INLINE void Sc::ConstraintGroupNode::purgeProjectionTrees()
{
	PX_ASSERT(this == parent);  // Only call for group roots
	PX_ASSERT(hasProjectionTreeRoot());
	ConstraintProjectionTree::purgeProjectionTrees(*this);
}

}

#endif
