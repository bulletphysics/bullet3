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


#include "PxcScratchAllocator.h"
#include "ScConstraintProjectionManager.h"
#include "ScBodySim.h"
#include "ScConstraintSim.h"
#include "ScConstraintInteraction.h"

using namespace physx;


namespace physx
{
namespace Sc
{

template<typename T, const PxU32 elementsPerBlock>
class ScratchAllocatorList
{
private:
	struct ElementBlock
	{
		PX_FORCE_INLINE ElementBlock() {}
		PX_FORCE_INLINE void init(PxU32 countAtStart) { next = NULL; count = countAtStart; }

		ElementBlock* next;
		PxU32 count;
		T elements[elementsPerBlock];
	};

	PX_FORCE_INLINE const ScratchAllocatorList& operator=(const ScratchAllocatorList&) {}


public:
	class Iterator
	{
		friend class ScratchAllocatorList;

	public:
		T const* getNext()
		{
			if (mCurrentBlock)
			{
				if (mIndex < mCurrentBlock->count)
				{
					return &mCurrentBlock->elements[mIndex++];
				}
				else
				{
					if (mCurrentBlock->next)
					{
						PX_ASSERT(mCurrentBlock->count == elementsPerBlock);
						mCurrentBlock = mCurrentBlock->next;
						PX_ASSERT(mCurrentBlock->count > 0);

						mIndex = 1;
						return &mCurrentBlock->elements[0];
					}
					else
						return NULL;
				}
			}
			else
				return NULL;
		}

	private:
		Iterator(const ElementBlock* startBlock) : mCurrentBlock(startBlock), mIndex(0) {}

	private:
		const ElementBlock* mCurrentBlock;
		PxU32 mIndex;
	};

	PX_FORCE_INLINE ScratchAllocatorList(PxcScratchAllocator& scratchAllocator) : mScratchAllocator(scratchAllocator)
	{
		mFirstBlock = reinterpret_cast<ElementBlock*>(scratchAllocator.alloc(sizeof(ElementBlock), true));
		if (mFirstBlock)
			mFirstBlock->init(0);

		mCurrentBlock = mFirstBlock;
	}

	PX_FORCE_INLINE ~ScratchAllocatorList()
	{
		freeMemory();
	}

	PX_FORCE_INLINE bool add(const T& element)
	{
		if (mCurrentBlock)
		{
			if (mCurrentBlock->count < elementsPerBlock)
			{
				mCurrentBlock->elements[mCurrentBlock->count] = element;
				mCurrentBlock->count++;
				return true;
			}
			else
			{
				PX_ASSERT(mCurrentBlock->next == NULL);
				PX_ASSERT(mCurrentBlock->count == elementsPerBlock);

				ElementBlock* newBlock = reinterpret_cast<ElementBlock*>(mScratchAllocator.alloc(sizeof(ElementBlock), true));
				if (newBlock)
				{
					newBlock->init(1);
					newBlock->elements[0] = element;
					mCurrentBlock->next = newBlock;
					mCurrentBlock = newBlock;
					return true;
				}
				else
					return false;
			}
		}
		else
			return false;
	}

	PX_FORCE_INLINE Iterator getIterator() const
	{
		return Iterator(mFirstBlock);
	}

	PX_FORCE_INLINE void freeMemory()
	{
		ElementBlock* block = mFirstBlock;

		while(block)
		{
			ElementBlock* blockToFree = block;
			block = block->next;

			mScratchAllocator.free(blockToFree);
		}
	}


private:
	PxcScratchAllocator& mScratchAllocator;
	ElementBlock* mFirstBlock;
	ElementBlock* mCurrentBlock;
};

}
}


Sc::ConstraintProjectionManager::ConstraintProjectionManager() : 
	mNodePool(PX_DEBUG_EXP("projectionNodePool"))
{
}


void Sc::ConstraintProjectionManager::addToPendingGroupUpdates(Sc::ConstraintSim& s)
{
	PX_ASSERT(!s.readFlag(ConstraintSim::ePENDING_GROUP_UPDATE));
	bool isNew = mPendingGroupUpdates.insert(&s);
	PX_UNUSED(isNew);
	PX_ASSERT(isNew);

	s.setFlag(ConstraintSim::ePENDING_GROUP_UPDATE);
}


void Sc::ConstraintProjectionManager::removeFromPendingGroupUpdates(Sc::ConstraintSim& s)
{
	PX_ASSERT(s.readFlag(ConstraintSim::ePENDING_GROUP_UPDATE));
	bool didExist = mPendingGroupUpdates.erase(&s);
	PX_UNUSED(didExist);
	PX_ASSERT(didExist);

	s.clearFlag(ConstraintSim::ePENDING_GROUP_UPDATE);
}


void Sc::ConstraintProjectionManager::addToPendingTreeUpdates(ConstraintGroupNode& n)
{
	PX_ASSERT(&n == &n.getRoot());
	PX_ASSERT(!n.readFlag(ConstraintGroupNode::ePENDING_TREE_UPDATE));
	bool isNew = mPendingTreeUpdates.insert(&n);
	PX_UNUSED(isNew);
	PX_ASSERT(isNew);

	n.raiseFlag(ConstraintGroupNode::ePENDING_TREE_UPDATE);
}


void Sc::ConstraintProjectionManager::removeFromPendingTreeUpdates(ConstraintGroupNode& n)
{
	PX_ASSERT(&n == &n.getRoot());
	PX_ASSERT(n.readFlag(ConstraintGroupNode::ePENDING_TREE_UPDATE));
	bool didExist = mPendingTreeUpdates.erase(&n);
	PX_UNUSED(didExist);
	PX_ASSERT(didExist);

	n.clearFlag(ConstraintGroupNode::ePENDING_TREE_UPDATE);
}


PX_INLINE Sc::ConstraintGroupNode* Sc::ConstraintProjectionManager::createGroupNode(BodySim& b)
{
	ConstraintGroupNode* n = mNodePool.construct(b);
	b.setConstraintGroup(n);
	return n;
}


//
// Implementation of UNION of 
// UNION-FIND algo.
// It also updates the group traversal
// linked list.
//
void Sc::ConstraintProjectionManager::groupUnion(ConstraintGroupNode& root0, ConstraintGroupNode& root1)
{
	// Should only get called for the roots
	PX_ASSERT(&root0 == root0.parent);
	PX_ASSERT(&root1 == root1.parent);

	if (&root0 != &root1)	//different groups?  If not, its already merged.
	{
		//UNION(this, other);	//union-find algo unites groups.
		ConstraintGroupNode* newRoot;
		ConstraintGroupNode* otherRoot;
		if (root0.rank > root1.rank)
		{
			//hisGroup appended to mygroup.
			newRoot = &root0;
			otherRoot = &root1;
		}
		else
		{
			//myGroup appended to hisGroup. 
			newRoot = &root1;
			otherRoot = &root0;
			//there is a chance that the two ranks were equal, in which case the tree depth just increased.
			root1.rank++;
		}

		PX_ASSERT(newRoot->parent == newRoot);
		otherRoot->parent = newRoot;
		
		//update traversal linked list:
		newRoot->tail->next = otherRoot;
		newRoot->tail = otherRoot->tail;
	}
}


//
// Add a body to a constraint projection group.
//
void Sc::ConstraintProjectionManager::addToGroup(BodySim& b, BodySim* other, ConstraintSim& c)
{
	// If both bodies of the constraint are defined, we want to fetch the reference to the group root
	// from body 0 by default (allows to avoid checking both)
	PX_ASSERT(&b == c.getBody(0) || (c.getBody(0) == NULL && &b == c.getBody(1)));
	PX_UNUSED(c);

	ConstraintGroupNode* myRoot;
	if (!b.getConstraintGroup())
		myRoot = createGroupNode(b);
	else
	{
		myRoot = &b.getConstraintGroup()->getRoot();
		if (myRoot->hasProjectionTreeRoot())
			myRoot->purgeProjectionTrees();  // If a new constraint gets added to a constraint group, projection trees need to be recreated
	}

	if (other)
	{
		ConstraintGroupNode* otherRoot;
		if (!other->getConstraintGroup())
			otherRoot = createGroupNode(*other);
		else
		{
			otherRoot = &other->getConstraintGroup()->getRoot();
			if (otherRoot->hasProjectionTreeRoot())
				otherRoot->purgeProjectionTrees();  // If a new constraint gets added to a constraint group, projection trees need to be recreated
		}

		//merge the two groups, if disjoint.
		groupUnion(*myRoot, *otherRoot);
	}
}


//
// Add all projection constraints connected to the specified body to the pending update list but
// ignore the specified constraint.
//
void Sc::ConstraintProjectionManager::markConnectedConstraintsForUpdate(BodySim& b, ConstraintSim* c)
{
	PxU32 size = b.getActorInteractionCount();
	Interaction** interactions = b.getActorInteractions();
	while(size--)
	{
		Interaction* interaction = *interactions++;
		if (interaction->getType() == InteractionType::eCONSTRAINTSHADER)
		{
			ConstraintSim* ct = static_cast<ConstraintInteraction*>(interaction)->getConstraint();

			if ((ct != c) && ct->needsProjection() && (!ct->readFlag(ConstraintSim::ePENDING_GROUP_UPDATE)))
			{
				//mark constraint for pending update:
				addToPendingGroupUpdates(*ct);
			}
		}
	}
}


//
// Add all constraints connected to the specified body to an array but
// ignore the specified constraint.
//
PX_FORCE_INLINE static void dumpConnectedConstraints(Sc::BodySim& b, Sc::ConstraintSim* c, Sc::ScratchAllocatorList<Sc::ConstraintSim*>& constraintList)
{
	PxU32 size = b.getActorInteractionCount();
	Sc::Interaction** interactions = b.getActorInteractions();
	while(size--)
	{
		Sc::Interaction* interaction = *interactions++;
		if (interaction->getType() == Sc::InteractionType::eCONSTRAINTSHADER)
		{
			Sc::ConstraintSim* ct = static_cast<Sc::ConstraintInteraction*>(interaction)->getConstraint();

			if ((ct != c) && (!ct->readFlag(Sc::ConstraintSim::ePENDING_GROUP_UPDATE)))
			{
				bool success = constraintList.add(ct);
				PX_UNUSED(success);
				PX_ASSERT(success);
			}
		}
	}
}


PX_FORCE_INLINE void Sc::ConstraintProjectionManager::processConstraintForGroupBuilding(ConstraintSim* c, ScratchAllocatorList<ConstraintSim*>& constraintList)
{
	c->clearFlag(ConstraintSim::ePENDING_GROUP_UPDATE);

	// Find all constraints connected to the two bodies of the dirty constraint.
	// - Constraints to static anchors are ignored (note: kinematics can't be ignored because they might get switched to dynamics any time which
	//   does trigger a projection tree rebuild but not a constraint tree rebuild
	// - Already processed bodies are ignored as well
	BodySim* b0 = c->getBody(0);
	if (b0 && !b0->getConstraintGroup())
	{
		dumpConnectedConstraints(*b0, c, constraintList);
	}
	BodySim* b1 = c->getBody(1);
	if (b1 && !b1->getConstraintGroup())
	{
		dumpConnectedConstraints(*b1, c, constraintList);
	}

	BodySim* b = c->getAnyBody();
	PX_ASSERT(b);

	addToGroup(*b, c->getOtherBody(b), *c);  //this will eventually merge some body's constraint groups.
}


void Sc::ConstraintProjectionManager::processPendingUpdates(PxcScratchAllocator& scratchAllocator)
{
	//
	// if there are dirty projection trees, then rebuild them
	//
	const PxU32 nbProjectionTreesToUpdate = mPendingTreeUpdates.size();
	if (nbProjectionTreesToUpdate)
	{
		ConstraintGroupNode* const* projectionTreesToUpdate = mPendingTreeUpdates.getEntries();
		for(PxU32 i=0; i < nbProjectionTreesToUpdate; i++)
		{
			ConstraintGroupNode* n = projectionTreesToUpdate[i];

			PX_ASSERT(n == &n->getRoot());  // only root nodes should be in that list
			PX_ASSERT(n->readFlag(ConstraintGroupNode::ePENDING_TREE_UPDATE));

			n->clearFlag(ConstraintGroupNode::ePENDING_TREE_UPDATE);

			// note: it is valid to get here and not have a projection root. This is the case if all nodes of a constraint graph are kinematic
			//       at some point (hence no projection root) and later some of those get switched to dynamic.
			if (n->hasProjectionTreeRoot())
				n->purgeProjectionTrees();
			n->buildProjectionTrees();
		}

		mPendingTreeUpdates.clear();
	}

	//
	// if there are new/dirty constraints, update groups
	//
	const PxU32 nbProjectionConstraintsToUpdate = mPendingGroupUpdates.size();

	if (nbProjectionConstraintsToUpdate)
	{
		ScratchAllocatorList<ConstraintSim*> nonProjectionConstraintList(scratchAllocator);

		ConstraintSim* const* projectionConstraintsToUpdate = mPendingGroupUpdates.getEntries();

#if PX_DEBUG
		// At the beginning the list should only contain constraints with projection.
		// Further below other constraints, connected to the constraints with projection, will be added too.
		for(PxU32 i=0; i < nbProjectionConstraintsToUpdate; i++)
		{
			PX_ASSERT(projectionConstraintsToUpdate[i]->needsProjection());
		}
#endif
		for(PxU32 i=0; i < nbProjectionConstraintsToUpdate; i++)
		{
			processConstraintForGroupBuilding(projectionConstraintsToUpdate[i], nonProjectionConstraintList);
		}

		ScratchAllocatorList<ConstraintSim*>::Iterator iter = nonProjectionConstraintList.getIterator();
		ConstraintSim* const* nextConstraint = iter.getNext();
		while(nextConstraint)
		{
			processConstraintForGroupBuilding(*nextConstraint, nonProjectionConstraintList);

			nextConstraint = iter.getNext();
		}

		// Now find all the newly made groups and build projection trees.
		// Don't need to iterate over the additionally constraints since the roots are supposed to be
		// fetchable from any node.
		for (PxU32 i=0; i < nbProjectionConstraintsToUpdate; i++)
		{
			ConstraintSim* c = projectionConstraintsToUpdate[i];
			BodySim* b = c->getAnyBody();
			PX_ASSERT(b);
			PX_ASSERT(b->getConstraintGroup());

			ConstraintGroupNode& root = b->getConstraintGroup()->getRoot();
			if (!root.hasProjectionTreeRoot())  // Build projection tree only once
				root.buildProjectionTrees();
		}

		mPendingGroupUpdates.clear();
	}
}


//
// Called if a body or a constraint gets deleted. All projecting constraints of the
// group (except the deleted one) are moved to the dirty list and all group nodes are destroyed.
//
void Sc::ConstraintProjectionManager::invalidateGroup(ConstraintGroupNode& node, ConstraintSim* deletedConstraint)
{
	ConstraintGroupNode* n = &node.getRoot();

	if (n->readFlag(ConstraintGroupNode::ePENDING_TREE_UPDATE))
	{
		removeFromPendingTreeUpdates(*n);
	}

	while (n) //go through nodes in constraint group
	{
		markConnectedConstraintsForUpdate(*n->body, deletedConstraint);

		//destroy the body's constraint group information

		ConstraintGroupNode* next = n->next;	//save next node ptr before we destroy it!

		BodySim* b = n->body;
		b->setConstraintGroup(NULL);
		if (n->hasProjectionTreeRoot())
			n->purgeProjectionTrees();
		mNodePool.destroy(n);

		n = next;
	}
}
