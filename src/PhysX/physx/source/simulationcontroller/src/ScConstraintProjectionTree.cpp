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


#include "ScConstraintProjectionTree.h"
#include "ScScene.h"
#include "ScBodySim.h"
#include "ScConstraintCore.h"
#include "ScConstraintSim.h"
#include "ScConstraintInteraction.h"

#include "PsFoundation.h"
#include "PsBasicTemplates.h"
#include "PsSort.h"
#include "PsArray.h"

using namespace physx;

//------------------------------------------------------------------------------------------
//
// The projection tree related code
// 
// Projection trees are built out of a constraint group/graph. The constraint group just tracks
// the constraint connectivity while the projection trees define the projection root and
// the projection order.
// A constraint group can contain multiple projection trees.
//
//------------------------------------------------------------------------------------------

class Sc::BodyRank		
{
public:
	PX_INLINE bool operator>(const BodyRank & b) const
	{
		return rank > b.rank;
	}

	Sc::ConstraintGroupNode* startingNode;
	Sc::ConstraintSim* constraintToFixedAnchor;
	PxU32 rank;

	//
	// The following weights are defined to fulfill the projection priorities described further below
	//
	static const PxU32 sOneWayProjection = PxU32(1 << 31);
	static const PxU32 sAttachedToStatic = (1 << 30);
	static const PxU32 sAttachedToKinematic = (1 << 29);
	static const PxU32 sAllDominantDynamic = (1 << 28);  // if for a dynamic body all connections with projection, are one-way towards the body
	static const PxU32 sDominantDynamic = (1 << 27);  // almost the same as above but there is at least one two-way projection
	static const PxU32 sAttachedToDynamic = 1;
	static const PxU32 sPrimaryTreeRootMinRank = sOneWayProjection | sAllDominantDynamic;
};


PX_INLINE bool isFixedBody(const Sc::BodySim* b)
{
	return (!b || (b->isKinematic()));
}


void Sc::ConstraintProjectionTree::getConstraintStatus(const ConstraintSim& c, const BodySim* b, BodySim*& otherBody, PxU32& projectToBody, PxU32& projectToOtherBody)
{
	const PxU32 isBroken = c.isBroken() ? 0 : 0xffffffff;
	const PxU32 projFlags = c.getCore().getFlags() & PxConstraintFlag::ePROJECTION;
	
	if (b == c.getBody(0))
	{
		projectToBody = isBroken & (projFlags & PxConstraintFlag::ePROJECT_TO_ACTOR0);
		projectToOtherBody = isBroken & (projFlags & PxConstraintFlag::ePROJECT_TO_ACTOR1);

		otherBody = c.getBody(1);
	}
	else
	{
		projectToBody = isBroken & (projFlags & PxConstraintFlag::ePROJECT_TO_ACTOR1);
		projectToOtherBody = isBroken & (projFlags & PxConstraintFlag::ePROJECT_TO_ACTOR0);

		otherBody = c.getBody(0);
	}
}


void Sc::ConstraintProjectionTree::rankConstraint(ConstraintSim& c, BodyRank& br, PxU32& dominanceTracking, PxU32& constraintsToProjectCount)
{
	PxU32 projectToBody, projectToOtherBody;
	BodySim* otherB;
	getConstraintStatus(c, br.startingNode->body, otherB, projectToBody, projectToOtherBody);

	if (isFixedBody(otherB))	// joint to fixed anchor
	{
		PxU32 rank;
		if (projectToOtherBody)
		{
			dominanceTracking = 0;  // makes sure that the flags below will never get raised again for the body
			br.rank &= ~(BodyRank::sAllDominantDynamic | BodyRank::sDominantDynamic);
			rank = BodyRank::sOneWayProjection;  //we should prefer picking projected constraints as the root over non-projected ones.
			constraintsToProjectCount++;
		}
		else
			rank = 0;

		if (!otherB)
			rank |= BodyRank::sAttachedToStatic;
		else
		{
			PX_ASSERT(otherB->isKinematic());
			rank |= BodyRank::sAttachedToKinematic;
		}

		// the highest ranked fixed anchor constraint should get tracked
		if ((!br.constraintToFixedAnchor) || (rank > br.rank))
			br.constraintToFixedAnchor = &c;

		br.rank |= rank;
	}
	else
	{
		if (projectToBody && projectToOtherBody)
		{
			dominanceTracking &= ~BodyRank::sAllDominantDynamic;  // makes sure that from now on this will never get raised again for the body
			br.rank &= ~BodyRank::sAllDominantDynamic;
			constraintsToProjectCount++;
		}
		else if (projectToOtherBody)
		{
			dominanceTracking &= ~(BodyRank::sAllDominantDynamic | BodyRank::sDominantDynamic);  // makes sure that from now on these will never get raised again for the body
			br.rank &= ~(BodyRank::sAllDominantDynamic | BodyRank::sDominantDynamic);
			constraintsToProjectCount++;
		}
		else if (projectToBody)
		{
			br.rank |= BodyRank::sOneWayProjection | (dominanceTracking & (BodyRank::sAllDominantDynamic | BodyRank::sDominantDynamic));
			constraintsToProjectCount++;
		}

		br.rank += BodyRank::sAttachedToDynamic;
	}
}


/*
the goal here is to take the constraint group whose root is passed, and create one or more projection trees.

At the moment, the group has to be acyclic and have at most 1 constraint with the ground to be accepted
without being broken up into multiple trees.

We 'flood fill' the constraint graph several times, starting at bodies where projection trees can be rooted. 
Projection tree roots are always dynamic bodies which either need to get projected to a fixed anchor directly 
or have projecting constraints between dynamics some way along the tree branches. Static and kinematic actors
are never roots and will not be explicitly part of any tree (but a tree root can project to at most one such fixed node).

The algo looks like this:

for all bodies
mark body as undiscovered
rank this body

The rank of a body depends on the constraints it's connected to. It defines the projection priority which
should be (highest first):
- dynamic attached to static/world with projection
- dynamic attached to kinematic with projection
- all dominant dynamic (has projecting constraints and all of them are one-way towards this dynamic)
---- all the ones above are guaranteed tree roots
- dominant dynamic (same as above but there is at least one projecting two-way constraint as well)
- partially dominant dynamic (has at least one projecting one-way constraints towards this dynamic and at least one projecting one-way constraints towards an other body)
- dynamic attached to static/world without projection
- dynamic attached to kinematic without projection
- dynamic with or without two-way projecting constraints to other dynamics (among these, the one with the highest connectivity count wins)

for the first three priority types sorted according to rank:
create a projection tree root and grow the tree one connectivity layer at a time

do the same for dominant dynamic bodies that have not been visited/discovered yet

for all remaining bodies sorted according to rank:
if the body still hasn't been visited/discovered start a projection tree there and build the whole tree in one go
before moving to the next potential root.
*/
void Sc::ConstraintProjectionTree::buildProjectionTrees(ConstraintGroupNode& root)
{
	PX_ASSERT(&root == root.parent);
	PX_ASSERT(!root.hasProjectionTreeRoot());

	Ps::InlineArray<BodyRank, 64> bodyRankArray PX_DEBUG_EXP("bodyRankArray");
	BodyRank br;
	PxU32 constraintsToProjectCount = 0;
	ConstraintGroupNode* node0 = &root;
	while (node0)	//for all nodes in group
	{
		PX_ASSERT(node0->body);
		if (!node0->body->isKinematic())
		{
			node0->clearFlag(ConstraintGroupNode::eDISCOVERED);

			//rank 
			br.startingNode = node0;
			br.rank = 0;
			br.constraintToFixedAnchor = 0;
			PxU32 dominanceTracking = BodyRank::sAllDominantDynamic | BodyRank::sDominantDynamic;

			//go through all constraints connected to body
			PxU32 size = node0->body->getActorInteractionCount();
			Sc::Interaction** interactions = node0->body->getActorInteractions();
			while(size--)
			{
				Interaction* interaction = *interactions++;
				if (interaction->getType() == InteractionType::eCONSTRAINTSHADER)
				{
					ConstraintSim* c = static_cast<ConstraintInteraction*>(interaction)->getConstraint();
					rankConstraint(*c, br, dominanceTracking, constraintsToProjectCount);
				}
			}

			PX_ASSERT(br.rank);	//if it has no constraints then why is it in the constraint group?

			if (br.rank >= BodyRank::sPrimaryTreeRootMinRank)
				node0->raiseFlag(ConstraintGroupNode::eDISCOVERED);	// we create a tree for each node attached to a fixed anchor, or a node which is an all dominating dynamic
																	// -> make sure they do not include each other

			bodyRankArray.pushBack(br);
		}
		else
			node0->raiseFlag(ConstraintGroupNode::eDISCOVERED);  // a kinematic does not get projected, it might only get projected to and it is never part of a tree.

		node0 = node0->next;
	}

	root.setProjectionCountHint(constraintsToProjectCount);

	if (bodyRankArray.size())  // all of the bodies might have been switched to kinematic in which case there will be no ranked body
	{
		//sort bodyRankArray

		Ps::sort(&bodyRankArray.front(), bodyRankArray.size(), Ps::Greater<BodyRank>());

		ConstraintGroupNode** nodeQueue = reinterpret_cast<ConstraintGroupNode**>(PX_ALLOC_TEMP(sizeof(ConstraintGroupNode*)*bodyRankArray.size(), "ProjectionNodeQueue"));
		if (nodeQueue)
		{
			//build the projectionTree

			ConstraintGroupNode* firstProjectionTreeRoot = NULL;

			//go through it in sorted order

			//
			// bodies attached to fixed anchors with projecting constraints or all dominant rigid dynamics should get processed first.
			// For each of those we create a projection tree for sure, by extending one connectivity level from the root at a time. 
			// This way we make sure that scenarios like a bridge that is attached to fixed anchors at both ends breaks in the middle 
			// and not at one of the fixed anchors.
			//
			// this gets repeated for dominant dynamics. The reason for this is to cover cases where a dominant dynamic is connected to
			// a higher ranked node by a chain of two-way constraints. In such a case the two-way constraint should project the dominant 
			// dynamic towards the higher ranked node and not start a tree on its own.
			//
			PxU32 brIdx = 0;
			PxU32 stopIdx = bodyRankArray.size();
			PxU32 skipCount = 0;
			PxU32 ranksToProcess = BodyRank::sPrimaryTreeRootMinRank;
			ConstraintGroupNode** nodeQueueEnd;
			ConstraintGroupNode** nodeQueueCurrent;
			for(PxU32 i=0; i < 2; i++)
			{
				nodeQueueEnd = nodeQueue;
				while((brIdx < stopIdx) && (bodyRankArray[brIdx].rank >= ranksToProcess))
				{
					BodyRank& bRank = bodyRankArray[brIdx];
					PX_ASSERT((brIdx == 0) || (bRank.rank <= bodyRankArray[brIdx-1].rank));

					ConstraintGroupNode& node = *bRank.startingNode;
					PX_ASSERT(node.readFlag(ConstraintGroupNode::eDISCOVERED));

					node.initProjectionData(NULL, bRank.constraintToFixedAnchor);

					if (bRank.rank & (BodyRank::sAttachedToStatic | BodyRank::sAttachedToKinematic))
					{
						// for static/kinematic attached, the current node is already a child, so we must not traverse the neighborhood yet
						// but rather add the current node to the queue.
						PX_ASSERT(bRank.constraintToFixedAnchor);
						*nodeQueueEnd = &node;
						nodeQueueEnd++;
					}
					else
					{
						PX_ASSERT(!bRank.constraintToFixedAnchor);
						PxU32 addedNodeCount = projectionTreeBuildStep(node, bRank.constraintToFixedAnchor, nodeQueueEnd);
						nodeQueueEnd += addedNodeCount;
					}

					node.projectionNextRoot = firstProjectionTreeRoot;
					firstProjectionTreeRoot = &node;

					brIdx++;
				}

				// first neighbor connectivity level has been pushed to a queue for all chosen tree roots. Now extend the trees one level at a time.
				nodeQueueCurrent = nodeQueue;
				while(nodeQueueCurrent != nodeQueueEnd)
				{
					ConstraintGroupNode* node = *nodeQueueCurrent;
					PX_ASSERT(node->readFlag(ConstraintGroupNode::eDISCOVERED));
					nodeQueueCurrent++;

					PxU32 addedNodeCount = projectionTreeBuildStep(*node, node->projectionConstraint, nodeQueueEnd);
					nodeQueueEnd += addedNodeCount;
				}

				brIdx += skipCount;
				skipCount = 0;

				// find dominant dynamics that have not been discovered yet and arrange them in a consecutive block
				ranksToProcess = BodyRank::sOneWayProjection | BodyRank::sDominantDynamic;
				stopIdx = brIdx;
				PxU32 k = brIdx;
				while((k < bodyRankArray.size()) && (bodyRankArray[k].rank >= ranksToProcess))
				{
					ConstraintGroupNode* node = bodyRankArray[k].startingNode;
					if (!node->readFlag(ConstraintGroupNode::eDISCOVERED))
					{
						node->raiseFlag(ConstraintGroupNode::eDISCOVERED);
						bodyRankArray[stopIdx] = bodyRankArray[k];
						stopIdx++;
					}
					else
						skipCount++;
					
					k++;
				}
			}

			//
			// for every body that has not been discovered yet, we build a tree. Here we do not advance one connectivity level 
			// at a time because there should be no fight over the nodes among equal roots anymore (or rather no fight that could
			// break one-way projection in an unfair way).
			//
			PX_ASSERT((brIdx == 0) || (brIdx == bodyRankArray.size()) || (bodyRankArray[brIdx].rank < bodyRankArray[brIdx-1].rank));
			for(PxU32 i=brIdx; i < bodyRankArray.size(); i++)
			{
				nodeQueueEnd = nodeQueue;

				BodyRank& bRank = bodyRankArray[i];
				PX_ASSERT((i == brIdx) || (bRank.rank <= bodyRankArray[i-1].rank));
#ifdef _DEBUG
				if (bRank.rank & (BodyRank::sAttachedToStatic | BodyRank::sAttachedToKinematic))
				{ PX_ASSERT(bRank.constraintToFixedAnchor); }
				else
				{ PX_ASSERT(!bRank.constraintToFixedAnchor); }
#endif

				ConstraintGroupNode& node = *bRank.startingNode;
				if (!node.readFlag(ConstraintGroupNode::eDISCOVERED))
				{
					node.raiseFlag(ConstraintGroupNode::eDISCOVERED);

					PxU32 addedNodeCount = projectionTreeBuildStep(node, bRank.constraintToFixedAnchor, nodeQueueEnd);
					nodeQueueEnd += addedNodeCount;

					nodeQueueCurrent = nodeQueue;
					while(nodeQueueCurrent != nodeQueueEnd)
					{
						ConstraintGroupNode* n = *nodeQueueCurrent;
						PX_ASSERT(n->readFlag(ConstraintGroupNode::eDISCOVERED));
						PX_ASSERT(n->projectionConstraint);
						nodeQueueCurrent++;

						PxU32 nodeCount = projectionTreeBuildStep(*n, n->projectionConstraint, nodeQueueEnd);
						nodeQueueEnd += nodeCount;
					}

					node.projectionNextRoot = firstProjectionTreeRoot;
					firstProjectionTreeRoot = &node;
				}
			}

			root.setProjectionTreeRoot(firstProjectionTreeRoot);

			PX_FREE(nodeQueue);
		}
		else
			Ps::getFoundation().error(PxErrorCode::eOUT_OF_MEMORY, __FILE__, __LINE__, "Allocating projection node queue failed!");
	}
}


PxU32 Sc::ConstraintProjectionTree::projectionTreeBuildStep(ConstraintGroupNode& node, ConstraintSim* cToParent, ConstraintGroupNode** nodeQueue)
{
	PX_ASSERT(node.readFlag(ConstraintGroupNode::eDISCOVERED));

	PxU32 nodeQueueFillCount = 0;

	//go through all constraints attached to the body.
	BodySim* body = node.body;
	PxU32 size = body->getActorInteractionCount();
	Sc::Interaction** interactions = body->getActorInteractions();
	while(size--)
	{
		Interaction* interaction = *interactions++;
		if (interaction->getType() == InteractionType::eCONSTRAINTSHADER)
		{
			ConstraintSim* c = static_cast<ConstraintInteraction*>(interaction)->getConstraint();

			if (c != cToParent)	//don't go back along the edge we came from (not really necessary I guess since the ConstraintGroupNode::eDISCOVERED marker should solve this)
			{
				PxU32 projectToBody, projectToOtherBody;
				BodySim* neighbor;
				getConstraintStatus(*c, body, neighbor, projectToBody, projectToOtherBody);

				if(!isFixedBody(neighbor) && (!projectToOtherBody || projectToBody))	//just ignore the eventual constraint with environment over here. Body might be attached to multiple fixed anchors.
																						//Also make sure to ignore one-way projection that goes the opposite way.
				{
					ConstraintGroupNode* neighborNode = neighbor->getConstraintGroup();
					PX_ASSERT(neighborNode);

					if (!neighborNode->readFlag(ConstraintGroupNode::eDISCOVERED))
					{
						*nodeQueue = neighborNode;

						neighborNode->initProjectionData(&node, c);
						neighborNode->raiseFlag(ConstraintGroupNode::eDISCOVERED);	//flag body nodes that we process so we can detect loops

						nodeQueueFillCount++;
						nodeQueue++;
					}
				}
			}
		}
	}

	return nodeQueueFillCount;
}


void Sc::ConstraintProjectionTree::purgeProjectionTrees(ConstraintGroupNode& root)
{
	PX_ASSERT(&root == root.parent);
	PX_ASSERT(root.hasProjectionTreeRoot());

	// CA: New code (non recursive: recursive calls can cause stack overflow with huge trees)
	ConstraintGroupNode* projRoot = root.projectionFirstRoot;
	do
	{
		ConstraintGroupNode* currentNode = projRoot;
		projRoot = projRoot->projectionNextRoot;  // need to do it here because the info might get cleared below

		do
		{
			// Go down the tree until we find a leaf
			if (currentNode->projectionFirstChild)
			{
				currentNode = currentNode->projectionFirstChild;
				continue;
			}

			// Delete current node and go to next sibling or parent
			ConstraintGroupNode* nodeToDelete = currentNode;
			ConstraintGroupNode* parent = currentNode->projectionParent;
			currentNode = currentNode->projectionNextSibling;

			// Mark parent as leaf
			if (nodeToDelete->projectionParent)
				nodeToDelete->projectionParent->projectionFirstChild = NULL;

			// Clear projection info
			nodeToDelete->clearProjectionData();

			if (currentNode != NULL)
				continue;

			// No more siblings jump back to parent
			currentNode = parent;

		} while (currentNode != NULL);

	} while (projRoot != NULL);

	root.projectionFirstRoot = NULL;	// it can happen that the constraint graph root is not part of a projection tree (if it is a kinematic, for example) but it still points to the
										// first projection tree root and that needs to get cleaned up as well.
	PX_ASSERT(!root.projectionNextRoot);
	PX_ASSERT(!root.projectionParent);
	PX_ASSERT(!root.projectionFirstChild);
	PX_ASSERT(!root.projectionNextSibling);
	PX_ASSERT(!root.projectionConstraint);
}


void Sc::ConstraintProjectionTree::projectPoseForTree(ConstraintGroupNode& node, Ps::Array<BodySim*>& projectedBodies)
{
	// create a dummy node to keep the loops compact while covering the special case of the first node
	PX_ASSERT(node.body);
	ConstraintGroupNode dummyNode(*node.body);
	dummyNode.projectionNextSibling = &node;
	ConstraintGroupNode* currentNode = &dummyNode;

	// non recursive: recursive calls can cause stack overflow with huge trees
	do
	{
		ConstraintGroupNode* nextSiblingNode = currentNode->projectionNextSibling;

		while (nextSiblingNode)
		{
			currentNode = nextSiblingNode;
			ConstraintGroupNode* nextChildNode = currentNode;

			do
			{
				currentNode = nextChildNode;

				//-----------------------------------------------------------------------------
				ConstraintSim* c = currentNode->projectionConstraint;

				if (c && c->hasDynamicBody() && c->needsProjection())
				{
					c->projectPose(currentNode->body, projectedBodies);
				}
				//-----------------------------------------------------------------------------

				nextChildNode = currentNode->projectionFirstChild;

			} while (nextChildNode);

			nextSiblingNode = currentNode->projectionNextSibling;
		}

		currentNode = currentNode->projectionParent;

	} while (currentNode != NULL);
}


void Sc::ConstraintProjectionTree::projectPose(ConstraintGroupNode& root, Ps::Array<BodySim*>& projectedBodies)
{
	PX_ASSERT(&root == root.parent);
	PX_ASSERT(root.hasProjectionTreeRoot());

	ConstraintGroupNode* projRoot = root.projectionFirstRoot;
	do
	{
		projectPoseForTree(*projRoot, projectedBodies);
		projRoot = projRoot->projectionNextRoot;

	} while (projRoot != NULL);
}
