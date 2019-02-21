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

#include "foundation/PxMemory.h"
#include "SqIncrementalAABBTree.h"
#include "SqAABBTree.h"
#include "SqAABBTreeUpdateMap.h"
#include "SqBounds.h"
#include "GuBVHStructure.h"
#include "PsVecMath.h"
#include "PsFPU.h"

using namespace physx;
using namespace Sq;
using namespace Gu;
using namespace shdfnd::aos;

#define SUPPORT_TREE_ROTATION 1
#define DEALLOCATE_RESET 0

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

IncrementalAABBTree::IncrementalAABBTree():
	mIndicesPool("AABBTreeIndicesPool", 256),
	mNodesPool("AABBTreeNodesPool", 256	),
	mRoot(NULL)
{
	
}

IncrementalAABBTree::~IncrementalAABBTree()
{
	release();
}

void IncrementalAABBTree::release()
{
	if(mRoot)
	{
		releaseNode(mRoot);
		mRoot = NULL;
	}
}

void IncrementalAABBTree::releaseNode(IncrementalAABBTreeNode* node)
{
	PX_ASSERT(node);
	if(node->isLeaf())
	{
		mIndicesPool.deallocate(node->mIndices);
	}
	else
	{
		releaseNode(node->mChilds[0]);
		releaseNode(node->mChilds[1]);
	}
	if(!node->mParent)
	{
		mNodesPool.deallocate(reinterpret_cast<IncrementalAABBTreeNodePair*>(node));
		return;
	}
	if(node->mParent->mChilds[1] == node)
	{
		mNodesPool.deallocate(reinterpret_cast<IncrementalAABBTreeNodePair*>(node->mParent->mChilds[0]));
	}
}


// check if node is inside the given bounds
PX_FORCE_INLINE static bool nodeInsideBounds(const Vec4V& nodeMin, const Vec4V& nodeMax, const Vec4V& parentMin, const Vec4V& parentMax)
{
	return !(Ps::IntBool(V4AnyGrtr3(parentMin, nodeMin)) || Ps::IntBool(V4AnyGrtr3(nodeMax, parentMax)));
}

// update the node parent hierarchy, when insert happen, we can early exit when the node is inside its parent
// no further update is needed
PX_FORCE_INLINE static void updateHierarchyAfterInsert(IncrementalAABBTreeNode* node)
{
	IncrementalAABBTreeNode* parent = node->mParent;
	IncrementalAABBTreeNode* testNode = node;
	while(parent)
	{
		// check if we can early exit
		if(!nodeInsideBounds(testNode->mBVMin, testNode->mBVMax, parent->mBVMin, parent->mBVMax))
		{
			parent->mBVMin = V4Min(parent->mChilds[0]->mBVMin, parent->mChilds[1]->mBVMin);
			parent->mBVMax = V4Max(parent->mChilds[0]->mBVMax, parent->mChilds[1]->mBVMax);
		}
		else
			break;
		testNode = parent;
		parent = parent->mParent;
	}
}

// add an index into the leaf indices list and update the node bounds
PX_FORCE_INLINE static void addPrimitiveIntoNode(IncrementalAABBTreeNode* node, const PoolIndex index,  const Vec4V& minV,  const Vec4V& maxV)
{
	PX_ASSERT(node->isLeaf());
	AABBTreeIndices& nodeIndices = *node->mIndices;
	PX_ASSERT(nodeIndices.nbIndices < NB_OBJECTS_PER_NODE);

	// store the new handle
	nodeIndices.indices[nodeIndices.nbIndices++] = index;

	// increase the node bounds
	node->mBVMin = V4Min(node->mBVMin, minV);
	node->mBVMax = V4Max(node->mBVMax, maxV);

	updateHierarchyAfterInsert(node);
}

// check if node does intersect with given bounds
PX_FORCE_INLINE static bool nodeIntersection(IncrementalAABBTreeNode& node, const Vec4V& minV,  const Vec4V& maxV)
{
	return !(Ps::IntBool(V4AnyGrtr3(node.mBVMin, maxV)) || Ps::IntBool(V4AnyGrtr3(minV, node.mBVMax)));
}

// traversal strategy
PX_FORCE_INLINE static PxU32 traversalDirection(const IncrementalAABBTreeNode& child0, const IncrementalAABBTreeNode& child1, const Vec4V& testCenterV,
	bool testRotation, bool& rotateNode, PxU32& largesRotateNode)
{
	// traverse in the direction of a node which is closer
	// we compare the node and object centers
	const Vec4V centerCh0V = V4Add(child0.mBVMax, child0.mBVMin);
	const Vec4V centerCh1V = V4Add(child1.mBVMax, child1.mBVMin);

	const Vec4V ch0D = V4Sub(testCenterV, centerCh0V);
	const Vec4V ch1D = V4Sub(testCenterV, centerCh1V);

	if(testRotation)
	{
		// if some volume is 3x larger than we do a rotation
		const float volumeCompare = 3.0f;

		PX_ALIGN(16, PxVec4) sizeCh0;
		PX_ALIGN(16, PxVec4) sizeCh1;
		const Vec4V sizeCh0V = V4Sub(child0.mBVMax, child0.mBVMin);
		const Vec4V sizeCh1V = V4Sub(child1.mBVMax, child1.mBVMin);
		V4StoreA(sizeCh0V, &sizeCh0.x);
		V4StoreA(sizeCh1V, &sizeCh1.x);
		
		const float volumeCh0 = sizeCh0.x*sizeCh0.y*sizeCh0.z;
		const float volumeCh1 = sizeCh1.x*sizeCh1.y*sizeCh1.z;

		if((volumeCh0*volumeCompare < volumeCh1) || (volumeCh1*volumeCompare < volumeCh0))
		{
			largesRotateNode = (volumeCh0 > volumeCh1) ? 0u : 1u;
			rotateNode = true;
		}
	}

	const BoolV con = FIsGrtr(V4Dot3(ch0D, ch0D), V4Dot3(ch1D, ch1D));
	return (BAllEqTTTT(con) == 1) ? PxU32(1) : PxU32(0);
}

// remove an index from the leaf
PX_FORCE_INLINE static void removePrimitiveFromNode(IncrementalAABBTreeNode* node, const PoolIndex index)
{
	AABBTreeIndices& indices = *node->mIndices;
	PX_ASSERT(indices.nbIndices > 1);

	for (PxU32 i = indices.nbIndices; i--; )
	{
		if(node->mIndices->indices[i] == index)
		{
			node->mIndices->indices[i] = node->mIndices->indices[--indices.nbIndices];
			return;
		}
	}
	// if handle was not found something is wrong here
	PX_ASSERT(0);
}

// check if bounds are equal with given node min/max
PX_FORCE_INLINE static bool boundsEqual(const Vec4V& testMin, const Vec4V& testMax, const Vec4V& nodeMin, const Vec4V& nodeMax)
{
	return (Ps::IntBool(V4AllEq(nodeMin, testMin)) && Ps::IntBool(V4AllEq(testMax, nodeMax)));
}

// update the node hierarchy bounds when remove happen, we can early exit if the bounds are equal and no bounds update
// did happen
PX_FORCE_INLINE static void updateHierarchyAfterRemove(IncrementalAABBTreeNode* node, const PxBounds3* bounds)
{	
	if(node->isLeaf())
	{		
		const AABBTreeIndices& indices = *node->mIndices;
		PX_ASSERT(indices.nbIndices > 0);

		Vec4V bvMin = V4LoadU(&bounds[indices.indices[0]].minimum.x);
		Vec4V bvMax = V4LoadU(&bounds[indices.indices[0]].maximum.x);
		for(PxU32 i = 1; i < indices.nbIndices; i++)
		{
			const Vec4V minV = V4LoadU(&bounds[indices.indices[i]].minimum.x);
			const Vec4V maxV = V4LoadU(&bounds[indices.indices[i]].maximum.x);

			bvMin = V4Min(bvMin, minV);
			bvMax = V4Max(bvMax, maxV);
		}

		node->mBVMin = V4ClearW(bvMin);
		node->mBVMax = V4ClearW(bvMax);
	}
	else
	{
		node->mBVMin = V4Min(node->mChilds[0]->mBVMin, node->mChilds[1]->mBVMin);
		node->mBVMax = V4Max(node->mChilds[0]->mBVMax, node->mChilds[1]->mBVMax);
	}

	IncrementalAABBTreeNode* parent = node->mParent;
	while(parent)
	{
		const Vec4V newMinV = V4Min(parent->mChilds[0]->mBVMin, parent->mChilds[1]->mBVMin);
		const Vec4V newMaxV = V4Max(parent->mChilds[0]->mBVMax, parent->mChilds[1]->mBVMax);

		const bool earlyExit = boundsEqual(newMinV, newMaxV, parent->mBVMin, parent->mBVMax);
		if(earlyExit)
			break;

		parent->mBVMin = newMinV;
		parent->mBVMax = newMaxV;

		parent = parent->mParent;
	}
}

// split the leaf node along the most significant axis
IncrementalAABBTreeNode* IncrementalAABBTree::splitLeafNode(IncrementalAABBTreeNode* node, const PoolIndex index,  const Vec4V& minV,  const Vec4V& maxV, const PxBounds3* bounds)
{
	PX_ASSERT(node->isLeaf());

	IncrementalAABBTreeNode* returnNode = NULL;

	// create new pairs of nodes, parent will remain the node (the one we split)
	IncrementalAABBTreeNode* child0 = reinterpret_cast<IncrementalAABBTreeNode*>(mNodesPool.allocate());
	IncrementalAABBTreeNode* child1 = child0 + 1;	
	AABBTreeIndices* newIndices = mIndicesPool.allocate();

	// get the split axis
	PX_ALIGN(16, PxVec4) vars;
	PX_ALIGN(16, PxVec4) center;
	const float half = 0.5f;
	const FloatV halfV = FLoad(half);
	const Vec4V newMinV = V4Min(node->mBVMin, minV);
	const Vec4V newMaxV = V4Max(node->mBVMax, maxV);
	const Vec4V centerV = V4Scale(V4Add(newMaxV, newMinV), halfV);
	const Vec4V varsV = V4Sub(newMaxV, newMinV);
	V4StoreA(varsV, &vars.x);
	V4StoreA(centerV, &center.x);
	const PxU32 axis = Ps::largestAxis(PxVec3(vars.x, vars.y, vars.z));

	// setup parent
	child0->mParent = node;
	child1->mParent = node;
	child0->mIndices = node->mIndices;
	child0->mChilds[1] = NULL;
	child1->mIndices = newIndices;
	child1->mChilds[1] = NULL;

	AABBTreeIndices& child0Indices = *child0->mIndices;	// the original node indices
	AABBTreeIndices& child1Indices = *child1->mIndices; // new empty indices
	child1Indices.nbIndices = 0;

	// split the node
	for(PxU32 i = child0Indices.nbIndices; i--;)
	{
		const PxBounds3& primitiveBounds = bounds[child0Indices.indices[i]];
		const float pCenter = primitiveBounds.getCenter(axis);
		if(center[axis] >= pCenter)
		{
			// move to new node
			child1Indices.indices[child1Indices.nbIndices++] = child0Indices.indices[i];						
			child0Indices.nbIndices--;
			child0Indices.indices[i] = child0Indices.indices[child0Indices.nbIndices];
		}
	}

	// check where to put the new node, if there is still a free space
	if(child0Indices.nbIndices == 0 || child1Indices.nbIndices == NB_OBJECTS_PER_NODE)
	{
		child0Indices.nbIndices = 1;
		child0Indices.indices[0] = index;
		returnNode = child0;
	}
	else
	{
		if(child0Indices.nbIndices == NB_OBJECTS_PER_NODE)
		{
			child1Indices.nbIndices = 1;
			child1Indices.indices[0] = index;
			returnNode = child1;
		}
		else
		{
			const PxBounds3& primitiveBounds = bounds[index];
			const float pCenter = primitiveBounds.getCenter(axis);
			if(center[axis] >= pCenter)
			{
				// move to new node
				child1Indices.indices[child1Indices.nbIndices++] = index;
				returnNode = child1;
			}
			else
			{
				// move to old node
				child0Indices.indices[child0Indices.nbIndices++] = index;	
				returnNode = child0;
			}
		}
	}

	// update bounds for the new nodes
	Vec4V bvMin = V4LoadU(&bounds[child0Indices.indices[0]].minimum.x);
	Vec4V bvMax = V4LoadU(&bounds[child0Indices.indices[0]].maximum.x);	
	for(PxU32 i = 1; i < child0Indices.nbIndices; i++)
	{
		const Vec4V nodeMinV = V4LoadU(&bounds[child0Indices.indices[i]].minimum.x);
		const Vec4V nodeMaxV = V4LoadU(&bounds[child0Indices.indices[i]].maximum.x);	

		bvMin = V4Min(bvMin, nodeMinV);
		bvMax = V4Max(bvMax, nodeMaxV);
	}
	child0->mBVMin = V4ClearW(bvMin);
	child0->mBVMax = V4ClearW(bvMax);	

	bvMin = V4LoadU(&bounds[child1Indices.indices[0]].minimum.x);
	bvMax = V4LoadU(&bounds[child1Indices.indices[0]].maximum.x);	
	for(PxU32 i = 1; i < child1Indices.nbIndices; i++)
	{
		const Vec4V nodeMinV = V4LoadU(&bounds[child1Indices.indices[i]].minimum.x);
		const Vec4V nodeMaxV = V4LoadU(&bounds[child1Indices.indices[i]].maximum.x);	

		bvMin = V4Min(bvMin, nodeMinV);
		bvMax = V4Max(bvMax, nodeMaxV);
	}
	child1->mBVMin = V4ClearW(bvMin);
	child1->mBVMax = V4ClearW(bvMax);

	// node parent is the same, setup the new childs
	node->mChilds[0] = child0;
	node->mChilds[1] = child1;
	node->mBVMin = newMinV;
	node->mBVMax = newMaxV;

	updateHierarchyAfterInsert(node);

	PX_ASSERT(returnNode);
	return returnNode;
}

void IncrementalAABBTree::rotateTree(IncrementalAABBTreeNode* node, NodeList& changedLeaf, PxU32 largesRotateNodeIn, const PxBounds3* bounds, bool rotateAgain)
{
	PX_ASSERT(!node->isLeaf());	

	IncrementalAABBTreeNode* smallerNode = node->mChilds[(largesRotateNodeIn == 0) ? 1 : 0];
	IncrementalAABBTreeNode* largerNode = node->mChilds[largesRotateNodeIn];
	PX_ASSERT(!largerNode->isLeaf());

	// take a leaf from larger node and add it to the smaller node
	const Vec4V testCenterV = V4Add(smallerNode->mBVMax, smallerNode->mBVMin);
	IncrementalAABBTreeNode* rotationNode = NULL;	// store a node that seems not balanced
	PxU32 largesRotateNode = 0;
	bool rotateNode = false; 	
	PxU32 traversalIndex = traversalDirection(*largerNode->mChilds[0], *largerNode->mChilds[1], testCenterV, false, rotateNode, largesRotateNode);
	IncrementalAABBTreeNode* closestNode = largerNode->mChilds[traversalIndex];
	while(!closestNode->isLeaf())
	{
		Ps::prefetchLine(closestNode->mChilds[0]->mChilds[0]);
		Ps::prefetchLine(closestNode->mChilds[1]->mChilds[0]);

		traversalIndex = traversalDirection(*closestNode->mChilds[0], *closestNode->mChilds[1], testCenterV, false, rotateNode, largesRotateNode);
		closestNode = closestNode->mChilds[traversalIndex];
	}
	
	// we have the leaf that we want to rotate
	// create new parent and remove the current leaf
	changedLeaf.findAndReplaceWithLast(closestNode);
	IncrementalAABBTreeNode* parent = closestNode->mParent;
	IncrementalAABBTreeNodePair* removedPair = reinterpret_cast<IncrementalAABBTreeNodePair*>(parent->mChilds[0]);
	PX_ASSERT(!parent->isLeaf());

	// copy the remaining child into parent
	IncrementalAABBTreeNode* remainingChild = (parent->mChilds[0] == closestNode) ? parent->mChilds[1] : parent->mChilds[0];
	parent->mBVMax = remainingChild->mBVMax;
	parent->mBVMin = remainingChild->mBVMin;
	if(remainingChild->isLeaf())
	{
		parent->mIndices = remainingChild->mIndices;
		parent->mChilds[1] = NULL;
		changedLeaf.findAndReplaceWithLast(remainingChild);
		changedLeaf.pushBack(parent);
	}
	else
	{
		parent->mChilds[0] = remainingChild->mChilds[0];
		parent->mChilds[0]->mParent = parent;
		parent->mChilds[1] = remainingChild->mChilds[1];
		parent->mChilds[1]->mParent = parent;
	}

	// update the hieararchy after the node removal
	if(parent->mParent)
	{
		updateHierarchyAfterRemove(parent->mParent, bounds);
	}

	// find new spot for the node
	// take a leaf from larger node and add it to the smaller node
	IncrementalAABBTreeNode* newSpotNode = NULL;
	if(smallerNode->isLeaf())
	{
		newSpotNode = smallerNode;
	}
	else
	{
		const Vec4V testClosestNodeCenterV = V4Add(closestNode->mBVMax, closestNode->mBVMin);
		rotationNode = NULL;	// store a node that seems not balanced
		largesRotateNode = 0;
		rotateNode = false;
		bool testRotation = rotateAgain;
		traversalIndex = traversalDirection(*smallerNode->mChilds[0], *smallerNode->mChilds[1], testClosestNodeCenterV, testRotation, rotateNode, largesRotateNode);
		if(rotateNode && !smallerNode->mChilds[largesRotateNode]->isLeaf())
		{
			rotationNode = smallerNode;
			testRotation = false;
		}
		newSpotNode = smallerNode->mChilds[traversalIndex];
		while(!newSpotNode->isLeaf())
		{
			Ps::prefetchLine(newSpotNode->mChilds[0]->mChilds[0]);
			Ps::prefetchLine(newSpotNode->mChilds[1]->mChilds[0]);

			traversalIndex = traversalDirection(*newSpotNode->mChilds[0], *newSpotNode->mChilds[1], testClosestNodeCenterV, testRotation, rotateNode, largesRotateNode);
			if(!rotationNode && rotateNode && !newSpotNode->mChilds[largesRotateNode]->isLeaf())
			{
				rotationNode = newSpotNode;
				testRotation = false;
			}
			newSpotNode = newSpotNode->mChilds[traversalIndex];
		}
	}

	// we have the closest leaf in the smaller child, lets merge it with the closestNode
	if(newSpotNode->getNbPrimitives() + closestNode->getNbPrimitives() <= NB_OBJECTS_PER_NODE)
	{
		// all primitives fit into new spot, we merge here simply
		AABBTreeIndices* targetIndices = newSpotNode->mIndices;
		const AABBTreeIndices* sourceIndices = closestNode->mIndices;
		for(PxU32 i = 0; i < sourceIndices->nbIndices; i++)
		{
			targetIndices->indices[targetIndices->nbIndices++] = sourceIndices->indices[i];			
		}
		PX_ASSERT(targetIndices->nbIndices <= NB_OBJECTS_PER_NODE);
		if(changedLeaf.find(newSpotNode) == changedLeaf.end())
			changedLeaf.pushBack(newSpotNode);
		mIndicesPool.deallocate(closestNode->mIndices);

		newSpotNode->mBVMin = V4Min(newSpotNode->mBVMin, closestNode->mBVMin);
		newSpotNode->mBVMax = V4Max(newSpotNode->mBVMax, closestNode->mBVMax);
		updateHierarchyAfterInsert(newSpotNode);		
	}
	else
	{
		// we need to make new parent with newSpotNode and closestNode as childs
		// create new pairs of nodes, parent will remain the node (the one we split)
		IncrementalAABBTreeNode* child0 = reinterpret_cast<IncrementalAABBTreeNode*>(mNodesPool.allocate());
		IncrementalAABBTreeNode* child1 = child0 + 1;	

		// setup parent
		child0->mParent = newSpotNode;
		child1->mParent = newSpotNode;
		child0->mIndices = newSpotNode->mIndices;
		child0->mChilds[1] = NULL;
		child0->mBVMin = newSpotNode->mBVMin;
		child0->mBVMax = newSpotNode->mBVMax;
		child1->mIndices = closestNode->mIndices;
		child1->mChilds[1] = NULL;
		child1->mBVMin = closestNode->mBVMin;
		child1->mBVMax = closestNode->mBVMax;

		// node parent is the same, setup the new childs
		newSpotNode->mChilds[0] = child0;
		newSpotNode->mChilds[1] = child1;

		newSpotNode->mBVMin = V4Min(child0->mBVMin, child1->mBVMin);
		newSpotNode->mBVMax = V4Max(child0->mBVMax, child1->mBVMax);

		updateHierarchyAfterInsert(newSpotNode);

		changedLeaf.findAndReplaceWithLast(newSpotNode);
		changedLeaf.pushBack(child0);
		changedLeaf.pushBack(child1);
	}

	// deallocate the closestNode, it has been moved
#if DEALLOCATE_RESET
	removedPair->mNode0.mChilds[0] = NULL;
	removedPair->mNode0.mChilds[1] = NULL;

	removedPair->mNode1.mChilds[0] = NULL;
	removedPair->mNode1.mChilds[1] = NULL;
#endif
	mNodesPool.deallocate(removedPair);

	// try to do one more rotation for the newly added node part of tree
	if(rotationNode)
	{
		rotateTree(rotationNode, changedLeaf, largesRotateNode, bounds, false);
	}
}


// insert new bounds into tree
IncrementalAABBTreeNode* IncrementalAABBTree::insert(const PoolIndex index, const PxBounds3* bounds, NodeList& changedLeaf)
{	
	PX_SIMD_GUARD;

	// get the bounds, reset the W value
	const Vec4V minV = V4ClearW(V4LoadU(&bounds[index].minimum.x));
	const Vec4V maxV = V4ClearW(V4LoadU(&bounds[index].maximum.x));

	// check if tree is empty
	if(!mRoot)
	{
		// make it a leaf
		AABBTreeIndices* indices = mIndicesPool.construct(index);
		mRoot = reinterpret_cast<IncrementalAABBTreeNode*> (mNodesPool.allocate());
		mRoot->mBVMin = minV;
		mRoot->mBVMax = maxV;
		mRoot->mIndices = indices;
		mRoot->mChilds[1] = NULL;
		mRoot->mParent = NULL;
		
		return mRoot;
	}
	else
	{
		// check if root is a leaf
		if(mRoot->isLeaf())
		{
			// if we still can insert the primitive into the leaf, or we need to split			
			if(mRoot->getNbPrimitives() < NB_OBJECTS_PER_NODE)
			{
				// simply add the primitive into the current leaf
				addPrimitiveIntoNode(mRoot, index, minV, maxV);
				return mRoot;
			}
			else
			{
				// need to split the node
				// check if the leaf is not marked as changed, we need to remove it
				if(!changedLeaf.empty())
				{
					PX_ASSERT(changedLeaf.size() == 1);
					if(changedLeaf[0] == mRoot)
						changedLeaf.popBack();
				}
				IncrementalAABBTreeNode* retNode = splitLeafNode(mRoot, index, minV, maxV, bounds);
				mRoot = retNode->mParent;
				IncrementalAABBTreeNode* sibling = (mRoot->mChilds[0] == retNode) ? mRoot->mChilds[1] : mRoot->mChilds[0];
				if(sibling->isLeaf())
					changedLeaf.pushBack(sibling);
				changedLeaf.pushBack(retNode);
				return retNode;
			}
		}
		else
		{
			const Vec4V testCenterV = V4Add(maxV, minV);
			IncrementalAABBTreeNode* returnNode = NULL;
			IncrementalAABBTreeNode* rotationNode = NULL;	// store a node that seems not balanced
			PxU32 largesRotateNode = 0;
			bool rotateNode = false;
#if SUPPORT_TREE_ROTATION
			bool testRotation = true;
#else
			bool testRotation = false;
#endif
 			// we dont need to modify root, lets traverse the tree to find the right spot
			PxU32 traversalIndex = traversalDirection(*mRoot->mChilds[0], *mRoot->mChilds[1], testCenterV, testRotation, rotateNode, largesRotateNode);
			if(rotateNode && !mRoot->mChilds[largesRotateNode]->isLeaf())
			{
				rotationNode = mRoot;
				testRotation = false;
			}
			IncrementalAABBTreeNode* baseNode = mRoot->mChilds[traversalIndex];
			while(!baseNode->isLeaf())
			{
				Ps::prefetchLine(baseNode->mChilds[0]->mChilds[0]);
				Ps::prefetchLine(baseNode->mChilds[1]->mChilds[0]);

				traversalIndex = traversalDirection(*baseNode->mChilds[0], *baseNode->mChilds[1], testCenterV, testRotation, rotateNode, largesRotateNode);
				if(!rotationNode && rotateNode && !baseNode->mChilds[largesRotateNode]->isLeaf())
				{
					rotationNode = baseNode;
					testRotation = false;
				}
				baseNode = baseNode->mChilds[traversalIndex];
			}
			
			// if we still can insert the primitive into the leaf, or we need to split			
			if(baseNode->getNbPrimitives() < NB_OBJECTS_PER_NODE)
			{
				// simply add the primitive into the current leaf
				addPrimitiveIntoNode(baseNode, index, minV, maxV);
				returnNode = baseNode;
				if(!changedLeaf.empty())
				{
					PX_ASSERT(changedLeaf.size() == 1);
					if(changedLeaf[0] != baseNode)
						changedLeaf.pushBack(baseNode);
				}
				else
					changedLeaf.pushBack(baseNode);
			}
			else
			{
				// split
				// check if the leaf is not marked as changed, we need to remove it
				if(!changedLeaf.empty())
				{
					PX_ASSERT(changedLeaf.size() == 1);
					if(changedLeaf[0] == baseNode)
						changedLeaf.popBack();
				}
				IncrementalAABBTreeNode* retNode = splitLeafNode(baseNode, index, minV, maxV, bounds);
				const IncrementalAABBTreeNode* splitParent = retNode->mParent;
				changedLeaf.pushBack(splitParent->mChilds[0]);
				changedLeaf.pushBack(splitParent->mChilds[1]);

				returnNode = retNode;
			}

			if(rotationNode)
			{
				rotateTree(rotationNode, changedLeaf, largesRotateNode, bounds, true);
				returnNode = NULL;
			}

			return returnNode;
		}
	}
}

// update the index, do a full remove/insert update
IncrementalAABBTreeNode* IncrementalAABBTree::update(IncrementalAABBTreeNode* node, const PoolIndex index, const PxBounds3* bounds, NodeList& changedLeaf)
{
	PX_SIMD_GUARD;

	IncrementalAABBTreeNode* removedNode = remove(node, index, bounds);
	if(removedNode && removedNode->isLeaf())
	{
		changedLeaf.pushBack(removedNode);
	}
	return insert(index, bounds, changedLeaf);	
}

// update the index, faster version with a lazy update of objects that moved just a bit
IncrementalAABBTreeNode* IncrementalAABBTree::updateFast(IncrementalAABBTreeNode* node, const PoolIndex index, const PxBounds3* bounds, NodeList& changedLeaf)
{
	PX_SIMD_GUARD;

	const Vec4V minV = V4ClearW(V4LoadU(&bounds[index].minimum.x));
	const Vec4V maxV = V4ClearW(V4LoadU(&bounds[index].maximum.x));

	// for update fast, we dont care if the tree gets slowly unbalanced, we are building a new tree already
	if(nodeIntersection(*node, minV, maxV))
	{
		updateHierarchyAfterRemove(node, bounds);
		return node;
	}
	else
	{
		IncrementalAABBTreeNode* removedNode = remove(node, index, bounds);	
		if(removedNode && removedNode->isLeaf())
		{
			changedLeaf.pushBack(removedNode);
		}
		return insert(index, bounds, changedLeaf);	
	}
}

// remove primitive from the tree, return a node if it moved to its parent
IncrementalAABBTreeNode* IncrementalAABBTree::remove(IncrementalAABBTreeNode* node, const PoolIndex index, const PxBounds3* bounds)
{
	PX_SIMD_GUARD;
	PX_ASSERT(node->isLeaf());
	// if we just remove the primitive from the list
	if(node->getNbPrimitives() > 1)
	{
		removePrimitiveFromNode(node, index);

		// update the hierarchy		
		updateHierarchyAfterRemove(node, bounds);
		return NULL;
	}
	else
	{
		// if root node and the last primitive remove root
		if(node == mRoot)
		{
#if DEALLOCATE_RESET
			IncrementalAABBTreeNodePair* removedPair = reinterpret_cast<IncrementalAABBTreeNodePair*>(node);
			removedPair->mNode0.mChilds[0] = NULL;
			removedPair->mNode0.mChilds[1] = NULL;

			removedPair->mNode1.mChilds[0] = NULL;
			removedPair->mNode1.mChilds[1] = NULL;
#endif
			mNodesPool.deallocate(reinterpret_cast<IncrementalAABBTreeNodePair*>(node));
			mRoot = NULL;
			return NULL;
		}
		else
		{
			// create new parent and remove the current leaf
			IncrementalAABBTreeNode* parent = node->mParent;
			IncrementalAABBTreeNodePair* removedPair = reinterpret_cast<IncrementalAABBTreeNodePair*>(parent->mChilds[0]);
			PX_ASSERT(!parent->isLeaf());

			// copy the remaining child into parent
			IncrementalAABBTreeNode* remainingChild = (parent->mChilds[0] == node) ? parent->mChilds[1] : parent->mChilds[0];
			parent->mBVMax = remainingChild->mBVMax;
			parent->mBVMin = remainingChild->mBVMin;
			if(remainingChild->isLeaf())
			{
				parent->mIndices = remainingChild->mIndices;
				parent->mChilds[1] = NULL;
			}
			else
			{
				parent->mChilds[0] = remainingChild->mChilds[0];
				parent->mChilds[0]->mParent = parent;
				parent->mChilds[1] = remainingChild->mChilds[1];
				parent->mChilds[1]->mParent = parent;
			}

			if(parent->mParent)
			{
				updateHierarchyAfterRemove(parent->mParent, bounds);
			}

			mIndicesPool.deallocate(node->mIndices);
#if DEALLOCATE_RESET
			removedPair->mNode0.mChilds[0] = NULL;
			removedPair->mNode0.mChilds[1] = NULL;

			removedPair->mNode1.mChilds[0] = NULL;
			removedPair->mNode1.mChilds[1] = NULL;
#endif
			mNodesPool.deallocate(removedPair);	
			return parent;
		}
	}
}

// fixup the indices
void IncrementalAABBTree::fixupTreeIndices(IncrementalAABBTreeNode* node, const PoolIndex index, const PoolIndex newIndex)
{
	PX_ASSERT(node->isLeaf());

	AABBTreeIndices& indices = *node->mIndices;
	for(PxU32 i = 0; i < indices.nbIndices; i++)
	{
		if(indices.indices[i] == index)			
		{
			indices.indices[i] = newIndex;
			return;
		}
	}	
	PX_ASSERT(0);
}

// shift node
static void shiftNode(IncrementalAABBTreeNode* node, const Vec4V& shiftV)
{
	node->mBVMax = V4Sub(node->mBVMax, shiftV);
	node->mBVMin = V4Sub(node->mBVMin, shiftV);

	if(!node->isLeaf())
	{
		shiftNode(node->mChilds[0], shiftV);
		shiftNode(node->mChilds[1], shiftV);
	}
}

// shift origin
void IncrementalAABBTree::shiftOrigin(const PxVec3& shift)
{
	if(mRoot)
	{
		const Vec4V shiftV = V4ClearW(V4LoadU(&shift.x));

		shiftNode(mRoot, shiftV);
	}
}

static void checkNode(IncrementalAABBTreeNode* node, IncrementalAABBTreeNode* parent, const PxBounds3* bounds, PoolIndex maxIndex, PxU32& numIndices, PxU32& numNodes)
{
	PX_ASSERT(node->mParent == parent);
	PX_ASSERT(!parent->isLeaf());
	PX_ASSERT(parent->mChilds[0] == node || parent->mChilds[1] == node);

	numNodes++;
	if(!node->isLeaf())
	{
		PX_ASSERT(nodeInsideBounds(node->mChilds[0]->mBVMin, node->mChilds[0]->mBVMax, node->mBVMin, node->mBVMax));
		PX_ASSERT(nodeInsideBounds(node->mChilds[1]->mBVMin, node->mChilds[1]->mBVMax, node->mBVMin, node->mBVMax));

		const Vec4V testMinV = V4Min(parent->mChilds[0]->mBVMin, parent->mChilds[1]->mBVMin);
		const Vec4V testMaxV = V4Max(parent->mChilds[0]->mBVMax, parent->mChilds[1]->mBVMax);

		PX_UNUSED(testMinV);
		PX_UNUSED(testMaxV);
		PX_ASSERT(nodeInsideBounds(node->mBVMin, node->mBVMax, testMinV, testMaxV));

		checkNode(node->mChilds[0], node, bounds, maxIndex, numIndices, numNodes);
		checkNode(node->mChilds[1], node, bounds, maxIndex, numIndices, numNodes);
	}
	else
	{
		const AABBTreeIndices& indices = *node->mIndices;
		PX_ASSERT(indices.nbIndices);
		Vec4V testMinV = V4ClearW(V4LoadU(&bounds[indices.indices[0]].minimum.x));
		Vec4V testMaxV = V4ClearW(V4LoadU(&bounds[indices.indices[0]].maximum.x));
		for(PxU32 i = 0; i < indices.nbIndices; i++)
		{
			PX_ASSERT(indices.indices[i] < maxIndex);
			numIndices++;

			const Vec4V minV = V4ClearW(V4LoadU(&bounds[indices.indices[i]].minimum.x));
			const Vec4V maxV = V4ClearW(V4LoadU(&bounds[indices.indices[i]].maximum.x));

			testMinV = V4Min(testMinV, minV);
			testMaxV = V4Max(testMaxV, maxV);

			PX_ASSERT(nodeInsideBounds(minV, maxV, node->mBVMin, node->mBVMax));
		}

		PX_ASSERT(boundsEqual(testMinV, testMaxV, node->mBVMin, node->mBVMax));
	}
}

void IncrementalAABBTree::hierarchyCheck(PoolIndex maxIndex, const PxBounds3* bounds)
{
	PxU32 numHandles = 0;
	PxU32 numPosNodes = 0;
	PxU32 numNegNodes = 0;
	if(mRoot && !mRoot->isLeaf())
	{
		checkNode(mRoot->mChilds[0], mRoot, bounds, maxIndex, numHandles, numPosNodes);
		checkNode(mRoot->mChilds[1], mRoot, bounds, maxIndex, numHandles, numNegNodes);

		PX_ASSERT(numHandles == maxIndex);
	}	
}

void IncrementalAABBTree::hierarchyCheck(const PxBounds3* bounds)
{
	PxU32 numHandles = 0;
	PxU32 numPosNodes = 0;
	PxU32 numNegNodes = 0;
	if(mRoot && !mRoot->isLeaf())
	{
		checkNode(mRoot->mChilds[0], mRoot, bounds, 0xFFFFFFFF, numHandles, numPosNodes);
		checkNode(mRoot->mChilds[1], mRoot, bounds, 0xFFFFFFFF, numHandles, numNegNodes);
	}	
}

void IncrementalAABBTree::checkTreeLeaf(IncrementalAABBTreeNode* leaf, PoolIndex h)
{
	PX_ASSERT(leaf->isLeaf());

	const AABBTreeIndices& indices = *leaf->mIndices;
	bool found = false;
	for(PxU32 i = 0; i < indices.nbIndices; i++)
	{
		if(indices.indices[i] == h)
		{
			found = true;
			break;
		}
	}
	PX_UNUSED(found);
	PX_ASSERT(found);
}

PxU32 IncrementalAABBTree::getTreeLeafDepth(IncrementalAABBTreeNode* leaf)
{
	PxU32 depth = 1;
	IncrementalAABBTreeNode* parent = leaf->mParent;
	while(parent)
	{
		depth++;
		parent = parent->mParent;
	}
	return depth;
}

// build the tree from given bounds
bool IncrementalAABBTree::build(AABBTreeBuildParams& params, Ps::Array<IncrementalAABBTreeNode*>& mapping)
{
	// Init stats
	BuildStats stats;
	const PxU32 nbPrimitives = params.mNbPrimitives;
	if (!nbPrimitives)
		return false;

	PxU32* indices = NULL; 
	const bool buildStatus = buildAABBTree(params, mNodeAllocator, stats, indices);
	PX_UNUSED(buildStatus);
	PX_ASSERT(buildStatus);

	PX_FREE_AND_RESET(params.mCache);

	IncrementalAABBTreeNode** treeNodes = reinterpret_cast<IncrementalAABBTreeNode**>(PX_ALLOC(sizeof(IncrementalAABBTreeNode*)*(stats.getCount()), "temp node helper array"));
	PxMemSet(treeNodes, 0, sizeof(IncrementalAABBTreeNode*)*(stats.getCount()));

	clone(mapping, indices, treeNodes);
	mRoot = treeNodes[0];
	mRoot->mParent = NULL;
	
	PX_FREE_AND_RESET(indices);
	PX_FREE_AND_RESET(treeNodes);

	mNodeAllocator.release();
	return true;
}

// clone the tree, the tree is computed in the NodeAllocator, similar to AABBTree flatten
void IncrementalAABBTree::clone(Ps::Array<IncrementalAABBTreeNode*>& mapping, const PxU32* _indices, IncrementalAABBTreeNode** treeNodes)
{		
	PxU32 offset = 0;
	const PxU32 nbSlabs = mNodeAllocator.mSlabs.size();
	for (PxU32 s = 0; s<nbSlabs; s++)
	{
		const NodeAllocator::Slab& currentSlab = mNodeAllocator.mSlabs[s];

		AABBTreeBuildNode* pool = currentSlab.mPool;
		for (PxU32 i = 0; i < currentSlab.mNbUsedNodes; i++)
		{
			IncrementalAABBTreeNode* destNode = treeNodes[offset];
			if(!destNode)
			{
				destNode = reinterpret_cast<IncrementalAABBTreeNode*>(mNodesPool.allocate());
				treeNodes[offset] = destNode;
			}

			destNode->mBVMin = V4ClearW(V4LoadU(&pool[i].mBV.minimum.x));
			destNode->mBVMax = V4ClearW(V4LoadU(&pool[i].mBV.maximum.x));

			if (pool[i].isLeaf())
			{
				AABBTreeIndices* indices = mIndicesPool.allocate();
				destNode->mIndices = indices;
				destNode->mChilds[1] = NULL;
				indices->nbIndices = pool[i].getNbPrimitives();
				PX_ASSERT(indices->nbIndices <= 16);
				const PxU32* sourceIndices = _indices + pool[i].mNodeIndex;
				for (PxU32 iIndices = 0; iIndices < indices->nbIndices; iIndices++)
				{
					const PxU32 sourceIndex = sourceIndices[iIndices];
					indices->indices[iIndices] = sourceIndex;
					PX_ASSERT(sourceIndex < mapping.size());
					mapping[sourceIndex] = destNode;
				}
			}
			else
			{
				PX_ASSERT(pool[i].mPos);
				PxU32 localNodeIndex = 0xffffffff;
				PxU32 nodeBase = 0;
				for (PxU32 j = 0; j<nbSlabs; j++)
				{
					if (pool[i].mPos >= mNodeAllocator.mSlabs[j].mPool && pool[i].mPos < mNodeAllocator.mSlabs[j].mPool + mNodeAllocator.mSlabs[j].mNbUsedNodes)
					{
						localNodeIndex = PxU32(pool[i].mPos - mNodeAllocator.mSlabs[j].mPool);
						break;
					}
					nodeBase += mNodeAllocator.mSlabs[j].mNbUsedNodes;
				}
				const PxU32 nodeIndex = nodeBase + localNodeIndex;

				IncrementalAABBTreeNode* child0 = treeNodes[nodeIndex];
				IncrementalAABBTreeNode* child1 = treeNodes[nodeIndex + 1];
				if(!child0)
				{
					PX_ASSERT(!child1);
					child0 = reinterpret_cast<IncrementalAABBTreeNode*>(mNodesPool.allocate());
					child1 = child0 + 1;
					treeNodes[nodeIndex] = child0;
					treeNodes[nodeIndex + 1] = child1;
				}

				destNode->mChilds[0] = child0;
				destNode->mChilds[1] = child1;
				child0->mParent = destNode;
				child1->mParent = destNode;
			}
			offset++;
		}
	}
}

void IncrementalAABBTree::copyNode(IncrementalAABBTreeNode& destNode, const BVHNode& sourceNode, 
		const BVHNode* nodeBase, IncrementalAABBTreeNode* parent, const PxU32* primitivesBase,
		Ps::Array<IncrementalAABBTreeNode*>& mapping)
{
	destNode.mParent = parent;
	destNode.mBVMin = V4ClearW(V4LoadU(&sourceNode.mBV.minimum.x));
	destNode.mBVMax = V4ClearW(V4LoadU(&sourceNode.mBV.maximum.x));
	if(sourceNode.isLeaf())
	{
		AABBTreeIndices* indices = mIndicesPool.allocate();
		destNode.mIndices = indices;
		indices->nbIndices = sourceNode.getNbPrimitives();
		const PxU32* sourceIndices = sourceNode.getPrimitives(primitivesBase);
		for(PxU32 i = 0; i < indices->nbIndices; i++)
		{
			const PxU32 sourceIndex = sourceIndices[i];
			indices->indices[i] = sourceIndex;
			mapping[sourceIndex] = &destNode;
		}
	}
	else
	{
		IncrementalAABBTreeNodePair* nodePair = mNodesPool.construct();
		IncrementalAABBTreeNode* child0 = &nodePair->mNode0;
		IncrementalAABBTreeNode* child1 = &nodePair->mNode1;

		destNode.mChilds[0] = child0;
		destNode.mChilds[1] = child1;

		copyNode(*destNode.mChilds[0], *sourceNode.getPos(nodeBase), nodeBase, &destNode, primitivesBase, mapping);
		copyNode(*destNode.mChilds[1], *sourceNode.getNeg(nodeBase), nodeBase, &destNode, primitivesBase, mapping);
	}
}

// build the tree from the prebuild AABB tree
void IncrementalAABBTree::copy(const BVHStructure& bvhStructure, Ps::Array<IncrementalAABBTreeNode*>& mapping)
{
	if(bvhStructure.getNbBounds() == 0)
		return;

	IncrementalAABBTreeNodePair* nodePair = mNodesPool.construct();
	mRoot = &nodePair->mNode0;

	const BVHNode* nodes = bvhStructure.getNodes();
	copyNode(*mRoot, *nodes, nodes, NULL, bvhStructure.getIndices(), mapping);
}

