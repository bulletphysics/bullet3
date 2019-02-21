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

#include "SqAABBTree.h"
#include "SqAABBTreeUpdateMap.h"
#include "SqBounds.h"

#include "PsMathUtils.h"
#include "PsFoundation.h"
#include "GuInternal.h"

using namespace physx;
using namespace Sq;
using namespace Gu;

#define INVALID_ID	0xffffffff

// Progressive building
class Sq::FIFOStack : public Ps::UserAllocated
{
public:
	FIFOStack() : mStack(PX_DEBUG_EXP("SQFIFOStack")), mCurIndex(0) {}
	~FIFOStack() {}

	PX_FORCE_INLINE	PxU32				getNbEntries() const { return mStack.size(); }
	PX_FORCE_INLINE	void				push(AABBTreeBuildNode* entry) { mStack.pushBack(entry); }
	bool				pop(AABBTreeBuildNode*& entry);
private:
	Ps::Array<AABBTreeBuildNode*>	mStack;
	PxU32							mCurIndex;			//!< Current index within the container
};

bool Sq::FIFOStack::pop(AABBTreeBuildNode*& entry)
{
	const PxU32 NbEntries = mStack.size(); // Get current number of entries
	if (!NbEntries)
		return false; // Can be NULL when no value has been pushed. This is an invalid pop call.
	entry = mStack[mCurIndex++]; // Get oldest entry, move to next one
	if (mCurIndex == NbEntries)
	{
		// All values have been poped
		mStack.clear();
		mCurIndex = 0;
	}
	return true;
}
//~Progressive building

void flatten(const NodeAllocator& nodeAllocator, AABBTreeRuntimeNode* dest)
{
	// PT: gathers all build nodes allocated so far and flatten them to a linear destination array of smaller runtime nodes
	PxU32 offset = 0;
	const PxU32 nbSlabs = nodeAllocator.mSlabs.size();
	for(PxU32 s=0;s<nbSlabs;s++)
	{
		const NodeAllocator::Slab& currentSlab = nodeAllocator.mSlabs[s];

		AABBTreeBuildNode* pool = currentSlab.mPool;
		for(PxU32 i=0;i<currentSlab.mNbUsedNodes;i++)
		{
			dest[offset].mBV = pool[i].mBV;
			if(pool[i].isLeaf())
			{
				const PxU32 index = pool[i].mNodeIndex;

				const PxU32 nbPrims = pool[i].getNbPrimitives();
				PX_ASSERT(nbPrims<=16);

				dest[offset].mData = (index<<5)|((nbPrims&15)<<1)|1;
			}
			else
			{
				PX_ASSERT(pool[i].mPos);
				PxU32 localNodeIndex = 0xffffffff;
				PxU32 nodeBase = 0;
				for(PxU32 j=0;j<nbSlabs;j++)
				{
					if(pool[i].mPos>= nodeAllocator.mSlabs[j].mPool && pool[i].mPos < nodeAllocator.mSlabs[j].mPool + nodeAllocator.mSlabs[j].mNbUsedNodes)
					{
						localNodeIndex = PxU32(pool[i].mPos - nodeAllocator.mSlabs[j].mPool);
						break;
					}
					nodeBase += nodeAllocator.mSlabs[j].mNbUsedNodes;
				}
				const PxU32 nodeIndex = nodeBase + localNodeIndex;
				dest[offset].mData = nodeIndex<<1;
			}
			offset++;
		}
	}
}

AABBTree::AABBTree() :
	mIndices		(NULL),
	mNbIndices		(0),
	mRuntimePool	(NULL),
	mParentIndices	(NULL),
	mTotalNbNodes	(0),
	mTotalPrims		(0)
{
// Progressive building
	mStack = NULL;
//~Progressive building

// REFIT
	mRefitHighestSetWord = 0;
//~REFIT
}

AABBTree::~AABBTree()
{
	release(false);
}

void AABBTree::release(bool clearRefitMap)
{
// Progressive building
	PX_DELETE_AND_RESET(mStack);
//~Progressive building
	PX_FREE_AND_RESET(mParentIndices);
	PX_DELETE_ARRAY(mRuntimePool);
	mNodeAllocator.release();
	PX_FREE_AND_RESET(mIndices);
	mTotalNbNodes = 0;
	mNbIndices = 0;

// REFIT
	if(clearRefitMap)
		mRefitBitmask.clearAll();
	mRefitHighestSetWord = 0;
//~REFIT
}

// Initialize nodes/indices from the input tree merge data
void AABBTree::initTree(const AABBTreeMergeData& tree)
{
	PX_ASSERT(mIndices == NULL);
	PX_ASSERT(mRuntimePool == NULL);
	PX_ASSERT(mParentIndices == NULL);

	// allocate,copy indices
	mIndices = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32)*tree.mNbIndices, "AABB tree indices"));
	mNbIndices = tree.mNbIndices;
	PxMemCopy(mIndices, tree.mIndices, sizeof(PxU32)*tree.mNbIndices);

	// allocate,copy nodes
	mRuntimePool = PX_NEW(AABBTreeRuntimeNode)[tree.mNbNodes];
	mTotalNbNodes = tree.mNbNodes;
	PxMemCopy(mRuntimePool, tree.mNodes, sizeof(AABBTreeRuntimeNode)*tree.mNbNodes);
}

// Shift indices of the tree by offset. Used for merged trees, when initial indices needs to be shifted to match indices in current pruning pool
void AABBTree::shiftIndices(PxU32 offset)
{
	for (PxU32 i = 0; i < mNbIndices; i++)
	{
		mIndices[i] += offset;
	}
}

bool AABBTree::buildInit(AABBTreeBuildParams& params, BuildStats& stats)
{
	// Checkings
	const PxU32 nbPrimitives = params.mNbPrimitives;
	if(!nbPrimitives)
		return false;

	// Release previous tree
	release();

	// Initialize indices. This list will be modified during build.
	mNbIndices = nbPrimitives;
	return initAABBTreeBuild(params, mNodeAllocator, stats, mIndices);
}

void AABBTree::buildEnd(AABBTreeBuildParams& params, BuildStats& stats)
{
	PX_FREE_AND_RESET(params.mCache);
	// Get back total number of nodes
	mTotalNbNodes	= stats.getCount();
	mTotalPrims		= stats.mTotalPrims;

	mRuntimePool = PX_NEW(AABBTreeRuntimeNode)[mTotalNbNodes];
	PX_ASSERT(mTotalNbNodes==mNodeAllocator.mTotalNbNodes);
	flatten(mNodeAllocator, mRuntimePool);
	mNodeAllocator.release();
}

bool AABBTree::build(AABBTreeBuildParams& params)
{
	const PxU32 nbPrimitives = params.mNbPrimitives;
	if(!nbPrimitives)
		return false;

	// Release previous tree
	release();

	BuildStats stats;
	mNbIndices = nbPrimitives;

	const bool buildStatus = buildAABBTree(params, mNodeAllocator, stats, mIndices);
	PX_UNUSED(buildStatus);
	PX_ASSERT(buildStatus);

	buildEnd(params, stats);
	return true;
}

void AABBTree::shiftOrigin(const PxVec3& shift)
{
	AABBTreeRuntimeNode* const nodeBase = mRuntimePool;
	const PxU32 totalNbNodes = mTotalNbNodes;
	for(PxU32 i=0; i<totalNbNodes; i++)
	{
		AABBTreeRuntimeNode& current = nodeBase[i];
		if((i+1) < totalNbNodes)
			Ps::prefetch(nodeBase + i + 1);

		current.mBV.minimum -= shift;
		current.mBV.maximum -= shift;
	}
}

#if PX_DEBUG
void AABBTree::validate() const
{
}
#endif

// Progressive building
static PxU32 incrementalBuildHierarchy(FIFOStack& stack, AABBTreeBuildNode* node, AABBTreeBuildParams& params, BuildStats& stats, NodeAllocator& nodeBase, PxU32* const indices)
{
	node->subdivide(params, stats, nodeBase, indices);

	if(!node->isLeaf())
	{
		AABBTreeBuildNode* pos = const_cast<AABBTreeBuildNode*>(node->getPos());
		PX_ASSERT(pos);
		AABBTreeBuildNode* neg = pos + 1;
		stack.push(neg);
		stack.push(pos);
	}

	stats.mTotalPrims += node->mNbPrimitives;
	return node->mNbPrimitives;
}

PxU32 AABBTree::progressiveBuild(AABBTreeBuildParams& params, BuildStats& stats, PxU32 progress, PxU32 limit)
{
	if(progress==0)
	{
		if(!buildInit(params, stats))
			return PX_INVALID_U32;

		mStack = PX_NEW(FIFOStack);
		mStack->push(mNodeAllocator.mPool);
		return progress++;
	}
	else if(progress==1)
	{
		PxU32 stackCount = mStack->getNbEntries();
		if(stackCount)
		{
			PxU32 Total = 0;
			const PxU32 Limit = limit;
			while(Total<Limit)
			{
				AABBTreeBuildNode* Entry;
				if(mStack->pop(Entry))
					Total += incrementalBuildHierarchy(*mStack, Entry, params, stats, mNodeAllocator, mIndices);
				else
					break;
			}
			return progress;
		}

		buildEnd(params, stats);

		PX_DELETE_AND_RESET(mStack);

		return 0;	// Done!
	}
	return PX_INVALID_U32;
}
//~Progressive building



static PX_FORCE_INLINE PxU32 BitsToDwords(PxU32 nb_bits)
{
	return (nb_bits>>5) + ((nb_bits&31) ? 1 : 0);
}

bool Sq::BitArray::init(PxU32 nb_bits)
{
	mSize = BitsToDwords(nb_bits);
	// Get ram for n bits
	PX_FREE(mBits);
	mBits = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32)*mSize, "BitArray::mBits"));
	// Set all bits to 0
	clearAll();
	return true;
}

void Sq::BitArray::resize(PxU32 maxBitNumber)
{
	const PxU32 newSize = BitsToDwords(maxBitNumber);
	if (newSize <= mSize)
		return;

	PxU32* newBits = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32)*newSize, "BitArray::mBits"));
	PxMemZero(newBits + mSize, (newSize - mSize) * sizeof(PxU32));
	PxMemCopy(newBits, mBits, mSize*sizeof(PxU32));
	PX_FREE(mBits);
	mBits = newBits;
	mSize = newSize;
}

static PX_FORCE_INLINE	PxU32						getNbPrimitives(PxU32 data)							{ return (data>>1)&15;		}
static PX_FORCE_INLINE	const PxU32*				getPrimitives(const PxU32* base, PxU32 data)		{ return base + (data>>5);	}
static PX_FORCE_INLINE	const AABBTreeRuntimeNode*	getPos(const AABBTreeRuntimeNode* base, PxU32 data)	{ return base + (data>>1);	}
static PX_FORCE_INLINE	PxU32						isLeaf(PxU32 data)									{ return data&1;			}

static PX_FORCE_INLINE void refitNode(AABBTreeRuntimeNode* PX_RESTRICT current, const PxBounds3* PX_RESTRICT boxes, const PxU32* PX_RESTRICT indices, AABBTreeRuntimeNode* PX_RESTRICT const nodeBase)
{
	// PT: we can safely use V4 loads on both boxes and nodes here:
	// - it's safe on boxes because we allocated one extra box in the pruning pool
	// - it's safe on nodes because there's always some data within the node, after the BV

	const PxU32 data = current->mData;

	Vec4V resultMinV, resultMaxV;
	if(isLeaf(data))
	{
		const PxU32 nbPrims = getNbPrimitives(data);
		if(nbPrims)
		{
			const PxU32* primitives = getPrimitives(indices, data);
			resultMinV = V4LoadU(&boxes[*primitives].minimum.x);
			resultMaxV = V4LoadU(&boxes[*primitives].maximum.x);

			if(nbPrims>1)
			{
				const PxU32* last = primitives + nbPrims;
				primitives++;

				while(primitives!=last)
				{
					resultMinV = V4Min(resultMinV, V4LoadU(&boxes[*primitives].minimum.x));
					resultMaxV = V4Max(resultMaxV, V4LoadU(&boxes[*primitives].maximum.x));
					primitives++;
				}
			}
		}
		else
		{
			// Might happen after a node has been invalidated
			const float max = SQ_EMPTY_BOUNDS_EXTENTS;
			resultMinV = V4Load(max);
			resultMaxV = V4Load(-max);
		}
	}
	else
	{
		const AABBTreeRuntimeNode* pos = getPos(nodeBase, data);
		const AABBTreeRuntimeNode* neg = pos+1;

		const PxBounds3& posBox = pos->mBV;
		const PxBounds3& negBox = neg->mBV;

		resultMinV = V4Min(V4LoadU(&posBox.minimum.x), V4LoadU(&negBox.minimum.x));
//		resultMaxV = V4Max(V4LoadU(&posBox.maximum.x), V4LoadU(&negBox.maximum.x));

#if PX_INTEL_FAMILY && !defined(PX_SIMD_DISABLED)
		Vec4V posMinV = V4LoadU(&posBox.minimum.z);
		Vec4V negMinV = V4LoadU(&negBox.minimum.z);
		posMinV = _mm_shuffle_ps(posMinV, posMinV, _MM_SHUFFLE(0, 3, 2, 1));
		negMinV = _mm_shuffle_ps(negMinV, negMinV, _MM_SHUFFLE(0, 3, 2, 1));
		resultMaxV = V4Max(posMinV, negMinV);
#else
		// PT: fixes the perf issue but not really convincing
		resultMaxV = Vec4V_From_Vec3V(V3Max(V3LoadU(&posBox.maximum.x), V3LoadU(&negBox.maximum.x)));
#endif
	}

	// PT: the V4 stores overwrite the data after the BV, but we just put it back afterwards
	V4StoreU(resultMinV, &current->mBV.minimum.x);
	V4StoreU(resultMaxV, &current->mBV.maximum.x);
	current->mData = data;
}

void AABBTree::fullRefit(const PxBounds3* boxes)
{
	PX_ASSERT(boxes);

	const PxU32* indices = mIndices;
	AABBTreeRuntimeNode* const nodeBase = mRuntimePool;
	PX_ASSERT(nodeBase);

	// Bottom-up update
	PxU32 index = mTotalNbNodes;
	while(index--)
	{
		AABBTreeRuntimeNode* current = nodeBase + index;
		if(index)
			Ps::prefetch(current - 1);

		refitNode(current, boxes, indices, nodeBase);
	}
}

static void _createParentArray(PxU32 totalNbNodes, PxU32* parentIndices, const AABBTreeRuntimeNode* parentNode, const AABBTreeRuntimeNode* currentNode, const AABBTreeRuntimeNode* root)
{
	const PxU32 parentIndex = PxU32(parentNode - root);
	const PxU32 currentIndex = PxU32(currentNode - root);
	PX_ASSERT(parentIndex<totalNbNodes);
	PX_ASSERT(currentIndex<totalNbNodes);
	PX_UNUSED(totalNbNodes);
	parentIndices[currentIndex] = parentIndex;

	if(!currentNode->isLeaf())
	{
		_createParentArray(totalNbNodes, parentIndices, currentNode, currentNode->getPos(root), root);
		_createParentArray(totalNbNodes, parentIndices, currentNode, currentNode->getNeg(root), root);
	}
}

void AABBTree::markNodeForRefit(TreeNodeIndex nodeIndex)
{
	if(!mRefitBitmask.getBits())
		mRefitBitmask.init(mTotalNbNodes);

	PX_ASSERT(nodeIndex<mTotalNbNodes);

	// PT: lazy-create parent array. Memory is not wasted for purely static trees, or dynamic trees that only do "full refit".
	if(!mParentIndices)
	{
		mParentIndices = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32)*mTotalNbNodes, "AABB parent indices"));
		_createParentArray(mTotalNbNodes, mParentIndices, mRuntimePool, mRuntimePool, mRuntimePool);
	}

	PxU32 currentIndex = nodeIndex;
	while(1)
	{
		PX_ASSERT(currentIndex<mTotalNbNodes);
		if(mRefitBitmask.isSet(currentIndex))
		{
			// We can early exit if we already visited the node!
			return;
		}
		else
		{
			mRefitBitmask.setBit(currentIndex);
			const PxU32 currentMarkedWord = currentIndex>>5;
			mRefitHighestSetWord = PxMax(mRefitHighestSetWord, currentMarkedWord);

			const PxU32 parentIndex = mParentIndices[currentIndex];			
			PX_ASSERT(parentIndex == 0 || parentIndex < currentIndex);
			if(currentIndex == parentIndex)
				break;
			currentIndex = parentIndex;
		}
	}
}

#define FIRST_VERSION
#ifdef FIRST_VERSION
void AABBTree::refitMarkedNodes(const PxBounds3* boxes)
{
	if(!mRefitBitmask.getBits())
		return;	// No refit needed

	{
		/*const*/ PxU32* bits = const_cast<PxU32*>(mRefitBitmask.getBits());
		PxU32 size = mRefitHighestSetWord+1;
#ifdef _DEBUG
		if(1)
		{
			const PxU32 totalSize = mRefitBitmask.getSize();
			for(PxU32 i=size;i<totalSize;i++)
			{
				PX_ASSERT(!bits[i]);
			}
		}
		PxU32 nbRefit=0;
#endif
		const PxU32* indices = mIndices;
		AABBTreeRuntimeNode* const nodeBase = mRuntimePool;

		while(size--)
		{
			// Test 32 bits at a time
			const PxU32 currentBits = bits[size];
			if(!currentBits)
				continue;

			PxU32 index = (size+1)<<5;
			PxU32 mask = PxU32(1<<((index-1)&31));
			PxU32 _Count=32;
			while(_Count--)
			{
				index--;
				Ps::prefetch(nodeBase + index);

				PX_ASSERT(size==index>>5);
				PX_ASSERT(mask==PxU32(1<<(index&31)));
				if(currentBits & mask)
				{
					refitNode(nodeBase + index, boxes, indices, nodeBase);
#ifdef _DEBUG
					nbRefit++;
#endif
				}
				mask>>=1;
			}
			bits[size] = 0;
		}

		mRefitHighestSetWord = 0;
//		mRefitBitmask.clearAll();
	}
}
#endif


//#define SECOND_VERSION
#ifdef SECOND_VERSION
void AABBTree::refitMarkedNodes(const PxBounds3* boxes)
{
	/*const*/ PxU32* bits = const_cast<PxU32*>(mRefitBitmask.getBits());
	if(!bits)
		return;	// No refit needed

	const PxU32 lastSetBit = mRefitBitmask.findLast();

	const PxU32* indices = mIndices;
	AABBTreeRuntimeNode* const nodeBase = mRuntimePool;

	for(PxU32 w = 0; w <= lastSetBit >> 5; ++w)
	{
		for(PxU32 b = bits[w]; b; b &= b-1)
		{
			const PxU32 index = (PxU32)(w<<5|Ps::lowestSetBit(b));



		while(size--)
		{
			// Test 32 bits at a time
			const PxU32 currentBits = bits[size];
			if(!currentBits)
				continue;

			PxU32 index = (size+1)<<5;
			PxU32 mask = PxU32(1<<((index-1)&31));
			PxU32 _Count=32;
			while(_Count--)
			{
				index--;
				Ps::prefetch(nodeBase + index);

				PX_ASSERT(size==index>>5);
				PX_ASSERT(mask==PxU32(1<<(index&31)));
				if(currentBits & mask)
				{
					refitNode(nodeBase + index, boxes, indices, nodeBase);
#ifdef _DEBUG
					nbRefit++;
#endif
				}
				mask>>=1;
			}
			bits[size] = 0;
		}
		mRefitHighestSetWord = 0;
//		mRefitBitmask.clearAll();
	}
}
#endif

PX_FORCE_INLINE static void setLeafData(PxU32& leafData, const AABBTreeRuntimeNode& node, const PxU32 indicesOffset)
{
	const PxU32 index = indicesOffset + (node.mData >> 5);
	const PxU32 nbPrims = node.getNbPrimitives();
	PX_ASSERT(nbPrims <= 16);
	leafData = (index << 5) | ((nbPrims & 15) << 1) | 1;
}

// Copy the tree into nodes. Update node indices, leaf indices.
void AABBTree::addRuntimeChilds(PxU32& nodeIndex, const AABBTreeMergeData& treeParams)
{
	PX_ASSERT(nodeIndex < mTotalNbNodes + treeParams.mNbNodes + 1);	
	const PxU32 baseNodeIndex = nodeIndex;	

	// copy the src tree into dest tree nodes, update its data
	for (PxU32 i = 0; i < treeParams.mNbNodes; i++)
	{
		PX_ASSERT(nodeIndex < mTotalNbNodes + treeParams.mNbNodes  + 1);
		mRuntimePool[nodeIndex].mBV = treeParams.mNodes[i].mBV;
		if (treeParams.mNodes[i].isLeaf())
		{
			setLeafData(mRuntimePool[nodeIndex].mData, treeParams.mNodes[i], mNbIndices);
		}
		else
		{
			const PxU32 srcNodeIndex = baseNodeIndex + (treeParams.mNodes[i].getPosIndex());
			mRuntimePool[nodeIndex].mData = srcNodeIndex << 1;
			mParentIndices[srcNodeIndex] = nodeIndex;
			mParentIndices[srcNodeIndex + 1] = nodeIndex;
		}
		nodeIndex++;
	}
}

// Merge tree into targetNode, where target node is a leaf
// 1. Allocate new nodes/parent, copy all the nodes/parents
// 2. Create new node at the end, copy the data from target node	
// 3. Copy the merge tree after the new node, create the parent map for them, update the leaf indices
// Schematic view:
// Target Nodes: ...Tn...
// Input tree: R1->Rc0, Rc1...
// Merged tree: ...Tnc->...->Nc0,R1->Rc0,Rc1...
//		where new node:		Nc0==Tn and Tnc is not a leaf anymore and points to Nc0

void AABBTree::mergeRuntimeLeaf(AABBTreeRuntimeNode& targetNode, const AABBTreeMergeData& treeParams, PxU32 targetMergeNodeIndex)
{
	PX_ASSERT(mParentIndices);
	PX_ASSERT(targetNode.isLeaf());	

	// 1. Allocate new nodes/parent, copy all the nodes/parents
	// allocate new runtime pool with max combine number of nodes
	// we allocate only 1 additional node each merge
	AABBTreeRuntimeNode* newRuntimePool = PX_NEW(AABBTreeRuntimeNode)[mTotalNbNodes + treeParams.mNbNodes + 1];
	PxU32* newParentIndices = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32)*(mTotalNbNodes + treeParams.mNbNodes + 1), "AABB parent indices"));

	// copy the whole target nodes, we will add the new node at the end together with the merge tree
	PxMemCopy(newRuntimePool, mRuntimePool, sizeof(AABBTreeRuntimeNode)*(mTotalNbNodes));
	PxMemCopy(newParentIndices, mParentIndices, sizeof(PxU32)*(mTotalNbNodes));

	// 2. Create new node at the end, copy the data from target node	
	PxU32 nodeIndex = mTotalNbNodes;	
	// copy the targetNode at the end of the new nodes
	newRuntimePool[nodeIndex].mBV = targetNode.mBV;
	newRuntimePool[nodeIndex].mData = targetNode.mData;
	// update the parent information
	newParentIndices[nodeIndex] = targetMergeNodeIndex;

	// mark for refit
	if (mRefitBitmask.getBits() && mRefitBitmask.isSet(targetMergeNodeIndex))
	{
		mRefitBitmask.setBit(nodeIndex);
		const PxU32 currentMarkedWord = nodeIndex >> 5;
		mRefitHighestSetWord = PxMax(mRefitHighestSetWord, currentMarkedWord);
	}

	// swap pointers
	PX_DELETE_ARRAY(mRuntimePool);
	mRuntimePool = newRuntimePool;
	PX_FREE(mParentIndices);
	mParentIndices = newParentIndices;

	// 3. Copy the merge tree after the new node, create the parent map for them, update the leaf indices
	nodeIndex++;
	addRuntimeChilds(nodeIndex, treeParams);
	PX_ASSERT(nodeIndex == mTotalNbNodes + 1 + treeParams.mNbNodes);	

	// update the parent information for the input tree root node
	mParentIndices[mTotalNbNodes + 1] = targetMergeNodeIndex;

	// fix the child information for the target node, was a leaf before
	mRuntimePool[targetMergeNodeIndex].mData = mTotalNbNodes << 1;

	// update the total number of nodes
	mTotalNbNodes = mTotalNbNodes + 1 + treeParams.mNbNodes;
}

// Merge tree into targetNode, where target node is not a leaf
// 1. Allocate new nodes/parent, copy the nodes/parents till targetNodePosIndex
// 2. Create new node , copy the data from target node	
// 3. Copy the rest of the target tree nodes/parents at the end -> targetNodePosIndex + 1 + treeParams.mNbNodes
// 4. Copy the merge tree after the new node, create the parent map for them, update the leaf indices
// 5. Go through the nodes copied at the end and fix the parents/childs
// Schematic view:
// Target Nodes: ...Tn->...->Tc0,Tc1...
// Input tree: R1->Rc0, Rc1...
// Merged tree: ...Tn->...->Nc0,R1->Rc0,Rc1...,Tc0,Tc1...       
//		where new node:		Nc0->...->Tc0,Tc1
void AABBTree::mergeRuntimeNode(AABBTreeRuntimeNode& targetNode, const AABBTreeMergeData& treeParams, PxU32 targetMergeNodeIndex)
{
	PX_ASSERT(mParentIndices);	
	PX_ASSERT(!targetNode.isLeaf());

	// Get the target node child pos, this is where we insert the new node and the input tree
	const PxU32 targetNodePosIndex = targetNode.getPosIndex();

	// 1. Allocate new nodes/parent, copy the nodes/parents till targetNodePosIndex
	// allocate new runtime pool with max combine number of nodes
	// we allocate only 1 additional node each merge
	AABBTreeRuntimeNode* newRuntimePool = PX_NEW(AABBTreeRuntimeNode)[mTotalNbNodes + treeParams.mNbNodes + 1];
	PxU32* newParentIndices = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32)*(mTotalNbNodes + treeParams.mNbNodes + 1), "AABB parent indices"));
	// copy the untouched part of the nodes and parents
	PxMemCopy(newRuntimePool, mRuntimePool, sizeof(AABBTreeRuntimeNode)*(targetNodePosIndex));
	PxMemCopy(newParentIndices, mParentIndices, sizeof(PxU32)*(targetNodePosIndex));

	PxU32 nodeIndex = targetNodePosIndex;
	// 2. Create new node , copy the data from target node	
	newRuntimePool[nodeIndex].mBV = targetNode.mBV;
	newRuntimePool[nodeIndex].mData = ((targetNode.mData >> 1) + 1 + treeParams.mNbNodes) << 1;
	// update parent information
	newParentIndices[nodeIndex] = targetMergeNodeIndex;

	// handle mark for refit
	if(mRefitBitmask.getBits() && mRefitBitmask.isSet(targetMergeNodeIndex))
	{
		mRefitBitmask.setBit(nodeIndex);
		const PxU32 currentMarkedWord = nodeIndex >> 5;
		mRefitHighestSetWord = PxMax(mRefitHighestSetWord, currentMarkedWord);
	}

	// 3. Copy the rest of the target tree nodes/parents at the end -> targetNodePosIndex + 1 + treeParams.mNbNodes
	if(mTotalNbNodes - targetNodePosIndex)
	{
		PX_ASSERT(mTotalNbNodes - targetNodePosIndex > 0);
		PxMemCopy(newRuntimePool + targetNodePosIndex + 1 + treeParams.mNbNodes, mRuntimePool + targetNodePosIndex, sizeof(AABBTreeRuntimeNode)*(mTotalNbNodes - targetNodePosIndex));
		PxMemCopy(newParentIndices + targetNodePosIndex + 1 + treeParams.mNbNodes, mParentIndices + targetNodePosIndex, sizeof(PxU32)*(mTotalNbNodes - targetNodePosIndex));
	}
	// swap the pointers, release the old memory
	PX_DELETE_ARRAY(mRuntimePool);
	mRuntimePool = newRuntimePool;
	PX_FREE(mParentIndices);
	mParentIndices = newParentIndices;

	// 4. Copy the merge tree after the new node, create the parent map for them, update the leaf indices
	nodeIndex++;
	addRuntimeChilds(nodeIndex, treeParams);
	PX_ASSERT(nodeIndex == targetNodePosIndex + 1 + treeParams.mNbNodes);
	// update the total number of nodes
	mTotalNbNodes = mTotalNbNodes + 1 + treeParams.mNbNodes;	

	// update the parent information for the input tree root node
	mParentIndices[targetNodePosIndex + 1] = targetMergeNodeIndex;
	
	// 5. Go through the nodes copied at the end and fix the parents/childs
	for (PxU32 i = targetNodePosIndex + 1 + treeParams.mNbNodes; i < mTotalNbNodes; i++)
	{
		// check if the parent is the targetNode, if yes update the parent to new node
		if(mParentIndices[i] == targetMergeNodeIndex)
		{
			mParentIndices[i] = targetNodePosIndex;
		}
		else
		{
			// if parent node has been moved, update the parent node
			if(mParentIndices[i] >= targetNodePosIndex)
			{
				mParentIndices[i] = mParentIndices[i] + 1 + treeParams.mNbNodes;
			}
			else
			{
				// if parent has not been moved, update its child information
				const PxU32 parentIndex = mParentIndices[i];
				// update the child information to point to Pos child
				if(i % 2 != 0)
				{
					const PxU32 srcNodeIndex = mRuntimePool[parentIndex].getPosIndex();
					// if child index points to a node that has been moved, update the child index
					PX_ASSERT(!mRuntimePool[parentIndex].isLeaf());
					PX_ASSERT(srcNodeIndex > targetNodePosIndex);
					mRuntimePool[parentIndex].mData = (1 + treeParams.mNbNodes + srcNodeIndex) << 1;
				}
			}
		}
		if(!mRuntimePool[i].isLeaf())
		{
			// update the child node index
			const PxU32 srcNodeIndex = 1 + treeParams.mNbNodes + mRuntimePool[i].getPosIndex();
			mRuntimePool[i].mData = srcNodeIndex << 1;
		}
	}
}

// traverse the target node, the tree is inside the targetNode, and find the best place where merge the tree
void AABBTree::traverseRuntimeNode(AABBTreeRuntimeNode& targetNode, const AABBTreeMergeData& treeParams, PxU32 nodeIndex)
{
	const AABBTreeRuntimeNode& srcNode = treeParams.getRootNode();
	PX_ASSERT(srcNode.mBV.isInside(targetNode.mBV));

	// Check if the srcNode(tree) can fit inside any of the target childs. If yes, traverse the target tree child
	AABBTreeRuntimeNode& targetPosChild = *targetNode.getPos(mRuntimePool);	
	if(srcNode.mBV.isInside(targetPosChild.mBV))
	{
		return traverseRuntimeNode(targetPosChild, treeParams, targetNode.getPosIndex());		
	}

	AABBTreeRuntimeNode& targetNegChild = *targetNode.getNeg(mRuntimePool);
	if (srcNode.mBV.isInside(targetNegChild.mBV))
	{
		return traverseRuntimeNode(targetNegChild, treeParams, targetNode.getNegIndex());		
	}

	// we cannot traverse target anymore, lets add the srcTree to current target node
	if(targetNode.isLeaf())
		mergeRuntimeLeaf(targetNode, treeParams, nodeIndex);
	else
		mergeRuntimeNode(targetNode, treeParams, nodeIndex);	
}

// Merge the input tree into current tree.
// Traverse the tree and find the smallest node, where the whole new tree fits. When we find the node 
// we create one new node pointing to the original children and the to the input tree root.
void AABBTree::mergeTree(const AABBTreeMergeData& treeParams)
{ 
	// allocate new indices buffer 	
	PxU32* newIndices = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32)*(mNbIndices + treeParams.mNbIndices), "AABB tree indices"));
	PxMemCopy(newIndices, mIndices, sizeof(PxU32)*mNbIndices);
	PX_FREE(mIndices);
	mIndices = newIndices;
	mTotalPrims += treeParams.mNbIndices;

	// copy the new indices, re-index using the provided indicesOffset. Note that indicesOffset 
	// must be provided, as original mNbIndices can be different than indicesOffset dues to object releases.	
	for (PxU32 i = 0; i < treeParams.mNbIndices; i++)
	{
		mIndices[mNbIndices + i] = treeParams.mIndicesOffset + treeParams.mIndices[i];
	}	

	// check the mRefitBitmask if we fit all the new nodes
	mRefitBitmask.resize(mTotalNbNodes + treeParams.mNbNodes + 1);	

	// create the parent information so we can update it
	if(!mParentIndices)
	{
		mParentIndices = reinterpret_cast<PxU32*>(PX_ALLOC(sizeof(PxU32)*mTotalNbNodes, "AABB parent indices"));
		_createParentArray(mTotalNbNodes, mParentIndices, mRuntimePool, mRuntimePool, mRuntimePool);
	}		
	
	// if new tree is inside the root AABB we will traverse the tree to find better node where to attach the tree subnodes
	// if the root is a leaf we merge with the root. 		
	if(treeParams.getRootNode().mBV.isInside(mRuntimePool[0].mBV) && !mRuntimePool[0].isLeaf())
	{
		traverseRuntimeNode(mRuntimePool[0], treeParams, 0);
	}
	else
	{				
		if(mRuntimePool[0].isLeaf())
		{			
			mergeRuntimeLeaf(mRuntimePool[0], treeParams, 0);
		}
		else		
		{			
			mergeRuntimeNode(mRuntimePool[0], treeParams, 0);		
		}

		// increase the tree root AABB
		mRuntimePool[0].mBV.include(treeParams.getRootNode().mBV);
	}

#ifdef _DEBUG
	//verify parent indices
	for (PxU32 i = 0; i < mTotalNbNodes; i++)
	{
		if (i)
		{
			PX_ASSERT(mRuntimePool[mParentIndices[i]].getPosIndex() == i || mRuntimePool[mParentIndices[i]].getNegIndex() == i);
		}
		if (!mRuntimePool[i].isLeaf())
		{
			PX_ASSERT(mParentIndices[mRuntimePool[i].getPosIndex()] == i);
			PX_ASSERT(mParentIndices[mRuntimePool[i].getNegIndex()] == i);
		}
	}

	// verify the tree nodes, leafs
	for (PxU32 i = 0; i < mTotalNbNodes; i++)
	{
		if (mRuntimePool[i].isLeaf())
		{
			const PxU32 index = mRuntimePool[i].mData >> 5;
			const PxU32 nbPrim = mRuntimePool[i].getNbPrimitives();
			PX_ASSERT(index + nbPrim <= mNbIndices + treeParams.mNbIndices);
		}
		else
		{
			const PxU32 nodeIndex = (mRuntimePool[i].getPosIndex());
			PX_ASSERT(nodeIndex < mTotalNbNodes);
		}
	}
#endif // _DEBUG

	mNbIndices += treeParams.mNbIndices;
}



