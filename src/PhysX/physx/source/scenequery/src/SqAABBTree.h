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

#ifndef SQ_AABBTREE_H
#define SQ_AABBTREE_H

#include "foundation/PxMemory.h"
#include "foundation/PxBounds3.h"
#include "PsUserAllocated.h"
#include "PsVecMath.h"
#include "SqTypedef.h"
#include "GuAABBTreeBuild.h"
#include "PsArray.h"

namespace physx
{

using namespace shdfnd::aos;

namespace Sq
{
	class AABBTreeUpdateMap;

	typedef Ps::Pair<PxU32, PxU32>		TreeMergePair;
	typedef Ps::Array<TreeMergePair >	TreeMergeMap;

	class BitArray
	{
		public:
										BitArray() : mBits(NULL), mSize(0) {}
										BitArray(PxU32 nb_bits) { init(nb_bits); }
										~BitArray() { PX_FREE_AND_RESET(mBits); mBits = NULL; }

						bool			init(PxU32 nb_bits);

		// Data management
		PX_FORCE_INLINE	void			setBit(PxU32 bit_number)
										{
											mBits[bit_number>>5] |= 1<<(bit_number&31);
										}
		PX_FORCE_INLINE	void			clearBit(PxU32 bit_number)
										{
											mBits[bit_number>>5] &= ~(1<<(bit_number&31));
										}
		PX_FORCE_INLINE	void			toggleBit(PxU32 bit_number)
										{
											mBits[bit_number>>5] ^= 1<<(bit_number&31);
										}

		PX_FORCE_INLINE	void			clearAll()			{ PxMemZero(mBits, mSize*4);		}
		PX_FORCE_INLINE	void			setAll()			{ PxMemSet(mBits, 0xff, mSize*4);	}

						void			resize(PxU32 maxBitNumber);

		// Data access
		PX_FORCE_INLINE	Ps::IntBool		isSet(PxU32 bit_number)	const
										{
											return Ps::IntBool(mBits[bit_number>>5] & (1<<(bit_number&31)));
										}

		PX_FORCE_INLINE	const PxU32*	getBits()	const	{ return mBits;		}
		PX_FORCE_INLINE	PxU32			getSize()	const	{ return mSize;		}

		protected:
						PxU32*			mBits;		//!< Array of bits
						PxU32			mSize;		//!< Size of the array in dwords
	};


	//! AABB tree node used for runtime (smaller than for build)
	class AABBTreeRuntimeNode : public Ps::UserAllocated
	{
		public:
		PX_FORCE_INLINE								AABBTreeRuntimeNode()		{}
		PX_FORCE_INLINE								~AABBTreeRuntimeNode()		{}

		PX_FORCE_INLINE	PxU32						isLeaf()								const	{ return mData&1;			}

		PX_FORCE_INLINE	const PxU32*				getPrimitives(const PxU32* base)		const	{ return base + (mData>>5);	}
		PX_FORCE_INLINE	PxU32*						getPrimitives(PxU32* base)						{ return base + (mData>>5);	}
		PX_FORCE_INLINE	PxU32						getNbPrimitives()						const	{ return (mData>>1)&15;		}

		PX_FORCE_INLINE	PxU32						getPosIndex()							const	{ return mData>>1;			}
		PX_FORCE_INLINE	PxU32						getNegIndex()							const	{ return (mData>>1) + 1;			}
		PX_FORCE_INLINE	const AABBTreeRuntimeNode*	getPos(const AABBTreeRuntimeNode* base)	const	{ return base + (mData>>1);	}
		PX_FORCE_INLINE	const AABBTreeRuntimeNode*	getNeg(const AABBTreeRuntimeNode* base)	const	{ const AABBTreeRuntimeNode* P = getPos(base); return P ? P+1 : NULL;}

		PX_FORCE_INLINE	AABBTreeRuntimeNode*		getPos(AABBTreeRuntimeNode* base)				{ return base + (mData >> 1); }
		PX_FORCE_INLINE	AABBTreeRuntimeNode*		getNeg(AABBTreeRuntimeNode* base)				{ AABBTreeRuntimeNode* P = getPos(base); return P ? P + 1 : NULL; }

		PX_FORCE_INLINE	PxU32						getNbRuntimePrimitives()				const	{ return (mData>>1)&15;		}
		PX_FORCE_INLINE void						setNbRunTimePrimitives(PxU32 val)
													{
														PX_ASSERT(val<16);
														PxU32 data = mData & ~(15<<1);
														data |= val<<1;
														mData = data;
													}

		PX_FORCE_INLINE	void						getAABBCenterExtentsV(Vec3V* center, Vec3V* extents) const
													{
														const Vec4V minV = V4LoadU(&mBV.minimum.x);
														const Vec4V maxV = V4LoadU(&mBV.maximum.x);

														const float half = 0.5f;
														const FloatV halfV = FLoad(half);

														*extents = Vec3V_From_Vec4V(V4Scale(V4Sub(maxV, minV), halfV));
														*center = Vec3V_From_Vec4V(V4Scale(V4Add(maxV, minV), halfV));
													}

		PX_FORCE_INLINE	void						getAABBCenterExtentsV2(Vec3V* center, Vec3V* extents) const
													{
														const Vec4V minV = V4LoadU(&mBV.minimum.x);
														const Vec4V maxV = V4LoadU(&mBV.maximum.x);

														*extents = Vec3V_From_Vec4V(V4Sub(maxV, minV));
														*center = Vec3V_From_Vec4V(V4Add(maxV, minV));
													}

		PX_FORCE_INLINE	void						getAABBMinMaxV(Vec4V* minV, Vec4V* maxV) const
													{
														*minV = V4LoadU(&mBV.minimum.x);
														*maxV = V4LoadU(&mBV.maximum.x);
													}

						PxBounds3					mBV;	// Global bounding-volume enclosing all the node-related primitives
						PxU32						mData;	// 27 bits node or prim index|4 bits #prims|1 bit leaf
	};

	//! Contains AABB-tree merge parameters
	class AABBTreeMergeData
	{
	public:
		AABBTreeMergeData(PxU32 nbNodes, const AABBTreeRuntimeNode* nodes, PxU32 nbIndices, const PxU32* indices, PxU32 indicesOffset) :
			mNbNodes(nbNodes), mNodes(nodes), mNbIndices(nbIndices), mIndices(indices), mIndicesOffset(indicesOffset)
		{
		}

		~AABBTreeMergeData()		{}

		PX_FORCE_INLINE const AABBTreeRuntimeNode& getRootNode() const { return mNodes[0]; }

	public:
		PxU32			mNbNodes;		//!< Number of nodes of AABB tree merge
		const AABBTreeRuntimeNode*	mNodes;	//!< Nodes of AABB tree merge

		PxU32			mNbIndices;		//!< Number of indices of AABB tree merge
		const PxU32*	mIndices;		//!< Indices of AABB tree merge

		PxU32			mIndicesOffset;	//!< Indices offset from pruning pool
	};

	// Progressive building
	class FIFOStack;
	//~Progressive building

	//! AABB-tree, N primitives/leaf
	class AABBTree : public Ps::UserAllocated
	{
		public:
													AABBTree();													
													~AABBTree();
		// Build
						bool						build(Gu::AABBTreeBuildParams& params);
		// Progressive building
						PxU32						progressiveBuild(Gu::AABBTreeBuildParams& params, Gu::BuildStats& stats, PxU32 progress, PxU32 limit);
		//~Progressive building
						void						release(bool clearRefitMap=true);

		// Merge tree with another one
						void						mergeTree(const AABBTreeMergeData& tree);
		// Initialize tree from given merge data
						void						initTree(const AABBTreeMergeData& tree);

		// Data access
		PX_FORCE_INLINE	const PxU32*				getIndices()		const	{ return mIndices;		}
		PX_FORCE_INLINE	PxU32*						getIndices()				{ return mIndices;		}
		PX_FORCE_INLINE	void						setIndices(PxU32* indices)	{ mIndices = indices;	}
		PX_FORCE_INLINE	PxU32						getNbNodes()		const	{ return mTotalNbNodes;	}
		PX_FORCE_INLINE	const AABBTreeRuntimeNode*	getNodes()			const	{ return mRuntimePool;	}
		PX_FORCE_INLINE	AABBTreeRuntimeNode*		getNodes()					{ return mRuntimePool;	}		
		PX_FORCE_INLINE	void						setNodes(AABBTreeRuntimeNode* nodes) { mRuntimePool = nodes;	}		
		PX_FORCE_INLINE	PxU32						getTotalPrims()		const	{ return mTotalPrims;	}

#if PX_DEBUG 
						void						validate()			const;
#endif
						void						shiftOrigin(const PxVec3& shift);

		// Shift indices of the tree by offset. Used for merged trees, when initial indices needs to be shifted to match indices in current pruning pool
						void						shiftIndices(PxU32 offset);
				
		private:
						PxU32*						mIndices;			//!< Indices in the app list. Indices are reorganized during build (permutation).
						PxU32						mNbIndices;			//!< Nb indices
						AABBTreeRuntimeNode*		mRuntimePool;		//!< Linear pool of nodes.
						Gu::NodeAllocator			mNodeAllocator;
						PxU32*						mParentIndices;		//!< PT: hot/cold split, keep parent data in separate array
		// Stats
						PxU32						mTotalNbNodes;		//!< Number of nodes in the tree.
						PxU32						mTotalPrims;		//!< Copy of final BuildStats::mTotalPrims

	// Progressive building
						FIFOStack*					mStack;
	//~Progressive building
						bool						buildInit(Gu::AABBTreeBuildParams& params, Gu::BuildStats& stats);
						void						buildEnd(Gu::AABBTreeBuildParams& params, Gu::BuildStats& stats);

		// tree merge							
						void						mergeRuntimeNode(AABBTreeRuntimeNode& targetNode, const AABBTreeMergeData& tree, PxU32 targetNodeIndex);
						void						mergeRuntimeLeaf(AABBTreeRuntimeNode& targetNode, const AABBTreeMergeData& tree, PxU32 targetNodeIndex);
						void						addRuntimeChilds(PxU32& nodeIndex, const AABBTreeMergeData& tree);
						void						traverseRuntimeNode(AABBTreeRuntimeNode& targetNode, const AABBTreeMergeData& tree, PxU32 nodeIndex);
		// REFIT
		public:
						void						fullRefit(const PxBounds3* boxes);

		// adds node[index] to a list of nodes to refit when refitMarkedNodes is called
		// Note that this includes updating the hierarchy up the chain
						void						markNodeForRefit(TreeNodeIndex nodeIndex);
						void						refitMarkedNodes(const PxBounds3* boxes);
		private:
						BitArray					mRefitBitmask; //!< bit is set for each node index in markForRefit
						PxU32						mRefitHighestSetWord;
		//~REFIT
	};

} // namespace Sq

}

#endif // SQ_AABBTREE_H
