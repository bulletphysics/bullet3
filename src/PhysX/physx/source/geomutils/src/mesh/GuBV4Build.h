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

#ifndef GU_BV4_BUILD_H
#define GU_BV4_BUILD_H

#include "foundation/PxSimpleTypes.h"
#include "GuBV4.h"

namespace physx
{
namespace Gu
{
	class BV4Tree;
	class SourceMesh;

	//! Contains AABB-tree build statistics
	// PT: TODO: this is a duplicate of the SQ structure (TA34704)
	struct BuildStats
	{
								BuildStats() : mCount(0), mTotalPrims(0)	{}

						PxU32	mCount;			//!< Number of nodes created
						PxU32	mTotalPrims;	//!< Total accumulated number of primitives. Should be much higher than the source
												//!< number of prims, since it accumulates all prims covered by each node (i.e. internal
												//!< nodes too, not just leaf ones)

		PX_FORCE_INLINE	void	reset()							{ mCount = mTotalPrims = 0;	}

		PX_FORCE_INLINE	void	setCount(PxU32 nb)				{ mCount=nb;				}
		PX_FORCE_INLINE	void	increaseCount(PxU32 nb)			{ mCount+=nb;				}
		PX_FORCE_INLINE	PxU32	getCount()				const	{ return mCount;			}
	};

	// PT: TODO: refactor with SQ version (TA34704)
	class AABBTreeNode : public physx::shdfnd::UserAllocated
	{
		public:
		PX_FORCE_INLINE							AABBTreeNode() : mPos(0), mNodePrimitives(NULL), mNbPrimitives(0)
												{
												}
		PX_FORCE_INLINE							~AABBTreeNode()
												{
													mPos = 0;
													mNodePrimitives	= NULL;	// This was just a shortcut to the global list => no release
													mNbPrimitives	= 0;
												}
		// Data access
		PX_FORCE_INLINE	const PxBounds3&		getAABB()		const	{ return mBV;							}

		PX_FORCE_INLINE	const AABBTreeNode*		getPos()		const	{ return reinterpret_cast<const AABBTreeNode*>(mPos);		}
		PX_FORCE_INLINE	const AABBTreeNode*		getNeg()		const	{ const AABBTreeNode* P = getPos(); return P ? P+1 : NULL;	}

		PX_FORCE_INLINE	bool					isLeaf()		const	{ return !getPos();						}

						PxBounds3				mBV;		// Global bounding-volume enclosing all the node-related primitives
						size_t					mPos;		// "Positive" & "Negative" children

		// Data access
		PX_FORCE_INLINE	const PxU32*			getPrimitives()		const	{ return mNodePrimitives;	}
		PX_FORCE_INLINE	PxU32					getNbPrimitives()	const	{ return mNbPrimitives;		}

						PxU32*					mNodePrimitives;	//!< Node-related primitives (shortcut to a position in mIndices below)
						PxU32					mNbPrimitives;		//!< Number of primitives for this node
	};

	typedef		bool	(*WalkingCallback)	(const AABBTreeNode* current, PxU32 depth, void* userData);

	// PT: TODO: refactor with SQ version (TA34704)
	class AABBTree : public physx::shdfnd::UserAllocated
	{
		public:
											AABBTree();
											~AABBTree();

						bool				buildFromMesh(SourceMesh& mesh, PxU32 limit);
						void				release();

		PX_FORCE_INLINE	const PxU32*		getIndices()		const	{ return mIndices;		}	//!< Catch the indices
		PX_FORCE_INLINE	PxU32				getNbNodes()		const	{ return mTotalNbNodes;	}	//!< Catch the number of nodes

		PX_FORCE_INLINE	const PxU32*		getPrimitives()		const	{ return mPool->mNodePrimitives;	}
		PX_FORCE_INLINE	PxU32				getNbPrimitives()	const	{ return mPool->mNbPrimitives;		}
		PX_FORCE_INLINE	const AABBTreeNode*	getNodes()			const	{ return mPool;						}
		PX_FORCE_INLINE	const PxBounds3&	getBV()				const	{ return mPool->mBV;				}

						PxU32				walk(WalkingCallback callback, void* userData) const;
		private:
						PxU32*				mIndices;			//!< Indices in the app list. Indices are reorganized during build (permutation).
						AABBTreeNode*		mPool;				//!< Linear pool of nodes for complete trees. Null otherwise. [Opcode 1.3]
						PxU32				mTotalNbNodes;		//!< Number of nodes in the tree.
	};

	PX_PHYSX_COMMON_API bool BuildBV4Ex(BV4Tree& tree, SourceMesh& mesh, float epsilon, PxU32 nbTrisPerLeaf);

} // namespace Gu
}

#endif // GU_BV4_BUILD_H
