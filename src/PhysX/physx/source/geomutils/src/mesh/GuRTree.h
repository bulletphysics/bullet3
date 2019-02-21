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

#ifndef GU_RTREE_H
#define GU_RTREE_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec4.h"
#include "foundation/PxBounds3.h"
#include "foundation/PxAssert.h"
#include "PsUserAllocated.h" // for PxSerializationContext
#include "PxSerialFramework.h"
#include "PxTriangleMesh.h"
#include "PsAlignedMalloc.h"


#if PX_ENABLE_DYNAMIC_MESH_RTREE
#include "PsVecMath.h"
#endif

#define RTREE_N 4 // changing this number will affect the mesh format
PX_COMPILE_TIME_ASSERT(RTREE_N == 4 || RTREE_N == 8); // using the low 5 bits for storage of index(childPtr) for dynamic rtree

namespace physx
{


#if PX_VC
#pragma warning(push)
#pragma warning(disable: 4324)	// Padding was added at the end of a structure because of a __declspec(align) value.
#endif

namespace Gu {
	
	class Box;
	struct RTreePage;

	typedef PxF32 RTreeValue;

	/////////////////////////////////////////////////////////////////////////
	// quantized untransposed RTree node - used for offline build and dynamic insertion
	struct RTreeNodeQ
	{
		RTreeValue minx, miny, minz, maxx, maxy, maxz;
		PxU32 ptr; // lowest bit is leaf flag

		PX_FORCE_INLINE void setLeaf(bool set) { if (set) ptr |= 1; else ptr &= ~1; }
		PX_FORCE_INLINE PxU32 isLeaf() const { return ptr & 1; }
		PX_FORCE_INLINE void setEmpty();
		PX_FORCE_INLINE void grow(const RTreePage& page, int nodeIndex);
		PX_FORCE_INLINE void grow(const RTreeNodeQ& node);
	};

	/////////////////////////////////////////////////////////////////////////
	// RTreePage data structure, holds RTREE_N transposed nodes

	// RTreePage data structure, holds 8 transposed nodes
	PX_ALIGN_PREFIX(16)
	struct RTreePage
	{
	//= ATTENTION! =====================================================================================
	// Changing the data layout of this class breaks the binary serialization format.  See comments for 
	// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
	// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
	// accordingly.
	//==================================================================================================

		static const RTreeValue MN, MX;

		RTreeValue minx[RTREE_N]; // [min=MX, max=MN] is used as a sentinel range for empty bounds
		RTreeValue miny[RTREE_N];
		RTreeValue minz[RTREE_N];
		RTreeValue maxx[RTREE_N];
		RTreeValue maxy[RTREE_N];
		RTreeValue maxz[RTREE_N];
		PxU32 ptrs[RTREE_N]; // for static rtree this is an offset relative to the first page divided by 16, for dynamics it's an absolute pointer divided by 16

		PX_FORCE_INLINE PxU32	nodeCount() const; // returns the number of occupied nodes in this page
		PX_FORCE_INLINE void	setEmpty(PxU32 startIndex = 0);
		PX_FORCE_INLINE bool	isEmpty(PxU32 index) const { return minx[index] > maxx[index]; }
		PX_FORCE_INLINE void	copyNode(PxU32 targetIndex, const RTreePage& sourcePage, PxU32 sourceIndex);
		PX_FORCE_INLINE void	setNode(PxU32 targetIndex, const RTreeNodeQ& node);
		PX_FORCE_INLINE void	clearNode(PxU32 nodeIndex);
		PX_FORCE_INLINE void	getNode(PxU32 nodeIndex, RTreeNodeQ& result) const;
		PX_FORCE_INLINE void	computeBounds(RTreeNodeQ& bounds);
		PX_FORCE_INLINE void	adjustChildBounds(PxU32 index, const RTreeNodeQ& adjustedChildBounds);
		PX_FORCE_INLINE void	growChildBounds(PxU32 index, const RTreeNodeQ& adjustedChildBounds);
		PX_FORCE_INLINE PxU32	getNodeHandle(PxU32 index) const;
		PX_FORCE_INLINE PxU32	isLeaf(PxU32 index) const { return ptrs[index] & 1; }
	} PX_ALIGN_SUFFIX(16);

	/////////////////////////////////////////////////////////////////////////
	// RTree root data structure
	PX_ALIGN_PREFIX(16)
	struct RTree
	{
	//= ATTENTION! =====================================================================================
	// Changing the data layout of this class breaks the binary serialization format.  See comments for 
	// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
	// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
	// accordingly.
	//==================================================================================================
		// PX_SERIALIZATION
		RTree(const PxEMPTY);
		void	exportExtraData(PxSerializationContext&);
		void	importExtraData(PxDeserializationContext& context);
		static	void	getBinaryMetaData(PxOutputStream& stream);
		//~PX_SERIALIZATION

		PX_INLINE RTree(); // offline static rtree constructor used with cooking

		~RTree() { release(); }

		PX_INLINE void release();
		bool load(PxInputStream& stream, PxU32 meshVersion, bool mismatch);

		////////////////////////////////////////////////////////////////////////////
		// QUERIES
		struct Callback
		{
			// result buffer should have room for at least RTREE_N items
			// should return true to continue traversal. If false is returned, traversal is aborted
			virtual bool processResults(PxU32 count, PxU32* buf) = 0;
			virtual void profile() {}
            virtual ~Callback() {}
		};

		struct CallbackRaycast
		{
			// result buffer should have room for at least RTREE_N items
			// should return true to continue traversal. If false is returned, traversal is aborted
			// newMaxT serves as both input and output, as input it's the maxT so far
			// set it to a new value (which should be smaller) and it will become the new far clip t
			virtual bool processResults(PxU32 count, PxU32* buf, PxF32& newMaxT) = 0;
            virtual ~CallbackRaycast() {}
		};

		// callback will be issued as soon as the buffer overflows maxResultsPerBlock-RTreePage:SIZE entries
		// use maxResults = RTreePage:SIZE and return false from callback for "first hit" early out
		void		traverseAABB(
						const PxVec3& boxMin, const PxVec3& boxMax,
						const PxU32 maxResultsPerBlock, PxU32* resultsBlockBuf, Callback* processResultsBlockCallback) const;
		void		traverseOBB(
						const Gu::Box& obb,
						const PxU32 maxResultsPerBlock, PxU32* resultsBlockBuf, Callback* processResultsBlockCallback) const;

		template <int inflate>
		void		traverseRay(
						const PxVec3& rayOrigin, const PxVec3& rayDir, // dir doesn't have to be normalized and is B-A for raySegment
						const PxU32 maxResults, PxU32* resultsPtr,
						Gu::RTree::CallbackRaycast* callback,
						const PxVec3* inflateAABBs, // inflate tree's AABBs by this amount. This function turns into AABB sweep.
						PxF32 maxT = PX_MAX_F32 // maximum ray t parameter, p(t)=origin+t*dir; use 1.0f for ray segment
						) const;

#if PX_ENABLE_DYNAMIC_MESH_RTREE
		struct CallbackRefit
		{
			// In this callback index is the number stored in the RTree, which is a LeafTriangles object for current PhysX mesh
			virtual void recomputeBounds(PxU32 index, shdfnd::aos::Vec3V& mn, shdfnd::aos::Vec3V& mx) = 0;
			virtual ~CallbackRefit() {}
		};
		void		refitAllStaticTree(CallbackRefit& cb, PxBounds3* resultMeshBounds); // faster version of refit for static RTree only
#endif


		////////////////////////////////////////////////////////////////////////////
		// DEBUG HELPER FUNCTIONS
#if PX_ENABLE_DYNAMIC_MESH_RTREE
		PX_PHYSX_COMMON_API void validate(CallbackRefit* cb = NULL); // verify that all children are indeed included in parent bounds
#else
		PX_PHYSX_COMMON_API void validate(); // verify that all children are indeed included in parent bounds
#endif
		void		openTextDump();
		void		closeTextDump();
		void		textDump(const char* prefix);
		void		maxscriptExport();
		PxU32		computeBottomLevelCount(PxU32 storedToMemMultiplier) const;

		////////////////////////////////////////////////////////////////////////////
		// DATA
		// remember to update save() and load() when adding or removing data
		PxVec4			mBoundsMin, mBoundsMax, mInvDiagonal, mDiagonalScaler; // 16
		PxU32			mPageSize;
		PxU32			mNumRootPages;
		PxU32			mNumLevels;
		PxU32			mTotalNodes; // 16
		PxU32			mTotalPages;
		PxU32			mFlags; enum { USER_ALLOCATED = 0x1, IS_EDGE_SET = 0x2 };
		RTreePage*		mPages;

	protected:
		typedef PxU32 NodeHandle;
#if PX_ENABLE_DYNAMIC_MESH_RTREE
		void		validateRecursive(PxU32 level, RTreeNodeQ parentBounds, RTreePage* page, CallbackRefit* cb = NULL);
#else
		void		validateRecursive(PxU32 level, RTreeNodeQ parentBounds, RTreePage* page);
#endif

		friend struct RTreePage;
	} PX_ALIGN_SUFFIX(16);

#if PX_SUPPORT_EXTERN_TEMPLATE
	//explicit template instantiation declaration
	extern template
	void RTree::traverseRay<0>(const PxVec3&, const PxVec3&, const PxU32, PxU32*, Gu::RTree::CallbackRaycast*, const PxVec3*, PxF32) const;
	
	extern template
	void RTree::traverseRay<1>(const PxVec3&, const PxVec3&, const PxU32, PxU32*, Gu::RTree::CallbackRaycast*, const PxVec3*, PxF32) const;
#endif

#if PX_VC
#pragma warning(pop)
#endif

	/////////////////////////////////////////////////////////////////////////
	PX_INLINE RTree::RTree()
	{
		mFlags = 0;
		mPages = NULL;
		mTotalNodes = 0;
		mNumLevels = 0;
		mPageSize = RTREE_N;
	}

	/////////////////////////////////////////////////////////////////////////
	PX_INLINE void RTree::release()
	{
		if ((mFlags & USER_ALLOCATED) == 0 && mPages)
		{
			physx::shdfnd::AlignedAllocator<128>().deallocate(mPages);
			mPages = NULL;
		}
	}

	/////////////////////////////////////////////////////////////////////////
	PX_FORCE_INLINE void RTreeNodeQ::setEmpty()
	{
		minx = miny = minz = RTreePage::MX;
		maxx = maxy = maxz = RTreePage::MN;
	}


	// bit 1 is always expected to be set to differentiate between leaf and non-leaf node
	PX_FORCE_INLINE PxU32 LeafGetNbTriangles(PxU32 Data) { return ((Data>>1) & 15)+1; }
	PX_FORCE_INLINE PxU32 LeafGetTriangleIndex(PxU32 Data) { return Data>>5; }
	PX_FORCE_INLINE PxU32 LeafSetData(PxU32 nb, PxU32 index)
	{
		PX_ASSERT(nb>0 && nb<=16); PX_ASSERT(index < (1<<27));
		return (index<<5)|(((nb-1)&15)<<1) | 1;
	}

	struct LeafTriangles
	{
		PxU32			Data;

		// Gets number of triangles in the leaf, returns the number of triangles N, with 0 < N <= 16
		PX_FORCE_INLINE	PxU32	GetNbTriangles()				const	{ return LeafGetNbTriangles(Data); }

		// Gets triangle index for this leaf. Indexed model's array of indices retrieved with RTreeMidphase::GetIndices()
		PX_FORCE_INLINE	PxU32	GetTriangleIndex()				const	{ return LeafGetTriangleIndex(Data); }
		PX_FORCE_INLINE	void	SetData(PxU32 nb, PxU32 index)			{ Data = LeafSetData(nb, index); }
	};

	PX_COMPILE_TIME_ASSERT(sizeof(LeafTriangles)==4); // RTree has space for 4 bytes

} // namespace Gu

}

#endif // #ifdef PX_COLLISION_RTREE
