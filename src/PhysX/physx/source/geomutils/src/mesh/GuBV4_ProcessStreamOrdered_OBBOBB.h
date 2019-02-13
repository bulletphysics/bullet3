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

#ifndef GU_BV4_PROCESS_STREAM_ORDERED_OBB_OBB_H
#define GU_BV4_PROCESS_STREAM_ORDERED_OBB_OBB_H

#ifdef GU_BV4_USE_SLABS
/*	template<class LeafTestT, int i, class ParamsT>
	PX_FORCE_INLINE void BV4_ProcessNodeOrdered2_Swizzled(PxU32& code, const BVDataSwizzled* PX_RESTRICT node, ParamsT* PX_RESTRICT params)
	{
		OPC_SLABS_GET_CE(i)

		if(BV4_BoxBoxOverlap(centerV, extentsV, params))
		{
			if(node->isLeaf(i))
				LeafTestT::doLeafTest(params, node->getPrimitive(i));
			else
				code |= 1<<i;
		}
	}*/

	template<class LeafTestT, int i, class ParamsT>
	PX_FORCE_INLINE void BV4_ProcessNodeOrdered2_SwizzledQ(PxU32& code, const BVDataSwizzledQ* PX_RESTRICT node, ParamsT* PX_RESTRICT params)
	{
		OPC_SLABS_GET_CEQ(i)

		if(BV4_BoxBoxOverlap(centerV, extentsV, params))
		{
			if(node->isLeaf(i))
				LeafTestT::doLeafTest(params, node->getPrimitive(i));
			else
				code |= 1<<i;
		}
	}

	#ifdef GU_BV4_COMPILE_NON_QUANTIZED_TREE
	template<class LeafTestT, int i, class ParamsT>
	PX_FORCE_INLINE void BV4_ProcessNodeOrdered2_SwizzledNQ(PxU32& code, const BVDataSwizzledNQ* PX_RESTRICT node, ParamsT* PX_RESTRICT params)
	{
		OPC_SLABS_GET_CENQ(i)

		if(BV4_BoxBoxOverlap(centerV, extentsV, params))
		{
			if(node->isLeaf(i))
				LeafTestT::doLeafTest(params, node->getPrimitive(i));
			else
				code |= 1<<i;
		}
	}
	#endif
#else
	template<class LeafTestT, class ParamsT>
	PX_FORCE_INLINE void BV4_ProcessNodeOrdered(PxU32* PX_RESTRICT Stack, PxU32& Nb, const BVDataPacked* PX_RESTRICT node, ParamsT* PX_RESTRICT params, PxU32 i, PxU32 limit)
	{
	#ifdef GU_BV4_QUANTIZED_TREE
		if(i<limit && BV4_BoxBoxOverlap(node+i, params))
	#else
		if(i<limit && BV4_BoxBoxOverlap(node[i].mAABB.mExtents, node[i].mAABB.mCenter, params))
	#endif
		{
			if(node[i].isLeaf())
				LeafTestT::doLeafTest(params, node[i].getPrimitive());
			else
				Stack[Nb++] = node[i].getChildData();
		}
	}

	template<class LeafTestT, int i, class ParamsT>
	PX_FORCE_INLINE void BV4_ProcessNodeOrdered2(PxU32& code, const BVDataPacked* PX_RESTRICT node, ParamsT* PX_RESTRICT params)
	{
	#ifdef GU_BV4_QUANTIZED_TREE
		if(BV4_BoxBoxOverlap(node+i, params))
	#else
		if(BV4_BoxBoxOverlap(node[i].mAABB.mExtents, node[i].mAABB.mCenter, params))
	#endif
		{
			if(node[i].isLeaf())
				LeafTestT::doLeafTest(params, node[i].getPrimitive());
			else
				code |= 1<<i;
		}
	}
#endif

#endif	// GU_BV4_PROCESS_STREAM_ORDERED_OBB_OBB_H
