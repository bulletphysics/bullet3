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

#ifndef GU_BV4_SLABS_SWIZZLED_NO_ORDER_H
#define GU_BV4_SLABS_SWIZZLED_NO_ORDER_H

	// Generic, no sort
/*	template<class LeafTestT, class ParamsT>
	static Ps::IntBool BV4_ProcessStreamSwizzledNoOrder(const BVDataPacked* PX_RESTRICT node, PxU32 initData, ParamsT* PX_RESTRICT params)
	{
		const BVDataPacked* root = node;

		PxU32 nb=1;
		PxU32 stack[GU_BV4_STACK_SIZE];
		stack[0] = initData;

		do
		{
			const PxU32 childData = stack[--nb];
			node = root + getChildOffset(childData);

			const BVDataSwizzled* tn = reinterpret_cast<const BVDataSwizzled*>(node);

			const PxU32 nodeType = getChildType(childData);

			if(nodeType>1 && BV4_ProcessNodeNoOrder_Swizzled<LeafTestT, 3>(stack, nb, tn, params))
				return 1;
			if(nodeType>0 && BV4_ProcessNodeNoOrder_Swizzled<LeafTestT, 2>(stack, nb, tn, params))
				return 1;
			if(BV4_ProcessNodeNoOrder_Swizzled<LeafTestT, 1>(stack, nb, tn, params))
				return 1;
			if(BV4_ProcessNodeNoOrder_Swizzled<LeafTestT, 0>(stack, nb, tn, params))
				return 1;

		}while(nb);

		return 0;
	}*/


	template<class LeafTestT, class ParamsT>
	static Ps::IntBool BV4_ProcessStreamSwizzledNoOrderQ(const BVDataPackedQ* PX_RESTRICT node, PxU32 initData, ParamsT* PX_RESTRICT params)
	{
		const BVDataPackedQ* root = node;

		PxU32 nb=1;
		PxU32 stack[GU_BV4_STACK_SIZE];
		stack[0] = initData;

		do
		{
			const PxU32 childData = stack[--nb];
			node = root + getChildOffset(childData);

			const BVDataSwizzledQ* tn = reinterpret_cast<const BVDataSwizzledQ*>(node);

			const PxU32 nodeType = getChildType(childData);

			if(nodeType>1 && BV4_ProcessNodeNoOrder_SwizzledQ<LeafTestT, 3>(stack, nb, tn, params))
				return 1;
			if(nodeType>0 && BV4_ProcessNodeNoOrder_SwizzledQ<LeafTestT, 2>(stack, nb, tn, params))
				return 1;
			if(BV4_ProcessNodeNoOrder_SwizzledQ<LeafTestT, 1>(stack, nb, tn, params))
				return 1;
			if(BV4_ProcessNodeNoOrder_SwizzledQ<LeafTestT, 0>(stack, nb, tn, params))
				return 1;

		}while(nb);

		return 0;
	}

#ifdef GU_BV4_COMPILE_NON_QUANTIZED_TREE
	template<class LeafTestT, class ParamsT>
	static Ps::IntBool BV4_ProcessStreamSwizzledNoOrderNQ(const BVDataPackedNQ* PX_RESTRICT node, PxU32 initData, ParamsT* PX_RESTRICT params)
	{
		const BVDataPackedNQ* root = node;

		PxU32 nb=1;
		PxU32 stack[GU_BV4_STACK_SIZE];
		stack[0] = initData;

		do
		{
			const PxU32 childData = stack[--nb];
			node = root + getChildOffset(childData);

			const BVDataSwizzledNQ* tn = reinterpret_cast<const BVDataSwizzledNQ*>(node);

			const PxU32 nodeType = getChildType(childData);

			if(nodeType>1 && BV4_ProcessNodeNoOrder_SwizzledNQ<LeafTestT, 3>(stack, nb, tn, params))
				return 1;
			if(nodeType>0 && BV4_ProcessNodeNoOrder_SwizzledNQ<LeafTestT, 2>(stack, nb, tn, params))
				return 1;
			if(BV4_ProcessNodeNoOrder_SwizzledNQ<LeafTestT, 1>(stack, nb, tn, params))
				return 1;
			if(BV4_ProcessNodeNoOrder_SwizzledNQ<LeafTestT, 0>(stack, nb, tn, params))
				return 1;

		}while(nb);

		return 0;
	}
#endif

#endif // GU_BV4_SLABS_SWIZZLED_NO_ORDER_H
