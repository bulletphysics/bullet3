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


#ifndef GU_AABBTREEQUERY_H
#define GU_AABBTREEQUERY_H

#include "GuBVHTestsSIMD.h"
#include "PsInlineArray.h"

namespace physx
{
	namespace Gu
	{
		#define RAW_TRAVERSAL_STACK_SIZE 256

		//////////////////////////////////////////////////////////////////////////

		static PX_FORCE_INLINE void getBoundsTimesTwo(Vec4V& center, Vec4V& extents, const PxBounds3* boxes, PxU32 poolIndex)
		{
			const PxBounds3* objectBounds = boxes + poolIndex;

			const Vec4V minV = V4LoadU(&objectBounds->minimum.x);
			const Vec4V maxV = V4LoadU(&objectBounds->maximum.x);

			center = V4Add(maxV, minV);
			extents = V4Sub(maxV, minV);
		}

		//////////////////////////////////////////////////////////////////////////

		template<typename Test, typename Tree, typename Node, typename Payload, typename QueryCallback>
		class AABBTreeOverlap
		{
		public:
			bool operator()(const Payload* objects, const PxBounds3* boxes, const Tree& tree, const Test& test, QueryCallback& visitor)
			{
				using namespace Cm;

				Ps::InlineArray<const Node*, RAW_TRAVERSAL_STACK_SIZE> stack;
				stack.forceSize_Unsafe(RAW_TRAVERSAL_STACK_SIZE);
				const Node* const nodeBase = tree.getNodes();
				stack[0] = nodeBase;
				PxU32 stackIndex = 1;

				while (stackIndex > 0)
				{
					const Node* node = stack[--stackIndex];
					Vec3V center, extents;
					node->getAABBCenterExtentsV(&center, &extents);
					while (test(center, extents))
					{
						if (node->isLeaf())
						{
							PxU32 nbPrims = node->getNbPrimitives();
							const bool doBoxTest = nbPrims > 1;
							const PxU32* prims = node->getPrimitives(tree.getIndices());
							while (nbPrims--)
							{
								const PxU32* prunableIndex = prims;
								prims++;

								const PxU32 poolIndex = *prunableIndex;
								if (doBoxTest)
								{
									Vec4V center2, extents2;
									getBoundsTimesTwo(center2, extents2, boxes, poolIndex);

									const float half = 0.5f;
									const FloatV halfV = FLoad(half);

									const Vec4V extents_ = V4Scale(extents2, halfV);
									const Vec4V center_ = V4Scale(center2, halfV);

									if (!test(Vec3V_From_Vec4V(center_), Vec3V_From_Vec4V(extents_)))
										continue;
								}

								PxReal unusedDistance;
								if (!visitor.invoke(unusedDistance, objects[poolIndex]))
									return false;
							}
							break;
						}

						const Node* children = node->getPos(nodeBase);

						node = children;
						stack[stackIndex++] = children + 1;
						if(stackIndex == stack.capacity())
							stack.resizeUninitialized(stack.capacity() * 2);
						node->getAABBCenterExtentsV(&center, &extents);
					}
				}
				return true;
			}
		};

		//////////////////////////////////////////////////////////////////////////

		template <bool tInflate, typename Tree, typename Node, typename Payload, typename QueryCallback> // use inflate=true for sweeps, inflate=false for raycasts
		static PX_FORCE_INLINE bool doLeafTest(const Node* node, Gu::RayAABBTest& test, PxReal& md, PxReal oldMaxDist,
			const Payload* objects, const PxBounds3* boxes, const Tree& tree,
			PxReal& maxDist, QueryCallback& pcb)
		{
			PxU32 nbPrims = node->getNbPrimitives();
			const bool doBoxTest = nbPrims > 1;
			const PxU32* prims = node->getPrimitives(tree.getIndices());
			while (nbPrims--)
			{
				const PxU32* prunableIndex = prims;
				prims++;

				const PxU32 poolIndex = *prunableIndex;
				if (doBoxTest)
				{
					Vec4V center_, extents_;
					getBoundsTimesTwo(center_, extents_, boxes, poolIndex);

					if (!test.check<tInflate>(Vec3V_From_Vec4V(center_), Vec3V_From_Vec4V(extents_)))
						continue;
				}

				if (!pcb.invoke(md, objects[poolIndex]))
					return false;

				if (md < oldMaxDist)
				{
					maxDist = md;
					test.setDistance(md);
				}
			}
			return true;
		}

		//////////////////////////////////////////////////////////////////////////

		template <bool tInflate, typename Tree, typename Node, typename Payload, typename QueryCallback> // use inflate=true for sweeps, inflate=false for raycasts
		class AABBTreeRaycast
		{
		public:
			bool operator()(
				const Payload* objects, const PxBounds3* boxes, const Tree& tree,
				const PxVec3& origin, const PxVec3& unitDir, PxReal& maxDist, const PxVec3& inflation,
				QueryCallback& pcb)
			{
				using namespace Cm;

				// PT: we will pass center*2 and extents*2 to the ray-box code, to save some work per-box
				// So we initialize the test with values multiplied by 2 as well, to get correct results
				Gu::RayAABBTest test(origin*2.0f, unitDir*2.0f, maxDist, inflation*2.0f);

				Ps::InlineArray<const Node*, RAW_TRAVERSAL_STACK_SIZE> stack;
				stack.forceSize_Unsafe(RAW_TRAVERSAL_STACK_SIZE);
				const Node* const nodeBase = tree.getNodes();
				stack[0] = nodeBase;
				PxU32 stackIndex = 1;

				PxReal oldMaxDist;
				while (stackIndex--)
				{
					const Node* node = stack[stackIndex];
					Vec3V center, extents;
					node->getAABBCenterExtentsV2(&center, &extents);
					if (test.check<tInflate>(center, extents))	// TODO: try timestamp ray shortening to skip this
					{
						PxReal md = maxDist; // has to be before the goto below to avoid compile error
						while (!node->isLeaf())
						{
							const Node* children = node->getPos(nodeBase);

							Vec3V c0, e0;
							children[0].getAABBCenterExtentsV2(&c0, &e0);
							const PxU32 b0 = test.check<tInflate>(c0, e0);

							Vec3V c1, e1;
							children[1].getAABBCenterExtentsV2(&c1, &e1);
							const PxU32 b1 = test.check<tInflate>(c1, e1);

							if (b0 && b1)	// if both intersect, push the one with the further center on the stack for later
							{
								// & 1 because FAllGrtr behavior differs across platforms
								const PxU32 bit = FAllGrtr(V3Dot(V3Sub(c1, c0), test.mDir), FZero()) & 1;
								stack[stackIndex++] = children + bit;
								node = children + (1 - bit);
								if (stackIndex == stack.capacity())
									stack.resizeUninitialized(stack.capacity() * 2);
							}
							else if (b0)
								node = children;
							else if (b1)
								node = children + 1;
							else
								goto skip_leaf_code;
						}

						oldMaxDist = maxDist; // we copy since maxDist can be updated in the callback and md<maxDist test below can fail

						if (!doLeafTest<tInflate, Tree, Node>(node, test, md, oldMaxDist,
							objects, boxes, tree,
							maxDist,
							pcb))
							return false;
					skip_leaf_code:;
					}
				}
				return true;
			}
		};
	}
}

#endif   // SQ_AABBTREEQUERY_H
