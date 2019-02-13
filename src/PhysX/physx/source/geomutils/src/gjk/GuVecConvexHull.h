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

#ifndef GU_VEC_CONVEXHULL_H
#define GU_VEC_CONVEXHULL_H

#include "PxPhysXCommonConfig.h"
#include "GuVecConvex.h"
#include "GuConvexMeshData.h"
#include "GuBigConvexData.h"
#include "GuConvexSupportTable.h"
#include "GuCubeIndex.h"
#include "PsFPU.h"
#include "GuGeometryUnion.h"
#include "PsVecQuat.h"
#include "PxMeshScale.h"

namespace physx
{
namespace Gu
{
#define CONVEX_MARGIN_RATIO			0.1f
#define CONVEX_MIN_MARGIN_RATIO		0.05f
#define	CONVEX_SWEEP_MARGIN_RATIO	0.025f
#define TOLERANCE_MARGIN_RATIO		0.08f
#define TOLERANCE_MIN_MARGIN_RATIO	0.05f


	//This margin is used in Persistent contact manifold
	PX_SUPPORT_FORCE_INLINE Ps::aos::FloatV CalculatePCMConvexMargin(const Gu::ConvexHullData* hullData, const Ps::aos::Vec3VArg scale, 
		const PxReal toleranceLength, const PxReal toleranceRatio = TOLERANCE_MIN_MARGIN_RATIO)
	{
		
		using namespace Ps::aos;
		const Vec3V extents= V3Mul(V3LoadU(hullData->mInternal.mExtents), scale);
		const FloatV min = V3ExtractMin(extents);
		const FloatV toleranceMargin = FLoad(toleranceLength * toleranceRatio);
		//ML: 25% of the minimum extents of the internal AABB as this convex hull's margin
		return FMin(FMul(min, FLoad(0.25f)), toleranceMargin);
	}

	PX_SUPPORT_FORCE_INLINE Ps::aos::FloatV CalculateMTDConvexMargin(const Gu::ConvexHullData* hullData, const Ps::aos::Vec3VArg scale)
	{
		using namespace Ps::aos;
		const Vec3V extents = V3Mul(V3LoadU(hullData->mInternal.mExtents), scale);
		const FloatV min = V3ExtractMin(extents);
		//ML: 25% of the minimum extents of the internal AABB as this convex hull's margin
		return FMul(min, FLoad(0.25f));
	}


	//This minMargin is used in PCM contact gen
	PX_SUPPORT_FORCE_INLINE void CalculateConvexMargin(const Gu::ConvexHullData* hullData, PxReal& margin, PxReal& minMargin, PxReal& sweepMargin,
		const Ps::aos::Vec3VArg scale)
	{
		using namespace Ps::aos;
		
		const Vec3V extents = V3Mul(V3LoadU(hullData->mInternal.mExtents), scale);
		const FloatV min_ = V3ExtractMin(extents);

		PxReal minExtent;
		FStore(min_, &minExtent);

		//margin is used as acceptanceTolerance for overlap.
		margin = minExtent * CONVEX_MARGIN_RATIO;
		//minMargin is used in the GJK termination condition
		minMargin = minExtent * CONVEX_MIN_MARGIN_RATIO;
		//this is used for sweep(gjkRaycast)
		sweepMargin = minExtent * CONVEX_SWEEP_MARGIN_RATIO;
	}

	PX_SUPPORT_FORCE_INLINE Ps::aos::Mat33V ConstructSkewMatrix(const Ps::aos::Vec3VArg scale, const Ps::aos::QuatVArg rotation) 
	{
		using namespace Ps::aos;
		Mat33V rot;
		QuatGetMat33V(rotation, rot.col0, rot.col1, rot.col2);
		Mat33V trans = M33Trnsps(rot);
		trans.col0 = V3Scale(trans.col0, V3GetX(scale));
		trans.col1 = V3Scale(trans.col1, V3GetY(scale));
		trans.col2 = V3Scale(trans.col2, V3GetZ(scale));
		return M33MulM33(trans, rot);
	}

	PX_SUPPORT_FORCE_INLINE void ConstructSkewMatrix(const Ps::aos::Vec3VArg scale, const Ps::aos::QuatVArg rotation, Ps::aos::Mat33V& vertex2Shape, Ps::aos::Mat33V& shape2Vertex, Ps::aos::Vec3V& center, const bool idtScale) 
	{
		using namespace Ps::aos;

		PX_ASSERT(!V3AllEq(scale, V3Zero()));
	
		if(idtScale)
		{
			//create identity buffer
			const Mat33V identity = M33Identity();
			vertex2Shape = identity;
			shape2Vertex = identity;
		}
		else
		{
			const FloatV scaleX = V3GetX(scale);
			const Vec3V invScale = V3Recip(scale);

			//this is uniform scale
			if(V3AllEq(V3Splat(scaleX), scale))
			{	
				vertex2Shape = M33Diagonal(scale);
				shape2Vertex = M33Diagonal(invScale);
			}
			else
			{
				Mat33V rot;
				QuatGetMat33V(rotation, rot.col0, rot.col1, rot.col2);
				const Mat33V trans = M33Trnsps(rot);
				/*
					vertex2shape
					skewMat = Inv(R)*Diagonal(scale)*R;
				*/

				const Mat33V temp(V3Scale(trans.col0, scaleX), V3Scale(trans.col1, V3GetY(scale)), V3Scale(trans.col2, V3GetZ(scale)));
				vertex2Shape = M33MulM33(temp, rot);

				//don't need it in the support function
				/*
					shape2Vertex
					invSkewMat =(invSkewMat)= Inv(R)*Diagonal(1/scale)*R;
				*/
				
				shape2Vertex.col0 = V3Scale(trans.col0, V3GetX(invScale));
				shape2Vertex.col1 = V3Scale(trans.col1, V3GetY(invScale));
				shape2Vertex.col2 = V3Scale(trans.col2, V3GetZ(invScale));
				shape2Vertex = M33MulM33(shape2Vertex, rot);

				//shape2Vertex = M33Inverse(vertex2Shape);
			}

			//transform center to shape space
			center = M33MulV3(vertex2Shape, center);
		}
	}

	PX_SUPPORT_FORCE_INLINE Ps::aos::Mat33V ConstructVertex2ShapeMatrix(const Ps::aos::Vec3VArg scale, const Ps::aos::QuatVArg rotation) 
	{
		using namespace Ps::aos;
		Mat33V rot;
		QuatGetMat33V(rotation, rot.col0, rot.col1, rot.col2);
		const Mat33V trans = M33Trnsps(rot);
		/*
			vertex2shape
			skewMat = Inv(R)*Diagonal(scale)*R;
		*/

		const Mat33V temp(V3Scale(trans.col0, V3GetX(scale)), V3Scale(trans.col1, V3GetY(scale)), V3Scale(trans.col2, V3GetZ(scale)));
		return M33MulM33(temp, rot);
	}


	class ConvexHullV : public ConvexV
	{

		class TinyBitMap
		{
		public:
			PxU32 m[8];
			PX_FORCE_INLINE TinyBitMap() { m[0] = m[1] = m[2] = m[3] = m[4] = m[5] = m[6] = m[7] = 0; }
			PX_FORCE_INLINE void set(PxU8 v) { m[v >> 5] |= 1 << (v & 31); }
			PX_FORCE_INLINE bool get(PxU8 v) const { return (m[v >> 5] & 1 << (v & 31)) != 0; }
		};


	public:
		/**
		\brief Constructor
		*/
		PX_SUPPORT_INLINE ConvexHullV() : ConvexV(ConvexType::eCONVEXHULL)
		{
		}

		PX_SUPPORT_INLINE ConvexHullV(const Gu::ConvexHullData* _hullData, const Ps::aos::Vec3VArg _center, const Ps::aos::Vec3VArg scale, const Ps::aos::QuatVArg scaleRot,
			const bool idtScale) :
			ConvexV(ConvexType::eCONVEXHULL, _center)
		{
			using namespace Ps::aos;

			hullData = _hullData;
			const PxVec3* PX_RESTRICT tempVerts = _hullData->getHullVertices();
			verts = tempVerts;
			numVerts = _hullData->mNbHullVertices;
			CalculateConvexMargin(_hullData, margin, minMargin, sweepMargin, scale);
			ConstructSkewMatrix(scale, scaleRot, vertex2Shape, shape2Vertex, center, idtScale);
			data = _hullData->mBigConvexRawData;
		}

		//this is used by CCD system
		PX_SUPPORT_INLINE ConvexHullV(const PxGeometry& geom) : ConvexV(ConvexType::eCONVEXHULL, Ps::aos::V3Zero())
		{
			using namespace Ps::aos;
			const PxConvexMeshGeometryLL& convexGeom = static_cast<const PxConvexMeshGeometryLL&>(geom);
			const Gu::ConvexHullData* hData = convexGeom.hullData;

			const Vec3V vScale = V3LoadU_SafeReadW(convexGeom.scale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
			const QuatV vRot = QuatVLoadU(&convexGeom.scale.rotation.x);
			const bool idtScale = convexGeom.scale.isIdentity();

			hullData = hData;
			const PxVec3* PX_RESTRICT tempVerts = hData->getHullVertices();
			verts = tempVerts;
			numVerts = hData->mNbHullVertices;
			CalculateConvexMargin(hData, margin, minMargin, sweepMargin, vScale);
			ConstructSkewMatrix(vScale, vRot, vertex2Shape, shape2Vertex, center, idtScale);

			data = hData->mBigConvexRawData;

		}

		PX_SUPPORT_INLINE void initialize(const Gu::ConvexHullData* _hullData, const Ps::aos::Vec3VArg _center, const Ps::aos::Vec3VArg scale,
			const Ps::aos::QuatVArg scaleRot, const bool idtScale)
		{
			using namespace Ps::aos;

			const PxVec3* tempVerts = _hullData->getHullVertices();
			CalculateConvexMargin(_hullData, margin, minMargin, sweepMargin, scale);
			ConstructSkewMatrix(scale, scaleRot, vertex2Shape, shape2Vertex, center, idtScale);

			verts = tempVerts;
			numVerts = _hullData->mNbHullVertices;
			//rot = _rot;	

			center = _center;

			//	searchIndex = 0;
			data = _hullData->mBigConvexRawData;

			hullData = _hullData;
			if (_hullData->mBigConvexRawData)
			{
				Ps::prefetchLine(hullData->mBigConvexRawData->mValencies);
				Ps::prefetchLine(hullData->mBigConvexRawData->mValencies, 128);
				Ps::prefetchLine(hullData->mBigConvexRawData->mAdjacentVerts);
			}
		}


		PX_FORCE_INLINE void resetMargin(const PxReal toleranceLength)
		{
			const PxReal toleranceMinMargin = toleranceLength * TOLERANCE_MIN_MARGIN_RATIO;
			const PxReal toleranceMargin = toleranceLength * TOLERANCE_MARGIN_RATIO;

			margin = PxMin(margin, toleranceMargin);
			minMargin = PxMin(minMargin, toleranceMinMargin);
		}

		PX_FORCE_INLINE Ps::aos::Vec3V supportPoint(const PxI32 index)const
		{
			using namespace Ps::aos;
			
			return M33MulV3(vertex2Shape, V3LoadU_SafeReadW(verts[index]));	// PT: safe because of the way vertex memory is allocated in ConvexHullData (and 'verts' is initialized with ConvexHullData::getHullVertices())
		}

		PX_NOINLINE PxU32 hillClimbing(const Ps::aos::Vec3VArg _dir)const
		{
			using namespace Ps::aos;

			const Gu::Valency* valency = data->mValencies;
			const PxU8* adjacentVerts = data->mAdjacentVerts;
			
			//NotSoTinyBitMap visited;
			PxU32 smallBitMap[8] = {0,0,0,0,0,0,0,0};

		//	PxU32 index = searchIndex;
			PxU32 index = 0;

			{
				PxVec3 vertexSpaceDirection;
				V3StoreU(_dir, vertexSpaceDirection);
				const PxU32 offset = ComputeCubemapNearestOffset(vertexSpaceDirection, data->mSubdiv);
				//const PxU32 offset = ComputeCubemapOffset(vertexSpaceDirection, data->mSubdiv);
				index = data->mSamples[offset];
			}

			Vec3V maxPoint = V3LoadU_SafeReadW(verts[index]);	// PT: safe because of the way vertex memory is allocated in ConvexHullData (and 'verts' is initialized with ConvexHullData::getHullVertices())
			FloatV max = V3Dot(maxPoint, _dir);
	
			PxU32 initialIndex = index;
			
			do
			{
				initialIndex = index;
				const PxU32 numNeighbours = valency[index].mCount;
				const PxU32 offset = valency[index].mOffset;

				for(PxU32 a = 0; a < numNeighbours; ++a)
				{
					const PxU32 neighbourIndex = adjacentVerts[offset + a];

					const Vec3V vertex = V3LoadU_SafeReadW(verts[neighbourIndex]);	// PT: safe because of the way vertex memory is allocated in ConvexHullData (and 'verts' is initialized with ConvexHullData::getHullVertices())
					const FloatV dist = V3Dot(vertex, _dir);
					if(FAllGrtr(dist, max))
					{
						const PxU32 ind = neighbourIndex>>5;
						const PxU32 mask = PxU32(1 << (neighbourIndex & 31));
						if((smallBitMap[ind] & mask) == 0)
						{
							smallBitMap[ind] |= mask;
							max = dist;
							index = neighbourIndex;
						}
					}
				}

			}while(index != initialIndex);

			return index;
		}

		PX_SUPPORT_INLINE PxU32 bruteForceSearch(const Ps::aos::Vec3VArg _dir)const 
		{
			using namespace Ps::aos;
			//brute force
			PxVec3 dir;
			V3StoreU(_dir, dir);

			PxReal max = verts[0].dot(dir);
			PxU32 maxIndex = 0;

			for (PxU32 i = 1; i < numVerts; ++i)
			{
				const PxReal dist = verts[i].dot(dir);
				if (dist > max)
				{
					max = dist;
					maxIndex = i;
				}
			}
			return maxIndex;
		}

		//points are in vertex space, _dir in vertex space
		PX_NOINLINE PxU32 supportVertexIndex(const Ps::aos::Vec3VArg _dir)const
		{
			using namespace Ps::aos;
			if(data)
				return hillClimbing(_dir);
			else
				return bruteForceSearch(_dir);
		}

		//dir is in the vertex space
		PX_SUPPORT_INLINE void bruteForceSearchMinMax(const Ps::aos::Vec3VArg _dir, Ps::aos::FloatV& min, Ps::aos::FloatV& max)const 
		{
			using namespace Ps::aos;

			//brute force
			PxVec3 dir;
			V3StoreU(_dir, dir);
			//get the support point from the orignal margin
			PxReal _max = verts[0].dot(dir);
			PxReal _min = _max;

			for(PxU32 i = 1; i < numVerts; ++i)
			{ 
				const PxReal dist = verts[i].dot(dir);
				_max = PxMax(dist, _max);
				_min = PxMin(dist, _min);
			}
			min = FLoad(_min);
			max = FLoad(_max);
		}

		//This function is used in the full contact manifold generation code, points are in vertex space.
		//This function support scaling, _dir is in the shape space	
		PX_SUPPORT_INLINE void supportVertexMinMax(const Ps::aos::Vec3VArg _dir, Ps::aos::FloatV& min, Ps::aos::FloatV& max)const
		{
			using namespace Ps::aos;

			//dir is in the vertex space
			const Vec3V dir = M33TrnspsMulV3(vertex2Shape, _dir);

			if(data)
			{
				const PxU32 maxIndex= hillClimbing(dir);
				const PxU32 minIndex= hillClimbing(V3Neg(dir));
				const Vec3V maxPoint= M33MulV3(vertex2Shape, V3LoadU_SafeReadW(verts[maxIndex]));	// PT: safe because of the way vertex memory is allocated in ConvexHullData (and 'verts' is initialized with ConvexHullData::getHullVertices())
				const Vec3V minPoint= M33MulV3(vertex2Shape, V3LoadU_SafeReadW(verts[minIndex]));	// PT: safe because of the way vertex memory is allocated in ConvexHullData (and 'verts' is initialized with ConvexHullData::getHullVertices())
				min = V3Dot(_dir, minPoint);
				max = V3Dot(_dir, maxPoint);
			}
			else
			{
				//dir is in the vertex space
				bruteForceSearchMinMax(dir, min, max);
			}
		}  
 
		//This function is used in the full contact manifold generation code
		PX_SUPPORT_INLINE void populateVerts(const PxU8* inds, PxU32 numInds, const PxVec3* originalVerts, Ps::aos::Vec3V* _verts)const
		{
			using namespace Ps::aos;

			for(PxU32 i=0; i<numInds; ++i)
				_verts[i] = M33MulV3(vertex2Shape, V3LoadU_SafeReadW(originalVerts[inds[i]]));	// PT: safe because of the way vertex memory is allocated in ConvexHullData (and 'populateVerts' is always called with polyData.mVerts)
		}

		//This function is used in epa
		//dir is in the shape space
		PX_SUPPORT_INLINE Ps::aos::Vec3V supportLocal(const Ps::aos::Vec3VArg dir)const
		{
			using namespace Ps::aos;
			//scale dir and put it in the vertex space
			const Vec3V _dir = M33TrnspsMulV3(vertex2Shape, dir);
			const PxU32 maxIndex = supportVertexIndex(_dir);
			return M33MulV3(vertex2Shape, V3LoadU_SafeReadW(verts[maxIndex]));	// PT: safe because of the way vertex memory is allocated in ConvexHullData (and 'verts' is initialized with ConvexHullData::getHullVertices())
		}

		//this is used in the sat test for the full contact gen
		PX_SUPPORT_INLINE void supportLocal(const Ps::aos::Vec3VArg dir, Ps::aos::FloatV& min, Ps::aos::FloatV& max)const
		{
			using namespace Ps::aos;
			//dir is in the shape space
			supportVertexMinMax(dir, min, max);
		}

		//This function is used in epa
		PX_SUPPORT_INLINE Ps::aos::Vec3V supportRelative(const Ps::aos::Vec3VArg dir, const Ps::aos::PsMatTransformV& aTob, const Ps::aos::PsMatTransformV& aTobT) const
		{
			using namespace Ps::aos;
		
			//transform dir into the shape space
//			const Vec3V dir_ = aTob.rotateInv(dir);//relTra.rotateInv(dir);
			const Vec3V dir_ = aTobT.rotate(dir);//relTra.rotateInv(dir);
			const Vec3V maxPoint =supportLocal(dir_);
			//translate maxPoint from shape space of a back to the b space
			return aTob.transform(maxPoint);//relTra.transform(maxPoint);
		}

		//dir in the shape space, this function is used in gjk	
		PX_SUPPORT_INLINE Ps::aos::Vec3V supportLocal(const Ps::aos::Vec3VArg dir, PxI32& index)const
		{
			using namespace Ps::aos;
			//scale dir and put it in the vertex space, for non-uniform scale, we don't want the scale in the dir, therefore, we are using
			//the transpose of the inverse of shape2Vertex(which is vertex2shape). This will allow us igore the scale and keep the rotation
			const Vec3V dir_ = M33TrnspsMulV3(vertex2Shape, dir);
			//get the extreme point index
			const PxU32 maxIndex = supportVertexIndex(dir_);
			index = PxI32(maxIndex);
			//p is in the shape space
			return M33MulV3(vertex2Shape, V3LoadU_SafeReadW(verts[index]));	// PT: safe because of the way vertex memory is allocated in ConvexHullData (and 'verts' is initialized with ConvexHullData::getHullVertices())
		}

		//this function is used in gjk	
		PX_SUPPORT_INLINE Ps::aos::Vec3V supportRelative(	const Ps::aos::Vec3VArg dir, const Ps::aos::PsMatTransformV& aTob,
															const Ps::aos::PsMatTransformV& aTobT, PxI32& index)const
		{
			using namespace Ps::aos;

			//transform dir from b space to the shape space of a space
//			const Vec3V dir_ = aTob.rotateInv(dir);//relTra.rotateInv(dir);//M33MulV3(skewInvRot, dir);
			const Vec3V dir_ = aTobT.rotate(dir);//relTra.rotateInv(dir);//M33MulV3(skewInvRot, dir);
			const Vec3V p = supportLocal(dir_, index);
			//transfrom from a to b space
			return aTob.transform(p);
		}

		Ps::aos::Mat33V vertex2Shape;//inv(R)*S*R
		Ps::aos::Mat33V shape2Vertex;//inv(vertex2Shape)

		const Gu::ConvexHullData* hullData;
		const BigConvexRawData* data;  
		const PxVec3* verts;
		PxU8 numVerts;
	};

}

}

#endif	// 
