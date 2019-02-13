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

#ifndef	GU_EPA_FACET_H
#define	GU_EPA_FACET_H

#include "CmPhysXCommon.h"
#include "PsVecMath.h"
#include "PsFPU.h"
#include "PsUtilities.h"
#include "CmIDPool.h"

#if (defined __GNUC__ && defined _DEBUG)
#define PX_EPA_FORCE_INLINE
#else
#define PX_EPA_FORCE_INLINE PX_FORCE_INLINE
#endif

#define EPA_DEBUG	0

namespace physx
{
#define MaxEdges 32
#define MaxFacets 64
#define MaxSupportPoints 64

namespace Gu
{
	const PxU32 lookUp[3] = {1, 2, 0};
	
	PX_FORCE_INLINE PxU32 incMod3(PxU32 i) { return lookUp[i]; } 
	
	class EdgeBuffer;
	class Edge;
	typedef Cm::InlineDeferredIDPool<MaxFacets>	EPAFacetManager;

	class Facet	
	{
	public:
	
		Facet()
		{
		}

		PX_FORCE_INLINE Facet(const PxU32 _i0, const PxU32 _i1, const PxU32 _i2)
			: m_obsolete(false), m_inHeap(false)
		{
			m_indices[0]= Ps::toI8(_i0);
			m_indices[1]= Ps::toI8(_i1);
			m_indices[2]= Ps::toI8(_i2);
			 
			m_adjFacets[0] = m_adjFacets[1] = m_adjFacets[2] = NULL;
			m_adjEdges[0] = m_adjEdges[1] = m_adjEdges[2] = -1;
		}


		PX_FORCE_INLINE void invalidate()
		{
			m_adjFacets[0] = m_adjFacets[1] = m_adjFacets[2] = NULL;
			m_adjEdges[0] = m_adjEdges[1] = m_adjEdges[2] = -1;
		}

		PX_FORCE_INLINE bool Valid()
		{
			return (m_adjFacets[0] != NULL) & (m_adjFacets[1] != NULL) & (m_adjFacets[2] != NULL);
		}

		PX_FORCE_INLINE PxU32 operator[](const PxU32 i) const 
		{ 
			return PxU32(m_indices[i]);
		} 

		//create ajacency information
		bool link(const PxU32 edge0, Facet* PX_RESTRICT  facet, const PxU32 edge1);

		PX_FORCE_INLINE bool isObsolete() const { return m_obsolete; }

		//calculate the signed distance from a point to a plane
		PX_FORCE_INLINE Ps::aos::FloatV getPlaneDist(const Ps::aos::Vec3VArg p, const Ps::aos::Vec3V* PX_RESTRICT aBuf, const Ps::aos::Vec3V* PX_RESTRICT bBuf) const; 

		//check to see whether the triangle is a valid triangle, calculate plane normal and plane distance at the same time
		Ps::aos::BoolV isValid2(const PxU32 i0, const PxU32 i1, const PxU32 i2, const Ps::aos::Vec3V* PX_RESTRICT aBuf, const Ps::aos::Vec3V* PX_RESTRICT bBuf, 
		const Ps::aos::FloatVArg upper);

		//return the absolute value for the plane distance from origin 
		PX_FORCE_INLINE Ps::aos::FloatV getPlaneDist() const 
		{ 
			return Ps::aos::FLoad(m_planeDist);
		}

		//return the plane normal
		PX_FORCE_INLINE Ps::aos::Vec3V getPlaneNormal()const
		{
			return m_planeNormal; 
		}

		//calculate the closest points for a shape pair
		void getClosestPoint(const Ps::aos::Vec3V* PX_RESTRICT aBuf, const Ps::aos::Vec3V* PX_RESTRICT bBuf, Ps::aos::Vec3V& closestA, 
			Ps::aos::Vec3V& closestB);

		//calculate the closest points for a shape pair
		//void getClosestPoint(const Ps::aos::Vec3V* PX_RESTRICT aBuf, const Ps::aos::Vec3V* PX_RESTRICT bBuf, Ps::aos::FloatV& v, Ps::aos::FloatV& w);
		
		//performs a flood fill over the boundary of the current polytope. 
		void silhouette(const Ps::aos::Vec3VArg w, const Ps::aos::Vec3V* PX_RESTRICT aBuf, const Ps::aos::Vec3V* PX_RESTRICT bBuf, EdgeBuffer& edgeBuffer, EPAFacetManager& manager);
		

		//m_planeDist is positive
		bool operator <(const Facet& b) const
		{
			return m_planeDist < b.m_planeDist;
		}
	 
		//store all the boundary facets for the new polytope in the edgeBuffer and free indices when an old facet isn't part of the boundary anymore
		PX_FORCE_INLINE void silhouette(const PxU32 index, const Ps::aos::Vec3VArg w, const Ps::aos::Vec3V* PX_RESTRICT aBuf, const Ps::aos::Vec3V* PX_RESTRICT bBuf, EdgeBuffer& edgeBuffer, 
			EPAFacetManager& manager);

		Ps::aos::Vec3V m_planeNormal;																										//16
		PxF32 m_planeDist;
#if EPA_DEBUG
		PxF32 m_lambda1;
		PxF32 m_lambda2;
#endif
		
		Facet* PX_RESTRICT m_adjFacets[3]; //the triangle adjacent to edge i in this triangle												//32
		PxI8 m_adjEdges[3]; //the edge connected with the corresponding triangle															//35
		PxI8 m_indices[3]; //the index of vertices of the triangle																			//38
		bool m_obsolete; //a flag to denote whether the triangle are still part of the bundeary of the new polytope							//39
		bool m_inHeap;	//a flag to indicate whether the triangle is in the heap															//40
		PxU8 m_FacetId;																														//41																									//73

	};

	
	class Edge 
	{
	public:
		PX_FORCE_INLINE Edge() {}
		PX_FORCE_INLINE Edge(Facet * PX_RESTRICT facet, const PxU32 index) : m_facet(facet), m_index(index) {}
		PX_FORCE_INLINE Edge(const Edge& other) : m_facet(other.m_facet), m_index(other.m_index){}

		PX_FORCE_INLINE Edge& operator = (const Edge& other)
		{
			m_facet = other.m_facet;
			m_index = other.m_index;
			return *this;
		}

		PX_FORCE_INLINE Facet *getFacet() const { return m_facet; }
		PX_FORCE_INLINE PxU32 getIndex() const { return m_index; }

		//get out the associated start vertex index in this edge from the facet
		PX_FORCE_INLINE PxU32 getSource() const
		{
			PX_ASSERT(m_index < 3);
			return (*m_facet)[m_index];
		}

		//get out the associated end vertex index in this edge from the facet
		PX_FORCE_INLINE PxU32 getTarget() const
		{
			PX_ASSERT(m_index < 3);
			return (*m_facet)[incMod3(m_index)];
		}

		Facet* PX_RESTRICT m_facet;
		PxU32 m_index;
	};


	class EdgeBuffer
	{
	public:
		EdgeBuffer() : m_Size(0), m_OverFlow(false)
		{
		}

		Edge* Insert(Facet* PX_RESTRICT  facet, const PxU32 index)
		{
			if (m_Size < MaxEdges)
			{
				Edge* pEdge = &m_pEdges[m_Size++];
				pEdge->m_facet = facet;
				pEdge->m_index = index;
				return pEdge;
			}
			m_OverFlow = true;
			return NULL;
		}

		Edge* Get(const PxU32 index)
		{
			PX_ASSERT(index < m_Size);
			return &m_pEdges[index];
		}

		PxU32 Size()
		{
			return m_Size;
		}

		bool IsValid()
		{
			return m_Size > 0 && !m_OverFlow;
		}

		void MakeEmpty()
		{
			m_Size = 0;
			m_OverFlow = false;
		}

		Edge	m_pEdges[MaxEdges];
		PxU32	m_Size;
		bool	m_OverFlow;
	};

	//ML: calculate MTD points for a shape pair
	PX_FORCE_INLINE void Facet::getClosestPoint(const Ps::aos::Vec3V* PX_RESTRICT aBuf, const Ps::aos::Vec3V* PX_RESTRICT bBuf, Ps::aos::Vec3V& closestA, 
		Ps::aos::Vec3V& closestB)
	{
		using namespace Ps::aos;

		const Vec3V pa0(aBuf[m_indices[0]]);
		const Vec3V pa1(aBuf[m_indices[1]]);
		const Vec3V pa2(aBuf[m_indices[2]]);

		const Vec3V pb0(bBuf[m_indices[0]]);
		const Vec3V pb1(bBuf[m_indices[1]]);
		const Vec3V pb2(bBuf[m_indices[2]]);

		const Vec3V p0 = V3Sub(pa0, pb0);
		const Vec3V p1 = V3Sub(pa1, pb1);
		const Vec3V p2 = V3Sub(pa2, pb2);

		const Vec3V v0 = V3Sub(p1, p0);
		const Vec3V v1 = V3Sub(p2, p0);

		const Vec3V closestP = V3Scale(m_planeNormal, FLoad(m_planeDist));
		const Vec3V v2 = V3Sub(closestP, p0);
		//calculate barycentric coordinates
		const FloatV d00 = V3Dot(v0, v0);
		const FloatV d01 = V3Dot(v0, v1);
		const FloatV d11 = V3Dot(v1, v1);

		const FloatV d20 = V3Dot(v2, v0);
		const FloatV d21 = V3Dot(v2, v1);

		const FloatV det = FNegScaleSub(d01, d01, FMul(d00, d11));//FSub( FMul(v1dv1, v2dv2), FMul(v1dv2, v1dv2) ); // non-negative
		const FloatV recip = FSel(FIsGrtr(det, FEps()), FRecip(det), FZero());

		const FloatV lambda1 = FMul(FNegScaleSub(d01, d21, FMul(d11, d20)), recip);
		const FloatV lambda2 = FMul(FNegScaleSub(d01, d20, FMul(d00, d21)), recip);

#if EPA_DEBUG
		FStore(lambda1, &m_lambda1);
		FStore(lambda2, &m_lambda2);
#endif

		const FloatV u = FSub(FOne(), FAdd(lambda1, lambda2));
		closestA = V3ScaleAdd(pa0, u, V3ScaleAdd(pa1, lambda1, V3Scale(pa2, lambda2)));
		closestB = V3ScaleAdd(pb0, u, V3ScaleAdd(pb1, lambda1, V3Scale(pb2, lambda2)));
	}

	//ML: create adjacency informations for both facets
	PX_FORCE_INLINE bool Facet::link(const PxU32 edge0, Facet * PX_RESTRICT facet, const PxU32 edge1) 
	{
		m_adjFacets[edge0] = facet;
		m_adjEdges[edge0] = Ps::toI8(edge1);
		facet->m_adjFacets[edge1] = this;
		facet->m_adjEdges[edge1] = Ps::toI8(edge0);

		return (m_indices[edge0] == facet->m_indices[incMod3(edge1)]) && (m_indices[incMod3(edge0)] == facet->m_indices[edge1]);
	} 

}

}

#endif
