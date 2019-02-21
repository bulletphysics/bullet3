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


#ifndef PX_PHYSICS_GEOMUTILS_PX_ADJACENCIES
#define PX_PHYSICS_GEOMUTILS_PX_ADJACENCIES

#define MSH_ADJACENCIES_INCLUDE_CONVEX_BITS
#include "foundation/Px.h"
#include "GuTriangle32.h"

namespace physx
{

#ifdef MSH_ADJACENCIES_INCLUDE_CONVEX_BITS
	#define ADJ_TRIREF_MASK		0x1fffffff				//!< Masks 3 bits
	#define IS_CONVEX_EDGE(x)	(x & 0x20000000)		//!< Returns true for convex edges
#else
	#define ADJ_TRIREF_MASK		0x3fffffff				//!< Masks 2 bits
#endif

	#define MAKE_ADJ_TRI(x)		(x & ADJ_TRIREF_MASK)	//!< Transforms a link into a triangle reference.
	#define GET_EDGE_NB(x)		(x>>30)					//!< Transforms a link into a counterpart edge ID.
//	#define IS_BOUNDARY(x)		(x==PX_INVALID_U32)		//!< Returns true for boundary edges.
	#define IS_BOUNDARY(x)		((x & ADJ_TRIREF_MASK)==ADJ_TRIREF_MASK)		//!< Returns true for boundary edges.

	// Forward declarations
	class Adjacencies;

	enum SharedEdgeIndex
	{
		EDGE01	= 0,
		EDGE02	= 1,
		EDGE12	= 2
	};

/*	PX_INLINE void GetEdgeIndices(SharedEdgeIndex edge_index, PxU32& id0, PxU32& id1)
	{
		if(edge_index==0)
		{
			id0 = 0;
			id1 = 1;
		}
		else if(edge_index==1)
		{
			id0 = 0;
			id1 = 2;
		}
		else if(edge_index==2)
		{
			id0 = 1;
			id1 = 2;
		}
	}*/

	//! Sets a new edge code
	#define SET_EDGE_NB(link, code)	\
		link&=ADJ_TRIREF_MASK;		\
		link|=code<<30;				\

	//! A triangle class used to compute the adjacency structures.
	class AdjTriangle
#ifdef MSH_ADJACENCIES_INCLUDE_TOPOLOGY
		: public IndexedTriangle
#else
		: public Ps::UserAllocated
#endif
	{
		public:
		//! Constructor
		PX_INLINE						AdjTriangle()	{}
		//! Destructor
		PX_INLINE						~AdjTriangle()	{}

		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Computes the number of boundary edges in a triangle.
		 *	\return		the number of boundary edges. (0 => 3)
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				PxU32				ComputeNbBoundaryEdges()						const;
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Computes the number of valid neighbors.
		 *	\return		the number of neighbors. (0 => 3)
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				PxU32				ComputeNbNeighbors()							const;
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Checks whether the triangle has a particular neighbor or not.
		 *	\param		tref	[in] the triangle reference to look for
		 *	\param		index	[out] the corresponding index in the triangle (NULL if not needed)
		 *	\return		true if the triangle has the given neighbor
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				bool				HasNeighbor(PxU32 tref, PxU32* index=NULL)	const;
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/**
		 *	Flips the winding.
		 */
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				void				Flip();

		// Data access
		PX_INLINE	PxU32			GetLink(SharedEdgeIndex edge_index)				const	{ return mATri[edge_index];					}
		PX_INLINE	PxU32			GetAdjTri(SharedEdgeIndex edge_index)			const	{ return MAKE_ADJ_TRI(mATri[edge_index]);	}
		PX_INLINE	PxU32			GetAdjEdge(SharedEdgeIndex edge_index)			const	{ return GET_EDGE_NB(mATri[edge_index]);	}
		PX_INLINE	Ps::IntBool		IsBoundaryEdge(SharedEdgeIndex edge_index)		const	{ return IS_BOUNDARY(mATri[edge_index]);	}
#ifdef MSH_ADJACENCIES_INCLUDE_CONVEX_BITS
		PX_INLINE	Ps::IntBool		HasActiveEdge01()								const	{ return Ps::IntBool(IS_CONVEX_EDGE(mATri[EDGE01]));	}
		PX_INLINE	Ps::IntBool		HasActiveEdge20()								const	{ return Ps::IntBool(IS_CONVEX_EDGE(mATri[EDGE02]));	}
		PX_INLINE	Ps::IntBool		HasActiveEdge12()								const	{ return Ps::IntBool(IS_CONVEX_EDGE(mATri[EDGE12]));	}
		PX_INLINE	Ps::IntBool		HasActiveEdge(PxU32 i)							const	{ return Ps::IntBool(IS_CONVEX_EDGE(mATri[i]));			}
#endif
//		private:
		//! Links/References of adjacent triangles. The 2 most significant bits contains the counterpart edge in the adjacent triangle.
		//! mATri[0] refers to edge 0-1
		//! mATri[1] refers to edge 0-2
		//! mATri[2] refers to edge 1-2
				PxU32				mATri[3];
	};

	//! The adjacencies creation structure.
	struct ADJACENCIESCREATE
	{
		//! Constructor
									ADJACENCIESCREATE()	: NbFaces(0), DFaces(NULL), WFaces(NULL)
									{
#ifdef MSH_ADJACENCIES_INCLUDE_CONVEX_BITS
									Verts	= NULL;
									Epsilon	= 0.1f;
//									Epsilon	= 0.001f;
#endif
									}
				
				PxU32				NbFaces;		//!< Number of faces in source topo
				const PxU32*		DFaces;			//!< List of faces (dwords) or NULL
				const PxU16*		WFaces;			//!< List of faces (words) or NULL
#ifdef MSH_ADJACENCIES_INCLUDE_CONVEX_BITS
				const PxVec3*		Verts;
				float				Epsilon;
#endif
	};

	class Adjacencies : public Ps::UserAllocated
	{
		public:
									Adjacencies();
									~Adjacencies();

				PxU32				mNbFaces;		//!< Number of faces involved in the computation.
				AdjTriangle*		mFaces;			//!< A list of AdjTriangles (one/face)

				bool				Load(PxInputStream& stream);
		// Basic mesh walking
		PX_INLINE	const AdjTriangle*	GetAdjacentFace(const AdjTriangle& current_tri, SharedEdgeIndex edge_nb) const
									{
										// No checkings here, make sure mFaces has been created

										// Catch the link
										PxU32 Link = current_tri.GetLink(edge_nb);

										// Returns NULL for boundary edges
										if(IS_BOUNDARY(Link))	return NULL;

										// Else transform into face index
										PxU32 Id = MAKE_ADJ_TRI(Link);

										// Possible counterpart edge is:
										// PxU32 Edge = GET_EDGE_NB(Link);

										// And returns adjacent triangle
										return &mFaces[Id];
									}
		// Helpers
				PxU32				ComputeNbBoundaryEdges() const;
#ifdef MSH_ADJACENCIES_INCLUDE_TOPOLOGY
				bool				GetBoundaryVertices(PxU32 nb_verts, bool* bound_status) const;
#else
				bool				GetBoundaryVertices(PxU32 nb_verts, bool* bound_status, const Gu::TriangleT<PxU32>* faces) const;
#endif
		// 
#ifdef MSH_ADJACENCIES_INCLUDE_TOPOLOGY
				bool				MakeLastRef(AdjTriangle& cur_tri, PxU32 vref);
#else
				bool				MakeLastRef(AdjTriangle& cur_tri, PxU32 vref, Gu::TriangleT<PxU32>* cur_topo);
#endif
		private:
		// New edge codes assignment
				void				AssignNewEdgeCode(PxU32 link, PxU8 edge_nb);
	};

//#ifdef PX_COOKING
	class AdjacenciesBuilder : public Adjacencies
	{
		public:
									AdjacenciesBuilder();
									~AdjacenciesBuilder();

				bool				Init(const ADJACENCIESCREATE& create);
//				bool				Save(Stream& stream)	const;
	};
//#endif

}

#endif
