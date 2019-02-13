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

#include "foundation/PxMemory.h"
#include "EdgeList.h"
#include "Adjacencies.h"
#include "CmRadixSortBuffered.h"
#include "GuSerialize.h"
#include "PsFoundation.h"

using namespace physx;
using namespace Gu;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Flips the winding.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AdjTriangle::Flip()
{
#ifdef MSH_ADJACENCIES_INCLUDE_TOPOLOGY
	// Call the Triangle method
	IndexedTriangle::Flip();
#endif

	// Flip links. We flipped vertex references 1 & 2, i.e. links 0 & 1.
	physx::shdfnd::swap(mATri[0], mATri[1]);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the number of boundary edges in a triangle.
 *	\return		the number of boundary edges. (0 => 3)
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PxU32 AdjTriangle::ComputeNbBoundaryEdges() const
{
	// Look for boundary edges
	PxU32 Nb = 0;
	if(IS_BOUNDARY(mATri[0]))	Nb++;
	if(IS_BOUNDARY(mATri[1]))	Nb++;
	if(IS_BOUNDARY(mATri[2]))	Nb++;
	return Nb;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the number of valid neighbors.
 *	\return		the number of neighbors. (0 => 3)
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PxU32 AdjTriangle::ComputeNbNeighbors() const
{
	PxU32 Nb = 0;
	if(!IS_BOUNDARY(mATri[0]))	Nb++;
	if(!IS_BOUNDARY(mATri[1]))	Nb++;
	if(!IS_BOUNDARY(mATri[2]))	Nb++;
	return Nb;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Checks whether the triangle has a particular neighbor or not.
 *	\param		tref	[in] the triangle reference to look for
 *	\param		index	[out] the corresponding index in the triangle (NULL if not needed)
 *	\return		true if the triangle has the given neighbor
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AdjTriangle::HasNeighbor(PxU32 tref, PxU32* index) const
{
	// ### could be optimized
	if(!IS_BOUNDARY(mATri[0]) && MAKE_ADJ_TRI(mATri[0])==tref)	{ if(index)	*index = 0;	return true; }
	if(!IS_BOUNDARY(mATri[1]) && MAKE_ADJ_TRI(mATri[1])==tref)	{ if(index)	*index = 1;	return true; }
	if(!IS_BOUNDARY(mATri[2]) && MAKE_ADJ_TRI(mATri[2])==tref)	{ if(index)	*index = 2;	return true; }
	return false;
}




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Constructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Adjacencies::Adjacencies() : mNbFaces(0), mFaces(NULL)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Destructor.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Adjacencies::~Adjacencies()
{
	PX_DELETE_ARRAY(mFaces);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the number of boundary edges.
 *	\return		the number of boundary edges.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PxU32 Adjacencies::ComputeNbBoundaryEdges() const
{
	// Checking
	if(!mFaces)	return 0;

	// Look for boundary edges
	PxU32 Nb = 0;
	for(PxU32 i=0;i<mNbFaces;i++)
	{
		AdjTriangle* CurTri = &mFaces[i];
		Nb+=CurTri->ComputeNbBoundaryEdges();
	}
	return Nb;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Computes the boundary vertices. A boundary vertex is defined as a vertex shared by at least one boundary edge.
 *	\param		nb_verts		[in] the number of vertices
 *	\param		bound_status	[out] a user-provided array of bool
 *	\return		true if success. The user-array is filled with true or false (boundary vertex / not boundary vertex)
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef MSH_ADJACENCIES_INCLUDE_TOPOLOGY
bool Adjacencies::GetBoundaryVertices(PxU32 nb_verts, bool* bound_status) const
#else
bool Adjacencies::GetBoundaryVertices(PxU32 nb_verts, bool* bound_status, const Gu::TriangleT<PxU32>* faces) const
#endif
{
	// We need the adjacencies
	if(!mFaces || !bound_status || !nb_verts)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Adjacencies::GetBoundaryVertices: NULL parameter!");
		return false;
	}

#ifndef MSH_ADJACENCIES_INCLUDE_TOPOLOGY
	if(!faces)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Adjacencies::GetBoundaryVertices: NULL parameter!");
		return false;
	}
#endif

	// Init
	PxMemZero(bound_status, nb_verts*sizeof(bool));

	// Loop through faces
	for(PxU32 i=0;i<mNbFaces;i++)
	{
		AdjTriangle* CurTri = &mFaces[i];
		if(IS_BOUNDARY(CurTri->mATri[0]))
		{
			// Two boundary vertices: 0 - 1
#ifdef MSH_ADJACENCIES_INCLUDE_TOPOLOGY
			PxU32 VRef0 = CurTri->v[0];	if(VRef0>=nb_verts)	return false;	bound_status[VRef0] = true;
			PxU32 VRef1 = CurTri->v[1];	if(VRef1>=nb_verts)	return false;	bound_status[VRef1] = true;
#else
			PxU32 VRef0 = faces[i].v[0];	if(VRef0>=nb_verts)	return false;	bound_status[VRef0] = true;
			PxU32 VRef1 = faces[i].v[1];	if(VRef1>=nb_verts)	return false;	bound_status[VRef1] = true;
#endif
		}
		if(IS_BOUNDARY(CurTri->mATri[1]))
		{
			// Two boundary vertices: 0 - 2
#ifdef MSH_ADJACENCIES_INCLUDE_TOPOLOGY
			PxU32 VRef0 = CurTri->v[0];	if(VRef0>=nb_verts)	return false;	bound_status[VRef0] = true;
			PxU32 VRef1 = CurTri->v[2];	if(VRef1>=nb_verts)	return false;	bound_status[VRef1] = true;
#else
			PxU32 VRef0 = faces[i].v[0];	if(VRef0>=nb_verts)	return false;	bound_status[VRef0] = true;
			PxU32 VRef1 = faces[i].v[2];	if(VRef1>=nb_verts)	return false;	bound_status[VRef1] = true;
#endif
		}
		if(IS_BOUNDARY(CurTri->mATri[2]))
		{
			// Two boundary vertices: 1 - 2
#ifdef MSH_ADJACENCIES_INCLUDE_TOPOLOGY
			PxU32 VRef0 = CurTri->v[1];	if(VRef0>=nb_verts)	return false;	bound_status[VRef0] = true;
			PxU32 VRef1 = CurTri->v[2];	if(VRef1>=nb_verts)	return false;	bound_status[VRef1] = true;
#else
			PxU32 VRef0 = faces[i].v[1];	if(VRef0>=nb_verts)	return false;	bound_status[VRef0] = true;
			PxU32 VRef1 = faces[i].v[2];	if(VRef1>=nb_verts)	return false;	bound_status[VRef1] = true;
#endif
		}
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Assigns a new edge code to the counterpart link of a given link.
 *	\param		link		[in] the link to modify - shouldn't be a boundary link
 *	\param		edge_nb		[in] the new edge number
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Adjacencies::AssignNewEdgeCode(PxU32 link, PxU8 edge_nb)
{
	if(!IS_BOUNDARY(link))
	{
		PxU32 Id = MAKE_ADJ_TRI(link);				// Triangle ID
		PxU32 Edge = GET_EDGE_NB(link);			// Counterpart edge ID
		AdjTriangle* Tri = &mFaces[Id];				// Adjacent triangle

		// Get link whose edge code is invalid
		PxU32 AdjLink = Tri->mATri[Edge];			// Link to ourself (i.e. to 'link')
		SET_EDGE_NB(AdjLink, edge_nb);				// Assign new edge code
		Tri->mATri[Edge] = AdjLink;					// Put link back
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Modifies the existing database so that reference 'vref' of triangle 'curtri' becomes the last one.
 *	Provided reference must already exist in provided triangle.
 *	\param		cur_tri			[in] the triangle
 *	\param		vref			[in] the reference
 *	\return		true if success.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef MSH_ADJACENCIES_INCLUDE_TOPOLOGY
bool Adjacencies::MakeLastRef(AdjTriangle& cur_tri, PxU32 vref)
#else
bool Adjacencies::MakeLastRef(AdjTriangle& cur_tri, PxU32 vref, Gu::TriangleT<PxU32>* cur_topo)
#endif
{
#ifndef MSH_ADJACENCIES_INCLUDE_TOPOLOGY
	// Checkings
	if(!cur_topo)
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Adjacencies::MakeLastRef: NULL parameter!");
		return false;
	}
#endif
	// We want pattern (x y vref)
	// Edge 0-1 is (x y)
	// Edge 0-2 is (x vref)
	// Edge 1-2 is (y vref)

	// First thing is to scroll the existing references in order for vref to become the last one. Scrolling assures winding order is conserved.

	// Edge code need fixing as well:
	// The two MSB for each link encode the counterpart edge in adjacent triangle. We swap the link positions, but adjacent triangles remain the
	// same. In other words, edge codes are still valid for current triangle since counterpart edges have not been swapped. *BUT* edge codes of
	// the three possible adjacent triangles *are* now invalid. We need to fix edge codes, but for adjacent triangles...

#ifdef MSH_ADJACENCIES_INCLUDE_TOPOLOGY
	if(cur_tri.v[0]==vref)
#else
	if(cur_topo->v[0]==vref)
#endif
	{
		// Pattern is (vref x y)
		// Edge 0-1 is (vref x)
		// Edge 0-2 is (vref y)
		// Edge 1-2 is (x y)

		// Catch original data
#ifdef MSH_ADJACENCIES_INCLUDE_TOPOLOGY
		PxU32 Ref0 = cur_tri.v[0];		PxU32 Link01 = cur_tri.mATri[0];
		PxU32 Ref1 = cur_tri.v[1];		PxU32 Link02 = cur_tri.mATri[1];
		PxU32 Ref2 = cur_tri.v[2];		PxU32 Link12 = cur_tri.mATri[2];

		// Swap
		cur_tri.v[0] = Ref1;
		cur_tri.v[1] = Ref2;
		cur_tri.v[2] = Ref0;
#else
		PxU32 Ref0 = cur_topo->v[0];		PxU32 Link01 = cur_tri.mATri[0];
		PxU32 Ref1 = cur_topo->v[1];		PxU32 Link02 = cur_tri.mATri[1];
		PxU32 Ref2 = cur_topo->v[2];		PxU32 Link12 = cur_tri.mATri[2];

		// Swap
		cur_topo->v[0] = Ref1;
		cur_topo->v[1] = Ref2;
		cur_topo->v[2] = Ref0;
#endif
		cur_tri.mATri[0] = Link12;	// Edge 0-1 now encodes Ref1-Ref2, i.e. previous Link12
		cur_tri.mATri[1] = Link01;	// Edge 0-2 now encodes Ref1-Ref0, i.e. previous Link01
		cur_tri.mATri[2] = Link02;	// Edge 1-2 now encodes Ref2-Ref0, i.e. previous Link02

		// Fix edge codes
		AssignNewEdgeCode(Link01, 1);
		AssignNewEdgeCode(Link02, 2);
		AssignNewEdgeCode(Link12, 0);

		return true;
	}
#ifdef MSH_ADJACENCIES_INCLUDE_TOPOLOGY
	else if(cur_tri.v[1]==vref)
#else
	else if(cur_topo->v[1]==vref)
#endif
	{
		// Pattern is (x vref y)
		// Edge 0-1 is (x vref)
		// Edge 0-2 is (x y)
		// Edge 1-2 is (vref y)

		// Catch original data
#ifdef MSH_ADJACENCIES_INCLUDE_TOPOLOGY
		PxU32 Ref0 = cur_tri.v[0];		PxU32 Link01 = cur_tri.mATri[0];
		PxU32 Ref1 = cur_tri.v[1];		PxU32 Link02 = cur_tri.mATri[1];
		PxU32 Ref2 = cur_tri.v[2];		PxU32 Link12 = cur_tri.mATri[2];

		// Swap
		cur_tri.v[0] = Ref2;
		cur_tri.v[1] = Ref0;
		cur_tri.v[2] = Ref1;
#else
		PxU32 Ref0 = cur_topo->v[0];		PxU32 Link01 = cur_tri.mATri[0];
		PxU32 Ref1 = cur_topo->v[1];		PxU32 Link02 = cur_tri.mATri[1];
		PxU32 Ref2 = cur_topo->v[2];		PxU32 Link12 = cur_tri.mATri[2];

		// Swap
		cur_topo->v[0] = Ref2;
		cur_topo->v[1] = Ref0;
		cur_topo->v[2] = Ref1;
#endif
		cur_tri.mATri[0] = Link02;	// Edge 0-1 now encodes Ref2-Ref0, i.e. previous Link02
		cur_tri.mATri[1] = Link12;	// Edge 0-2 now encodes Ref2-Ref1, i.e. previous Link12
		cur_tri.mATri[2] = Link01;	// Edge 1-2 now encodes Ref0-Ref1, i.e. previous Link01

		// Fix edge codes
		AssignNewEdgeCode(Link01, 2);
		AssignNewEdgeCode(Link02, 0);
		AssignNewEdgeCode(Link12, 1);

		return true;
	}
#ifdef MSH_ADJACENCIES_INCLUDE_TOPOLOGY
	else if(cur_tri.v[2]==vref)
#else
	else if(cur_topo->v[2]==vref)
#endif
	{
		// Nothing to do, provided reference already is the last one
		return true;
	}

	// Here the provided reference doesn't belong to the provided triangle.
	return false;
}

bool Adjacencies::Load(PxInputStream& stream)
{
	// Import header
	PxU32 Version;
	bool Mismatch;
	if(!ReadHeader('A', 'D', 'J', 'A', Version, Mismatch, stream))
		return false;

	// Import adjacencies
	mNbFaces = readDword(Mismatch, stream);
	mFaces = PX_NEW(AdjTriangle)[mNbFaces];
	stream.read(mFaces, sizeof(AdjTriangle)*mNbFaces);

	return true;
}

//#ifdef PX_COOKING

	//! An edge class used to compute the adjacency structures.
	class AdjEdge : public Gu::EdgeData, public Ps::UserAllocated
	{
		public:
		//! Constructor
		PX_INLINE						AdjEdge()	{}
		//! Destructor
		PX_INLINE						~AdjEdge()	{}

				PxU32				mFaceNb;		//!< Owner face
	};

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	 *	Adds a new edge to the database.
	 *	\param		ref0	[in] vertex reference for the new edge
	 *	\param		ref1	[in] vertex reference for the new edge
	 *	\param		face	[in] owner face
	 */
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	static void AddEdge(PxU32 ref0, PxU32 ref1, PxU32 face, PxU32& nb_edges, AdjEdge* edges)
	{
		// Store edge data
		edges[nb_edges].Ref0	= ref0;
		edges[nb_edges].Ref1	= ref1;
		edges[nb_edges].mFaceNb	= face;
		nb_edges++;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	 *	Adds a new triangle to the database.
	 *	\param		ref0	[in] vertex reference for the new triangle
	 *	\param		ref1	[in] vertex reference for the new triangle
	 *	\param		ref2	[in] vertex reference for the new triangle
	 *	\param		id		[in] triangle index
	 */
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	static void AddTriangle(PxU32 ref0, PxU32 ref1, PxU32 ref2, PxU32 id, AdjTriangle* faces, PxU32& nb_edges, AdjEdge* edges)
	{
#ifdef MSH_ADJACENCIES_INCLUDE_TOPOLOGY
		// Store vertex-references
		faces[id].v[0]	= ref0;
		faces[id].v[1]	= ref1;
		faces[id].v[2]	= ref2;
#endif
		// Reset links
		faces[id].mATri[0]	= PX_INVALID_U32;
		faces[id].mATri[1]	= PX_INVALID_U32;
		faces[id].mATri[2]	= PX_INVALID_U32;

		// Add edge 01 to database
		if(ref0<ref1)	AddEdge(ref0, ref1, id, nb_edges, edges);
		else			AddEdge(ref1, ref0, id, nb_edges, edges);
		// Add edge 02 to database
		if(ref0<ref2)	AddEdge(ref0, ref2, id, nb_edges, edges);
		else			AddEdge(ref2, ref0, id, nb_edges, edges);
		// Add edge 12 to database
		if(ref1<ref2)	AddEdge(ref1, ref2, id, nb_edges, edges);
		else			AddEdge(ref2, ref1, id, nb_edges, edges);
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	 *	Updates the links in two adjacent triangles.
	 *	\param		first_tri	[in] index of the first triangle
	 *	\param		second_tri	[in] index of the second triangle
	 *	\param		ref0		[in] the common edge's first vertex reference
	 *	\param		ref1		[in] the common edge's second vertex reference
	 *	\return		true if success.
	 */
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef MSH_ADJACENCIES_INCLUDE_TOPOLOGY
	static bool UpdateLink(PxU32 first_tri, PxU32 second_tri, PxU32 ref0, PxU32 ref1, AdjTriangle* faces)
#else
	static bool UpdateLink(PxU32 first_tri, PxU32 second_tri, PxU32 ref0, PxU32 ref1, AdjTriangle* faces, const ADJACENCIESCREATE& create)
#endif
	{
#ifdef MSH_ADJACENCIES_INCLUDE_TOPOLOGY
		AdjTriangle& Tri0 = faces[first_tri];	// Catch the first triangle
		AdjTriangle& Tri1 = faces[second_tri];	// Catch the second triangle

		// Get the edge IDs. 0xff means input references are wrong.
		PxU8 EdgeNb0 = Tri0.FindEdge(ref0, ref1);		if(EdgeNb0==0xff)	return SetIceError("Adjacencies::UpdateLink: invalid edge reference in first triangle");
		PxU8 EdgeNb1 = Tri1.FindEdge(ref0, ref1);		if(EdgeNb1==0xff)	return SetIceError("Adjacencies::UpdateLink: invalid edge reference in second triangle");

		// Update links. The two most significant bits contain the counterpart edge's ID.
		Tri0.mATri[EdgeNb0] = second_tri	|(PxU32(EdgeNb1)<<30);
		Tri1.mATri[EdgeNb1] = first_tri		|(PxU32(EdgeNb0)<<30);
#else
		Gu::TriangleT<PxU32> FirstTri, SecondTri;
		if(create.DFaces)
		{
			FirstTri.v[0] = create.DFaces[first_tri*3+0];
			FirstTri.v[1] = create.DFaces[first_tri*3+1];
			FirstTri.v[2] = create.DFaces[first_tri*3+2];
			SecondTri.v[0] = create.DFaces[second_tri*3+0];
			SecondTri.v[1] = create.DFaces[second_tri*3+1];
			SecondTri.v[2] = create.DFaces[second_tri*3+2];
		}
		if(create.WFaces)
		{
			FirstTri.v[0] = create.WFaces[first_tri*3+0];
			FirstTri.v[1] = create.WFaces[first_tri*3+1];
			FirstTri.v[2] = create.WFaces[first_tri*3+2];
			SecondTri.v[0] = create.WFaces[second_tri*3+0];
			SecondTri.v[1] = create.WFaces[second_tri*3+1];
			SecondTri.v[2] = create.WFaces[second_tri*3+2];
		}

		// Get the edge IDs. 0xff means input references are wrong.
		const PxU8 EdgeNb0 = FirstTri.findEdge(ref0, ref1);
		const PxU8 EdgeNb1 = SecondTri.findEdge(ref0, ref1);
		if(EdgeNb0==0xff || EdgeNb1==0xff)
		{
			Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Adjacencies::UpdateLink: invalid edge reference");
			return false;
		}

		// Update links. The two most significant bits contain the counterpart edge's ID.
		faces[first_tri].mATri[EdgeNb0] = second_tri	|(PxU32(EdgeNb1)<<30);
		faces[second_tri].mATri[EdgeNb1] = first_tri	|(PxU32(EdgeNb0)<<30);
#endif
		return true;
	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/**
	 *	Creates the adjacency structures.
	 *	\return		true if success.
	 */
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef MSH_ADJACENCIES_INCLUDE_TOPOLOGY
	static bool CreateDatabase(AdjTriangle* faces, PxU32 nb_edges, const AdjEdge* edges)
#else
	static bool CreateDatabase(AdjTriangle* faces, PxU32 nb_edges, const AdjEdge* edges, const ADJACENCIESCREATE& create)
#endif
	{
		Cm::RadixSortBuffered Core;
		{
			// Multiple sorts - this rewritten version uses less ram
			// PT: TTP 2994: the mesh has 343000+ edges, so yeah, sure, allocating more than 1mb on the stack causes overflow...
			PxU32* VRefs = PX_NEW_TEMP(PxU32)[nb_edges];

			// Sort according to mRef0, then mRef1
			PxU32 i;
			for(i=0;i<nb_edges;i++)	
				VRefs[i] = edges[i].Ref0;	
			Core.Sort(VRefs, nb_edges);
			for(i=0;i<nb_edges;i++)	
				VRefs[i] = edges[i].Ref1;	
			Core.Sort(VRefs, nb_edges);

			PX_DELETE_POD(VRefs);
		}
		const PxU32* Sorted = Core.GetRanks();

		// Read the list in sorted order, look for similar edges
		PxU32 LastRef0 = edges[Sorted[0]].Ref0;
		PxU32 LastRef1 = edges[Sorted[0]].Ref1;
		PxU32 Count = 0;
		PxU32 TmpBuffer[3];

		while(nb_edges--)
		{
			PxU32 SortedIndex = *Sorted++;
			PxU32 Face = edges[SortedIndex].mFaceNb;	// Owner face
			PxU32 Ref0 = edges[SortedIndex].Ref0;		// Vertex ref #1
			PxU32 Ref1 = edges[SortedIndex].Ref1;		// Vertex ref #2
			if(Ref0==LastRef0 && Ref1==LastRef1)
			{
				// Current edge is the same as last one
				TmpBuffer[Count++] = Face;				// Store face number
				// Only works with manifold meshes (i.e. an edge is not shared by more than 2 triangles)
				if(Count==3)
				{
					Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Adjacencies::CreateDatabase: can't work on non-manifold meshes.");
					return false;
				}
			}
			else
			{
				// Here we have a new edge (LastRef0, LastRef1) shared by Count triangles stored in TmpBuffer
				if(Count==2)
				{
					// if Count==1 => edge is a boundary edge: it belongs to a single triangle.
					// Hence there's no need to update a link to an adjacent triangle.
#ifdef MSH_ADJACENCIES_INCLUDE_TOPOLOGY
					if(!UpdateLink(TmpBuffer[0], TmpBuffer[1], LastRef0, LastRef1, faces))	return false;
#else
					if(!UpdateLink(TmpBuffer[0], TmpBuffer[1], LastRef0, LastRef1, faces, create))	return false;
#endif
				}
				// Reset for next edge
				Count = 0;
				TmpBuffer[Count++] = Face;
				LastRef0 = Ref0;
				LastRef1 = Ref1;
			}
		}
		bool Status = true;
#ifdef MSH_ADJACENCIES_INCLUDE_TOPOLOGY
		if(Count==2)	Status = UpdateLink(TmpBuffer[0], TmpBuffer[1], LastRef0, LastRef1, faces);
#else
		if(Count==2)	Status = UpdateLink(TmpBuffer[0], TmpBuffer[1], LastRef0, LastRef1, faces, create);
#endif
		return Status;
	}

AdjacenciesBuilder::AdjacenciesBuilder()
{
}

AdjacenciesBuilder::~AdjacenciesBuilder()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Initializes the component.
 *	\param		create		[in] the creation structure
 *	\return		true if success.
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool AdjacenciesBuilder::Init(const ADJACENCIESCREATE& create)
{
	// Checkings
	if(!create.NbFaces)	return false;

	// Get some bytes
	mNbFaces	= create.NbFaces;
	mFaces		= PX_NEW(AdjTriangle)[mNbFaces];

	AdjEdge* Edges = PX_NEW_TEMP(AdjEdge)[mNbFaces*3];
	PxU32 NbEdges=0;

	// Feed me with triangles.....
	for(PxU32 i=0;i<mNbFaces;i++)
	{
		// Get correct vertex references
		const PxU32 Ref0 = create.DFaces ? create.DFaces[i*3+0] : create.WFaces ? create.WFaces[i*3+0] : 0;
		const PxU32 Ref1 = create.DFaces ? create.DFaces[i*3+1] : create.WFaces ? create.WFaces[i*3+1] : 1;
		const PxU32 Ref2 = create.DFaces ? create.DFaces[i*3+2] : create.WFaces ? create.WFaces[i*3+2] : 2;

		// Add a triangle to the database
		AddTriangle(Ref0, Ref1, Ref2, i, mFaces, NbEdges, Edges);
	}

	// At this point of the process we have mFaces & Edges filled with input data. That is:
	// - a list of triangles with 3 NULL links (i.e. PX_INVALID_U32)
	// - a list of mNbFaces*3 edges, each edge having 2 vertex references and an owner face.

	// Here NbEdges should be equal to mNbFaces*3.
	PX_ASSERT(NbEdges==mNbFaces*3);

#ifdef MSH_ADJACENCIES_INCLUDE_TOPOLOGY
	bool Status = CreateDatabase(mFaces, NbEdges, Edges);
#else
	bool Status = CreateDatabase(mFaces, NbEdges, Edges, create);
#endif

	// We don't need the edges anymore
	PX_DELETE_ARRAY(Edges);

#ifdef MSH_ADJACENCIES_INCLUDE_CONVEX_BITS
	// Now create convex information. This creates coupling between adjacencies & edge-list but in this case it's actually the goal:
	// mixing the two structures to save memory.
	if(Status && create.Verts)
	{
		Gu::EDGELISTCREATE ELC;
		ELC.NbFaces			= create.NbFaces;
		ELC.DFaces			= create.DFaces;	// That's where I like having a unified way to do things... We
		ELC.WFaces			= create.WFaces;	// can just directly copy the same pointers.
		ELC.FacesToEdges	= true;
		ELC.Verts			= create.Verts;
		ELC.Epsilon			= create.Epsilon;

		Gu::EdgeListBuilder EL;
		if(EL.init(ELC))
		{
			for(PxU32 i=0;i<mNbFaces;i++)
			{
				const Gu::EdgeTriangleData& ET = EL.getEdgeTriangle(i);
				if(Gu::EdgeTriangleAC::HasActiveEdge01(ET))	mFaces[i].mATri[EDGE01] |= 0x20000000;
				else										mFaces[i].mATri[EDGE01] &= ~0x20000000;
				if(Gu::EdgeTriangleAC::HasActiveEdge20(ET))	mFaces[i].mATri[EDGE02] |= 0x20000000;
				else										mFaces[i].mATri[EDGE02] &= ~0x20000000;
				if(Gu::EdgeTriangleAC::HasActiveEdge12(ET))	mFaces[i].mATri[EDGE12] |= 0x20000000;
				else										mFaces[i].mATri[EDGE12] &= ~0x20000000;

				PX_ASSERT((Gu::EdgeTriangleAC::HasActiveEdge01(ET) && mFaces[i].HasActiveEdge01()) || (!Gu::EdgeTriangleAC::HasActiveEdge01(ET) && !mFaces[i].HasActiveEdge01()));
				PX_ASSERT((Gu::EdgeTriangleAC::HasActiveEdge20(ET) && mFaces[i].HasActiveEdge20()) || (!Gu::EdgeTriangleAC::HasActiveEdge20(ET) && !mFaces[i].HasActiveEdge20()));
				PX_ASSERT((Gu::EdgeTriangleAC::HasActiveEdge12(ET) && mFaces[i].HasActiveEdge12()) || (!Gu::EdgeTriangleAC::HasActiveEdge12(ET) && !mFaces[i].HasActiveEdge12()));
			}
		}
	}
#endif

	return Status;
}
/*
bool AdjacenciesBuilder::Save(Stream& stream) const
{
	bool PlatformMismatch = PxPlatformMismatch();

	// Export header
	if(!WriteHeader('A', 'D', 'J', 'A', gVersion, PlatformMismatch, stream))
		return false;

	// Export adjacencies
//	stream.StoreDword(mNbFaces);
	WriteDword(mNbFaces, PlatformMismatch, stream);

//	stream.StoreBuffer(mFaces, sizeof(AdjTriangle)*mNbFaces);
	WriteDwordBuffer((const PxU32*)mFaces, mNbFaces*3, PlatformMismatch, stream);

	return true;
}*/
//#endif
