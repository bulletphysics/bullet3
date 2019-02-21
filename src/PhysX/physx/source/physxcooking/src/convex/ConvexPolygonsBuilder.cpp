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


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "foundation/PxMemory.h"
#include "EdgeList.h"
#include "Adjacencies.h"
#include "MeshCleaner.h"
#include "CmRadixSortBuffered.h"
#include "CookingUtils.h"
#include "PsArray.h"
#include "PsFoundation.h"

#include "ConvexPolygonsBuilder.h"


using namespace physx;

#define USE_PRECOMPUTED_HULL_PROJECTION

static PX_INLINE void Flip(HullTriangleData& data)
{
	PxU32 tmp = data.mRef[2];
	data.mRef[2] = data.mRef[1];
	data.mRef[1] = tmp;
}

//////////////////////////////////////////////////////////////////////////
//! A generic couple structure
class Pair : public Ps::UserAllocated
{
public:
	PX_FORCE_INLINE	Pair()										{}
	PX_FORCE_INLINE	Pair(PxU32 i0, PxU32 i1) : id0(i0), id1(i1)	{}
	PX_FORCE_INLINE	~Pair()										{}

	//! Operator for "if(Pair==Pair)"
	PX_FORCE_INLINE	bool			operator==(const Pair& p)	const	{ return (id0==p.id0) && (id1==p.id1);	}
	//! Operator for "if(Pair!=Pair)"
	PX_FORCE_INLINE	bool			operator!=(const Pair& p)	const	{ return (id0!=p.id0) || (id1!=p.id1);	}

	PxU32	id0;	//!< First index of the pair
	PxU32	id1;	//!< Second index of the pair
};
PX_COMPILE_TIME_ASSERT(sizeof(Pair)==8);

//////////////////////////////////////////////////////////////////////////
// construct a plane 
template <class T>
PX_INLINE PxPlane PlaneEquation(const T& t, const PxVec3* verts)
{
	const PxVec3& p0 = verts[t.v[0]];
	const PxVec3& p1 = verts[t.v[1]];
	const PxVec3& p2 = verts[t.v[2]];
	return PxPlane(p0, p1, p2);
}

//////////////////////////////////////////////////////////////////////////
// negate plane
static PX_FORCE_INLINE void negatePlane(Gu::HullPolygonData& data)
{
	data.mPlane.n = -data.mPlane.n;
	data.mPlane.d = -data.mPlane.d;
}

//////////////////////////////////////////////////////////////////////////
// Inverse a buffer in-place
static bool inverseBuffer(PxU32 nbEntries, PxU8* entries)
{
	if(!nbEntries || !entries)	return false;

	for(PxU32 i=0; i < (nbEntries>>1); i++)
		Ps::swap(entries[i], entries[nbEntries-1-i]);

	return true;
}

//////////////////////////////////////////////////////////////////////////
// Extracts a line-strip from a list of non-sorted line-segments (slow)
static bool findLineStrip(Ps::Array<PxU32>& lineStrip, const Ps::Array<Pair>& lineSegments)
{
	// Ex:
	//
	// 4-2
	// 0-1
	// 2-3
	// 4-0
	// 7-3
	// 7-1
	//
	// => 0-1-7-3-2-4-0

	// 0-0-1-1-2-2-3-3-4-4-7-7

	// 0-1
	// 0-4
	// 1-7
	// 2-3
	// 2-4
	// 3-7

	// Naive implementation below

	Ps::Array<Pair> Copy(lineSegments);

RunAgain:
	{		
		PxU32 nbSegments = Copy.size();
		for(PxU32 j=0;j<nbSegments;j++)
		{
			PxU32 ID0 = Copy[j].id0;
			PxU32 ID1 = Copy[j].id1;

			for(PxU32 i=j+1;i<nbSegments;i++)
			{
				if(
					(Copy[i].id0==ID0 && Copy[i].id1==ID1)
					||	(Copy[i].id1==ID0 && Copy[i].id0==ID1)
					)
				{
					// Duplicate segment found => remove both
					PX_ASSERT(Copy.size()>=2);
					Copy.remove(i);
					Copy.remove(j);
					goto RunAgain;	
				}
			}
		}
		// Goes through when everything's fine
	}

	PxU32 ref0 = 0xffffffff;
	PxU32 ref1 = 0xffffffff;
	if(Copy.size()>=1)
	{
		Pair* Segments = Copy.begin();
		if(Segments)
		{
			ref0 = Segments->id0;
			ref1 = Segments->id1;
			lineStrip.pushBack(ref0);
			lineStrip.pushBack(ref1);
			PX_ASSERT(Copy.size()>=1);
			Copy.remove(0);
		}
	}

Wrap:
	// Look for same vertex ref in remaining segments
	PxU32 nb = Copy.size();
	if(!nb)
	{
		// ### check the line is actually closed?
		return true;
	}

	for(PxU32 i=0;i<nb;i++)
	{
		PxU32 newRef0 = Copy[i].id0;
		PxU32 newRef1 = Copy[i].id1;

		// We look for Ref1 only
		if(newRef0==ref1)
		{
			// r0 - r1
			// r1 - x
			lineStrip.pushBack(newRef1);	// Output the other reference
			ref0 = newRef0;
			ref1 = newRef1;
			Copy.remove(i);			
			goto Wrap;
		}
		else if(newRef1==ref1)
		{
			// r0 - r1
			// x - r1	=> r1 - x
			lineStrip.pushBack(newRef0);	// Output the other reference
			ref0 = newRef1;
			ref1 = newRef0;
			Copy.remove(i);		
			goto Wrap;
		}
	}
	return false;
}

//////////////////////////////////////////////////////////////////////////
// Test for duplicate triangles
PX_COMPILE_TIME_ASSERT(sizeof(Gu::TriangleT<PxU32>)==sizeof(PxVec3));	// ...
static bool TestDuplicateTriangles(PxU32& nbFaces, Gu::TriangleT<PxU32>* faces, bool repair)
{
	if(!nbFaces || !faces)
		return true;

	Gu::TriangleT<PxU32>* indices32 = reinterpret_cast<Gu::TriangleT<PxU32>*>(PxAlloca(nbFaces*sizeof(Gu::TriangleT<PxU32>)));
	for(PxU32 i=0;i<nbFaces;i++)
	{
		indices32[i].v[0] = faces[i].v[0];
		indices32[i].v[1] = faces[i].v[1];
		indices32[i].v[2] = faces[i].v[2];
	}

	// Radix-sort power...
	ReducedVertexCloud	reducer(reinterpret_cast<PxVec3*>(indices32), nbFaces);
	REDUCEDCLOUD rc;
	reducer.Reduce(&rc);
	if(rc.NbRVerts<nbFaces)
	{
		if(repair)
		{
			nbFaces = rc.NbRVerts;
			for(PxU32 i=0;i<nbFaces;i++)
			{
				const Gu::TriangleT<PxU32>* curTri = reinterpret_cast<const Gu::TriangleT<PxU32>*>(&rc.RVerts[i]);
				faces[i].v[0] = curTri->v[0];
				faces[i].v[1] = curTri->v[1];
				faces[i].v[2] = curTri->v[2];
			}
		}
		return false;	// Test failed
	}
	return true;	// Test succeeded
}

//////////////////////////////////////////////////////////////////////////
// plane culling test
static PX_FORCE_INLINE bool testCulling(const Gu::TriangleT<PxU32>& triangle, const PxVec3* verts, const PxVec3& center)
{
	const PxPlane plane(verts[triangle.v[0]], verts[triangle.v[1]], verts[triangle.v[2]]);
	return plane.distance(center)>0.0f;
}

//////////////////////////////////////////////////////////////////////////
// face normals test
static bool TestUnifiedNormals(PxU32 nbVerts, const PxVec3* verts, PxU32 nbFaces, Gu::TriangleT<PxU32>* faces, bool repair)
{
	if(!nbVerts || !verts || !nbFaces || !faces)
		return false;

	// Unify normals so that all hull faces are well oriented

	// Compute geometric center - we need a vertex inside the hull
	const float coeff = 1.0f / float(nbVerts);
	PxVec3 geomCenter(0.0f, 0.0f, 0.0f);
	for(PxU32 i=0;i<nbVerts;i++)
	{
		geomCenter.x += verts[i].x * coeff;
		geomCenter.y += verts[i].y * coeff;
		geomCenter.z += verts[i].z * coeff;
	}

	// We know the hull is (hopefully) convex so we can easily test whether a point is inside the hull or not.
	// The previous geometric center must be invisible from any hull face: that's our test to decide whether a normal
	// must be flipped or not.
	bool status = true;
	for(PxU32 i=0;i<nbFaces;i++)
	{
		// Test face visibility from the geometric center (supposed to be inside the hull).
		// All faces must be invisible from this point to ensure a strict CCW order.
		if(testCulling(faces[i], verts, geomCenter))
		{
			if(repair)	faces[i].flip();
			status = false;
		}
	}

	return status;
}

//////////////////////////////////////////////////////////////////////////
// clean the mesh
static bool CleanFaces(PxU32& nbFaces, Gu::TriangleT<PxU32>* faces, PxU32& nbVerts, PxVec3* verts)
{
	// Brute force mesh cleaning.
	// PT: I added this back on Feb-18-05 because it fixes bugs with hulls from QHull.	
	MeshCleaner cleaner(nbVerts, verts, nbFaces, faces->v, 0.0f);
	if (!cleaner.mNbTris)
		return false;

	nbVerts = cleaner.mNbVerts;
	nbFaces = cleaner.mNbTris;

	PxMemCopy(verts, cleaner.mVerts, cleaner.mNbVerts*sizeof(PxVec3));

	for (PxU32 i = 0; i < cleaner.mNbTris; i++)
	{
		faces[i].v[0] = cleaner.mIndices[i * 3 + 0];
		faces[i].v[1] = cleaner.mIndices[i * 3 + 1];
		faces[i].v[2] = cleaner.mIndices[i * 3 + 2];
	}

	// Get rid of duplicates
	TestDuplicateTriangles(nbFaces, faces, true);

	// Unify normals
	TestUnifiedNormals(nbVerts, verts, nbFaces, faces, true);

	// Remove zero-area triangles
	//	TestZeroAreaTriangles(nbFaces, faces, verts, true);

	// Unify normals again
	TestUnifiedNormals(nbVerts, verts, nbFaces, faces, true);

	// Get rid of duplicates again
	TestDuplicateTriangles(nbFaces, faces, true);

	return true;
}

//////////////////////////////////////////////////////////////////////////
// check the newly constructed faces
static bool CheckFaces(PxU32 nbFaces, const Gu::TriangleT<PxU32>* faces, PxU32 nbVerts, const PxVec3* verts)
{
	// Remove const since we use functions that can do both testing & repairing. But we won't change the data.
	Gu::TriangleT<PxU32>* f = const_cast<Gu::TriangleT<PxU32>*>(faces);

	// Test duplicate faces
	if(!TestDuplicateTriangles(nbFaces, f, false))	
		return false;

	// Test unified normals
	if(!TestUnifiedNormals(nbVerts, verts, nbFaces, f, false))	
		return false;

	return true;
}

//////////////////////////////////////////////////////////////////////////
// compute the newell plane from the face verts
static bool computeNewellPlane(PxPlane& plane, PxU32 nbVerts, const PxU8* indices, const PxVec3* verts)
{
	if(!nbVerts || !indices || !verts)
		return false;

	PxVec3 centroid(0,0,0), normal(0,0,0);
	for(PxU32 i=nbVerts-1, j=0; j<nbVerts; i=j, j++)
	{
		normal.x += (verts[indices[i]].y - verts[indices[j]].y) * (verts[indices[i]].z + verts[indices[j]].z);
		normal.y += (verts[indices[i]].z - verts[indices[j]].z) * (verts[indices[i]].x + verts[indices[j]].x);
		normal.z += (verts[indices[i]].x - verts[indices[j]].x) * (verts[indices[i]].y + verts[indices[j]].y);
		centroid += verts[indices[j]];
	}
	plane.n = normal;
	plane.n.normalize();
	plane.d = -(centroid.dot(plane.n))/float(nbVerts);

	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
*	Analyses a redundant vertices and splits the polygons if necessary.
*	\relates	ConvexHull
*	\fn			extractHullPolygons(Container& polygon_data, const ConvexHull& hull)
*	\param		nb_polygons		[out] number of extracted polygons
*	\param		polygon_data	[out] polygon data: (Nb indices, index 0, index 1... index N)(Nb indices, index 0, index 1... index N)(...)
*	\param		hull			[in] convex hull
*	\param      redundantVertices [out] redundant vertices found inside the polygons - we want to remove them because of PCM
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void checkRedundantVertices(PxU32& nb_polygons, Ps::Array<PxU32>& polygon_data, const ConvexPolygonsBuilder& hull, Ps::Array<PxU32>& triangle_data, Ps::Array<PxU32>& redundantVertices)
{
	const PxU32* dFaces	= reinterpret_cast<const PxU32*>(hull.getFaces());
	bool needToSplitPolygons = false;

	bool* polygonMarkers = reinterpret_cast<bool*>(PxAlloca(nb_polygons*sizeof(bool)));
	PxMemZero(polygonMarkers, nb_polygons*sizeof(bool));

	bool* redundancyMarkers = reinterpret_cast<bool*>(PxAlloca(redundantVertices.size()*sizeof(bool)));
	PxMemZero(redundancyMarkers, redundantVertices.size()*sizeof(bool));

	// parse through the redundant vertices and if we cannot remove them split just the actual polygon if possible
	Ps::Array<PxU32> polygonsContainer;
	PxU32 numEntries = 0;
	for (PxU32 i = redundantVertices.size(); i--;)
	{
		numEntries = 0;
		polygonsContainer.clear();
		// go through polygons, if polygons does have only 3 verts we cannot remove any vertex from it, try to decompose the second one
		PxU32* Data = polygon_data.begin();		
		for(PxU32 t=0;t<nb_polygons;t++)
		{			
			PxU32 nbVerts = *Data++;
			PX_ASSERT(nbVerts>=3);			// Else something very wrong happened...

			for(PxU32 j=0;j<nbVerts;j++)
			{
				if(redundantVertices[i] == Data[j])
				{
					polygonsContainer.pushBack(t);
					polygonsContainer.pushBack(nbVerts);
					numEntries++;
					break;
				}
			}
			Data += nbVerts;
		}

		bool needToSplit = false;
		for (PxU32 j = 0; j < numEntries; j++)
		{
			PxU32 numInternalVertices = polygonsContainer[j*2 + 1];
			if(numInternalVertices == 3)
			{
				needToSplit = true;				
			}
		}

		// now lets mark the polygons for split
		if(needToSplit)
		{
			// mark the redundant vertex, it is solved by spliting, dont report it
			needToSplitPolygons = true;
			redundancyMarkers[i] = true;
			for (PxU32 j = 0; j < numEntries; j++)
			{
				PxU32 polygonNumber = polygonsContainer[j*2];
				PxU32 numInternalPolygons = polygonsContainer[j*2 + 1];
				if(numInternalPolygons != 3)
				{
					polygonMarkers[polygonNumber] = true;					
				}
			}
		}
	}

	if(needToSplitPolygons)
	{
		// parse from the end so we can remove it and not change the order
		for (PxU32 i = redundantVertices.size(); i--;)
		{
			// remove it
			if(redundancyMarkers[i])
			{
				redundantVertices.remove(i);
			}
		}

		Ps::Array<PxU32> newPolygon_data;
		Ps::Array<PxU32> newTriangle_data;
		PxU32 newNb_polygons = 0;

		PxU32* data = polygon_data.begin();		
		PxU32* triData = triangle_data.begin();		
		for(PxU32 i=0;i<nb_polygons;i++)
		{			
			PxU32 nbVerts = *data++;
			PxU32 nbTris = *triData++;
			if(polygonMarkers[i])
			{
				// split the polygon into triangles
				for(PxU32 k=0;k< nbTris; k++)
				{
					newNb_polygons++;
					const PxU32 faceIndex = triData[k];
					newPolygon_data.pushBack(PxU32(3));
					newPolygon_data.pushBack(dFaces[3*faceIndex]);
					newPolygon_data.pushBack(dFaces[3*faceIndex + 1]);
					newPolygon_data.pushBack(dFaces[3*faceIndex + 2]);
					newTriangle_data.pushBack(PxU32(1));
					newTriangle_data.pushBack(faceIndex);
				}
			}
			else
			{	
				newNb_polygons++;
				// copy the original polygon
				newPolygon_data.pushBack(nbVerts);
				for(PxU32 j=0;j<nbVerts;j++)				
					newPolygon_data.pushBack(data[j]);

				// copy the original polygon triangles
				newTriangle_data.pushBack(nbTris);
				for(PxU32 k=0;k< nbTris; k++)
				{
					newTriangle_data.pushBack(triData[k]);
				}
			}
			data += nbVerts;
			triData += nbTris;
		}

		// now put the data to output
		polygon_data.clear();
		triangle_data.clear();

		// the copy does copy even the data
		polygon_data = newPolygon_data;
		triangle_data = newTriangle_data;
		nb_polygons = newNb_polygons;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
*	Analyses a convex hull made of triangles and extracts polygon data out of it.
*	\relates	ConvexHull
*	\fn			extractHullPolygons(Ps::Array<PxU32>& polygon_data, const ConvexHull& hull)
*	\param		nb_polygons		[out] number of extracted polygons
*	\param		polygon_data	[out] polygon data: (Nb indices, index 0, index 1... index N)(Nb indices, index 0, index 1... index N)(...)
*	\param		hull			[in] convex hull
*	\param		triangle_data	[out] triangle data
*	\param      rendundantVertices [out] redundant vertices found inside the polygons - we want to remove them because of PCM
*	\return		true if success
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static bool extractHullPolygons(PxU32& nb_polygons, Ps::Array<PxU32>& polygon_data, const ConvexPolygonsBuilder& hull, Ps::Array<PxU32>* triangle_data, Ps::Array<PxU32>& rendundantVertices)
{
	PxU32 nbFaces	= hull.getNbFaces();
	const PxVec3* hullVerts	= hull.mHullDataHullVertices;
	const PxU32 nbVertices = hull.mHull->mNbHullVertices;

	const PxU16* wFaces	= NULL;
	const PxU32* dFaces	= reinterpret_cast<const PxU32*>(hull.getFaces());
	PX_ASSERT(wFaces || dFaces);

	ADJACENCIESCREATE create;
	create.NbFaces	= nbFaces;
	create.DFaces	= dFaces;
	create.WFaces	= wFaces;
	create.Verts	= hullVerts;
	//Create.Epsilon	= 0.01f;	// PT: trying to fix Rob Elam bug. Also fixes TTP 2467
	//	Create.Epsilon	= 0.001f;	// PT: for "Bruno's bug"
	create.Epsilon	= 0.005f;	// PT: middle-ground seems to fix both. Expose this param?


	AdjacenciesBuilder adj;
	if(!adj.Init(create))	return false;

	PxU32 nbBoundaryEdges = adj.ComputeNbBoundaryEdges();
	if(nbBoundaryEdges)	return false;	// A valid hull shouldn't have open edges!!

	bool* markers = reinterpret_cast<bool*>(PxAlloca(nbFaces*sizeof(bool)));
	PxMemZero(markers, nbFaces*sizeof(bool));

	PxU8* vertexMarkers = reinterpret_cast<PxU8*>(PxAlloca(nbVertices*sizeof(PxU8)));
	PxMemZero(vertexMarkers, nbVertices*sizeof(PxU8));

	PxU32 currentFace = 0;	// Start with first triangle
	nb_polygons = 0;
	do
	{
		currentFace = 0;
		while(currentFace<nbFaces && markers[currentFace])	currentFace++;

		// Start from "closest" face and floodfill through inactive edges
		struct Local
		{
			static void FloodFill(Ps::Array<PxU32>& indices, const AdjTriangle* faces, PxU32 current, bool* inMarkers)
			{
				if(inMarkers[current])	return;
				inMarkers[current] = true;

				indices.pushBack(current);
				const AdjTriangle& AT = faces[current];

				// We can floodfill through inactive edges since the mesh is convex (inactive==planar)
				if(!AT.HasActiveEdge01())	FloodFill(indices, faces, AT.GetAdjTri(EDGE01), inMarkers);
				if(!AT.HasActiveEdge20())	FloodFill(indices, faces, AT.GetAdjTri(EDGE02), inMarkers);
				if(!AT.HasActiveEdge12())	FloodFill(indices, faces, AT.GetAdjTri(EDGE12), inMarkers);
			}

			static bool GetNeighborFace(PxU32 index,PxU32 triangleIndex,const AdjTriangle* faces, const PxU32* dfaces, PxU32& neighbor, PxU32& current)
			{			
				PxU32 currentIndex = index;
				PxU32 previousIndex = index;
				bool firstFace = true;
				bool next = true;
				while (next)
				{
					const AdjTriangle& currentAT = faces[currentIndex];
					PxU32 refTr0 = dfaces[currentIndex*3 + 0];
					PxU32 refTr1 = dfaces[currentIndex*3 + 1];

					PxU32 edge[2];
					edge[0] = 1;
					edge[1] = 2;
					if(triangleIndex == refTr0)
					{
						edge[0] = 0;
						edge[1] = 1;
					}
					else
					{
						if(triangleIndex == refTr1)
						{
							edge[0] = 0;
							edge[1] = 2;
						}
					}

					if(currentAT.HasActiveEdge(edge[0]) && currentAT.HasActiveEdge(edge[1]))
					{
						return false;					
					}

					if(!currentAT.HasActiveEdge(edge[0]) && !currentAT.HasActiveEdge(edge[1]))
					{
						// not interested in testing transition vertices 
						if(currentIndex == index)
						{
							return false;
						}

						// transition one
						for (PxU32 i = 0; i < 2; i++)
						{
							PxU32 testIndex = currentAT.GetAdjTri(SharedEdgeIndex(edge[i]));

							// exit if we circle around the vertex back to beginning
							if(testIndex == index && previousIndex != index)
							{
								return false;
							}

							if(testIndex != previousIndex)
							{
								// move to next 
								previousIndex = currentIndex;
								currentIndex = testIndex;
								break;
							}							
						}
					}
					else
					{
						if(!currentAT.HasActiveEdge(edge[0]))
						{
							PxU32 t = edge[0];
							edge[0] = edge[1];
							edge[1] = t;
						}

						if(currentAT.HasActiveEdge(edge[0]))
						{
							PxU32 testIndex = currentAT.GetAdjTri(SharedEdgeIndex(edge[0]));
							if(firstFace)
							{
								firstFace = false;
							}
							else
							{
								neighbor = testIndex;
								current = currentIndex;
								return true;									
							}
						}

						if(!currentAT.HasActiveEdge(edge[1]))
						{
							PxU32 testIndex = currentAT.GetAdjTri(SharedEdgeIndex(edge[1]));
							if(testIndex != index)
							{
								previousIndex = currentIndex;
								currentIndex = testIndex;
							}
						}
					}

				}

				return false;
			}

			static bool CheckFloodFillFace(PxU32 index,const AdjTriangle* faces, const PxU32* dfaces)
			{
				if(!dfaces)
					return true;

				const AdjTriangle& checkedAT = faces[index];

				PxU32 refTr0 = dfaces[index*3 + 0];
				PxU32 refTr1 = dfaces[index*3 + 1];
				PxU32 refTr2 = dfaces[index*3 + 2];

				for (PxU32 i = 0; i < 3; i++)
				{
					if(!checkedAT.HasActiveEdge(i))
					{
						PxU32 testTr0 = refTr1;
						PxU32 testTr1 = refTr2;
						PxU32 testIndex0 = 0;
						PxU32 testIndex1 = 1;
						if(i == 0)
						{
							testTr0 = refTr0;
							testTr1 = refTr1;
							testIndex0 = 1;
							testIndex1 = 2;
						}
						else
						{
							if(i == 1)
							{
								testTr0 = refTr0;
								testTr1 = refTr2;
								testIndex0 = 0;
								testIndex1 = 2;
							}
						}

						PxU32 adjFaceTested = checkedAT.GetAdjTri(SharedEdgeIndex(testIndex0));

						PxU32 neighborIndex00;
						PxU32 neighborIndex01;
						bool found0 = GetNeighborFace(index,testTr0,faces,dfaces, neighborIndex00, neighborIndex01);
						PxU32 neighborIndex10;
						PxU32 neighborIndex11;
						bool found1 = GetNeighborFace(adjFaceTested,testTr0,faces,dfaces, neighborIndex10, neighborIndex11);

						if(found0 && found1 && neighborIndex00 == neighborIndex11 && neighborIndex01 == neighborIndex10)
						{
							return false;
						}

						adjFaceTested = checkedAT.GetAdjTri(SharedEdgeIndex(testIndex1));
						found0 = GetNeighborFace(index,testTr1,faces,dfaces,neighborIndex00,neighborIndex01);
						found1 = GetNeighborFace(adjFaceTested,testTr1,faces,dfaces,neighborIndex10,neighborIndex11);

						if(found0 && found1 && neighborIndex00 == neighborIndex11 && neighborIndex01 == neighborIndex10)
						{
							return false;
						}

					}
				}

				return true;
			}

			static bool CheckFloodFill(Ps::Array<PxU32>& indices,AdjTriangle* faces,bool* inMarkers, const PxU32* dfaces)
			{
				bool valid = true;

				for(PxU32 i=0;i<indices.size();i++)
				{
					//const AdjTriangle& AT = faces[indices.GetEntry(i)];

					for(PxU32 j= i + 1;j<indices.size();j++)
					{						
						const AdjTriangle& testAT = faces[indices[j]];

						if(testAT.GetAdjTri(EDGE01) == indices[i])
						{
							if(testAT.HasActiveEdge01())
							{								
								valid = false;
							}
						}
						if(testAT.GetAdjTri(EDGE02) == indices[i])
						{
							if(testAT.HasActiveEdge20())
							{							
								valid = false;
							}
						}
						if(testAT.GetAdjTri(EDGE12) == indices[i])
						{
							if(testAT.HasActiveEdge12())
							{							
								valid  = false;
							}
						}

						if(!valid)
							break;				
					}

					if(!CheckFloodFillFace(indices[i], faces, dfaces))
					{
						valid = false;
					}

					if(!valid)
						break;
				}

				if(!valid)
				{
					for(PxU32 i=0;i<indices.size();i++)
					{
						AdjTriangle& AT = faces[indices[i]];
						AT.mATri[0] |= 0x20000000;
						AT.mATri[1] |= 0x20000000;
						AT.mATri[2] |= 0x20000000;

						inMarkers[indices[i]] = false;
					}					

					indices.forceSize_Unsafe(0);

					return true;
				}

				return false;
			}
		};

		if(currentFace!=nbFaces)
		{
			Ps::Array<PxU32> indices;	// Indices of triangles forming hull polygon

			bool doFill = true;
			while (doFill)
			{
				Local::FloodFill(indices, adj.mFaces, currentFace, markers);

				doFill = Local::CheckFloodFill(indices,adj.mFaces,markers, dFaces);
			}			

			// Now it would be nice to recreate a closed linestrip, similar to silhouette extraction. The line is composed of active edges, this time.


			Ps::Array<Pair> activeSegments;
			//Container ActiveSegments;
			// Loop through triangles composing the polygon
			for(PxU32 i=0;i<indices.size();i++)
			{
				const PxU32 currentTriIndex = indices[i];	// Catch current triangle
				const PxU32 vRef0 = dFaces ? dFaces[currentTriIndex*3+0] : wFaces[currentTriIndex*3+0];
				const PxU32 vRef1 = dFaces ? dFaces[currentTriIndex*3+1] : wFaces[currentTriIndex*3+1];
				const PxU32 vRef2 = dFaces ? dFaces[currentTriIndex*3+2] : wFaces[currentTriIndex*3+2];

				// Keep active edges
				if(adj.mFaces[currentTriIndex].HasActiveEdge01())	{ activeSegments.pushBack(Pair(vRef0,vRef1));	}
				if(adj.mFaces[currentTriIndex].HasActiveEdge20())	{ activeSegments.pushBack(Pair(vRef0,vRef2));	}
				if(adj.mFaces[currentTriIndex].HasActiveEdge12())	{ activeSegments.pushBack(Pair(vRef1,vRef2));	}
			}

			// We assume the polygon is convex. In that case it should always be possible to retriangulate it so that the triangles are
			// implicit (in particular, it should always be possible to remove interior triangles)

			Ps::Array<PxU32> lineStrip;
			if(findLineStrip(lineStrip, activeSegments))
			{
				PxU32 nb = lineStrip.size();
				if(nb)
				{
					const PxU32* entries = lineStrip.begin();
					PX_ASSERT(entries[0] == entries[nb-1]);	// findLineStrip() is designed that way. Might not be what we want!

					// We get rid of the last (duplicated) index
					polygon_data.pushBack(nb-1);
					for (PxU32 i = 0; i < nb-1; i++)
					{
						vertexMarkers[entries[i]]++;
						polygon_data.pushBack(entries[i]);
					}					
					nb_polygons++;

					// Loop through vertices composing the line strip polygon end mark the redundant vertices inside the polygon 
					for(PxU32 i=0;i<indices.size();i++)
					{
						const PxU32 CurrentTriIndex = indices[i];	// Catch current triangle
						const PxU32 VRef0 = dFaces ? dFaces[CurrentTriIndex*3+0] : wFaces[CurrentTriIndex*3+0];
						const PxU32 VRef1 = dFaces ? dFaces[CurrentTriIndex*3+1] : wFaces[CurrentTriIndex*3+1];
						const PxU32 VRef2 = dFaces ? dFaces[CurrentTriIndex*3+2] : wFaces[CurrentTriIndex*3+2];

						bool found0 = false;
						bool found1 = false;
						bool found2 = false;

						for (PxU32 j=0;j < nb - 1; j++)
						{
							if(VRef0 == entries[j])
							{
								found0 = true;								
							}

							if(VRef1 == entries[j])
							{
								found1 = true;								
							}

							if(VRef2 == entries[j])
							{
								found2 = true;								
							}

							if(found0 && found1 && found2)
								break;
						}

						if(!found0)
						{
							if(rendundantVertices.find(VRef0) == rendundantVertices.end())
								rendundantVertices.pushBack(VRef0);
						}

						if(!found1)
						{
							if(rendundantVertices.find(VRef1) == rendundantVertices.end())
								rendundantVertices.pushBack(VRef1);

						}

						if(!found2)
						{
							if(rendundantVertices.find(VRef2) == rendundantVertices.end())
								rendundantVertices.pushBack(VRef2);
						}
					}					

					// If needed, output triangle indices used to build this polygon
					if(triangle_data)
					{
						triangle_data->pushBack(indices.size());
						for (PxU32 j = 0; j < indices.size(); j++)
							triangle_data->pushBack(indices[j]);
					}
				}
			}
			else
			{
				Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Meshmerizer::extractHullPolygons: line strip extraction failed");				
				return false;
			}
		}
	}
	while(currentFace!=nbFaces);

	for (PxU32 i = 0; i < nbVertices; i++)
	{
		if(vertexMarkers[i] < 3)
		{
			if(rendundantVertices.find(i) == rendundantVertices.end())
				rendundantVertices.pushBack(i);
		}
	}

	if(rendundantVertices.size() > 0 && triangle_data)
		checkRedundantVertices(nb_polygons,polygon_data,hull,*triangle_data,rendundantVertices);

	return true;
}

//////////////////////////////////////////////////////////////////////////

ConvexPolygonsBuilder::ConvexPolygonsBuilder(Gu::ConvexHullData* hull, const bool buildGRBData)
	: ConvexHullBuilder(hull, buildGRBData), mNbHullFaces(0), mFaces(NULL)
{
}

//////////////////////////////////////////////////////////////////////////

ConvexPolygonsBuilder::~ConvexPolygonsBuilder()
{
	PX_DELETE_POD(mFaces);
}

//////////////////////////////////////////////////////////////////////////
// compute hull polygons from given hull triangles
bool ConvexPolygonsBuilder::computeHullPolygons(const PxU32& nbVerts,const PxVec3* verts, const PxU32& nbTriangles, const PxU32* triangles)
{
	PX_ASSERT(triangles);
	PX_ASSERT(verts);

	mHullDataHullVertices			= NULL;
	mHullDataPolygons				= NULL;
	mHullDataVertexData8			= NULL;
	mHullDataFacesByEdges8			= NULL;
	mHullDataFacesByVertices8		= NULL;

	mNbHullFaces					= nbTriangles;
	mHull->mNbHullVertices			= Ps::to8(nbVerts);
	// allocate additional vec3 for V4 safe load in VolumeInteration
	mHullDataHullVertices			= reinterpret_cast<PxVec3*>(PX_ALLOC(sizeof(PxVec3) * mHull->mNbHullVertices + 1, "PxVec3"));
	PxMemCopy(mHullDataHullVertices, verts, mHull->mNbHullVertices*sizeof(PxVec3));

	mFaces = PX_NEW(HullTriangleData)[mNbHullFaces];
	for(PxU32 i=0;i<mNbHullFaces;i++)
	{
		PX_ASSERT(triangles[i*3+0]<=0xffff);
		PX_ASSERT(triangles[i*3+1]<=0xffff);
		PX_ASSERT(triangles[i*3+2]<=0xffff);
		mFaces[i].mRef[0] = triangles[i*3+0];
		mFaces[i].mRef[1] = triangles[i*3+1];
		mFaces[i].mRef[2] = triangles[i*3+2];
	}

	Gu::TriangleT<PxU32>* hullAsIndexedTriangle = reinterpret_cast<Gu::TriangleT<PxU32>*>(mFaces);

	// We don't trust the user at all... So, clean the hull.
	PxU32 nbHullVerts = mHull->mNbHullVertices;
	CleanFaces(mNbHullFaces, hullAsIndexedTriangle, nbHullVerts, mHullDataHullVertices);
	PX_ASSERT(nbHullVerts<256);
	mHull->mNbHullVertices = Ps::to8(nbHullVerts);

	// ...and then run the full tests again.
	if(!CheckFaces(mNbHullFaces, hullAsIndexedTriangle, mHull->mNbHullVertices, mHullDataHullVertices))	
		return false;	

	// Transform triangles-to-polygons
	if(!createPolygonData())	
		return false;	

	return checkHullPolygons();	
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
*	Computes polygon data.
*	\return		true if success
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool ConvexPolygonsBuilder::createPolygonData()
{
	// Cleanup
	mHull->mNbPolygons = 0;
	PX_DELETE_POD(mHullDataVertexData8);
	PX_DELETE_POD(mHullDataFacesByVertices8);
	PX_FREE_AND_RESET(mHullDataPolygons);

	// Extract polygon data from triangle data
	Ps::Array<PxU32> temp;
	Ps::Array<PxU32> temp2;
	Ps::Array<PxU32> rendundantVertices;
	PxU32 nbPolygons;
	if(!extractHullPolygons(nbPolygons, temp, *this, &temp2,rendundantVertices))
		return false;

	PxVec3*	 reducedHullDataHullVertices = mHullDataHullVertices;
	PxU8 numReducedHullDataVertices = mHull->mNbHullVertices;

	if(rendundantVertices.size() > 0)
	{
		numReducedHullDataVertices = Ps::to8(mHull->mNbHullVertices - rendundantVertices.size());
		reducedHullDataHullVertices = static_cast<PxVec3*> (PX_ALLOC_TEMP(sizeof(PxVec3)*numReducedHullDataVertices,"Reduced vertices hull data"));
		PxU8* remapTable = PX_NEW(PxU8)[mHull->mNbHullVertices];

		PxU8 currentIndex = 0;
		for (PxU8 i = 0; i < mHull->mNbHullVertices; i++)
		{
			if(rendundantVertices.find(i) == rendundantVertices.end())
			{
				PX_ASSERT(currentIndex < numReducedHullDataVertices);
				reducedHullDataHullVertices[currentIndex] = mHullDataHullVertices[i];
				remapTable[i] = currentIndex;
				currentIndex++;
			}
			else
			{
				remapTable[i] = 0xFF;
			}
		}

		PxU32* data = temp.begin();
		for(PxU32 i=0;i<nbPolygons;i++)
		{			
			PxU32 nbVerts = *data++;
			PX_ASSERT(nbVerts>=3);			// Else something very wrong happened...

			for(PxU32 j=0;j<nbVerts;j++)
			{
				PX_ASSERT(data[j] < mHull->mNbHullVertices);
				data[j] = remapTable[data[j]];
			}

			data += nbVerts;
		}

		PX_DELETE_POD(remapTable);
	}

	if(nbPolygons>255)
	{
		Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "ConvexHullBuilder: convex hull has more than 255 polygons!");
		return false;
	}

	// Precompute hull polygon structures
	mHull->mNbPolygons = Ps::to8(nbPolygons);
	mHullDataPolygons = reinterpret_cast<Gu::HullPolygonData*>(PX_ALLOC(sizeof(Gu::HullPolygonData)*mHull->mNbPolygons, "Gu::HullPolygonData"));
	PxMemZero(mHullDataPolygons, sizeof(Gu::HullPolygonData)*mHull->mNbPolygons);

	// The winding hasn't been preserved so we need to handle this. Basically we need to "unify normals"
	// exactly as we did at hull creation time - except this time we work on polygons
	PxVec3 geomCenter;
	computeGeomCenter(geomCenter, mNbHullFaces, mFaces);

	// Loop through polygons
	// We have N polygons => remove N entries for number of vertices
	PxU32 tmp = temp.size() - nbPolygons;
	mHullDataVertexData8 = PX_NEW(PxU8)[tmp];
	PxU8* dest = mHullDataVertexData8;
	const PxU32* data = temp.begin();
	const PxU32* triData = temp2.begin();
	for(PxU32 i=0;i<nbPolygons;i++)
	{
		mHullDataPolygons[i].mVRef8 = PxU16(dest - mHullDataVertexData8);	// Setup link for current polygon
		PxU32 nbVerts = *data++;
		PX_ASSERT(nbVerts>=3);			// Else something very wrong happened...
		mHullDataPolygons[i].mNbVerts = Ps::to8(nbVerts);

		PxU32 index = 0;
		for(PxU32 j=0;j<nbVerts;j++)
		{
			if(data[j] != 0xFF)
			{
				dest[index] = Ps::to8(data[j]);
				index++;
			}
			else
			{
				mHullDataPolygons[i].mNbVerts--;
			}
		}

		// Compute plane equation
		{
			computeNewellPlane(mHullDataPolygons[i].mPlane, mHullDataPolygons[i].mNbVerts, dest, reducedHullDataHullVertices);

			PxU32 nbTris = *triData++;		// #tris in current poly
			bool flip = false;
			for(PxU32 k=0;k< nbTris; k++)
			{
				PxU32 triIndex = *triData++;	// Index of one triangle composing polygon
				PX_ASSERT(triIndex<mNbHullFaces);
				const Gu::TriangleT<PxU32>& T = reinterpret_cast<const Gu::TriangleT<PxU32>&>(mFaces[triIndex]);
				const PxPlane PL = PlaneEquation(T, mHullDataHullVertices);
				if(k==0 && PL.n.dot(mHullDataPolygons[i].mPlane.n) < 0.0f) 
				{
					flip = true;
				}
			}
			if(flip)
			{
				negatePlane(mHullDataPolygons[i]);
				inverseBuffer(mHullDataPolygons[i].mNbVerts, dest);
			}

			for(PxU32 j=0;j<mHull->mNbHullVertices;j++)
			{
				float d = - (mHullDataPolygons[i].mPlane.n).dot(mHullDataHullVertices[j]);
				if(d<mHullDataPolygons[i].mPlane.d)	mHullDataPolygons[i].mPlane.d=d;
			}
		}

		// "Unify normal"
		if(mHullDataPolygons[i].mPlane.distance(geomCenter)>0.0f)
		{
			inverseBuffer(mHullDataPolygons[i].mNbVerts, dest);

			negatePlane(mHullDataPolygons[i]);
			PX_ASSERT(mHullDataPolygons[i].mPlane.distance(geomCenter)<=0.0f);
		}

		// Next one
		data += nbVerts;			// Skip vertex indices
		dest += mHullDataPolygons[i].mNbVerts;
	}

	if(reducedHullDataHullVertices != mHullDataHullVertices)
	{
		PxMemCopy(mHullDataHullVertices,reducedHullDataHullVertices,sizeof(PxVec3)*numReducedHullDataVertices);
		PX_FREE(reducedHullDataHullVertices);

		mHull->mNbHullVertices = numReducedHullDataVertices;
	}

	//calculate the vertex map table
	if(!calculateVertexMapTable(nbPolygons))
		return false;

#ifdef USE_PRECOMPUTED_HULL_PROJECTION
	// Loop through polygons
	for(PxU32 j=0;j<nbPolygons;j++)
	{
		// Precompute hull projection along local polygon normal
		PxU32 nbVerts = mHull->mNbHullVertices;
		const PxVec3* verts = mHullDataHullVertices;
		Gu::HullPolygonData& polygon = mHullDataPolygons[j];
		PxReal min = PX_MAX_F32;
		PxU8 minIndex = 0xff;
		for (PxU8 i = 0; i < nbVerts; i++)
		{
			float dp = (*verts++).dot(polygon.mPlane.n);
			if(dp < min)	
			{ 
				min = dp; 
				minIndex = i; 
			} 
		}
		polygon.mMinIndex = minIndex;
	}
#endif

	// Triangulate newly created polygons to recreate a clean vertex cloud.
	return createTrianglesFromPolygons();
}

//////////////////////////////////////////////////////////////////////////
// create back triangles from polygons
bool ConvexPolygonsBuilder::createTrianglesFromPolygons()
{
	if (!mHull->mNbPolygons || !mHullDataPolygons)	return false;

	PxU32 maxNbTriangles = 0;
	for (PxU32 i = 0; i < mHull->mNbPolygons; i++)
	{
		if (mHullDataPolygons[i].mNbVerts < 3)
		{
			Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, "ConvexHullBuilder::CreateTrianglesFromPolygons: convex hull has a polygon with less than 3 vertices!");
			return false;
		}
		maxNbTriangles += mHullDataPolygons[i].mNbVerts - 2;
	}

	HullTriangleData* tmpFaces = PX_NEW(HullTriangleData)[maxNbTriangles];

	HullTriangleData* currFace = tmpFaces;
	PxU32 nbTriangles = 0;
	const PxU8* vertexData = mHullDataVertexData8;
	const PxVec3* hullVerts = mHullDataHullVertices;
	for (PxU32 i = 0; i < mHull->mNbPolygons; i++)
	{
		const PxU8* data = vertexData + mHullDataPolygons[i].mVRef8;
		PxU32 nbVerts = mHullDataPolygons[i].mNbVerts;

		// Triangulate the polygon such that all all generated triangles have one and the same vertex
		// in common.
		//
		// Make sure to avoid creating zero area triangles. Imagine the following polygon:
		//
		// 4                  3
		// *------------------*
		// |                  |
		// *---*----*----*----*
		// 5   6    0    1    2
		//
		// Choosing vertex 0 as the shared vertex, the following zero area triangles will be created:
		// [0 1 2], [0 5 6]
		//
		// Check for these triangles and discard them
		// Note: Such polygons should only occur if the user defines the convex hull, i.e., the triangles
		//       of the convex shape, himself. If the convex hull is built from the vertices only, the
		//       hull algorithm removes the useless vertices.
		//
		for (PxU32 j = 0; j < nbVerts - 2; j++)
		{
			currFace->mRef[0] = data[0];
			currFace->mRef[1] = data[(j + 1) % nbVerts];
			currFace->mRef[2] = data[(j + 2) % nbVerts];

			const PxVec3& p0 = hullVerts[currFace->mRef[0]];
			const PxVec3& p1 = hullVerts[currFace->mRef[1]];
			const PxVec3& p2 = hullVerts[currFace->mRef[2]];

			const float area = ((p1 - p0).cross(p2 - p0)).magnitudeSquared();

			if (area != 0.0f)	// Else discard the triangle
			{
				nbTriangles++;
				currFace++;
			}
		}
	}

	PX_DELETE_POD(mFaces);
	HullTriangleData* faces;
	PX_ASSERT(nbTriangles <= maxNbTriangles);
	if (maxNbTriangles == nbTriangles)
	{
		// No zero area triangles, hence the face buffer has correct size and can be used directly.
		faces = tmpFaces;
	}
	else
	{
		// Resize face buffer because some triangles were discarded.
		faces = PX_NEW(HullTriangleData)[nbTriangles];
		if (!faces)
		{
			PX_DELETE_POD(tmpFaces);
			return false;
		}
		PxMemCopy(faces, tmpFaces, sizeof(HullTriangleData)*nbTriangles);
		PX_DELETE_POD(tmpFaces);
	}
	mFaces = faces;
	mNbHullFaces = nbTriangles;
	// TODO: at this point useless vertices should be removed from the hull. The current fix is to initialize
	// support vertices to known valid vertices, but it's not really convincing.

	// Re-unify normals
	PxVec3 geomCenter;
	computeGeomCenter(geomCenter, mNbHullFaces, mFaces);

	for (PxU32 i = 0; i < mNbHullFaces; i++)
	{
		const PxPlane P(hullVerts[mFaces[i].mRef[0]],
			hullVerts[mFaces[i].mRef[1]],
			hullVerts[mFaces[i].mRef[2]]);
		if (P.distance(geomCenter) > 0.0f)
		{
			Flip(mFaces[i]);
		}
	}
	return true;
}

