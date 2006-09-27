// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
//	
//
// Hull.cpp
//
// Copyright (c) 2006 Simon Hobbs
//
// This software is provided 'as-is', without any express or implied warranty. In no event will the authors be held liable for any damages arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose, including commercial applications, and to alter it and redistribute it freely, subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source distribution.
#ifdef WIN32
#if _MSC_VER >= 1310

#include "hull.h"
#include "Geometry.h"
#include <assert.h>

#include "HullContactCollector.h"

Hull::Hull()
{
	m_numVerts = 0;
	m_numFaces = 0;
	m_numEdges = 0;

	m_pVerts = 0;
	m_pFaces = 0;
	m_pEdges = 0;
	m_pPlanes = 0;
}

Hull::~Hull()
{
	delete[] m_pVerts;
	delete[] m_pFaces;
	delete[] m_pEdges;
	delete[] m_pPlanes;
}

Point3 Hull::GetFaceCentroid(short face) const
{
	short edge;

	edge = GetFaceFirstEdge(face);
	Vector3 c = Vector3(GetVertex(GetEdgeVertex0(face, edge)));

	for (edge = GetFaceNextEdge(face, edge); edge >= 0; edge = GetFaceNextEdge(face, edge))
		c += Vector3(GetVertex(GetEdgeVertex0(face, edge)));

	c /= Scalar(GetFace(face).m_numEdges);

	return Point3(c);
}


// helper stuff for MakeHull
short Hull::s_firstFreeTmpFace = 0;
short Hull::s_firstUsedTmpFace = 0;
Hull::TmpFace* Hull::s_pTmpFaces = 0;

short Hull::s_firstFreeTmpEdge = 0;
short Hull::s_firstUsedTmpEdge = 0;
Hull::TmpEdge* Hull::s_pTmpEdges = 0;

const Point3* Hull::s_pPoints = 0;


short Hull::AllocTmpFace()
{
	short face = s_firstFreeTmpFace;
	assert(face != -1);

	TmpFace* pFace = GetTmpFace(face);
	s_firstFreeTmpFace = pFace->m_next;

	pFace->m_next = s_firstUsedTmpFace;
	s_firstUsedTmpFace = face;

	for (int i = 0; i < kTmpFaceMaxVerts; i++)
		pFace->m_verts[i] = pFace->m_edges[i] = -1;

	pFace->m_plane = Maths::Zero;

	return face;
}

void Hull::FreeTmpFace(short face)
{
	assert(face >= 0);

	TmpFace* pFace = GetTmpFace(face);

	if (face == s_firstUsedTmpFace)
		s_firstUsedTmpFace = pFace->m_next;
	else
	{
		TmpFace* pPrev;
		for (pPrev = GetTmpFace(s_firstUsedTmpFace); pPrev->m_next != face; pPrev = GetTmpFace(pPrev->m_next))
		{
		}
		pPrev->m_next = pFace->m_next;
	}
	
	pFace->m_next = s_firstFreeTmpFace;
	s_firstFreeTmpFace = face;
}

short Hull::AllocTmpEdge()
{
	short edge = s_firstFreeTmpEdge;
	assert(edge != -1);

	TmpEdge* pEdge = GetTmpEdge(edge);
	s_firstFreeTmpEdge = pEdge->m_next;

	pEdge->m_next = s_firstUsedTmpEdge;
	s_firstUsedTmpEdge = edge;

	pEdge->m_verts[0] = pEdge->m_verts[1] = -1;
	pEdge->m_faces[0] = pEdge->m_faces[1] = -1;

	return edge;
}

void Hull::FreeTmpEdge(short edge)
{
	assert(edge >= 0);

	TmpEdge* pEdge = GetTmpEdge(edge);

	if (edge == s_firstUsedTmpEdge)
		s_firstUsedTmpEdge = pEdge->m_next;
	else
	{
		TmpEdge* pPrev;
		for ( pPrev= GetTmpEdge(s_firstUsedTmpEdge); pPrev->m_next != edge; pPrev = GetTmpEdge(pPrev->m_next))
		{
		}
		pPrev->m_next = pEdge->m_next;
	}

	pEdge->m_next = s_firstFreeTmpEdge;
	s_firstFreeTmpEdge = edge;
}

short Hull::MatchOrAddEdge(short vert0, short vert1, short face)
{
	assert(vert0 >= 0);
	assert(vert1 >= 0);
	assert(vert0 != vert1);

	TmpEdge* pEdge;

	// see if edge already exists
	for (pEdge = GetTmpEdge(s_firstUsedTmpEdge); pEdge; pEdge = GetTmpEdge(pEdge->m_next))
	{
		if (pEdge->m_verts[0] == vert0 && pEdge->m_verts[1] == vert1)
		{
			assert(pEdge->m_faces[0] == -1);
			pEdge->m_faces[0] = face;
			return pEdge->m_index;
		}

		else if (pEdge->m_verts[0] == vert1 && pEdge->m_verts[1] == vert0)
		{
			assert(pEdge->m_faces[1] == -1);
			pEdge->m_faces[1] = face;
			return pEdge->m_index;
		}
	}

	// doesn't exist so add new face
	short edge = AllocTmpEdge();
	assert(edge >= 0);

	pEdge = GetTmpEdge(edge);

	pEdge->m_verts[0] = vert0;
	pEdge->m_verts[1] = vert1;
	pEdge->m_faces[0] = face;

	return edge;
}

void Hull::UnmatchOrRemoveEdge(short edge, short face)
{
	assert(edge >= 0);
	TmpEdge* pEdge = GetTmpEdge(edge);

	if (pEdge->m_faces[0] == face)
	{
		pEdge->m_faces[0] = -1;
	}
	else
	{
		assert(pEdge->m_faces[1] == face);
		pEdge->m_faces[1] = -1;
	}

	// if edge is now redundant then free it
	if (pEdge->m_faces[0] == -1 && pEdge->m_faces[1] == -1)
		FreeTmpEdge(edge);
}

short Hull::AddTmpFace(short vert0, short vert1, short vert2)
{
	short verts[3] = {vert0, vert1, vert2};
	return AddTmpFace(3, verts);
}

short Hull::AddTmpFace(short vert0, short numOtherVerts, short* pVerts)
{
	short verts[256];
	verts[0] = vert0;
	for (short i = 0; i < numOtherVerts; i++)
		verts[i + 1] = pVerts[i];
	return AddTmpFace(numOtherVerts + 1, verts);
}

short Hull::AddTmpFace(short numVerts, short* pVerts)
{
	assert(numVerts >= 3);
	assert(numVerts <= kTmpFaceMaxVerts);
	assert(pVerts);

	short face = AllocTmpFace();
	assert(face >= 0);
	
	TmpFace* pFace = GetTmpFace(face);

	pFace->m_numVerts = numVerts;

	for (short i = 0; i < numVerts; i++)
	{
		pFace->m_verts[i] = pVerts[i];
		pFace->m_edges[i] = MatchOrAddEdge(pVerts[i], pVerts[(i + 1) % numVerts], face);
	}

	pFace->m_plane = Plane(s_pPoints[pVerts[0]], s_pPoints[pVerts[1]], s_pPoints[pVerts[2]]);

	return face;
}

void Hull::RemoveTmpFace(short face)
{
	assert(face >= 0);

	TmpFace* pFace = GetTmpFace(face);

	for (short i = 0; i < pFace->m_numVerts; i++)
		UnmatchOrRemoveEdge(pFace->m_edges[i], face);

	FreeTmpFace(face);
}

bool Hull::TmpFaceAddPoint(short point, short face)
{
	assert(face >= 0);

	TmpFace* pFace = GetTmpFace(face);

	// vertex limit reached?
	if (pFace->m_numVerts == kTmpFaceMaxVerts)
		return false;

	// in same plane?
	if (Abs(Dot(pFace->m_plane, s_pPoints[point])) > 0.001f)
		return false;

	// remove last edge
	UnmatchOrRemoveEdge(pFace->m_edges[pFace->m_numVerts - 1], face);

	// add 2 new edges
	MatchOrAddEdge(pFace->m_verts[pFace->m_numVerts - 1], point, face);
	MatchOrAddEdge(point, pFace->m_verts[0], face);

	// add new vertex
	pFace->m_verts[pFace->m_numVerts++] = point;

	return true;
}

// The resulting hull will not contain interior points
// Note that this is a cheap and cheerful implementation that can only handle
// reasonably small point sets. However, except for the final hull it doesn't do any dynamic
// memory allocation - all the work happens on the stack.
Hull* Hull::MakeHull(int numPoints, const Point3* pPoints)
{
	assert(numPoints >= 4);
	assert(pPoints);

	// check first 4 points are disjoint
	// TODO: make it search points so it can cope better with this
	assert(Length(pPoints[1] - pPoints[0]) > 0.01f);
	assert(Length(pPoints[2] - pPoints[0]) > 0.01f);
	assert(Length(pPoints[3] - pPoints[0]) > 0.01f);
	assert(Length(pPoints[2] - pPoints[1]) > 0.01f);
	assert(Length(pPoints[3] - pPoints[1]) > 0.01f);
	assert(Length(pPoints[3] - pPoints[2]) > 0.01f);

	s_pPoints = pPoints;

	// put all temp faces on a free list
	TmpFace tmpFaces[kMaxFaces];

	s_firstFreeTmpFace = 0;
	s_firstUsedTmpFace = -1;
	s_pTmpFaces = tmpFaces;

	for (short i = 0; i < kMaxEdges; i++)
	{
		tmpFaces[i].m_index = i;
		tmpFaces[i].m_next = i + 1;
	}
	tmpFaces[kMaxFaces - 1].m_next = -1;

	// put all temp edges on a free list
	TmpEdge tmpEdges[kMaxEdges];

	s_firstFreeTmpEdge = 0;
	s_firstUsedTmpEdge = -1;
	s_pTmpEdges = tmpEdges;

	for (short i = 0; i < kMaxEdges; i++)
	{
		tmpEdges[i].m_index = i;
		tmpEdges[i].m_next = i + 1;
	}
	tmpEdges[kMaxEdges - 1].m_next = -1;

	// make initial tetrahedron
	Plane plane = Plane(pPoints[0], pPoints[1], pPoints[2]);

	Scalar dot = Dot(plane, pPoints[3]);
	assert(Abs(dot) > 0.01f);			// first 4 points are co-planar

	if (IsNegative(dot))
	{
		AddTmpFace(0, 1, 2);
		AddTmpFace(1, 0, 3);
		AddTmpFace(0, 2, 3);
		AddTmpFace(2, 1, 3);
	}
	else
	{
		AddTmpFace(2, 1, (short)0);
		AddTmpFace(1, 2, 3);
		AddTmpFace(2, 0, 3);
		AddTmpFace(0, 1, 3);
	}

	// merge all remaining points
	for (int i = 4; i < numPoints; i++)
		if (RemoveVisibleFaces(pPoints[i]) > 0)
			FillHole(i);

	return MakeHullFromTemp();
}

int Hull::RemoveVisibleFaces(const Point3& point)
{
	// test point for containment in current hull
	int numRemoved = 0;
	TmpFace* pNextFace;
	for (TmpFace* pFace = GetTmpFace(s_firstUsedTmpFace); pFace; pFace = pNextFace)
	{
		pNextFace = GetTmpFace(pFace->m_next);

		if (Dot(pFace->m_plane, point) > -0.001f)
		{
			RemoveTmpFace(pFace->m_index);
			numRemoved++;
		}
	}

	return numRemoved;
}

void Hull::FillHole(short newVertex)
{
	// gather unmatched edges (they form the silhouette of the hole)
	short edgeVert0[kMaxEdges];
	short edgeVert1[kMaxEdges];
	int numEdges = 0;

	for (TmpEdge* pEdge = GetTmpEdge(s_firstUsedTmpEdge); pEdge; pEdge = GetTmpEdge(pEdge->m_next))
	{
		if (pEdge->m_faces[0] == -1)
		{
			assert(numEdges < kMaxEdges);
			edgeVert0[numEdges] = pEdge->m_verts[0];
			edgeVert1[numEdges] = pEdge->m_verts[1];
			numEdges++;
		}
		else if (pEdge->m_faces[1] == -1)
		{
			assert(numEdges < kMaxEdges);
			edgeVert0[numEdges] = pEdge->m_verts[1];
			edgeVert1[numEdges] = pEdge->m_verts[0];
			numEdges++;
		}
	}

	// extract vertex winding by sorting edges
	short verts[kMaxEdges + 1];		// +1 for repeat of first vertex

	verts[0] = edgeVert0[0];
	verts[1] = edgeVert1[0];
	short numVerts = 2;

	while (numVerts < (numEdges + 1))
	{
		for (int i = 0; i < numEdges; i++)
		{
			// if leading vertex of edge matches our last vertex
			if (edgeVert0[i] == verts[numVerts - 1])
			{
				// then add trailing vertex of edge to our vertex list
				verts[numVerts++] = edgeVert1[i];
			}
		}
	}

	// fill the hole
	int i = 1;
	while (i < numVerts)
	{
		// make plane for first 3 vertices
		Plane plane = Plane(s_pPoints[newVertex], s_pPoints[verts[i - 1]], s_pPoints[verts[i]]);

		// any subequent vertices that lie in the same plane are also added
		int numFaceVerts = 2;
		for (int j = i + 1; j < numVerts; j++)
		{
			if (Abs(Dot(plane, s_pPoints[verts[j]])) > 0.001f)
				break;
			numFaceVerts++;
		}

		// add the polygon
		AddTmpFace(newVertex, numFaceVerts, verts + i - 1);

		// skip to next
		i += numFaceVerts - 1;
	}
}

// make hull from temporary working structures
Hull* Hull::MakeHullFromTemp()
{
	Hull* pHull = new Hull();

	// count and index faces
	short index = 0;

	for (TmpFace* pFace = GetTmpFace(s_firstUsedTmpFace); pFace; pFace = GetTmpFace(pFace->m_next))
	{
		pFace->m_index = index++;
		pHull->m_numFaces++;
	}

	// count and index edges, also count and index used vertices
	short vertIndexes[kMaxVerts];
	for (int i = 0; i < kMaxVerts; i++)
		vertIndexes[i] = -1;

	index = 0;
	for (TmpEdge* pEdge = GetTmpEdge(s_firstUsedTmpEdge); pEdge; pEdge = GetTmpEdge(pEdge->m_next))
	{
		pEdge->m_index = index++;
		pHull->m_numEdges++;

		if (vertIndexes[pEdge->m_verts[0]] == -1)
			vertIndexes[pEdge->m_verts[0]] = pHull->m_numVerts++;
		if (vertIndexes[pEdge->m_verts[1]] == -1)
			vertIndexes[pEdge->m_verts[1]] = pHull->m_numVerts++;
	}

	// allocate hull arrays
	pHull->m_pVerts = new Point3[pHull->m_numVerts];
	pHull->m_pEdges = new Hull::Edge[pHull->m_numEdges];
	pHull->m_pFaces = new Hull::Face[pHull->m_numFaces];
	pHull->m_pPlanes = new Plane[pHull->m_numFaces];

	// fill in hull arrays
	// verts:
	for (int i = 0; i < kMaxVerts; i++)
		if (vertIndexes[i] != -1)
			pHull->m_pVerts[vertIndexes[i]] = s_pPoints[i];

	// edges:
	index = 0;
	for (TmpEdge* pEdge = GetTmpEdge(s_firstUsedTmpEdge); pEdge; pEdge = GetTmpEdge(pEdge->m_next))
	{
		pHull->m_pEdges[index].m_faces[0] = GetTmpFace(pEdge->m_faces[0])->m_index;
		pHull->m_pEdges[index].m_faces[1] = GetTmpFace(pEdge->m_faces[1])->m_index;
		pHull->m_pEdges[index].m_verts[0] = vertIndexes[pEdge->m_verts[0]];
		pHull->m_pEdges[index].m_verts[1] = vertIndexes[pEdge->m_verts[1]];

		index++;
	}

	// faces:
	index = 0;
	for (TmpFace* pFace = GetTmpFace(s_firstUsedTmpFace); pFace; pFace = GetTmpFace(pFace->m_next))
	{
		// set edge count
		pHull->m_pFaces[index].m_numEdges = pFace->m_numVerts;

		// make linked list of face edges
		short prevEdge = GetTmpEdge(pFace->m_edges[0])->m_index;
		pHull->m_pFaces[index].m_firstEdge = prevEdge;

		for (int i = 1; i < pFace->m_numVerts; i++)
		{
			Edge& e = pHull->m_pEdges[prevEdge];
			prevEdge = GetTmpEdge(pFace->m_edges[i])->m_index;
			e.m_nextEdge[pFace->m_index == e.m_faces[1]] = prevEdge;
		}

		Edge& e = pHull->m_pEdges[prevEdge];
		e.m_nextEdge[pFace->m_index == e.m_faces[1]] = -1;

		// set plane
		pHull->m_pPlanes[index] = pFace->m_plane;

		index++;
	}


	return pHull;
}



void Hull::ProcessHullHull(btSeparation& sep,const Hull& shapeA,const Hull& shapeB,const Transform& trA,const Transform& trB,HullContactCollector* collector)
{
	Point3 vertsA[Hull::kMaxVerts];
	Point3 vertsB[Hull::kMaxVerts];

//	const Hull& shapeA((const Hull&)*sep.m_pBodyA->GetShape());
//	const Hull& shapeB((const Hull&)*sep.m_pBodyB->GetShape());

//	Transform trA(sep.m_pBodyA->GetTransform());
//	Transform trB(sep.m_pBodyB->GetTransform());

	// transform verts of A to world space
	Point3* pVertsA = vertsA;
	for (short v = 0; v < shapeA.m_numVerts; v++)
		pVertsA[v] = shapeA.m_pVerts[v] * trA;

	// transform verts of B to world space
	Point3* pVertsB = vertsB;
	for (short v = 0; v < shapeB.m_numVerts; v++)
		pVertsB[v] = shapeB.m_pVerts[v] * trB;

#ifdef SHAPE_COLLIDER_USE_CACHING
	// update cached pair
	if (sep.m_separator != btSeparation::kFeatureNone)
	{
		// re-use the separation
		if (UpdateSeparationHullHull(sep, pVertsA, pVertsB, trA, trB) == true)
			return;

		// also re-use penetration if only slight
		if (sep.m_dist > -0.02f)
		{
			// except if no contacts were generated, in which case we continue through to full test
			if (AddContactsHullHull(sep, pVertsA, pVertsB, trA, trB,shapeA,shapeB) > 0)
				return;
		}
	}
#endif

	if (GetSeparationHullHull(sep, pVertsA, pVertsB, trA, trB,shapeA,shapeB) == false)
	{
		AddContactsHullHull(sep, pVertsA, pVertsB, trA, trB,shapeA,shapeB,collector);
	}
}


void Hull::ComputeInertia(const Transform& transform, Point3& centerOfMass, Matrix33& inertia, float totalMass) const
{
	assert(totalMass > 0.0f);

	// order: 1, x, y, z, x^2, y^2, z^2, xy, yz, zx
	float integral[10] = {0,0,0,0,0,0,0,0,0,0};

	// for each triangle
	for (short face = 0; face < m_numFaces; face++)
	{
		short edge = GetFaceFirstEdge(face);
		Point3 v0 = m_pVerts[ GetEdgeVertex0(face, edge) ] * transform;

		edge = GetFaceNextEdge(face, edge);
		Point3 v1 = m_pVerts[ GetEdgeVertex0(face, edge) ] * transform;

		for (edge = GetFaceNextEdge(face, edge); edge != -1; edge = GetFaceNextEdge(face, edge))
		{
			Point3 v2 = m_pVerts[ GetEdgeVertex0(face, edge) ] * transform;

			// get cross product of triangle edges
			Vector3 d = Cross(v2 - v0, v1 - v0);

			// compute integral terms
			Vector3 w0 = Vector3(v0);
			Vector3 w1 = Vector3(v1);
			Vector3 w2 = Vector3(v2);

			Vector3 temp0 = w0 + w1;
			Vector3 f1 = temp0 + w2;
			Vector3 temp1 = w0 * w0;
			Vector3 temp2 = temp1 + w1 * temp0;
			Vector3 f2 = temp2 + w2 * f1;
			Vector3 f3 = w0 * temp1 + w1 * temp2 + w2 * f2;
			Vector3 g0 = f2 + w0 * (f1 + w0);
			Vector3 g1 = f2 + w1 * (f1 + w1);
			Vector3 g2 = f2 + w2 * (f1 + w2);

			// update integrals
			integral[0] += d[0] * f1[0];
			integral[1] += d[0] * f2[0];
			integral[2] += d[1] * f2[1];
			integral[3] += d[2] * f2[2];
			integral[4] += d[0] * f3[0];
			integral[5] += d[1] * f3[1];
			integral[6] += d[2] * f3[2];
			integral[7] += d[0] * (v0[1] * g0[0] + v1[1] * g1[0] + v2[1] * g2[0]);
			integral[8] += d[1] * (v0[2] * g0[1] + v1[2] * g1[1] + v2[2] * g2[1]);
			integral[9] += d[2] * (v0[0] * g0[2] + v1[0] * g1[2] + v2[0] * g2[2]);

			// next edge
			v1 = v2;
		}
	}

	integral[0] *= 1.0f / 6.0f;
	integral[1] *= 1.0f / 24.0f;
	integral[2] *= 1.0f / 24.0f;
	integral[3] *= 1.0f / 24.0f;
	integral[4] *= 1.0f / 60.0f;
	integral[5] *= 1.0f / 60.0f;
	integral[6] *= 1.0f / 60.0f;
	integral[7] *= 1.0f / 120.0f;
	integral[8] *= 1.0f / 120.0f;
	integral[9] *= 1.0f / 120.0f;

	// scale all integrals to get desired total mass
	assert(integral[0] > 0.0f);

	float invMassRatio = totalMass / integral[0];
	for (int i = 0; i < 10; i++)
		integral[i] *= invMassRatio;

	// center of mass
	centerOfMass = Point3(integral[1] / totalMass, integral[2] / totalMass, integral[3] / totalMass);

	// inertia relative to world
	inertia[0][0] = integral[5] + integral[6];
	inertia[0][1] = -integral[7];
	inertia[0][2] = -integral[9];

	inertia[1][0] = -integral[7];
	inertia[1][1] = integral[4] + integral[6];
	inertia[1][2] = -integral[8];

	inertia[2][0] = -integral[9];
	inertia[2][1] = -integral[8];
	inertia[2][2] = integral[5] + integral[5];

	// inertia relative to center of mass
	inertia[0][0] -= totalMass * (centerOfMass[1] * centerOfMass[1] + centerOfMass[2] * centerOfMass[2]);
	inertia[0][1] += totalMass * centerOfMass[0] * centerOfMass[1];
	inertia[0][2] += totalMass * centerOfMass[2] * centerOfMass[0];

	inertia[1][0] += totalMass * centerOfMass[0] * centerOfMass[1];
	inertia[1][1] -= totalMass * (centerOfMass[2] * centerOfMass[2] + centerOfMass[0] * centerOfMass[0]);
	inertia[1][2] += totalMass * centerOfMass[1] * centerOfMass[2];

	inertia[2][0] += totalMass * centerOfMass[2] * centerOfMass[0];
	inertia[2][1] += totalMass * centerOfMass[1] * centerOfMass[2];
	inertia[2][2] -= totalMass * (centerOfMass[0] * centerOfMass[0] + centerOfMass[1] * centerOfMass[1]);
}

Bounds3 Hull::ComputeBounds(const Transform& transform) const
{
	Bounds3 b(m_pVerts[0] * transform);
	for (int i = 1; i < m_numVerts; i++)
		b += m_pVerts[i] * transform;
	return b;
}







// Clips a face to the back of a plane
int Hull::ClipFace(int numVerts, Point3** ppVtxIn, Point3** ppVtxOut, const Plane& plane)
{
	int ve, numVertsOut;
	Point3 *pVtxOut, *pVtxS, *pVtxE;
	Scalar ds, de;

	if (numVerts == 0)
		return 0;

	pVtxE = *ppVtxIn;
	pVtxS = pVtxE + numVerts - 1;
	pVtxOut = *ppVtxOut;

	Scalar zero(0.0f);

	ds = Dot(plane, *pVtxS);

	for (ve = 0; ve < numVerts; ve++, pVtxE++)
	{
		de = Dot(plane, *pVtxE);

		if (ds <= zero)
		{
			*pVtxOut++ = *pVtxS;
			if (de > zero)
				*pVtxOut++ = Lerp(*pVtxS, *pVtxE, ds * RcpNr(ds - de));
		}
		else if (de <= zero)
			*pVtxOut++ = Lerp(*pVtxS, *pVtxE, ds * RcpNr(ds - de));

		if (ve == 0)
			pVtxS = *ppVtxIn;
		else
			pVtxS++;

		ds = de;
	}

	numVertsOut = pVtxOut - *ppVtxOut;

	// swap in and out arrays ready for next time
	pVtxOut = *ppVtxIn;
	*ppVtxIn = *ppVtxOut;
	*ppVtxOut = pVtxOut;

	return numVertsOut;
}





int Hull::AddContactsHullHull(btSeparation& sep, const Point3* pVertsA, const Point3* pVertsB,
									   const Transform& trA, const Transform& trB,const Hull& hullA,const Hull& hullB,
									   HullContactCollector* hullContactCollector)
{
	const int maxContacts = hullContactCollector->GetMaxNumContacts();

	Vector3 normalWorld = sep.m_axis;

	// edge->edge contact is always a single point
	if (sep.m_separator == btSeparation::kFeatureBoth)
	{
		const Hull::Edge& edgeA = hullA.GetEdge(sep.m_featureA);
		const Hull::Edge& edgeB = hullB.GetEdge(sep.m_featureB);

		float ta, tb;
		Line la(pVertsA[edgeA.m_verts[0]], pVertsA[edgeA.m_verts[1]]);
		Line lb(pVertsB[edgeB.m_verts[0]], pVertsB[edgeB.m_verts[1]]);

		Intersect(la, lb, ta, tb);
	
#ifdef VALIDATE_CONTACT_POINTS
		AssertPointInsideHull(contact.m_points[0].m_pos, trA, hullA);
		AssertPointInsideHull(contact.m_points[0].m_pos, trB, hullB);
#endif

		
		Point3 posWorld = Lerp(la.m_start, la.m_end, ta);
		float depth = -sep.m_dist;
        Vector3 tangent = Normalize(pVertsA[edgeA.m_verts[1]] - pVertsA[edgeA.m_verts[0]]);

		sep.m_contact = hullContactCollector->BatchAddContactGroup(sep,1,normalWorld,tangent,&posWorld,&depth);
	
	}
	// face->face contact is polygon
	else
	{
		short faceA = sep.m_featureA;
		short faceB = sep.m_featureB;

		Vector3 tangent;

		// find face of hull A that is most opposite contact axis
		// TODO: avoid having to transform planes here
		if (sep.m_separator == btSeparation::kFeatureB)
		{
			const Hull::Edge& edgeB = hullB.GetEdge(hullB.GetFaceFirstEdge(faceB));
			tangent = Normalize(pVertsB[edgeB.m_verts[1]] - pVertsB[edgeB.m_verts[0]]);

			Scalar dmin = Scalar::Consts::MaxValue;
			for (short face = 0; face < hullA.m_numFaces; face++)
			{
				Vector3 normal = hullA.GetPlane(face).GetNormal() * trA;
				Scalar d = Dot(normal, sep.m_axis);
				if (d < dmin)
				{
					dmin = d;
					faceA = face;
				}
			}
		}
		else
		{
			const Hull::Edge& edgeA = hullA.GetEdge(hullA.GetFaceFirstEdge(faceA));
			tangent = Normalize(pVertsA[edgeA.m_verts[1]] - pVertsA[edgeA.m_verts[0]]);

			Scalar dmin = Scalar::Consts::MaxValue;
			for (short face = 0; face < hullB.m_numFaces; face++)
			{
				Vector3 normal = hullB.GetPlane(face).GetNormal() * trB;
				Scalar d = Dot(normal, -sep.m_axis);
				if (d < dmin)
				{
					dmin = d;
					faceB = face;
				}
			}
		}

		Point3 workspace[2][Hull::kMaxVerts];

		// setup initial clip face (minimizing face from hull B)
		int numContacts = 0;
		for (short edge = hullB.GetFaceFirstEdge(faceB); edge != -1; edge = hullB.GetFaceNextEdge(faceB, edge))
			workspace[0][numContacts++] = pVertsB[ hullB.GetEdgeVertex0(faceB, edge) ];

		// clip polygon to back of planes of all faces of hull A that are adjacent to witness face
		Point3* pVtxIn = workspace[0];
		Point3* pVtxOut = workspace[1];

#if 0
		for (short edge = hullA.GetFaceFirstEdge(faceA); edge != -1; edge = hullA.GetFaceNextEdge(faceA, edge))
		{
			Plane planeA = hullA.GetPlane( hullA.GetEdgeOtherFace(edge, faceA) ) * trA;
			numContacts = ClipFace(numContacts, &pVtxIn, &pVtxOut, planeA);
		}
#else
		for (short f = 0; f < hullA.GetNumFaces(); f++)
		{
			Plane planeA = hullA.GetPlane(f) * trA;
			numContacts = ClipFace(numContacts, &pVtxIn, &pVtxOut, planeA);
		}
#endif

		// only keep points that are behind the witness face
		Plane planeA = hullA.GetPlane(faceA) * trA;

		float depths[Hull::kMaxVerts];
		int numPoints = 0;
		for (int i = 0; i < numContacts; i++)
		{
			Scalar d = Dot(planeA, pVtxIn[i]);
			if (IsNegative(d))
			{
				depths[numPoints] = (float)-d;
				pVtxIn[numPoints] = pVtxIn[i];

#ifdef VALIDATE_CONTACT_POINTS
					AssertPointInsideHull(pVtxIn[numPoints], trA, hullA);
					AssertPointInsideHull(pVtxIn[numPoints], trB, hullB);
#endif
				numPoints++;
			}
		}

		//we can also use a persistentManifold/reducer class
		// keep maxContacts points at most
		if (numPoints > 0)
		{
			if (numPoints > maxContacts)
			{
				int step = (numPoints << 8) / maxContacts;

				numPoints = maxContacts;
				for (int i = 0; i < numPoints; i++)
				{
					int nth = (step * i) >> 8;

					depths[i] = depths[nth];
					pVtxIn[i] = pVtxIn[nth];
					

#ifdef VALIDATE_CONTACT_POINTS
					AssertPointInsideHull(contact.m_points[i].m_pos, trA, hullA);
					AssertPointInsideHull(contact.m_points[i].m_pos, trB, hullB);
#endif
				}
			}
			
			sep.m_contact = hullContactCollector->BatchAddContactGroup(sep,numPoints,normalWorld,tangent,pVtxIn,depths);

		}
		return numPoints;
	}

	// shut up compiler
	return 0;
}




// returns true if a separating axis was found
// if no separating axis was found then details of least penetrating axis are returned
// resulting axis always points away from hullB
// either transform can be null in which case it is treated as identity (this avoids a bunch of work)
bool Hull::GetSeparationHullHull(btSeparation& sep, const Point3* pVertsA, const Point3* pVertsB,
										  const Transform& trA, const Transform& trB,
										  const Hull& hullA,
										  const Hull& hullB
										  )
{
	//const Hull& hullA((const Hull&)*sep.m_pShapeA->GetShape());
	//const Hull& hullB((const Hull&)*sep.m_pShapeB->GetShape());

	sep.m_separator = btSeparation::kFeatureNone;
	sep.m_dist = MinValueF;
	sep.m_featureA = sep.m_featureB = -1;
	sep.m_contact = -1;

	// test verts of A to planes of B
	Scalar minDistB = Scalar::Consts::MinValue;
	for (short p = 0; p < hullB.m_numFaces; p++)
	{
		Plane planeWorld = hullB.m_pPlanes[p] * trB;

		Scalar minDist = Dot(planeWorld, pVertsA[0]);
		for (short v = 1; v < hullA.m_numVerts; v++)
			minDist = Min(minDist, Dot(planeWorld, pVertsA[v]));

		// keep min overlap
		if (minDist > minDistB)
		{
			minDistB = minDist;
			sep.m_featureB = p;
			sep.m_dist = minDist;
			sep.m_axis = planeWorld.GetNormal();
			sep.m_separator = btSeparation::kFeatureB;
		}

		// got a separating plane?
		if (!IsNegative(minDist))
			return true;
	}

	// test verts of B to planes of A
	Scalar minDistA = Scalar::Consts::MinValue;
	for (short p = 0; p < hullA.m_numFaces; p++)
	{
		// get plane in world space
		Plane planeWorld = hullA.m_pPlanes[p] * trA;

		// get min dist
		Scalar minDist = Dot(planeWorld, pVertsB[0]);
		for (short v = 1; v < hullB.m_numVerts; v++)
			minDist = Min(minDist, Dot(planeWorld, pVertsB[v]));

		// keep min overlap
		if (minDist > minDistA)
		{
			minDistA = minDist;
			sep.m_featureA = p;

			if ((float)minDist > sep.m_dist)
			{
				sep.m_dist = (float)minDist;
				sep.m_axis = -planeWorld.GetNormal();
				sep.m_separator = btSeparation::kFeatureA;
			}
		}

		// got a separating plane?
		if (!IsNegative(minDist))
			return true;
	}

	// test edge pairs on two minimizing faces
	short faceA( sep.m_featureA );
	short faceB( sep.m_featureB );

	Vector3 faceBnormal = Vector3(hullB.m_pPlanes[sep.m_featureB]) * trB;

//	Plane bestPlane;
//	Point3 bestPlanePos;

	for (short ea = hullA.GetFaceFirstEdge(faceA); ea != -1; ea = hullA.GetFaceNextEdge(faceA, ea))
	{
		const Hull::Edge& edgeA = hullA.GetEdge(ea);

		for (short eb = hullB.GetFaceFirstEdge(faceB); eb != -1; eb = hullB.GetFaceNextEdge(faceB, eb))
		{
			const Hull::Edge& edgeB = hullB.GetEdge(eb);

			Vector3 va = pVertsA[edgeA.m_verts[1]] - pVertsA[edgeA.m_verts[0]];
			Vector3 vb = pVertsB[edgeB.m_verts[1]] - pVertsB[edgeB.m_verts[0]];
			Vector3 axis = Cross(va, vb);
			Scalar rcpLen = LengthRcp(axis);

			// if the two edges have nearly equal or opposite directions then try the next pair
			if (IsNan(rcpLen) || rcpLen > 10000.0f)
				continue;
			axis *= rcpLen;

			// if axis is very close to current best axis then don't use it
			if (Abs(Dot(sep.m_axis, axis)) > 0.99f)
				continue;

			// ensure the axis points away from hullB
			if (Dot(faceBnormal, axis) < Scalar::Consts::Zero)
				axis = -axis;

			Plane plane(axis, pVertsB[edgeB.m_verts[0]]);

			// hull B must be entirely behind plane
			Scalar dmaxB = Dot(plane, pVertsB[0]);	
			for (short v = 1; v < hullB.m_numVerts; v++)
				dmaxB = Max(dmaxB, Dot(plane, pVertsB[v]));

			if (dmaxB > 0.001f)
				continue;

			// get hull A distance from plane
			Scalar dminA = Dot(plane, pVertsA[0]);	
			for (short v = 1; v < hullA.m_numVerts; v++)
				dminA = Min(dminA, Dot(plane, pVertsA[v]));

			// got a separating plane?
			if (!IsNegative(dminA))
			{
				sep.m_featureA = ea;
				sep.m_featureB = eb;
				sep.m_dist = dminA;
				sep.m_axis = axis;
				sep.m_separator = btSeparation::kFeatureBoth;
				return true;
			}

			// keep min overlap
			if (dminA > sep.m_dist)
			{
				// if edge of A is in front of the plane, then we haven't identified the correct edge pair
				if (IsNegative(Dot(plane, pVertsA[edgeA.m_verts[0]])))
				{
	//				bestPlane = plane;
	//				bestPlanePos = Lerp(pVertsB[edgeB.m_verts[0]], pVertsB[edgeB.m_verts[1]], 0.5f);

//					sep.m_featureA = ea;
//					sep.m_featureB = eb;
					sep.m_dist = dminA;
					sep.m_axis = axis;
//					sep.m_separator = btSeparation::kFeatureBoth;
				}
			}

		}

//			RenderPlane(bestPlane, bestPlanePos);
	}

	return false;
}
#endif //MSC_VER >= 1310
#endif //WIN32
