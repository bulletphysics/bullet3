// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
//
//
// Hull.inl
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
#pragma once

#include <assert.h>

inline short Hull::GetNumVertices() const
{
	return m_numVerts;
}

inline short Hull::GetNumFaces() const
{
	return m_numFaces;
}

inline short Hull::GetNumEdges() const
{
	return m_numEdges;
}

inline const Point3& Hull::GetVertex(short index) const
{
	return m_pVerts[index];
}

inline const Hull::Face& Hull::GetFace(short index) const
{
	return m_pFaces[index];
}

inline const Hull::Edge& Hull::GetEdge(short index) const
{
	return m_pEdges[index];
}

inline const Plane& Hull::GetPlane(short index) const
{
	return m_pPlanes[index];
} 

inline short Hull::GetFaceFirstEdge(short face) const
{
	assert(face >= 0 && face < m_numFaces);

	return m_pFaces[face].m_firstEdge;
}

inline short Hull::GetFaceNextEdge(short face, short prevEdge) const
{
	assert(face >= 0 && face < m_numFaces);
	assert(prevEdge >= 0 && prevEdge < m_numEdges);

	const Edge& e = m_pEdges[prevEdge];
	return e.m_nextEdge[face == e.m_faces[1]];
}

inline short Hull::GetEdgeVertex0(short face, short edge) const
{
	assert(face >= 0 && face < m_numFaces);
	assert(edge >= 0 && edge < m_numEdges);

	const Edge& e = m_pEdges[edge];
	return e.m_verts[face == e.m_faces[0]];
}

inline short Hull::GetEdgeVertex1(short face, short edge) const
{
	assert(face >= 0 && face < m_numFaces);
	assert(edge >= 0 && edge < m_numEdges);

	const Edge& e = m_pEdges[edge];
	return e.m_verts[face == e.m_faces[1]];
}

inline short Hull::GetEdgeOtherFace(short edge, short face) const
{
	assert(face >= 0 && face < m_numFaces);
	assert(edge >= 0 && edge < m_numEdges);

	const Edge& e = m_pEdges[edge];
	assert(e.m_faces[0] == face || e.m_faces[1] == face);

	return e.m_faces[face == e.m_faces[0]];
}
