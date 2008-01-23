/*
btShapeHull implemented by John McCutchan.

Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef _SHAPE_HULL_H
#define _SHAPE_HULL_H

#include "LinearMath/btAlignedObjectArray.h"
#include "btConvexShape.h"


///btShapeHull takes a btConvexShape, builds the convex hull using btConvexHull and provides triangle indices and vertices.
class btShapeHull
{
public:
	btShapeHull (const btConvexShape* shape);
	~btShapeHull ();

	bool buildHull (btScalar margin);

	int numTriangles () const;
	int numVertices () const;
	int numIndices () const;

	const btVector3* getVertexPointer() const
	{
		return &m_vertices[0];
	}
	const unsigned int* getIndexPointer() const
	{
		return m_indices;
	}

protected:
	btAlignedObjectArray<btVector3> m_vertices;
	unsigned int* m_indices;
	unsigned int m_numIndices;
	const btConvexShape* m_shape;
};

#endif //_SHAPE_HULL_H
