/*
Copyright (c) 2012 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Erwin Coumans

#include "CustomConvexShape.h"
#include "ConvexHeightFieldShape.h"
#include "BulletCollision/CollisionShapes/btConvexPolyhedron.h"


CustomConvexShape::CustomConvexShape(const btScalar* points,int numPoints, int stride)
:btConvexHullShape(points,numPoints,stride),
m_acceleratedCompanionShapeIndex(-1)
{
	m_shapeType = CUSTOM_POLYHEDRAL_SHAPE_TYPE;

	initializePolyhedralFeatures();
	int numFaces= m_polyhedron->m_faces.size();
	float4* eqn = new float4[numFaces];
	for (int i=0;i<numFaces;i++)
	{
		eqn[i].x = m_polyhedron->m_faces[i].m_plane[0];
		eqn[i].y = m_polyhedron->m_faces[i].m_plane[1];
		eqn[i].z = m_polyhedron->m_faces[i].m_plane[2];
		eqn[i].w = m_polyhedron->m_faces[i].m_plane[3];
	}
	
	m_ConvexHeightField = new ConvexHeightField(eqn,numFaces);

}

CustomConvexShape::~CustomConvexShape()
{
	delete m_ConvexHeightField;
}