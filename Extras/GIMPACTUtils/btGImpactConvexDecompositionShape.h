/*! \file btGImpactConvexDecompositionShape.h
\author Francisco León Nájera
*/
/*
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com


This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef GIMPACT_CONVEX_DECOMPOSITION_SHAPE_H
#define GIMPACT_CONVEX_DECOMPOSITION_SHAPE_H

#include "BulletCollision/Gimpact/btGImpactShape.h"  // box tree class

//! This class creates a decomposition from a trimesh.
/*!

*/
class btGImpactConvexDecompositionShape : public btGImpactCompoundShape
{
protected:
	btAlignedObjectArray<btGImpactMeshShapePart::TrimeshPrimitiveManager> m_trimeshInterfaces;

	class GIM_ConvexDecomposition* m_decomposition;

	void buildConvexDecomposition(bool transformSubShapes);

public:
	btGImpactConvexDecompositionShape(
		btStridingMeshInterface* meshInterface,
		const btVector3& mesh_scale,
		btScalar margin = btScalar(0.01), bool children_has_transform = true)
		: btGImpactCompoundShape(children_has_transform)
	{
		m_collisionMargin = margin;

		btGImpactMeshShapePart::TrimeshPrimitiveManager triInterface;
		triInterface.m_meshInterface = meshInterface;
		triInterface.m_scale = mesh_scale;
		triInterface.m_margin = btScalar(1.0);

		//add parts
		int part_count = meshInterface->getNumSubParts();
		for (int i = 0; i < part_count; i++)
		{
			triInterface.m_part = i;
			m_trimeshInterfaces.push_back(triInterface);
		}

		m_decomposition = 0;

		buildConvexDecomposition(children_has_transform);
	}

	virtual ~btGImpactConvexDecompositionShape();

	SIMD_FORCE_INLINE btGImpactMeshShapePart::TrimeshPrimitiveManager* getTrimeshInterface(int part)
	{
		return &m_trimeshInterfaces[part];
	}

	virtual void processAllTriangles(btTriangleCallback* callback, const btVector3& aabbMin, const btVector3& aabbMax) const;
};

#endif  //GIMPACT_MESH_SHAPE_H
