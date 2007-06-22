/*
-----------------------------------------------------------------------------
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2006 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com

 This library is free software; you can redistribute it and/or
 modify it under the terms of EITHER:
   (1) The GNU Lesser General Public License as published by the Free
       Software Foundation; either version 2.1 of the License, or (at
       your option) any later version. The text of the GNU Lesser
       General Public License is included with this library in the
       file GIMPACT-LICENSE-LGPL.TXT.
   (2) The BSD-style license that is included with this library in
       the file GIMPACT-LICENSE-BSD.TXT.
   (3) The zlib/libpng license that is included with this library in
       the file GIMPACT-LICENSE-ZLIB.TXT.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
 GIMPACT-LICENSE-LGPL.TXT, GIMPACT-LICENSE-ZLIB.TXT and GIMPACT-LICENSE-BSD.TXT for more details.

-----------------------------------------------------------------------------
*/


#include "GIMPACT/Bullet/btGImpactShape.h"
#include "GIMPACT/Bullet/btGImpactMassUtil.h"


#define CALC_EXACT_INERTIA 1

void btGImpactCompoundShape::calculateLocalInertia(btScalar mass,btVector3& inertia)
{

#ifdef CALC_EXACT_INERTIA
	inertia.setValue(0.f,0.f,0.f);

	GUINT i = this->getNumChildShapes();
	GREAL shapemass = mass/btScalar(i);

	while(i--)
	{
		btVector3 temp_inertia;
		m_childShapes[i]->calculateLocalInertia(shapemass,temp_inertia);
		inertia = gim_inertia_add_transformed( inertia,temp_inertia,m_childTransforms[i]);
	}

#else

	// Calc box inertia

	btScalar lx= m_localAABB.m_max[0] - m_localAABB.m_min[0];
	btScalar ly= m_localAABB.m_max[1] - m_localAABB.m_min[1];
	btScalar lz= m_localAABB.m_max[2] - m_localAABB.m_min[2];
	const btScalar x2 = lx*lx;
	const btScalar y2 = ly*ly;
	const btScalar z2 = lz*lz;
	const btScalar scaledmass = mass * btScalar(0.08333333);

	inertia = scaledmass * (btVector3(y2+z2,x2+z2,x2+y2));

#endif
}



void btGImpactMeshShapePart::calculateLocalInertia(btScalar mass,btVector3& inertia)
{
	lock();


#ifdef CALC_EXACT_INERTIA
	inertia.setValue(0.f,0.f,0.f);

	GUINT i = this->getVertexCount();
	GREAL pointmass = mass/btScalar(i);

	while(i--)
	{
		btVector3 pointintertia;
		this->getVertex(i,pointintertia);
		pointintertia = gim_get_point_inertia(pointintertia,pointmass);
		inertia+=pointintertia;
	}

#else

	// Calc box inertia

	btScalar lx= m_localAABB.m_max[0] - m_localAABB.m_min[0];
	btScalar ly= m_localAABB.m_max[1] - m_localAABB.m_min[1];
	btScalar lz= m_localAABB.m_max[2] - m_localAABB.m_min[2];
	const btScalar x2 = lx*lx;
	const btScalar y2 = ly*ly;
	const btScalar z2 = lz*lz;
	const btScalar scaledmass = mass * btScalar(0.08333333);

	inertia = scaledmass * (btVector3(y2+z2,x2+z2,x2+y2));

#endif

	unlock();
}

void btGImpactMeshShape::calculateLocalInertia(btScalar mass,btVector3& inertia)
{

#ifdef CALC_EXACT_INERTIA
	inertia.setValue(0.f,0.f,0.f);

	GUINT i = this->getMeshPartCount();
	GREAL partmass = mass/btScalar(i);

	while(i--)
	{
		btVector3 partinertia;
		getMeshPart(i)->calculateLocalInertia(partmass,partinertia);
		inertia+=partinertia;
	}

#else

	// Calc box inertia

	btScalar lx= m_localAABB.m_max[0] - m_localAABB.m_min[0];
	btScalar ly= m_localAABB.m_max[1] - m_localAABB.m_min[1];
	btScalar lz= m_localAABB.m_max[2] - m_localAABB.m_min[2];
	const btScalar x2 = lx*lx;
	const btScalar y2 = ly*ly;
	const btScalar z2 = lz*lz;
	const btScalar scaledmass = mass * btScalar(0.08333333);

	inertia = scaledmass * (btVector3(y2+z2,x2+z2,x2+y2));

#endif
}



void btGImpactCompoundShape::rayTest(const btVector3& rayFrom, const btVector3& rayTo, btCollisionWorld::RayResultCallback& resultCallback) const
{

}

void btGImpactMeshShape::rayTest(const btVector3& rayFrom, const btVector3& rayTo, btCollisionWorld::RayResultCallback& resultCallback) const
{
}

void btGImpactMeshShapePart::rayTest(const btVector3& rayFrom, const btVector3& rayTo, btCollisionWorld::RayResultCallback& resultCallback) const
{
}


void btGImpactMeshShapePart::processAllTriangles(btTriangleCallback* callback,const btVector3& aabbMin,const btVector3& aabbMax) const
{
	lock();
	GIM_AABB box;
	box.m_min = aabbMin;
	box.m_max = aabbMax;

	gim_array<GUINT> collided;	
	m_box_set.boxQuery(box,collided);

	if(collided.size()==0)
	{
		unlock();
		return;
	}

	int part = (int)getPart();
	GIM_TRIANGLE triangle;
	GUINT i = collided.size();	
	while(i--)
	{
		this->getTriangle(collided[i],triangle);
		callback->processTriangle(triangle.m_vertices,part,collided[i]);
	}
	unlock();

}

void btGImpactMeshShape::processAllTriangles(btTriangleCallback* callback,const btVector3& aabbMin,const btVector3& aabbMax) const
{
	GUINT i = m_mesh_parts.size();
	while(i--)
	{
		m_mesh_parts[i]->processAllTriangles(callback,aabbMin,aabbMax);
	}
}
