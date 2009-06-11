/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#include "btMultiSphereShape.h"
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"
#include "LinearMath/btQuaternion.h"

btMultiSphereShape::btMultiSphereShape (const btVector3* positions,const btScalar* radi,int numSpheres)
:btConvexInternalAabbCachingShape ()
{
	m_shapeType = MULTI_SPHERE_SHAPE_PROXYTYPE;
	btScalar startMargin = btScalar(BT_LARGE_FLOAT);

	m_localPositionArray.resize(numSpheres);
	m_radiArray.resize(numSpheres);
	for (int i=0;i<numSpheres;i++)
	{
		m_localPositionArray[i] = positions[i];
		m_radiArray[i] = radi[i];
		
	}

	recalcLocalAabb();

}

 
 btVector3	btMultiSphereShape::localGetSupportingVertexWithoutMargin(const btVector3& vec0)const
{
	int i;
	btVector3 supVec(0,0,0);

	btScalar maxDot(btScalar(-BT_LARGE_FLOAT));


	btVector3 vec = vec0;
	btScalar lenSqr = vec.length2();
	if (lenSqr < (SIMD_EPSILON*SIMD_EPSILON))
	{
		vec.setValue(1,0,0);
	} else
	{
		btScalar rlen = btScalar(1.) / btSqrt(lenSqr );
		vec *= rlen;
	}

	btVector3 vtx;
	btScalar newDot;

	const btVector3* pos = &m_localPositionArray[0];
	const btScalar* rad = &m_radiArray[0];
	int numSpheres = m_localPositionArray.size();

	for (i=0;i<numSpheres;i++)
	{
		vtx = (*pos) +vec*m_localScaling*(*rad) - vec * getMargin();
		pos++;
		rad++;
		newDot = vec.dot(vtx);
		if (newDot > maxDot)
		{
			maxDot = newDot;
			supVec = vtx;
		}
	}

	return supVec;

}

 void	btMultiSphereShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors,btVector3* supportVerticesOut,int numVectors) const
{

	for (int j=0;j<numVectors;j++)
	{
		btScalar maxDot(btScalar(-BT_LARGE_FLOAT));

		const btVector3& vec = vectors[j];

		btVector3 vtx;
		btScalar newDot;

		const btVector3* pos = &m_localPositionArray[0];
		const btScalar* rad = &m_radiArray[0];
		int numSpheres = m_localPositionArray.size();
		for (int i=0;i<numSpheres;i++)
		{
			vtx = (*pos) +vec*m_localScaling*(*rad) - vec * getMargin();
			pos++;
			rad++;
			newDot = vec.dot(vtx);
			if (newDot > maxDot)
			{
				maxDot = newDot;
				supportVerticesOut[j] = vtx;
			}
		}
	}
}








void	btMultiSphereShape::calculateLocalInertia(btScalar mass,btVector3& inertia) const
{
	//as an approximation, take the inertia of the box that bounds the spheres

	btVector3 localAabbMin,localAabbMax;
	getCachedLocalAabb(localAabbMin,localAabbMax);
	btVector3 halfExtents = (localAabbMax-localAabbMin)*btScalar(0.5);

	btScalar lx=btScalar(2.)*(halfExtents.x());
	btScalar ly=btScalar(2.)*(halfExtents.y());
	btScalar lz=btScalar(2.)*(halfExtents.z());

	inertia.setValue(mass/(btScalar(12.0)) * (ly*ly + lz*lz),
					mass/(btScalar(12.0)) * (lx*lx + lz*lz),
					mass/(btScalar(12.0)) * (lx*lx + ly*ly));

}


