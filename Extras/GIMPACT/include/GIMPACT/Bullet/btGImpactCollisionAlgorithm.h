/*! \file btGImpactShape.h
\author Francisco León Nájera
*/
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

#ifndef BVH_CONCAVE_COLLISION_ALGORITHM_H
#define BVH_CONCAVE_COLLISION_ALGORITHM_H

#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
class btDispatcher;
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"

#include "LinearMath/btAlignedObjectArray.h"

#include "GIMPACT/Bullet/btGImpactShape.h"
#include "BulletCollision/CollisionShapes/btStaticPlaneShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h"




//! Collision Algorithm for GImpact Shapes
/*!
For register this algorithm in Bullet, proceed as following:
 \code
btCollisionDispatcher * dispatcher = static_cast<btCollisionDispatcher *>(m_dynamicsWorld ->getDispatcher());
btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);
 \endcode
*/
class btGImpactCollisionAlgorithm : public btCollisionAlgorithm
{
protected:
	btCollisionAlgorithm * m_convex_algorithm;
    btPersistentManifold* m_manifoldPtr;
	btManifoldResult* m_resultOut;
	const btDispatcherInfo * m_dispatchInfo;
	int m_triface0;
	int m_part0;
	int m_triface1;
	int m_part1;

	SIMD_FORCE_INLINE btPersistentManifold* newContactManifold(btCollisionObject* body0,btCollisionObject* body1)
	{
		clearCache();
		m_manifoldPtr = m_dispatcher->getNewManifold(body0,body1);
		return m_manifoldPtr;
	}

	SIMD_FORCE_INLINE void destroyConvexAlgorithm()
	{
		if(m_convex_algorithm)
		{
			m_convex_algorithm->~btCollisionAlgorithm();
			m_dispatcher->freeCollisionAlgorithm( m_convex_algorithm);
			m_convex_algorithm = NULL;
		}
	}

	SIMD_FORCE_INLINE void clearCache()
	{
		if(m_manifoldPtr)
		{
			//m_manifoldPtr->clearManifold();
			m_dispatcher->releaseManifold(m_manifoldPtr);
			m_manifoldPtr = NULL;
		}
		destroyConvexAlgorithm();

		m_triface0 = -1;
		m_part0 = -1;
		m_triface1 = -1;
		m_part1 = -1;
	}


	// Call before process collision
	SIMD_FORCE_INLINE void checkManifold(btCollisionObject* body0,btCollisionObject* body1)
	{
		if(m_manifoldPtr == NULL)
		{
			newContactManifold(body0,body1);
		}
		/*else if(m_manifoldPtr->getBody0()!=body0)
		{
			clearCache();
			newContactManifold(body0,body1);
		}*/
		m_resultOut->setPersistentManifold(m_manifoldPtr);
	}

	// Call before process collision
	SIMD_FORCE_INLINE btCollisionAlgorithm * newAlgorithm(btCollisionObject* body0,btCollisionObject* body1)
	{
		checkManifold(body0,body1);
		/*btConvexConvexAlgorithm::CreateFunc convexcreatefunc;
		btCollisionAlgorithmConstructionInfo cinfo;
		cinfo.m_dispatcher = m_dispatcher;
		cinfo.m_manifold = m_manifoldPtr;*/
		btCollisionAlgorithm * convex_algorithm = m_dispatcher->findAlgorithm(body0,body1,m_manifoldPtr);
		return convex_algorithm ;
	}

	// Call before process collision
	SIMD_FORCE_INLINE void checkConvexAlgorithm(btCollisionObject* body0,btCollisionObject* body1)
	{
		if(m_convex_algorithm) return;
		m_convex_algorithm = newAlgorithm(body0,body1);
	}




	SIMD_FORCE_INLINE void addContactPoint(btCollisionObject * body0,
					btCollisionObject * body1,
					const btVector3 & point,
					const btVector3 & normal,
					btScalar distance)
	{
		checkManifold(body0,body1);
		m_resultOut->addContactPoint(normal,point,distance);
	}

	void gimpactcompound_vs_gimpactcompound_find_pairs(
					  const btTransform & trans0,
					  const btTransform & trans1,
					  btGImpactCompoundShape * shape0,
					  btGImpactCompoundShape * shape1,gim_pair_set & pairset) const;

	void gimpacttrimeshpart_vs_gimpacttrimeshpart_find_pairs(
					  const btTransform & trans0,
					  const btTransform & trans1,
					  btGImpactMeshShapePart * shape0,
					  btGImpactMeshShapePart * shape1,gim_pair_set & pairset) const;

	void gimpactcompound_vs_gimpacttrimeshpart_find_pairs(
					  const btTransform & trans0,
					  const btTransform & trans1,
					  btGImpactCompoundShape * shape0,
					  btGImpactMeshShapePart * shape1,gim_pair_set & pairset) const;

public:

	btGImpactCollisionAlgorithm( const btCollisionAlgorithmConstructionInfo& ci,btCollisionObject* body0,btCollisionObject* body1);

	virtual ~btGImpactCollisionAlgorithm();

	virtual void processCollision (btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut);

	btScalar	calculateTimeOfImpact(btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut);




	struct CreateFunc :public 	btCollisionAlgorithmCreateFunc
	{
		virtual	btCollisionAlgorithm* CreateCollisionAlgorithm(btCollisionAlgorithmConstructionInfo& ci, btCollisionObject* body0,btCollisionObject* body1)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btGImpactCollisionAlgorithm));
			return new(mem) btGImpactCollisionAlgorithm(ci,body0,body1);
		}
	};

	//! Use this function for register the algorithm externally
	static void registerAlgorithm(btCollisionDispatcher * dispatcher);

	//! Collision algorithms
	//!@{





	void shape_vs_shape_collision(
					  btCollisionObject * body0,
					  btCollisionObject * body1,
					  btCollisionShape * shape0,
					  btCollisionShape * shape1,bool swapped);

	void convex_vs_convex_collision(btCollisionObject * body0,
					  btCollisionObject * body1,
					  btCollisionShape * shape0,
					  btCollisionShape * shape1);

	void gimpacttrimesh_vs_shape_collision(
					  btCollisionObject * body0,
					  btCollisionObject * body1,
					  btGImpactMeshShape * shape0,
					  btCollisionShape * shape1,bool swapped);

	void gimpacttrimesh_vs_gimpacttrimesh(
					  btCollisionObject * body0,
					  btCollisionObject * body1,
					  btGImpactMeshShape * shape0,
					  btGImpactMeshShape * shape1);

	void gimpacttrimesh_vs_gimpactcompound(
					  btCollisionObject * body0,
					  btCollisionObject * body1,
					  btGImpactMeshShape * shape0,
					  btGImpactCompoundShape * shape1,bool swapped);

	void gimpacttrimesh_vs_trimeshpart(
					  btCollisionObject * body0,
					  btCollisionObject * body1,
					  btGImpactMeshShape * shape0,
					  btGImpactMeshShapePart * shape1,bool swapped);


	void gimpactcompound_vs_gimpactcompound_collision(
					  btCollisionObject * body0,
					  btCollisionObject * body1,
					  btGImpactCompoundShape * shape0,
					  btGImpactCompoundShape * shape1);


	void gimpactcompound_vs_gimpacttrimeshpart_collision(
					  btCollisionObject * body0,
					  btCollisionObject * body1,
					  btGImpactCompoundShape * shape0,
					  btGImpactMeshShapePart * shape1,bool swapped);


	void gimpactcompound_vs_shape_collision(
					  btCollisionObject * body0,
					  btCollisionObject * body1,
					  btGImpactCompoundShape * shape0,
					  btCollisionShape * shape1,bool swapped);

	void gimpacttrimeshpart_vs_gimpacttrimeshpart_collision(
					  btCollisionObject * body0,
					  btCollisionObject * body1,
					  btGImpactMeshShapePart * shape0,
					  btGImpactMeshShapePart * shape1,bool swapped);

	void gimpacttrimeshpart_vs_plane_collision(
					  btCollisionObject * body0,
					  btCollisionObject * body1,
					  btGImpactMeshShapePart * shape0,
					  btStaticPlaneShape * shape1,bool swapped);


	void gimpacttrimeshpart_vs_concave_collision(
					  btCollisionObject * body0,
					  btCollisionObject * body1,
					  btGImpactMeshShapePart * shape0,
					  btConcaveShape * shape1,bool swapped);

	void gimpacttrimeshpart_vs_shape_collision(
					  btCollisionObject * body0,
					  btCollisionObject * body1,
					  btGImpactMeshShapePart * shape0,
					  btCollisionShape * shape1,bool swapped);

	void gimpact_vs_compoundshape(btCollisionObject * body0,
					  btCollisionObject * body1,
					  btGImpactShapeInterface * shape0,
					  btCompoundShape * shape1,bool swapped);


	void gimpact_vs_shape(btCollisionObject * body0,
					  btCollisionObject * body1,
					  btGImpactShapeInterface * shape0,
					  btCollisionShape * shape1,bool swapped);

	void gimpact_vs_gimpact(btCollisionObject * body0,
					  btCollisionObject * body1,
					  btGImpactShapeInterface * shape0,
					  btGImpactShapeInterface * shape1);

	//!@}

};



#endif //BVH_CONCAVE_COLLISION_ALGORITHM_H
