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

#include "CustomConvexPairCollision.h"
#include "ConvexHeightFieldShape.h"
#include "CustomConvexShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "Stubs/AdlContact4.h"
#include "Stubs/AdlTransform.h"


CustomConvexConvexPairCollision::CustomConvexConvexPairCollision(btPersistentManifold* mf,const btCollisionAlgorithmConstructionInfo& ci,btCollisionObject* body0,btCollisionObject* body1, btSimplexSolverInterface* simplexSolver, btConvexPenetrationDepthSolver* pdSolver, int numPerturbationIterations, int minimumPointsPerturbationThreshold)
:btConvexConvexAlgorithm(mf,ci,body0,body1,simplexSolver,pdSolver,numPerturbationIterations, minimumPointsPerturbationThreshold)
{

}

CustomConvexConvexPairCollision::~CustomConvexConvexPairCollision()
{

}


#include <Windows.h>

template<typename T>
T atomAdd(const T* ptr, int value)
{
	return (T)InterlockedExchangeAdd((LONG*)ptr, value);
}



#define PARALLEL_SUM(v, n) for(int j=1; j<n; j++) v[0] += v[j];
#define PARALLEL_DO(execution, n) for(int ie=0; ie<n; ie++){execution;}
#define REDUCE_MAX(v, n) {int i=0;\
	for(int offset=0; offset<n; offset++) v[i] = (v[i].y > v[i+offset].y)? v[i]: v[i+offset]; }
#define REDUCE_MIN(v, n) {int i=0;\
	for(int offset=0; offset<n; offset++) v[i] = (v[i].y < v[i+offset].y)? v[i]: v[i+offset]; }

int extractManifold(const float4* p, int nPoints, float4& nearNormal, float4& centerOut, 
					 int contactIdx[4])
{
	if( nPoints == 0 ) return 0;

	nPoints = min2( nPoints, 64 );

	float4 center = make_float4(0.f);
	{
		float4 v[64];
		memcpy( v, p, nPoints*sizeof(float4) );
		PARALLEL_SUM( v, nPoints );
		center = v[0]/(float)nPoints;
	}

	centerOut = center;

	{	//	sample 4 directions
		if( nPoints < 4 )
		{
			for(int i=0; i<nPoints; i++) contactIdx[i] = i;
			return nPoints;
		}

		float4 aVector = p[0] - center;
		float4 u = cross3( nearNormal, aVector );
		float4 v = cross3( nearNormal, u );
		u = normalize3( u );
		v = normalize3( v );

		int idx[4];

		float2 max00 = make_float2(0,FLT_MAX);
		{
			float4 dir0 = u;
			float4 dir1 = -u;
			float4 dir2 = v;
			float4 dir3 = -v;

			//	idx, distance
			{
				{
					int4 a[64];
					for(int ie = 0; ie<nPoints; ie++ )
					{
						float4 f;
						float4 r = p[ie]-center;
						f.x = dot3F4( dir0, r );
						f.y = dot3F4( dir1, r );
						f.z = dot3F4( dir2, r );
						f.w = dot3F4( dir3, r );

						a[ie].x = ((*(u32*)&f.x) & 0xffffff00);
						a[ie].x |= (0xff & ie);

						a[ie].y = ((*(u32*)&f.y) & 0xffffff00);
						a[ie].y |= (0xff & ie);

						a[ie].z = ((*(u32*)&f.z) & 0xffffff00);
						a[ie].z |= (0xff & ie);

						a[ie].w = ((*(u32*)&f.w) & 0xffffff00);
						a[ie].w |= (0xff & ie);
					}

					for(int ie=0; ie<nPoints; ie++)
					{
						a[0].x = (a[0].x > a[ie].x )? a[0].x: a[ie].x;
						a[0].y = (a[0].y > a[ie].y )? a[0].y: a[ie].y;
						a[0].z = (a[0].z > a[ie].z )? a[0].z: a[ie].z;
						a[0].w = (a[0].w > a[ie].w )? a[0].w: a[ie].w;
					}

					idx[0] = (int)a[0].x & 0xff;
					idx[1] = (int)a[0].y & 0xff;
					idx[2] = (int)a[0].z & 0xff;
					idx[3] = (int)a[0].w & 0xff;
				}
			}

			{
				float2 h[64];
				PARALLEL_DO( h[ie] = make_float2((float)ie, p[ie].w), nPoints );
				REDUCE_MIN( h, nPoints );
				max00 = h[0];
			}
		}

		contactIdx[0] = idx[0];
		contactIdx[1] = idx[1];
		contactIdx[2] = idx[2];
		contactIdx[3] = idx[3];

//		if( max00.y < 0.0f )
//			contactIdx[0] = (int)max00.x;

		std::sort( contactIdx, contactIdx+4 );

		return 4;
	}
}

#undef PARALLEL_SUM
#undef PARALLEL_DO
#undef REDUCE_MAX
#undef REDUCE_MIX

int collideStraight(const ConvexHeightField* shapeA,const ConvexHeightField* shapeB,
		const float4& bodyApos, Quaternion& bodyAquat,const float4& bodyBpos,const Quaternion& bodyBquat,
		ContactPoint4* contactsOut, int& numContacts, int contactCapacity,
		float collisionMargin )
{
//	Stopwatch sw;

	Transform trA;
	trA = trSetTransform(bodyApos,bodyAquat);
	Transform trB;
	trB = trSetTransform(bodyBpos, bodyBquat);
	
	Transform B2A;
	{
		Transform invTrA = trInvert( trA );
		B2A = trMul( invTrA, trB );
	}

	int nContacts = 0;
	{	// testB against A
		float4 p[ConvexHeightField::HEIGHT_RES*ConvexHeightField::HEIGHT_RES*6];
		int nHits = 0;

		const float4* pInB = shapeB->getSamplePoints();

		float4 baInB = qtInvRotate( bodyBquat, bodyApos - bodyBpos );
		if( shapeA->m_type == CollisionShape::SHAPE_HEIGHT_FIELD ) 
			baInB = make_float4(0,0,0,0);

//		sw.start();
		for(int iface=0; iface<6; iface++)
		{
			Aabb aabb = shapeB->m_faceAabbs[iface];

			aabb.transform( B2A.m_translation, B2A.m_rotation );

			if( !shapeA->m_aabb.overlaps( aabb ) ) continue;
			
			for(int ip=0; ip<ConvexHeightField::HEIGHT_RES*ConvexHeightField::HEIGHT_RES; ip++)
			{
				int i = iface*ConvexHeightField::HEIGHT_RES*ConvexHeightField::HEIGHT_RES+ip;

				if( dot3F4( baInB, pInB[i] ) < 0.f ) continue;

				float4 pInA = trMul1( B2A, pInB[i] );

				if( shapeA->m_aabb.overlaps( pInA ) )
				{
//					Stopwatch sw1;
//					sw1.start();
					float dist = shapeA->queryDistance( pInA );
//					sw1.stop();
//					m_times[TIME_SAMPLE] += sw1.getMs();

					if( dist < collisionMargin )
					{
						p[nHits] = make_float4(pInA.x, pInA.y, pInA.z, dist);
						nHits++;
					}
				}
			}
		}
//		sw.stop();
//		m_times[TIME_TEST] += sw.getMs();

//		sw.start();
		if( nHits )
		{
			float4 ab = bodyBpos - bodyApos;
			ab = qtInvRotate( bodyAquat, ab );
			if( shapeA->m_type == CollisionShape::SHAPE_HEIGHT_FIELD )
			{
				//todo.	sample normal from height field but just fake here
				ab = make_float4(0,1,0,0);
			}

			int cIdx[4];
			float4 center;
			
			nContacts = extractManifold( p, nHits, ab, center, cIdx );

			float4 contactNormal;
			{
				shapeA->queryDistanceWithNormal( center, contactNormal );
				contactNormal = normalize3( contactNormal );

//				u32 cmp = u8vCompress( contactNormal );
//				contactNormal = make_float4( u8vGetX(cmp), u8vGetY(cmp), u8vGetZ(cmp), 0 );
			}

			int writeIdx = atomAdd( &numContacts, 1 );
			if( writeIdx+1 < contactCapacity )
			{
				ContactPoint4& c = contactsOut[writeIdx];
				nContacts = min2( nContacts, 4 );
				for(int i=0; i<nContacts; i++)
				{
					c.m_worldPos[i] = transform( p[cIdx[i]], bodyApos, bodyAquat );
					c.m_worldPos[i].w = max2( p[cIdx[i]].w - collisionMargin, -2*collisionMargin );
				}
				c.m_worldNormal = normalize3( qtRotate( bodyAquat, contactNormal ) );
				c.m_restituitionCoeff = 0.f;
				c.m_frictionCoeff = 0.7f;
				//c.m_bodyAPtr = (void*)bodyAIdx;
				//c.m_bodyBPtr = (void*)bodyBIdx;
				c.getNPoints() = nContacts;
			}
		}
//		sw.stop();
//		m_times[TIME_MANIFOLD] += sw.getMs();
	}

	return nContacts;
}


void	CustomConvexConvexPairCollision::createManifoldPtr(btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo)
{
	m_manifoldPtr = m_dispatcher->getNewManifold(body0,body1);
	m_ownManifold = true;
}

	
void CustomConvexConvexPairCollision::processCollision (btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
{
#if 1
	if (!m_manifoldPtr)
	{
		//swapped?
		m_manifoldPtr = m_dispatcher->getNewManifold(body0,body1);
		m_ownManifold = true;
	}
	resultOut->setPersistentManifold(m_manifoldPtr);


	CustomConvexShape* convex0 = (CustomConvexShape*)body0->getCollisionShape();
	CustomConvexShape* convex1 = (CustomConvexShape*)body1->getCollisionShape();

	
	float4 bodyApos;
	float4 bodyBpos;
	Quaternion bodyAquat;
	Quaternion bodyBquat;

	const btTransform& transA = body0->getWorldTransform();
	const btTransform& transB = body1->getWorldTransform();

	const btVector3& pA = body0->getWorldTransform().getOrigin();
	const btVector3& pB = body1->getWorldTransform().getOrigin();

	btQuaternion qA = body0->getWorldTransform().getRotation();
	btQuaternion qB = body1->getWorldTransform().getRotation();

	bodyApos.x = pA.getX();
	bodyApos.y = pA.getY();
	bodyApos.z = pA.getZ();
	bodyApos.w = 0.f;
	
	bodyBpos.x = pB.getX();
	bodyBpos.y = pB.getY();
	bodyBpos.z = pB.getZ();
	bodyBpos.w = 0.f;
	
	bodyAquat.x = qA.getX();
	bodyAquat.y = qA.getY();
	bodyAquat.z = qA.getZ();
	bodyAquat.w = qA.getW();

	bodyBquat.x = qB.getX();
	bodyBquat.y = qB.getY();
	bodyBquat.z = qB.getZ();
	bodyBquat.w = qB.getW();


#define CAPACITY_CONTACTS 4

	ContactPoint4 contactsOut[CAPACITY_CONTACTS];
	int freeContactIndex = 0;
	int contactCapacity = CAPACITY_CONTACTS;
	float collisionMargin = 0.001f;

	m_manifoldPtr->refreshContactPoints(body0->getWorldTransform(),body1->getWorldTransform());

	collideStraight(convex0->m_ConvexHeightField,convex1->m_ConvexHeightField,
		bodyApos, bodyAquat,bodyBpos,bodyBquat,
		contactsOut, freeContactIndex, contactCapacity,
		collisionMargin );
	collideStraight(convex1->m_ConvexHeightField,convex0->m_ConvexHeightField,
		bodyBpos, bodyBquat,bodyApos,bodyAquat,
		contactsOut, freeContactIndex, contactCapacity,
		collisionMargin );

	//copy points into manifold
	//refresh manifold

	btAssert(freeContactIndex<3);
	for (int j=0;j<freeContactIndex;j++)
	{
		int numPoints = contactsOut[j].getNPoints();
//		printf("numPoints = %d\n",numPoints);

		for (int i=0;i<numPoints;i++)
		{

			ContactPoint4& c = contactsOut[j];

			btVector3 normalOnBInWorld(
				c.m_worldNormal.x,
				c.m_worldNormal.y,
				c.m_worldNormal.z);
			btVector3 pointInWorldOnB(
				c.m_worldPos[i].x,
				c.m_worldPos[i].y,
				c.m_worldPos[i].z);
			btScalar depth = c.m_worldPos[i].w;
			if (depth<0)
			{

				const btVector3 deltaC = transB.getOrigin() - transA.getOrigin();
				if((deltaC.dot(normalOnBInWorld))>0.0f)
				{
					normalOnBInWorld= -normalOnBInWorld;
				}
				normalOnBInWorld.normalize();
				if (j)
				{
					resultOut->addContactPoint(normalOnBInWorld, pointInWorldOnB, depth);
				} else
				{
					resultOut->addContactPoint(normalOnBInWorld, pointInWorldOnB-normalOnBInWorld*depth, depth);
				}
			}
		}
	}
#else
	btConvexConvexAlgorithm::processCollision(body0,body1,dispatchInfo,resultOut);
#endif
}



CustomConvexConvexPairCollision::CreateFunc::CreateFunc(btSimplexSolverInterface*			simplexSolver, btConvexPenetrationDepthSolver* pdSolver)
:btConvexConvexAlgorithm::CreateFunc(simplexSolver,pdSolver)
{
}
		
CustomConvexConvexPairCollision::CreateFunc::~CreateFunc()
{

}