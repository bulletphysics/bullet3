/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "SpuSubSimplexConvexCast.h"


#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/CollisionShapes/btMinkowskiSumShape.h"
#include "BulletCollision/NarrowPhaseCollision/btSimplexSolverInterface.h"


SpuSubsimplexRayCast::SpuSubsimplexRayCast (void* shapeB, SpuConvexPolyhedronVertexData* convexDataB, int shapeTypeB, float marginB,
										    SpuVoronoiSimplexSolver* simplexSolver)
	:m_simplexSolver(simplexSolver), m_shapeB(shapeB), m_convexDataB(convexDataB), m_shapeTypeB(shapeTypeB), m_marginB(marginB)
{
}

///Typically the conservative advancement reaches solution in a few iterations, clip it to 32 for degenerate cases.
///See discussion about this here http://continuousphysics.com/Bullet/phpBB2/viewtopic.php?t=565
#ifdef BT_USE_DOUBLE_PRECISION
#define MAX_ITERATIONS 64
#else
#define MAX_ITERATIONS 32
#endif

/* Returns the support point of the minkowski sum:
 * MSUM(Pellet, ConvexShape)
 *
 */
void supportPoints (const btTransform& xformRay,
		    const btTransform& xformB,
		    const int shapeType,
		    const void* shape,
		    SpuConvexPolyhedronVertexData* convexVertexData,
		    const btScalar marginB,
		    const btVector3& seperatingAxis,
		    btVector3& w,
		    btVector3& supVertexRay,
		    btVector3& supVertexB)
{
	btVector3 saUnit = seperatingAxis;
	saUnit.normalize();
	btVector3 SupportPellet = xformRay(0.0001 * -saUnit);
	btVector3 rotatedSeperatingAxis = seperatingAxis * xformB.getBasis();
	btVector3 SupportShape = xformB(localGetSupportingVertexWithoutMargin(shapeType, (void*)shape, rotatedSeperatingAxis, convexVertexData));
	SupportShape += saUnit * marginB;
	w = SupportPellet - SupportShape;
	supVertexRay = SupportPellet;
	supVertexB = SupportShape;
}

bool	SpuSubsimplexRayCast::calcTimeOfImpact(const btTransform& fromRay,
											   const btTransform& toRay,
											   const btTransform& fromB,
											   const btTransform& toB,
											   SpuCastResult& result)
{
	m_simplexSolver->reset();

	btVector3 linVelRay, linVelB;
	linVelRay = toRay.getOrigin() - fromRay.getOrigin();
	linVelB = toB.getOrigin() - fromB.getOrigin ();

	btScalar lambda = btScalar(0.);
	
	btTransform interpolatedTransRay = fromRay;
	btTransform interpolatedTransB = fromB;

	btVector3 r = (linVelRay-linVelB);
	btVector3 supVertexRay;
	btVector3 supVertexB;
	btVector3 v;
	supportPoints (fromRay, fromB, m_shapeTypeB, m_shapeB, m_convexDataB, m_marginB, r, v, supVertexRay, supVertexB);

	btVector3 n;
	n.setValue(btScalar(0.), btScalar(0.), btScalar(0.));
	bool hasResult = false;
	btVector3 c;
	int maxIter = MAX_ITERATIONS;

	btScalar lastLambda = lambda;

	btScalar dist2 = v.length2();

#ifdef BT_USE_DOUBLE_PRECISION
	btScalar epsilon = btScalar(0.0001);
#else
	btScalar epsilon = btScalar(0.0001);
#endif //BT_USE_DOUBLE_PRECISION
	btVector3 w,p;
	btScalar VdotR;
	
	while ( (dist2 > epsilon) && maxIter--)
	{
		supportPoints (interpolatedTransRay, interpolatedTransB, m_shapeTypeB, m_shapeB, m_convexDataB, m_marginB, v, w, supVertexRay, supVertexB);

		btScalar VdotW = v.dot(w);

		if (lambda > btScalar(1.0))
		{
			return false;
		}

		if ( VdotW > btScalar(0.))
		{
			VdotR = v.dot(r);

			if (VdotR >= -(SIMD_EPSILON*SIMD_EPSILON))
				return false;
			else
			{
				lambda = lambda - VdotW / VdotR;
				interpolatedTransRay.getOrigin().setInterpolate3(fromRay.getOrigin(), toRay.getOrigin(), lambda);
				interpolatedTransB.getOrigin().setInterpolate3(fromB.getOrigin(), toB.getOrigin(), lambda);
				lastLambda = lambda;
				n = v;
				hasResult = true;
			}
		} 
		m_simplexSolver->addVertex(w, supVertexRay, supVertexB);
		if (m_simplexSolver->closest(v))
		{
			dist2 = v.length2();
			hasResult = true;
			//printf("V=%f , %f, %f\n",v[0],v[1],v[2]);
			//printf("DIST2=%f\n",dist2);
			//printf("numverts = %i\n",m_simplexSolver->numVertices());
		} else
		{
			dist2 = btScalar(0.);
		} 
	}

	result.m_fraction = lambda;
	result.m_normal = n;
	btVector3 hitRay, hitB;
	m_simplexSolver->compute_points (hitRay, hitB);
	/* TODO: We could output hit point here (hitB) */
	return true;
}
