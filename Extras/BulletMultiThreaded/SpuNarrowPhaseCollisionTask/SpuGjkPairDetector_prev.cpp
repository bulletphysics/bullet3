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


#include "SpuGjkPairDetector.h"
#include "SpuConvexPenetrationDepthSolver.h"
#include "SpuLocalSupport.h"

//#include "BulletCollision/CollisionShapes/btConvexShape.h"

#if defined(DEBUG) || defined (_DEBUG)
#include <stdio.h> //for debug printf
#ifdef __SPU__
#include <spu_printf.h>
#define printf spu_printf
#endif //__SPU__
#endif

//must be above the machine epsilon
#define REL_ERROR2 float(1.0e-6)

//temp globals, to improve GJK/EPA/penetration calculations
int gSpuNumDeepPenetrationChecks = 0;
int gSpuNumGjkChecks = 0;



SpuGjkPairDetector::SpuGjkPairDetector(void* objectA,void* objectB,int shapeTypeA, int shapeTypeB, float marginA,float marginB,SpuVoronoiSimplexSolver* simplexSolver, const SpuConvexPenetrationDepthSolver*	penetrationDepthSolver)
:m_cachedSeparatingAxis(float(0.),float(0.),float(1.)),
m_penetrationDepthSolver(penetrationDepthSolver),
m_simplexSolver(simplexSolver),
m_minkowskiA(objectA),
m_minkowskiB(objectB),
m_shapeTypeA(shapeTypeA),
m_shapeTypeB(shapeTypeB),
m_marginA(marginA),
m_marginB(marginB),
m_ignoreMargin(false),
m_lastUsedMethod(-1),
m_catchDegeneracies(1)
{
}

void SpuGjkPairDetector::getClosestPoints(const SpuClosestPointInput& input,SpuContactResult& output)
{
	float distance=float(0.);
	Vectormath::Aos::Vector3	normalInB(float(0.),float(0.),float(0.));
	Vectormath::Aos::Point3 pointOnA,pointOnB;
	Vectormath::Aos::Transform3 localTransA = input.m_transformA;
	Vectormath::Aos::Transform3 localTransB = input.m_transformB;

	// World space coordinate 
    Vectormath::Aos::Vector3 localOriginA = localTransA.getTranslation();
    Vectormath::Aos::Vector3 localOriginB = localTransB.getTranslation();

	// Average instance position.
	Vectormath::Aos::Vector3 positionOffset = (localOriginA + localOriginB) * float(0.5);

	// Adjust the instance positions so that they're equidistant from the origin.
	localTransA.setTranslation(localOriginA - positionOffset);
	localTransB.setTranslation(localOriginB - positionOffset);

	float marginA = m_marginA;
	float marginB = m_marginB;

	gSpuNumGjkChecks++;

	//for CCD we don't use margins
	if (m_ignoreMargin)
	{
		marginA = float(0.);
		marginB = float(0.);
	}

	m_curIter = 0;
	int gGjkMaxIter = 1000;//this is to catch invalid input, perhaps check for #NaN?
    m_cachedSeparatingAxis = Vectormath::Aos::Vector3(0.f,1.f,0.f);

	bool isValid = false;
	bool checkSimplex = false;
	bool checkPenetration = true;
	m_degenerateSimplex = 0;

	m_lastUsedMethod = -1;

	{
		float squaredDistance = 1e30f;
		// There's no reason to have this delta declared out here, it's set in the loop before it's used and it's not used outside of the loop.
		float delta = float(0.);
		
		float margin = marginA + marginB;

		m_simplexSolver->reset();

		while (true)
		{
			// Get the separating axes into each bound's local space.
			Vectormath::Aos::Vector3 seperatingAxisInA = orthoInverse(input.m_transformA) * (-m_cachedSeparatingAxis);
			Vectormath::Aos::Vector3 seperatingAxisInB = orthoInverse(input.m_transformB) * m_cachedSeparatingAxis;

            int shapeTypeA = m_shapeTypeA;
            int shapeTypeB = m_shapeTypeB;
//			int featureIndexA = 0, featureIndexB = 0;			// Feature index basically means vertex index.
			Vectormath::Aos::Point3 pInA = localGetSupportingVertexWithoutMargin(shapeTypeA, m_minkowskiA, seperatingAxisInA);//, &featureIndexA);
			Vectormath::Aos::Point3 qInB = localGetSupportingVertexWithoutMargin(shapeTypeB, m_minkowskiB, seperatingAxisInB);//, &featureIndexB);

			// These are in a 'translated' world space where the origin of that world space corresponds to 'positionOffset' in the 'real' world space.
            Vectormath::Aos::Point3  pWorld = localTransA * pInA;	
			Vectormath::Aos::Point3  qWorld = localTransB * qInB;

            //spu_printf("support point A: %f %f %f\n", pWorld.getX(), pWorld.getY(), pWorld.getZ());
            //spu_printf("support point B: %f %f %f\n", qWorld.getX(), qWorld.getY(), qWorld.getZ());

			// delta is sort of the distance between the current 'closest points' ...
			// This seems kind of weird to me since m_cachedSeparatingAxis isn't a unit vector, so I'm not really sure what this signifies.
			Vectormath::Aos::Vector3 w	= pWorld - qWorld;
			delta = dot(m_cachedSeparatingAxis, w);

			// potential exit, they don't overlap
			if ((delta > float(0.0)) && (delta * delta > squaredDistance * input.m_maximumDistanceSquared)) 
			{
				checkPenetration = false;
				break;
			}

			//exit 0: the new point is already in the simplex, or we didn't come any closer
			if (m_simplexSolver->inSimplex(w))
			{
				m_degenerateSimplex = 1;
				checkSimplex = true;
				break;
			}

			// are we getting any closer ?
			float f0 = squaredDistance - delta;
			float f1 = squaredDistance * REL_ERROR2;

			// Are we close enough
			if (f0 <= f1)
			{
				if (f0 <= float(0.))
				{
					m_degenerateSimplex = 2;
				}
				checkSimplex = true;
				break;
			}
			//add current vertex to simplex
			m_simplexSolver->addVertex(w, pWorld, qWorld);//, featureIndexA, featureIndexB);

			//calculate the closest point to the origin (update vector v)
			if (!m_simplexSolver->closest(m_cachedSeparatingAxis))
			{
				m_degenerateSimplex = 3;
				checkSimplex = true;
				break;
			}

			float previousSquaredDistance = squaredDistance;
			squaredDistance = lengthSqr(m_cachedSeparatingAxis);
			
			//redundant m_simplexSolver->compute_points(pointOnA, pointOnB);

			//are we getting any closer ?
			if (previousSquaredDistance - squaredDistance <= FLT_EPSILON * previousSquaredDistance) 
			{ 
				m_simplexSolver->backup_closest(m_cachedSeparatingAxis);
				checkSimplex = true;
				break;
			}

			  //degeneracy, this is typically due to invalid/uninitialized worldtransforms for a btCollisionObject   
              if (m_curIter++ > gGjkMaxIter)   
              {   
                      #if defined(DEBUG) || defined (_DEBUG)   

                              printf("SpuGjkPairDetector maxIter exceeded:%i\n",m_curIter);   
                              printf("sepAxis=(%f,%f,%f), squaredDistance = %f, shapeTypeA=%i,shapeTypeB=%i\n",   
                              m_cachedSeparatingAxis.getX(),   
                              m_cachedSeparatingAxis.getY(),   
                              m_cachedSeparatingAxis.getZ(),   
                              squaredDistance,   
                              shapeTypeA,   
                              shapeTypeB);   

                      #endif   
                      break;   

              } 


			bool check = (!m_simplexSolver->fullSimplex());
			//bool check = (!m_simplexSolver->fullSimplex() && squaredDistance > FLT_EPSILON * m_simplexSolver->maxVertex());

			if (!check)
			{
				//do we need this backup_closest here ?
				m_simplexSolver->backup_closest(m_cachedSeparatingAxis);
				break;
			}
		}

		if (checkSimplex)
		{
			m_simplexSolver->compute_points(pointOnA, pointOnB);
			normalInB = pointOnA-pointOnB;
			float lenSqr = lengthSqr(m_cachedSeparatingAxis);
			//valid normal
			if (lenSqr < 0.0001)
			{
				m_degenerateSimplex = 5;
			} 
			if (lenSqr > FLT_EPSILON*FLT_EPSILON)
			{
				float rlen = float(1.) / sqrtf(lenSqr );
				normalInB *= rlen; //normalize
				float s = sqrtf(squaredDistance);
			
				btAssert(s > float(0.0));
				pointOnA -= m_cachedSeparatingAxis * (marginA / s);
				pointOnB += m_cachedSeparatingAxis * (marginB / s);
				distance = ((float(1.)/rlen) - margin);
				isValid = true;
				
				m_lastUsedMethod = 1;
			} else
			{
				m_lastUsedMethod = 2;
			}
		}

		bool catchDegeneratePenetrationCase = 
			(m_catchDegeneracies && m_penetrationDepthSolver && m_degenerateSimplex && ((distance+margin) < 0.01));

		//if (checkPenetration && !isValid)
		if (checkPenetration && (!isValid || catchDegeneratePenetrationCase ))
		{
			//penetration case
		
			//if there is no way to handle penetrations, bail out
			if (m_penetrationDepthSolver)
			{
				// Penetration depth case.
				Vectormath::Aos::Point3 tmpPointOnA,tmpPointOnB;
				
//				spu_printf("SPU: deep penetration check\n");
				gSpuNumDeepPenetrationChecks++;

				bool isValid2 = m_penetrationDepthSolver->calcPenDepth( 
					*m_simplexSolver, 
					m_minkowskiA,m_minkowskiB,
                    m_shapeTypeA, m_shapeTypeB,
                    marginA, marginB,
					localTransA,localTransB,
					m_cachedSeparatingAxis, tmpPointOnA, tmpPointOnB,
					0,input.m_stackAlloc
					);
					

				if (isValid2)
				{
					Vectormath::Aos::Vector3 tmpNormalInB = tmpPointOnB-tmpPointOnA;
					float lenSqr = lengthSqr(tmpNormalInB);
					if (lenSqr > (FLT_EPSILON*FLT_EPSILON))
					{
						tmpNormalInB /= sqrtf(lenSqr);
						float distance2 = -dist(tmpPointOnA,tmpPointOnB);
						//only replace valid penetrations when the result is deeper (check)
						if (!isValid || (distance2 < distance))
						{
							distance = distance2;
							pointOnA = tmpPointOnA;
							pointOnB = tmpPointOnB;
							normalInB = tmpNormalInB;
							isValid = true;
							m_lastUsedMethod = 3;
						} else
						{
							
						}
					} else
					{
						//isValid = false;
						m_lastUsedMethod = 4;
					}
				} else
				{
					m_lastUsedMethod = 5;
				}
				
			}
		}
	}

	if (isValid)
	{
#ifdef __SPU__
		//spu_printf("distance\n");
#endif //__CELLOS_LV2__


		Vectormath::Aos::Point3	tmpPtOnB=pointOnB+positionOffset;
		Vectormath::Aos::Point3	vmPtOnB(tmpPtOnB.getX(),tmpPtOnB.getY(),tmpPtOnB.getZ());
		Vectormath::Aos::Vector3	vmNormalOnB(normalInB.getX(),normalInB.getY(),normalInB.getZ());

		output.addContactPoint(
			vmNormalOnB,
			vmPtOnB,
			distance
            );
			

		//printf("gjk add:%f",distance);
	}
}




