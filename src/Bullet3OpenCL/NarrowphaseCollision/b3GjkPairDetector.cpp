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

#include "b3GjkPairDetector.h"
#include "Bullet3Common/b3Transform.h"
#include "b3VoronoiSimplexSolver.h"
#include "b3ConvexPolyhedronCL.h"
#include "b3VectorFloat4.h"
#include "b3GjkEpa.h"
#include "b3SupportMappings.h"

//must be above the machine epsilon
#define REL_ERROR2 b3Scalar(1.0e-6)

//temp globals, to improve GJK/EPA/penetration calculations
int gNumDeepPenetrationChecks2 = 0;
int gNumGjkChecks2 = 0;
int gGjkSeparatingAxis2=0;
int gEpaSeparatingAxis2=0;



b3GjkPairDetector::b3GjkPairDetector(b3VoronoiSimplexSolver* simplexSolver,b3GjkEpaSolver2*	penetrationDepthSolver)
:m_cachedSeparatingAxis(b3MakeVector3(b3Scalar(0.),b3Scalar(-1.),b3Scalar(0.))),
m_penetrationDepthSolver(penetrationDepthSolver),
m_simplexSolver(simplexSolver),
m_ignoreMargin(false),
m_lastUsedMethod(-1),
m_catchDegeneracies(1),
m_fixContactNormalDirection(1)
{
}




bool calcPenDepth( b3VoronoiSimplexSolver& simplexSolver,
											  const b3Transform&	transformA, const b3Transform&	transformB,
	const b3ConvexPolyhedronCL& hullA, const b3ConvexPolyhedronCL& hullB, 
	const b3AlignedObjectArray<b3Vector3>& verticesA,
	const b3AlignedObjectArray<b3Vector3>& verticesB,
											  b3Vector3& v, b3Vector3& wWitnessOnA, b3Vector3& wWitnessOnB)
{

	(void)v;
	(void)simplexSolver;

	b3Vector3	guessVector(transformB.getOrigin()-transformA.getOrigin());
	b3GjkEpaSolver2::sResults	results;
	

	if(b3GjkEpaSolver2::Penetration(transformA,transformB,&hullA,&hullB,verticesA,verticesB,guessVector,results))
	{
		wWitnessOnA = results.witnesses[0];
		wWitnessOnB = results.witnesses[1];
		v = results.normal;
		return true;		
	} 
	else
	{
		if(b3GjkEpaSolver2::Distance(transformA,transformB,&hullA,&hullB,verticesA,verticesB,guessVector,results))
		{
			wWitnessOnA = results.witnesses[0];
			wWitnessOnB = results.witnesses[1];
			v = results.normal;
			return false;
		}
	}

	return false;
}
#define dot3F4 b3Dot

inline void project(const b3ConvexPolyhedronCL& hull,  const float4& pos, const b3Quaternion& orn, const float4& dir, const b3AlignedObjectArray<b3Vector3>& vertices, b3Scalar& min, b3Scalar& max)
{
	min = FLT_MAX;
	max = -FLT_MAX;
	int numVerts = hull.m_numVertices;

	const float4 localDir = b3QuatRotate(orn.inverse(),dir);

	b3Scalar offset = dot3F4(pos,dir);

	for(int i=0;i<numVerts;i++)
	{
		//b3Vector3 pt = trans * vertices[m_vertexOffset+i];
		//b3Scalar dp = pt.dot(dir);
		b3Vector3 vertex = vertices[hull.m_vertexOffset+i];
		b3Scalar dp = dot3F4((float4&)vertices[hull.m_vertexOffset+i],localDir);
		//b3Assert(dp==dpL);
		if(dp < min)	min = dp;
		if(dp > max)	max = dp;
	}
	if(min>max)
	{
		b3Scalar tmp = min;
		min = max;
		max = tmp;
	}
	min += offset;
	max += offset;
}


static bool TestSepAxis(const b3ConvexPolyhedronCL& hullA, const b3ConvexPolyhedronCL& hullB, 
	const float4& posA,const b3Quaternion& ornA,
	const float4& posB,const b3Quaternion& ornB,
	float4& sep_axis, const b3AlignedObjectArray<b3Vector3>& verticesA,const b3AlignedObjectArray<b3Vector3>& verticesB,b3Scalar& depth)
{
	b3Scalar Min0,Max0;
	b3Scalar Min1,Max1;
	project(hullA,posA,ornA,sep_axis,verticesA, Min0, Max0);
	project(hullB,posB,ornB, sep_axis,verticesB, Min1, Max1);

	if(Max0<Min1 || Max1<Min0)
		return false;

	b3Scalar d0 = Max0 - Min1;
	b3Assert(d0>=0.0f);
	b3Scalar d1 = Max1 - Min0;
	b3Assert(d1>=0.0f);
	if (d0<d1)
	{
		depth = d0;
		sep_axis *=-1;
	} else
	{
		depth = d1;
		
	}
	
	return true;
}


bool getClosestPoints(b3GjkPairDetector* gjkDetector, const b3Transform&	transA, const b3Transform&	transB,
	const b3ConvexPolyhedronCL& hullA, const b3ConvexPolyhedronCL& hullB, 
	const b3AlignedObjectArray<b3Vector3>& verticesA,
	const b3AlignedObjectArray<b3Vector3>& verticesB,
	b3Scalar maximumDistanceSquared,
	b3Vector3& resultSepNormal,
	float& resultSepDistance,
	b3Vector3& resultPointOnB)
{
	//resultSepDistance = maximumDistanceSquared;

	gjkDetector->m_cachedSeparatingDistance = 0.f;

	b3Scalar distance=b3Scalar(0.);
	b3Vector3	normalInB= b3MakeVector3(b3Scalar(0.),b3Scalar(0.),b3Scalar(0.));
	b3Vector3 pointOnA,pointOnB;

	b3Transform localTransA = transA;
	b3Transform localTransB = transB;
	
	b3Vector3 positionOffset = b3MakeVector3(0,0,0);// = (localTransA.getOrigin() + localTransB.getOrigin()) * b3Scalar(0.5);
	localTransA.getOrigin() -= positionOffset;
	localTransB.getOrigin() -= positionOffset;

	bool check2d = false;//m_minkowskiA->isConvex2d() && m_minkowskiB->isConvex2d();

	b3Scalar marginA = 0.f;//m_marginA;
	b3Scalar marginB = 0.f;//m_marginB;

	gNumGjkChecks2++;


	//for CCD we don't use margins
	if (gjkDetector->m_ignoreMargin)
	{
		marginA = b3Scalar(0.);
		marginB = b3Scalar(0.);
	}

	gjkDetector->m_curIter = 0;
	int gGjkMaxIter = 1000;//this is to catch invalid input, perhaps check for #NaN?
	gjkDetector->m_cachedSeparatingAxis.setValue(1,1,1);//0,0,0);

	bool isValid = false;
	bool checkSimplex = false;
	bool checkPenetration = true;
	gjkDetector->m_degenerateSimplex = 0;

	gjkDetector->m_lastUsedMethod = -1;

	{
		b3Scalar squaredDistance = B3_LARGE_FLOAT;
		b3Scalar delta = -1e30f;//b3Scalar(0.);
		b3Scalar prevDelta = -1e30f;//b3Scalar(0.);
		
		b3Scalar margin = marginA + marginB;
		b3Scalar bestDeltaN = -1e30f;
		b3Vector3 bestSepAxis= b3MakeVector3(0,0,0);
		b3Vector3 bestPointOnA;
		b3Vector3 bestPointOnB;

		

		gjkDetector->m_simplexSolver->reset();
		
		for ( ; ; )
		//while (true)
		{

		
			b3Vector3 seperatingAxisInA = (-gjkDetector->m_cachedSeparatingAxis)* localTransA.getBasis();
			b3Vector3 seperatingAxisInB = gjkDetector->m_cachedSeparatingAxis* localTransB.getBasis();

			b3Vector3 pInA = localGetSupportVertexWithoutMargin(seperatingAxisInA,&hullA,verticesA);
			b3Vector3 qInB = localGetSupportVertexWithoutMargin(seperatingAxisInB,&hullB,verticesB);

			b3Vector3  pWorld = localTransA(pInA);	
			b3Vector3  qWorld = localTransB(qInB);

			{
				b3Scalar l2 = gjkDetector->m_cachedSeparatingAxis.length2();
				if (l2>B3_EPSILON*B3_EPSILON)
				{

					b3Vector3 testAxis = gjkDetector->m_cachedSeparatingAxis*(1.f/b3Sqrt(l2));
					float computedDepth=1e30f;
					if (!TestSepAxis(hullA,hullB,transA.getOrigin(),transA.getRotation(),
						transB.getOrigin(),transB.getRotation(),testAxis,verticesA,verticesB,computedDepth))
					{
						return false;
					}
			

				
					if(computedDepth<resultSepDistance)
					{
						if (testAxis.length2()>B3_EPSILON*B3_EPSILON)
						{
							resultSepDistance = computedDepth;
							resultSepNormal = testAxis;
						}
					}
				}
			}



			if (check2d)
			{
				pWorld[2] = 0.f;
				qWorld[2] = 0.f;
			}

			b3Vector3 w	= pWorld - qWorld;
			delta = gjkDetector->m_cachedSeparatingAxis.dot(w);
			if (delta>0)
				return false;
			b3Scalar deltaN = gjkDetector->m_cachedSeparatingAxis.normalized().dot(w.normalized());
			

			if (deltaN < bestDeltaN)
			{
				bestDeltaN = deltaN;
				//printf("new solution?\n");
				bestSepAxis = gjkDetector->m_cachedSeparatingAxis;
				gjkDetector->m_simplexSolver->compute_points(bestPointOnA, bestPointOnB);
			}
			prevDelta = delta;
			b3Scalar dist = 0;
			if (delta<0)
				dist = -b3Sqrt(b3Fabs(delta));
			else
				dist = b3Sqrt(delta);

			//printf("gjkDetector->m_cachedSeparatingAxis = %f,%f,%f delta/dist = %f\n",gjkDetector->m_cachedSeparatingAxis.x,gjkDetector->m_cachedSeparatingAxis.y,gjkDetector->m_cachedSeparatingAxis.z,dist);
			// potential exit, they don't overlap
			if ((delta > b3Scalar(0.0)) && (delta * delta > squaredDistance * maximumDistanceSquared)) 
			{
				gjkDetector->m_degenerateSimplex = 10;
				checkSimplex=true;
				//checkPenetration = false;
				break;
			}

			//exit 0: the new point is already in the simplex, or we didn't come any closer
			if (gjkDetector->m_simplexSolver->inSimplex(w))
			{
				gjkDetector->m_degenerateSimplex = 1;
				checkSimplex = true;
				break;
			}
			// are we getting any closer ?
			b3Scalar f0 = squaredDistance - delta;
			b3Scalar f1 = squaredDistance * REL_ERROR2;

			if (f0 <= f1)
			{
				if (f0 <= b3Scalar(0.))
				{
					gjkDetector->m_degenerateSimplex = 2;
				} else
				{
					gjkDetector->m_degenerateSimplex = 11;
				}
				checkSimplex = true;
				break;
			}

			//add current vertex to simplex
			gjkDetector->m_simplexSolver->addVertex(w, pWorld, qWorld);
			b3Vector3 newCachedSeparatingAxis;

			//calculate the closest point to the origin (update vector v)
			if (!gjkDetector->m_simplexSolver->closest(newCachedSeparatingAxis))
			{
				gjkDetector->m_degenerateSimplex = 3;
				checkSimplex = true;
				break;
			}

			if(0)//newCachedSeparatingAxis.length2()<REL_ERROR2)
            {
				if (delta<bestDeltaN)
				{
					gjkDetector->m_cachedSeparatingAxis = newCachedSeparatingAxis;
				}
                gjkDetector->m_degenerateSimplex = 6;
                checkSimplex = true;
                break;
            }

			b3Scalar previousSquaredDistance = squaredDistance;
			squaredDistance = newCachedSeparatingAxis.length2();
		
			b3Vector3 sepAxis=newCachedSeparatingAxis.normalized();

			


			//redundant m_simplexSolver->compute_points(pointOnA, pointOnB);

			//are we getting any closer ?
			if (previousSquaredDistance - squaredDistance <= B3_EPSILON * previousSquaredDistance) 
			{ 
				checkSimplex = true;
				gjkDetector->m_degenerateSimplex = 12;
				
				break;
			}

			gjkDetector->m_cachedSeparatingAxis = newCachedSeparatingAxis;

			{
				b3Scalar l2 = gjkDetector->m_cachedSeparatingAxis.length2();
				if (l2>B3_EPSILON*B3_EPSILON)
				{

					b3Vector3 testAxis = gjkDetector->m_cachedSeparatingAxis*(1.f/b3Sqrt(l2));
					float computedDepth=1e30f;
					if (!TestSepAxis(hullA,hullB,transA.getOrigin(),transA.getRotation(),
						transB.getOrigin(),transB.getRotation(),testAxis,verticesA,verticesB,computedDepth))
					{
						return false;
					}
			

				
					if(computedDepth<resultSepDistance)
					{
						if (testAxis.length2()>B3_EPSILON*B3_EPSILON)
						{
							resultSepDistance = computedDepth;
							resultSepNormal = testAxis;
						}
					}
				}
			}

			//degeneracy, this is typically due to invalid/uninitialized worldtransforms for a btCollisionObject   
            if (gjkDetector->m_curIter++ > gGjkMaxIter)   
            {   
                    break;   
            } 


			bool check = (!gjkDetector->m_simplexSolver->fullSimplex());

			float projectedDepth = 0;
			
			if (delta<0)
			{
				projectedDepth = -b3Sqrt(b3Fabs(delta));
			} else
			{
				projectedDepth = b3Sqrt(delta);
			}
			
			//printf("dist2 = %f dist= %f projectedDepth = %f\n", squaredDistance,b3Sqrt(squaredDistance),projectedDepth);

			if (!check)
			{
				gjkDetector->m_degenerateSimplex = 13;
				break;
			}
		}

		if (checkSimplex)
		{
			if (bestSepAxis.length2())
			{
				pointOnA = bestPointOnA;
				pointOnB = bestPointOnB;
				gjkDetector->m_cachedSeparatingAxis = bestSepAxis;
			} else
			{
				gjkDetector->m_simplexSolver->compute_points(pointOnA, pointOnB);
			}
			
			normalInB = gjkDetector->m_cachedSeparatingAxis;
			b3Scalar lenSqr =gjkDetector->m_cachedSeparatingAxis.length2();
			
			//valid normal
			if (lenSqr < 0.0001)
			{
				gjkDetector->m_degenerateSimplex = 5;
			} 
			if (lenSqr > B3_EPSILON*B3_EPSILON)
			{
				b3Scalar rlen = b3Scalar(1.) / b3Sqrt(lenSqr );
				normalInB *= rlen; //normalize
				b3Scalar s = b3Sqrt(squaredDistance);
			
				b3Assert(s > b3Scalar(0.0));
				pointOnA -= gjkDetector->m_cachedSeparatingAxis * (marginA / s);
				pointOnB += gjkDetector->m_cachedSeparatingAxis * (marginB / s);
				distance = ((b3Scalar(1.)/rlen) - margin);
				isValid = true;
				
				gjkDetector->m_lastUsedMethod = 1;
			} else
			{
				gjkDetector->m_lastUsedMethod = 2;
			}
		}

		bool catchDegeneratePenetrationCase = (gjkDetector->m_catchDegeneracies && gjkDetector->m_penetrationDepthSolver && gjkDetector->m_degenerateSimplex && ((distance+margin) < 0.01));

		//if (checkPenetration && !isValid)
		if (checkPenetration && (!isValid || catchDegeneratePenetrationCase ))
		{
			//penetration case

			//if there is no way to handle penetrations, bail out
			if (gjkDetector->m_penetrationDepthSolver)
			{
				// Penetration depth case.
				b3Vector3 tmpPointOnA,tmpPointOnB;
				
				gNumDeepPenetrationChecks2++;
				gjkDetector->m_cachedSeparatingAxis.setZero();

				bool isValid2 = calcPenDepth( 
					*gjkDetector->m_simplexSolver, 
					transA,transB,hullA,hullB,verticesA,verticesB,
					gjkDetector->m_cachedSeparatingAxis, tmpPointOnA, tmpPointOnB
					);


				if (isValid2)
				{
					b3Vector3 tmpNormalInB = tmpPointOnB-tmpPointOnA;
					b3Scalar lenSqr = tmpNormalInB.length2();
					if (lenSqr <= (B3_EPSILON*B3_EPSILON))
					{
						tmpNormalInB = gjkDetector->m_cachedSeparatingAxis;
						lenSqr = gjkDetector->m_cachedSeparatingAxis.length2();
					}

					if (lenSqr > (B3_EPSILON*B3_EPSILON))
					{
						tmpNormalInB /= b3Sqrt(lenSqr);
						b3Scalar distance2 = -(tmpPointOnA-tmpPointOnB).length();
						//only replace valid penetrations when the result is deeper (check)
						if (!isValid || (distance2 < distance))
						{
							distance = distance2;
							pointOnA = tmpPointOnA;
							pointOnB = tmpPointOnB;
							normalInB = tmpNormalInB;
							isValid = true;
							gjkDetector->m_lastUsedMethod = 3;
						} else
						{
							gjkDetector->m_lastUsedMethod = 8;
						}
					} else
					{
						gjkDetector->m_lastUsedMethod = 9;
					}
				} else

				{
					///this is another degenerate case, where the initial GJK calculation reports a degenerate case
					///EPA reports no penetration, and the second GJK (using the supporting vector without margin)
					///reports a valid positive distance. Use the results of the second GJK instead of failing.
					///thanks to Jacob.Langford for the reproduction case
					///http://code.google.com/p/bullet/issues/detail?id=250

				
					if (gjkDetector->m_cachedSeparatingAxis.length2() > b3Scalar(0.))
					{
						b3Scalar distance2 = (tmpPointOnA-tmpPointOnB).length()-margin;
						//only replace valid distances when the distance is less
						if (!isValid || (distance2 < distance))
						{
							distance = distance2;
							pointOnA = tmpPointOnA;
							pointOnB = tmpPointOnB;
							pointOnA -= gjkDetector->m_cachedSeparatingAxis * marginA ;
							pointOnB += gjkDetector->m_cachedSeparatingAxis * marginB ;
							normalInB = gjkDetector->m_cachedSeparatingAxis;
							normalInB.normalize();
							isValid = true;
							gjkDetector->m_lastUsedMethod = 6;
						} else
						{
							gjkDetector->m_lastUsedMethod = 5;
						}
					}
				}
				
			}

		}
	}

	

	if (isValid && ((distance < 0) || (distance*distance < maximumDistanceSquared)))
	{

		if (gjkDetector->m_fixContactNormalDirection)
		{
			///@workaround for sticky convex collisions
			//in some degenerate cases (usually when the use uses very small margins) 
			//the contact normal is pointing the wrong direction
			//so fix it now (until we can deal with all degenerate cases in GJK and EPA)
			//contact normals need to point from B to A in all cases, so we can simply check if the contact normal really points from B to A
			//We like to use a dot product of the normal against the difference of the centroids, 
			//once the centroid is available in the API
			//until then we use the center of the aabb to approximate the centroid
			b3Vector3 aabbMin,aabbMax;
			//m_minkowskiA->getAabb(localTransA,aabbMin,aabbMax);
			//b3Vector3 posA  = (aabbMax+aabbMin)*b3Scalar(0.5);
		
			//m_minkowskiB->getAabb(localTransB,aabbMin,aabbMax);
			//b3Vector3 posB = (aabbMin+aabbMax)*b3Scalar(0.5);


			b3Vector3 diff = transA.getOrigin()-transB.getOrigin();
			if (diff.dot(normalInB) < 0.f)
				normalInB *= -1.f;
		}
		gjkDetector->m_cachedSeparatingAxis = normalInB;
		gjkDetector->m_cachedSeparatingDistance = distance;

		{
				b3Scalar l2 = gjkDetector->m_cachedSeparatingAxis.length2();
				if (l2>B3_EPSILON*B3_EPSILON)
				{

					b3Vector3 testAxis = gjkDetector->m_cachedSeparatingAxis*(1.f/b3Sqrt(l2));
					float computedDepth=1e30f;
					if (!TestSepAxis(hullA,hullB,transA.getOrigin(),transA.getRotation(),
						transB.getOrigin(),transB.getRotation(),testAxis,verticesA,verticesB,computedDepth))
					{
						return false;
					}
			

				
					if(computedDepth<resultSepDistance)
					{
						if (testAxis.length2()>B3_EPSILON*B3_EPSILON)
						{
							resultSepDistance = computedDepth;
							resultSepNormal = testAxis;
						}
					}
				}
			}

		
		resultSepNormal = normalInB;
		//printf("normalInB = %f,%f,%f, distance = %f\n",normalInB.x,normalInB.y,normalInB.z,distance);
		resultSepDistance = distance;
		

		b3Scalar lsqr = resultSepNormal.length2();
		b3Assert(lsqr>B3_EPSILON*B3_EPSILON);
		if (lsqr<B3_EPSILON*B3_EPSILON)
			return false;
		resultSepNormal *= 1.f/b3Sqrt(lsqr);

#if 0
		float dot = resultSepNormal.dot(b3Vector3(0,-1,0));
		//float dot = b3Vector3(-0.7,-0.6,-0.2).dot(b3Vector3(0,-1,0));

		static float minDot = 1e30f;

		if (dot<minDot)
		{
			//printf("minDot = %f\n", dot);
			minDot=dot;
		}

		//printf("gNumGjkChecks = %d, gNumDeepPenetrationChecks = %d, minDot = %f\n", gNumGjkChecks,gNumDeepPenetrationChecks,minDot);
		if (0)//dot<0.64)
		{
			{
				b3Scalar l2 = resultSepNormal.length2();
				if (l2>B3_EPSILON*B3_EPSILON)
				{

					b3Vector3 testAxis = gjkDetector->m_cachedSeparatingAxis*(1./b3Sqrt(l2));
					float computedDepth=1e30f;
					if (!TestSepAxis(hullA,hullB,transA.getOrigin(),transA.getRotation(),
						transB.getOrigin(),transB.getRotation(),testAxis,verticesA,verticesB,computedDepth))
					{
						return false;
					}
			

				
					if(computedDepth<resultSepDistance)
					{
						if (testAxis.length2()>B3_EPSILON*B3_EPSILON)
						{
							resultSepDistance = computedDepth;
							resultSepNormal = testAxis;
						}
					}
				}
			}
		}
#endif
		resultPointOnB = pointOnB+positionOffset;
		return true;
	}
	return false;

}





