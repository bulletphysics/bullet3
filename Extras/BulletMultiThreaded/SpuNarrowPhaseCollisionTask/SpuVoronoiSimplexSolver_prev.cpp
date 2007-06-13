
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
	
	Elsevier CDROM license agreements grants nonexclusive license to use the software
	for any purpose, commercial or non-commercial as long as the following credit is included
	identifying the original source of the software:

	Parts of the source are "from the book Real-Time Collision Detection by
	Christer Ericson, published by Morgan Kaufmann Publishers,
	(c) 2005 Elsevier Inc."
		
*/


// Needed to be able to DMA.
#ifdef WIN32
#include "SpuFakeDma.h"
#else
#include "SPU_Common/SpuDefines.h"
#include <cell/spurs/common.h>
#include <cell/dma.h>
#endif //WIN32



#include "SpuVoronoiSimplexSolver.h"
#include "LinearMath/btScalar.h"

#include <assert.h>
#include <stdio.h>


#define VERTA  0
#define VERTB  1
#define VERTC  2
#define VERTD  3

#define CATCH_DEGENERATE_TETRAHEDRON 1
void	SpuVoronoiSimplexSolver::removeVertex(int index)
{
	assert(m_numVertices>0);
	m_numVertices--;
	m_simplexVectorW[index] = m_simplexVectorW[m_numVertices];
	m_simplexPointsP[index] = m_simplexPointsP[m_numVertices];
	m_simplexPointsQ[index] = m_simplexPointsQ[m_numVertices];
//	m_VertexIndexA[index] = m_VertexIndexA[m_numVertices];
//	m_VertexIndexB[index] = m_VertexIndexB[m_numVertices];
}

void	SpuVoronoiSimplexSolver::reduceVertices (const SpuUsageBitfield& usedVerts)
{
	if ((numVertices() >= 4) && (!usedVerts.usedVertexD))
		removeVertex(3);

	if ((numVertices() >= 3) && (!usedVerts.usedVertexC))
		removeVertex(2);

	if ((numVertices() >= 2) && (!usedVerts.usedVertexB))
		removeVertex(1);
	
	if ((numVertices() >= 1) && (!usedVerts.usedVertexA))
		removeVertex(0);

}





//clear the simplex, remove all the vertices
void SpuVoronoiSimplexSolver::reset()
{
	m_cachedValidClosest = false;
	m_numVertices = 0;
	m_needsUpdate = true;
	m_lastW = Vectormath::Aos::Vector3(float(1e30),float(1e30),float(1e30));
	m_cachedBC.reset();
}



	//add a vertex
void SpuVoronoiSimplexSolver::addVertex(const Vectormath::Aos::Vector3& w, const Vectormath::Aos::Point3& p, const Vectormath::Aos::Point3& q)//, int vertexIndexA, int vertexIndexB)
{
	m_lastW = w;
	m_needsUpdate = true;

	m_simplexVectorW[m_numVertices] = w;
	m_simplexPointsP[m_numVertices] = Vectormath::Aos::Vector3(p);
	m_simplexPointsQ[m_numVertices] = Vectormath::Aos::Vector3(q);

	//m_VertexIndexA[m_numVertices] = vertexIndexA;
	//m_VertexIndexB[m_numVertices] = vertexIndexB;

	m_numVertices++;
}

bool	SpuVoronoiSimplexSolver::updateClosestVectorAndPoints()
{
	
	if (m_needsUpdate)
	{
		m_cachedBC.reset();

		m_needsUpdate = false;

		switch (numVertices())
		{
		case 0:
				m_cachedValidClosest = false;
				break;
		case 1:
			{
				m_cachedP1 = m_simplexPointsP[0];
				m_cachedP2 = m_simplexPointsQ[0];
				m_cachedV = m_cachedP1-m_cachedP2; //== m_simplexVectorW[0]
				m_cachedBC.reset();
				m_cachedBC.setBarycentricCoordinates(float(1.),float(0.),float(0.),float(0.));
				m_cachedValidClosest = m_cachedBC.isValid();
				break;
			};
		case 2:
			{
			//closest point origin from line segment
					const Vectormath::Aos::Vector3& from = m_simplexVectorW[0];
					const Vectormath::Aos::Vector3& to = m_simplexVectorW[1];
					Vectormath::Aos::Vector3 nearest;

					Vectormath::Aos::Vector3 p (float(0.),float(0.),float(0.));
					Vectormath::Aos::Vector3 diff = p - from;
					Vectormath::Aos::Vector3 v = to - from;
					float t = dot(v, diff);
					
					if (t > 0) {
						float dotVV = dot(v, v);
						if (t < dotVV) {
							t /= dotVV;
							diff -= t*v;
							m_cachedBC.m_usedVertices.usedVertexA = true;
							m_cachedBC.m_usedVertices.usedVertexB = true;
						} else {
							t = 1;
							diff -= v;
							//reduce to 1 point
							m_cachedBC.m_usedVertices.usedVertexB = true;
						}
					} else
					{
						t = 0;
						//reduce to 1 point
						m_cachedBC.m_usedVertices.usedVertexA = true;
					}
					m_cachedBC.setBarycentricCoordinates(1-t,t);
					nearest = from + t*v;

					m_cachedP1 = m_simplexPointsP[0] + t * (m_simplexPointsP[1] - m_simplexPointsP[0]);
					m_cachedP2 = m_simplexPointsQ[0] + t * (m_simplexPointsQ[1] - m_simplexPointsQ[0]);
					m_cachedV = m_cachedP1 - m_cachedP2;

					reduceVertices(m_cachedBC.m_usedVertices);

					m_cachedValidClosest = m_cachedBC.isValid();
					break;
			}
		case 3:
			{
				//closest point origin from triangle
				Vectormath::Aos::Vector3 p (float(0.),float(0.),float(0.));
				
				const Vectormath::Aos::Vector3& a = m_simplexVectorW[0];
				const Vectormath::Aos::Vector3& b = m_simplexVectorW[1];
				const Vectormath::Aos::Vector3& c = m_simplexVectorW[2];

				closestPtPointTriangle(p,a,b,c,m_cachedBC);
				m_cachedP1 = m_simplexPointsP[0] * m_cachedBC.m_barycentricCoords[0] +
								m_simplexPointsP[1] * m_cachedBC.m_barycentricCoords[1] +
								m_simplexPointsP[2] * m_cachedBC.m_barycentricCoords[2];

				m_cachedP2 = m_simplexPointsQ[0] * m_cachedBC.m_barycentricCoords[0] +
					m_simplexPointsQ[1] * m_cachedBC.m_barycentricCoords[1] +
					m_simplexPointsQ[2] * m_cachedBC.m_barycentricCoords[2];

				m_cachedV = m_cachedP1-m_cachedP2;

				reduceVertices (m_cachedBC.m_usedVertices);
				m_cachedValidClosest =  m_cachedBC.isValid();

				break;
			}
		case 4:
			{

				
				Vectormath::Aos::Vector3 p (float(0.),float(0.),float(0.));
				
				const Vectormath::Aos::Vector3& a = m_simplexVectorW[0];
				const Vectormath::Aos::Vector3& b = m_simplexVectorW[1];
				const Vectormath::Aos::Vector3& c = m_simplexVectorW[2];
				const Vectormath::Aos::Vector3& d = m_simplexVectorW[3];

				bool hasSeperation = closestPtPointTetrahedron(p,a,b,c,d,m_cachedBC);

				if (hasSeperation)
				{

					m_cachedP1 = m_simplexPointsP[0] * m_cachedBC.m_barycentricCoords[0] +
						m_simplexPointsP[1] * m_cachedBC.m_barycentricCoords[1] +
						m_simplexPointsP[2] * m_cachedBC.m_barycentricCoords[2] +
						m_simplexPointsP[3] * m_cachedBC.m_barycentricCoords[3];

					m_cachedP2 = m_simplexPointsQ[0] * m_cachedBC.m_barycentricCoords[0] +
						m_simplexPointsQ[1] * m_cachedBC.m_barycentricCoords[1] +
						m_simplexPointsQ[2] * m_cachedBC.m_barycentricCoords[2] +
						m_simplexPointsQ[3] * m_cachedBC.m_barycentricCoords[3];

					m_cachedV = m_cachedP1-m_cachedP2;
					reduceVertices (m_cachedBC.m_usedVertices);
				} else
				{
//					printf("sub distance got penetration\n");

					if (m_cachedBC.m_degenerate)
					{
						m_cachedValidClosest = false;
					} else
					{
						m_cachedValidClosest = true;
						//degenerate case == false, penetration = true + zero
                        m_cachedV = Vectormath::Aos::Vector3(float(0.),float(0.),float(0.));
					}
					break;
				}

				m_cachedValidClosest = m_cachedBC.isValid();

				//closest point origin from tetrahedron
				break;
			}
		default:
			{
				m_cachedValidClosest = false;
			}
		};
	}

	return m_cachedValidClosest;

}

//return/calculate the closest vertex
bool SpuVoronoiSimplexSolver::closest(Vectormath::Aos::Vector3& v)
{
	bool succes = updateClosestVectorAndPoints();
	v = m_cachedV;
	return succes;
}



float SpuVoronoiSimplexSolver::maxVertex()
{
	int i, numverts = numVertices();
	float maxV = float(0.);
	for (i=0;i<numverts;i++)
	{
		float curLen2 = lengthSqr(m_simplexVectorW[i]);
		if (maxV < curLen2)
			maxV = curLen2;
	}
	return maxV;
}



	//return the current simplex
int SpuVoronoiSimplexSolver::getSimplex(Vectormath::Aos::Vector3 *pBuf, Vectormath::Aos::Vector3 *qBuf, Vectormath::Aos::Vector3 *yBuf) const
{
	int i;
	for (i=0;i<numVertices();i++)
	{
		yBuf[i] = m_simplexVectorW[i];
		pBuf[i] = m_simplexPointsP[i];
		qBuf[i] = m_simplexPointsQ[i];
	}
	return numVertices();
}




bool SpuVoronoiSimplexSolver::inSimplex(const Vectormath::Aos::Vector3& w)
{
	bool found = false;
	int i, numverts = numVertices();
	//float maxV = float(0.);
	
	//w is in the current (reduced) simplex
	for (i=0;i<numverts;i++)
	{
        // TODO: find a better way to determine equality
		if (m_simplexVectorW[i].getX() == w.getX() &&
            m_simplexVectorW[i].getY() == w.getY() &&
            m_simplexVectorW[i].getZ() == w.getZ())
			found = true;
	}

	//check in case lastW is already removed
        // TODO: find a better way to determine equality
	if (w.getX() == m_lastW.getX() &&
        w.getY() == m_lastW.getY() &&
        w.getZ() == m_lastW.getZ())
		return true;
    	
	return found;
}

void SpuVoronoiSimplexSolver::backup_closest(Vectormath::Aos::Vector3& v) 
{
	v = m_cachedV;
}


bool SpuVoronoiSimplexSolver::emptySimplex() const 
{
	return (numVertices() == 0);

}

void SpuVoronoiSimplexSolver::compute_points(Vectormath::Aos::Point3& p1, Vectormath::Aos::Point3& p2) 
{
	updateClosestVectorAndPoints();
	p1 = Vectormath::Aos::Point3(m_cachedP1);
	p2 = Vectormath::Aos::Point3(m_cachedP2);

}




bool	SpuVoronoiSimplexSolver::closestPtPointTriangle(const Vectormath::Aos::Vector3& p, const Vectormath::Aos::Vector3& a, const Vectormath::Aos::Vector3& b, const Vectormath::Aos::Vector3& c,SpuSubSimplexClosestResult& result)
{
	result.m_usedVertices.reset();

    // Check if P in vertex region outside A
    Vectormath::Aos::Vector3 ab = b - a;
    Vectormath::Aos::Vector3 ac = c - a;
    Vectormath::Aos::Vector3 ap = p - a;
    float d1 = dot(ab,ap);
    float d2 = dot(ac,ap);
    if (d1 <= float(0.0) && d2 <= float(0.0)) 
	{
		result.m_closestPointOnSimplex = Vectormath::Aos::Point3(a);
		result.m_usedVertices.usedVertexA = true;
		result.setBarycentricCoordinates(1,0,0);
		return true;// a; // barycentric coordinates (1,0,0)
	}

    // Check if P in vertex region outside B
    Vectormath::Aos::Vector3 bp = p - b;
    float d3 = dot(ab,bp);
    float d4 = dot(ac,bp);
    if (d3 >= float(0.0) && d4 <= d3) 
	{
		result.m_closestPointOnSimplex = Vectormath::Aos::Point3(b);
		result.m_usedVertices.usedVertexB = true;
		result.setBarycentricCoordinates(0,1,0);

		return true; // b; // barycentric coordinates (0,1,0)
	}
    // Check if P in edge region of AB, if so return projection of P onto AB
    float vc = d1*d4 - d3*d2;
    if (vc <= float(0.0) && d1 >= float(0.0) && d3 <= float(0.0)) {
        float v = d1 / (d1 - d3);
		result.m_closestPointOnSimplex = Vectormath::Aos::Point3(a + v * ab);
		result.m_usedVertices.usedVertexA = true;
		result.m_usedVertices.usedVertexB = true;
		result.setBarycentricCoordinates(1-v,v,0);
		return true;
        //return a + v * ab; // barycentric coordinates (1-v,v,0)
    }

    // Check if P in vertex region outside C
    Vectormath::Aos::Vector3 cp = p - c;
    float d5 = dot(ab,cp);
    float d6 = dot(ac,cp);
    if (d6 >= float(0.0) && d5 <= d6) 
	{
		result.m_closestPointOnSimplex = Vectormath::Aos::Point3(c);
		result.m_usedVertices.usedVertexC = true;
		result.setBarycentricCoordinates(0,0,1);
		return true;//c; // barycentric coordinates (0,0,1)
	}

    // Check if P in edge region of AC, if so return projection of P onto AC
    float vb = d5*d2 - d1*d6;
    if (vb <= float(0.0) && d2 >= float(0.0) && d6 <= float(0.0)) {
        float w = d2 / (d2 - d6);
		result.m_closestPointOnSimplex = Vectormath::Aos::Point3(a + w * ac);
		result.m_usedVertices.usedVertexA = true;
		result.m_usedVertices.usedVertexC = true;
		result.setBarycentricCoordinates(1-w,0,w);
		return true;
        //return a + w * ac; // barycentric coordinates (1-w,0,w)
    }

    // Check if P in edge region of BC, if so return projection of P onto BC
    float va = d3*d6 - d5*d4;
    if (va <= float(0.0) && (d4 - d3) >= float(0.0) && (d5 - d6) >= float(0.0)) {
        float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		
		result.m_closestPointOnSimplex = Vectormath::Aos::Point3(b + w * (c - b));
		result.m_usedVertices.usedVertexB = true;
		result.m_usedVertices.usedVertexC = true;
		result.setBarycentricCoordinates(0,1-w,w);
		return true;		
       // return b + w * (c - b); // barycentric coordinates (0,1-w,w)
    }

    // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
    float denom = float(1.0) / (va + vb + vc);
    float v = vb * denom;
    float w = vc * denom;
    
	result.m_closestPointOnSimplex = Vectormath::Aos::Point3(a + ab * v + ac * w);
	result.m_usedVertices.usedVertexA = true;
	result.m_usedVertices.usedVertexB = true;
	result.m_usedVertices.usedVertexC = true;
	result.setBarycentricCoordinates(1-v-w,v,w);
	
	return true;
//	return a + ab * v + ac * w; // = u*a + v*b + w*c, u = va * denom = float(1.0) - v - w
}


// This is specifically just removing duplicate indices.
int SpuVoronoiSimplexSolver::RemoveDegenerateIndices (const int* inArray, int numIndices, int* outArray) const
{
	int outIndex = 0;
	for (int firstIndex=0; firstIndex<numIndices; firstIndex++)
	{
		bool duplicate = false;
		for (int secondIndex=0; secondIndex<firstIndex; secondIndex++)
		{
			if (inArray[secondIndex]==inArray[firstIndex])
			{
				duplicate = true;
				break;
			}
		}

		if (!duplicate)
		{
			outArray[outIndex++] = inArray[firstIndex];
		}
	}

	return outIndex;
}




/// Test if point p and d lie on opposite sides of plane through abc
int SpuVoronoiSimplexSolver::pointOutsideOfPlane(const Vectormath::Aos::Vector3& p, const Vectormath::Aos::Vector3& a, const Vectormath::Aos::Vector3& b, const Vectormath::Aos::Vector3& c, const Vectormath::Aos::Vector3& d)
{
	Vectormath::Aos::Vector3 normal = cross(b-a,c-a);

    float signp = dot(p - a, normal); // [AP AB AC]
    float signd = dot(d - a, normal); // [AD AB AC]

#ifdef CATCH_DEGENERATE_TETRAHEDRON
	if (signd * signd < (float(1e-4) * float(1e-4)))
	{
//		printf("affine dependent/degenerate\n");//
		return -1;
	}
#endif
	// Points on opposite sides if expression signs are opposite
    return signp * signd < float(0.);
}


bool	SpuVoronoiSimplexSolver::closestPtPointTetrahedron(const Vectormath::Aos::Vector3& p, const Vectormath::Aos::Vector3& a, const Vectormath::Aos::Vector3& b, const Vectormath::Aos::Vector3& c, const Vectormath::Aos::Vector3& d, SpuSubSimplexClosestResult& finalResult)
{
	SpuSubSimplexClosestResult tempResult;

    // Start out assuming point inside all halfspaces, so closest to itself
	finalResult.m_closestPointOnSimplex = Vectormath::Aos::Point3(p);
	finalResult.m_usedVertices.reset();
    finalResult.m_usedVertices.usedVertexA = true;
	finalResult.m_usedVertices.usedVertexB = true;
	finalResult.m_usedVertices.usedVertexC = true;
	finalResult.m_usedVertices.usedVertexD = true;

	// Check only the tetrahedron faces that are closest to p.  We do that by checking each face (which itself is a triangle) to see if the excluded
	//   vertex (the one vertex that isn't part of that face) is on the other side of that face from the point p.
    int pointOutsideABC = pointOutsideOfPlane(p, a, b, c, d);
	int pointOutsideACD = pointOutsideOfPlane(p, a, c, d, b);
  	int	pointOutsideADB = pointOutsideOfPlane(p, a, d, b, c);
	int	pointOutsideBDC = pointOutsideOfPlane(p, b, d, c, a);

   if (pointOutsideABC < 0 || pointOutsideACD < 0 || pointOutsideADB < 0 || pointOutsideBDC < 0)
   {
	   finalResult.m_degenerate = true;
	   return false;
   }

   if (!pointOutsideABC  && !pointOutsideACD && !pointOutsideADB && !pointOutsideBDC)
	 {
		 return false;
	 }


    float bestSqDist = 1e30f;//FLT_MAX;
    // If point outside face abc then compute closest point on abc
	if (pointOutsideABC) 
	{
        closestPtPointTriangle(p, a, b, c,tempResult);
        Vectormath::Aos::Vector3 q = Vectormath::Aos::Vector3(tempResult.m_closestPointOnSimplex);

        float sqDist = dot(q - p, q - p);
        // Update best closest point if (squared) distance is less than current best
        //if (sqDist < bestSqDist)
		btAssert(sqDist < bestSqDist);		// This has to be true; we haven't actually tested any other combinations.
		{
			bestSqDist = sqDist;
            finalResult.m_closestPointOnSimplex = Vectormath::Aos::Point3(q);
			//convert result bitmask!
			finalResult.m_usedVertices.reset();
			finalResult.m_usedVertices.usedVertexA = tempResult.m_usedVertices.usedVertexA;
			finalResult.m_usedVertices.usedVertexB = tempResult.m_usedVertices.usedVertexB;
			finalResult.m_usedVertices.usedVertexC = tempResult.m_usedVertices.usedVertexC;
			finalResult.setBarycentricCoordinates(
					tempResult.m_barycentricCoords[VERTA],
					tempResult.m_barycentricCoords[VERTB],
					tempResult.m_barycentricCoords[VERTC],
					0
			);

		}
    }
  

	// Repeat test for face acd
	if (pointOutsideACD) 
	{
        closestPtPointTriangle(p, a, c, d,tempResult);
		Vectormath::Aos::Vector3 q = Vectormath::Aos::Vector3(tempResult.m_closestPointOnSimplex);
		//convert result bitmask!

        float sqDist = dot(q - p, q - p);
        if (sqDist < bestSqDist) 
		{
			bestSqDist = sqDist;
			finalResult.m_closestPointOnSimplex = Vectormath::Aos::Point3(q);
			finalResult.m_usedVertices.reset();
			finalResult.m_usedVertices.usedVertexA = tempResult.m_usedVertices.usedVertexA;
			finalResult.m_usedVertices.usedVertexC = tempResult.m_usedVertices.usedVertexB;
			finalResult.m_usedVertices.usedVertexD = tempResult.m_usedVertices.usedVertexC;
			finalResult.setBarycentricCoordinates(
					tempResult.m_barycentricCoords[VERTA],
					0,
					tempResult.m_barycentricCoords[VERTB],
					tempResult.m_barycentricCoords[VERTC]
			);

		}
    }
    // Repeat test for face adb

	
	if (pointOutsideADB)
	{
		closestPtPointTriangle(p, a, d, b,tempResult);
		Vectormath::Aos::Vector3 q = Vectormath::Aos::Vector3(tempResult.m_closestPointOnSimplex);
		//convert result bitmask!

        float sqDist = dot(q - p, q - p);
        if (sqDist < bestSqDist) 
		{
			bestSqDist = sqDist;
			finalResult.m_closestPointOnSimplex = Vectormath::Aos::Point3(q);
			finalResult.m_usedVertices.reset();
			finalResult.m_usedVertices.usedVertexA = tempResult.m_usedVertices.usedVertexA;
			finalResult.m_usedVertices.usedVertexD = tempResult.m_usedVertices.usedVertexB;
			finalResult.m_usedVertices.usedVertexB = tempResult.m_usedVertices.usedVertexC;
			finalResult.setBarycentricCoordinates(
					tempResult.m_barycentricCoords[VERTA],
					tempResult.m_barycentricCoords[VERTC],
					0,
					tempResult.m_barycentricCoords[VERTB]
			);

		}
    }
    // Repeat test for face bdc
    

	if (pointOutsideBDC)
	{
        closestPtPointTriangle(p, b, d, c,tempResult);
		Vectormath::Aos::Vector3 q = Vectormath::Aos::Vector3(tempResult.m_closestPointOnSimplex);
		//convert result bitmask!
        float sqDist = dot(q - p, q - p);
        if (sqDist < bestSqDist) 
		{
			bestSqDist = sqDist;
			finalResult.m_closestPointOnSimplex = Vectormath::Aos::Point3(q);
			finalResult.m_usedVertices.reset();
			finalResult.m_usedVertices.usedVertexB = tempResult.m_usedVertices.usedVertexA;
			finalResult.m_usedVertices.usedVertexD = tempResult.m_usedVertices.usedVertexB;
			finalResult.m_usedVertices.usedVertexC = tempResult.m_usedVertices.usedVertexC;

			finalResult.setBarycentricCoordinates(
					0,
					tempResult.m_barycentricCoords[VERTA],
					tempResult.m_barycentricCoords[VERTC],
					tempResult.m_barycentricCoords[VERTB]
			);

		}
    }

	//help! we ended up full !
	
	if (finalResult.m_usedVertices.usedVertexA &&
		finalResult.m_usedVertices.usedVertexB &&
		finalResult.m_usedVertices.usedVertexC &&
		finalResult.m_usedVertices.usedVertexD) 
	{
		return true;
	}

    return true;
}


