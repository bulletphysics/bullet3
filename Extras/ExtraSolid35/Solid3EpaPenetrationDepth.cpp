/*
 * SOLID - Software Library for Interference Detection
 * 
 * Copyright (C) 2001-2003  Dtecta.  All rights reserved.
 *
 * This library may be distributed under the terms of the Q Public License
 * (QPL) as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE.QPL included in the packaging of this file.
 *
 * This library may be distributed and/or modified under the terms of the
 * GNU bteral Public License (GPL) version 2 as published by the Free Software
 * Foundation and appearing in the file LICENSE.GPL included in the
 * packaging of this file.
 *
 * This library is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Commercial use or any other use of this library not covered by either 
 * the QPL or the GPL requires an additional license from Dtecta. 
 * Please contact info@dtecta.com for enquiries about the terms of commercial
 * use of this library.
 */

#include "Solid3EpaPenetrationDepth.h"
#include <algorithm>
#include <vector>
#include "BulletCollision/NarrowPhaseCollision/btSimplexSolverInterface.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "LinearMath/btMinMax.h"

#define ASSERT_MESSAGE

class ReplaceMeAccuracy {
public:
	static btScalar rel_error2; // squared relative error in the computed distance
	static btScalar depth_tolerance; // terminate EPA if upper_bound <= depth_tolerance * dist2
	static btScalar tol_error; // error tolerance if the distance is almost zero
	
	static void setAccuracy(btScalar rel_error) 
	{ 
		rel_error2 = rel_error * rel_error;
		depth_tolerance = btScalar(1.0f) + btScalar(2.0f) * rel_error;
	}	
   
	static void setTolerance(btScalar epsilon) 
	{ 
		tol_error = epsilon;
	}
};


static const btScalar rel_error = btScalar(1.0e-3);

btScalar ReplaceMeAccuracy::rel_error2 = rel_error * rel_error;
btScalar ReplaceMeAccuracy::depth_tolerance = btScalar(1.0) + btScalar(2.0) * rel_error; 
btScalar ReplaceMeAccuracy::tol_error = SIMD_EPSILON;




class ReplaceMeFacet;


class ReplaceMeEdge {
public:
    ReplaceMeEdge() {}
    ReplaceMeEdge(ReplaceMeFacet *facet, int index) : 
	m_facet(facet), 
	m_index(index) {}

    ReplaceMeFacet *getFacet() const { return m_facet; }
    int       getIndex() const { return m_index; }

    int getSource() const;
    int getTarget() const;

private:    
    ReplaceMeFacet *m_facet;
    int       m_index;
};

typedef std::vector<ReplaceMeEdge> ReplaceMeEdgeBuffer;


class ReplaceMeFacet {
public:
    ReplaceMeFacet() {}
    ReplaceMeFacet(int i0, int i1, int i2) 
	  :	m_obsolete(false) 
    {
		m_indices[0] = i0; 
		m_indices[1] = i1; 
		m_indices[2] = i2;
    }
	
    inline int operator[](int i) const { return m_indices[i]; } 

    bool link(int edge0, ReplaceMeFacet *facet, int edge1);

    
    bool isObsolete() const { return m_obsolete; }
    

    bool computeClosest(const btVector3 *verts);
    
    const btVector3& getClosest() const { return m_closest; } 
    
    bool isClosestInternal() const
	{ 
		return m_lambda1 >= btScalar(0.0) && 
			m_lambda2 >= btScalar(0.0) && 
			m_lambda1 + m_lambda2 <= m_det;
    } 

    btScalar getDist2() const { return m_dist2; }
	
    btPoint3 getClosestPoint(const btPoint3 *points) const 
	{
		const btPoint3& p0 = points[m_indices[0]];
		
		return p0 + (m_lambda1 * (points[m_indices[1]] - p0) + 
					 m_lambda2 * (points[m_indices[2]] - p0)) / m_det;
    }
    
    void silhouette(const btVector3& w, ReplaceMeEdgeBuffer& edgeBuffer) 
	{
		edgeBuffer.clear();
		m_obsolete = true;
		m_adjFacets[0]->silhouette(m_adjEdges[0], w, edgeBuffer);
		m_adjFacets[1]->silhouette(m_adjEdges[1], w, edgeBuffer);
		m_adjFacets[2]->silhouette(m_adjEdges[2], w, edgeBuffer);
    }
	
private:
    void silhouette(int index, const btVector3& w, ReplaceMeEdgeBuffer& edgeBuffer);
	
    int         m_indices[3];
    bool        m_obsolete;
    ReplaceMeFacet   *m_adjFacets[3];
    int         m_adjEdges[3];
	
    btScalar   m_det;
    btScalar   m_lambda1;
    btScalar   m_lambda2;
    btVector3  m_closest;
    btScalar   m_dist2;
};


inline int incMod3(int i) { return ++i % 3; } 


bool ReplaceMeFacet::link(int edge0, ReplaceMeFacet *facet, int edge1) 
{
    m_adjFacets[edge0] = facet;
    m_adjEdges[edge0] = edge1;
    facet->m_adjFacets[edge1] = this;
    facet->m_adjEdges[edge1] = edge0;

    bool b = m_indices[edge0] == facet->m_indices[incMod3(edge1)] &&
	m_indices[incMod3(edge0)] == facet->m_indices[edge1];
    return b;
}


bool ReplaceMeFacet::computeClosest(const btVector3 *verts)
{
    const btVector3& p0 = verts[m_indices[0]]; 

    btVector3 v1 = verts[m_indices[1]] - p0;
    btVector3 v2 = verts[m_indices[2]] - p0;
    btScalar v1dv1 = v1.length2();
    btScalar v1dv2 = v1.dot(v2);
    btScalar v2dv2 = v2.length2();
    btScalar p0dv1 = p0.dot(v1); 
    btScalar p0dv2 = p0.dot(v2);

    m_det = v1dv1 * v2dv2 - v1dv2 * v1dv2; // non-negative
    //printf("m_det = %f\n",m_det);
    //ASSERT(m_det >= 0.f);

    if (m_det >= (SIMD_EPSILON*SIMD_EPSILON)) {	

    	m_lambda1 = p0dv2 * v1dv2 - p0dv1 * v2dv2;
	    m_lambda2 = p0dv1 * v1dv2 - p0dv2 * v1dv1; 
	
		m_closest = p0 + (m_lambda1 * v1 + m_lambda2 * v2) / m_det;
		m_dist2 = m_closest.length2();
		return true;
    }
    
    return false;
} 

void ReplaceMeFacet::silhouette(int index, const btVector3& w, 
			  ReplaceMeEdgeBuffer& edgeBuffer) 
{
    if (!m_obsolete) {
		if (m_closest.dot(w) < m_dist2) {
			edgeBuffer.push_back(ReplaceMeEdge(this, index));
		}	
	else {
	    m_obsolete = true; // Facet is visible 
	    int next = incMod3(index);
	    m_adjFacets[next]->silhouette(m_adjEdges[next], w, edgeBuffer);
	    next = incMod3(next);
	    m_adjFacets[next]->silhouette(m_adjEdges[next], w, edgeBuffer);
	}
    }
}



inline int ReplaceMeEdge::getSource() const 
{
    return (*m_facet)[m_index];
}

inline int ReplaceMeEdge::getTarget() const 
{
    return (*m_facet)[incMod3(m_index)];
}


//#define DEBUG

const int       MaxSupportPoints = 100;//1000;
const int       MaxFacets         = 200;//b2000;

static btPoint3  pBuf[MaxSupportPoints];
static btPoint3  qBuf[MaxSupportPoints];
static btVector3 yBuf[MaxSupportPoints];

static ReplaceMeFacet facetBuf[MaxFacets];
static int  freeFacet = 0;
static ReplaceMeFacet *facetHeap[MaxFacets];
static int  num_facets;

class ReplaceMeFacetComp {
public:
    
    bool operator()(const ReplaceMeFacet *face1, const ReplaceMeFacet *face2) 
	{ 
		return face1->getDist2() > face2->getDist2();
    }
    
};

ReplaceMeFacetComp myFacetComp;

inline ReplaceMeFacet *addFacet(int i0, int i1, int i2,
						  btScalar lower2, btScalar upper2) 
{
    assert(i0 != i1 && i0 != i2 && i1 != i2);
    if (freeFacet < MaxFacets)
	{
		ReplaceMeFacet *facet = new(&facetBuf[freeFacet++]) ReplaceMeFacet(i0, i1, i2);
#ifdef DEBUG
		std::cout << "Facet " << i0 << ' ' << i1 << ' ' << i2;
#endif
		if (facet->computeClosest(yBuf)) 
		{
			if (facet->isClosestInternal() && 
				lower2 <= facet->getDist2() && facet->getDist2() <= upper2) 
			{
				facetHeap[num_facets++] = facet;
				ASSERT_MESSAGE(num_facets<MaxFacets,"Error in facet/pendepth");

				std::push_heap(&facetHeap[0], &facetHeap[num_facets], myFacetComp);
#ifdef DEBUG
				std::cout << " accepted" << std::endl;
#endif
			}
			else 
			{
#ifdef DEBUG
				std::cout << " rejected, ";
				if (!facet->isClosestInternal()) 
				{
					std::cout << "closest point not internal";
				}
				else if (lower2 > facet->getDist2()) 
				{
					std::cout << "facet is closer than orignal facet";
				}
				else 
				{
					std::cout << "facet is further than upper bound";
				}
				std::cout << std::endl;
#endif
			}
			
			return facet;
		}
    }
    
    return 0;
}



inline bool originInTetrahedron(const btVector3& p1, const btVector3& p2, 
								const btVector3& p3, const btVector3& p4)
{
    btVector3 normal1 = (p2 - p1).cross(p3 - p1);
    btVector3 normal2 = (p3 - p2).cross(p4 - p2);
    btVector3 normal3 = (p4 - p3).cross(p1 - p3);
    btVector3 normal4 = (p1 - p4).cross(p2 - p4);
    
    return 
		(normal1.dot(p1) > btScalar(0.0)) != (normal1.dot(p4) > btScalar(0.0)) &&
		(normal2.dot(p2) > btScalar(0.0)) != (normal2.dot(p1) > btScalar(0.0)) &&
		(normal3.dot(p3) > btScalar(0.0)) != (normal3.dot(p2) > btScalar(0.0)) &&
		(normal4.dot(p4) > btScalar(0.0)) != (normal4.dot(p3) > btScalar(0.0));
}



bool Solid3EpaPenetrationDepth::calcPenDepth( btSimplexSolverInterface& simplexSolver,
		btConvexShape* convexA,btConvexShape* convexB,
					const btTransform& transformA,const btTransform& transformB,
				btVector3& v, btPoint3& pa, btPoint3& pb,
				class btIDebugDraw* debugDraw
				)
{
	
    int num_verts = simplexSolver.getSimplex(pBuf, qBuf, yBuf);

    switch (num_verts) 
	{
	case 1:
	    // Touching contact. Yes, we have a collision,
	    // but no penetration.
	    return false;
	case 2:	
	{
	    // We have a line segment inside the Minkowski sum containing the
	    // origin. Blow it up by adding three additional support points.
	    
	    btVector3 dir  = (yBuf[1] - yBuf[0]).normalized();
	    int        axis = dir.furthestAxis();
	    
	    static btScalar sin_60 = 0.8660254037f;//84438646763723170752941.22474487f;//13915890490986420373529;//
	    
	    btQuaternion rot(dir[0] * sin_60, dir[1] * sin_60, dir[2] * sin_60, btScalar(0.5));
	    btMatrix3x3 rot_mat(rot);
	    
	    btVector3 aux1 = dir.cross(btVector3(axis == 0, axis == 1, axis == 2));
	    btVector3 aux2 = rot_mat * aux1;
	    btVector3 aux3 = rot_mat * aux2;
	    
	    pBuf[2] = transformA(convexA->localGetSupportingVertex(aux1*transformA.getBasis()));
		qBuf[2] = transformB(convexB->localGetSupportingVertex((-aux1)*transformB.getBasis()));
	    yBuf[2] = pBuf[2] - qBuf[2];
	    
	    pBuf[3] = transformA(convexA->localGetSupportingVertex(aux2*transformA.getBasis()));
		qBuf[3] = transformB(convexB->localGetSupportingVertex((-aux2)*transformB.getBasis()));
	    yBuf[3] = pBuf[3] - qBuf[3];
	    
		pBuf[4] = transformA(convexA->localGetSupportingVertex(aux3*transformA.getBasis()));
		qBuf[4] = transformB(convexB->localGetSupportingVertex((-aux3)*transformB.getBasis()));
	    yBuf[4] = pBuf[4] - qBuf[4];
	    
	    if (originInTetrahedron(yBuf[0], yBuf[2], yBuf[3], yBuf[4])) 
		{
			pBuf[1] = pBuf[4];
			qBuf[1] = qBuf[4];
			yBuf[1] = yBuf[4];
	    }
	    else if (originInTetrahedron(yBuf[1], yBuf[2], yBuf[3], yBuf[4])) 
		{
			pBuf[0] = pBuf[4];
			qBuf[0] = qBuf[4];
			yBuf[0] = yBuf[4];
	    } 
	    else 
		{
			// Origin not in initial polytope
			return false;
	    }
	    
	    num_verts = 4;
	    
	    break;
	}
	case 3: 
	{
	    // We have a triangle inside the Minkowski sum containing
	    // the origin. First blow it up.
	    
	    btVector3 v1     = yBuf[1] - yBuf[0];
	    btVector3 v2     = yBuf[2] - yBuf[0];
	    btVector3 vv     = v1.cross(v2);
	    
		pBuf[3] = transformA(convexA->localGetSupportingVertex(vv*transformA.getBasis()));
		qBuf[3] = transformB(convexB->localGetSupportingVertex((-vv)*transformB.getBasis()));
	    yBuf[3] = pBuf[3] - qBuf[3];
		pBuf[4] = transformA(convexA->localGetSupportingVertex((-vv)*transformA.getBasis()));
		qBuf[4] = transformB(convexB->localGetSupportingVertex(vv*transformB.getBasis()));
	    yBuf[4] = pBuf[4] - qBuf[4];
	    
	   
	    if (originInTetrahedron(yBuf[0], yBuf[1], yBuf[2], yBuf[4])) 
		{
			pBuf[3] = pBuf[4];
			qBuf[3] = qBuf[4];
			yBuf[3] = yBuf[4];
	    }
	    else if (!originInTetrahedron(yBuf[0], yBuf[1], yBuf[2], yBuf[3]))
		{ 
			// Origin not in initial polytope
			return false;
	    }
	    
	    num_verts = 4;
	    
	    break;
	}
    }
    
    // We have a tetrahedron inside the Minkowski sum containing
    // the origin (if GJK did it's job right ;-)
      
    
    if (!originInTetrahedron(yBuf[0], yBuf[1], yBuf[2], yBuf[3])) 
	{
		//	assert(false);
		return false;
	}
    
	num_facets = 0;
    freeFacet = 0;

    ReplaceMeFacet *f0 = addFacet(0, 1, 2, btScalar(0.0), SIMD_INFINITY);
    ReplaceMeFacet *f1 = addFacet(0, 3, 1, btScalar(0.0), SIMD_INFINITY);
    ReplaceMeFacet *f2 = addFacet(0, 2, 3, btScalar(0.0), SIMD_INFINITY);
    ReplaceMeFacet *f3 = addFacet(1, 3, 2, btScalar(0.0), SIMD_INFINITY);
    
    if (!f0 || f0->getDist2() == btScalar(0.0) ||
		!f1 || f1->getDist2() == btScalar(0.0) ||
		!f2 || f2->getDist2() == btScalar(0.0) ||
		!f3 || f3->getDist2() == btScalar(0.0)) 
	{
		return false;
    }
    
    f0->link(0, f1, 2);
    f0->link(1, f3, 2);
    f0->link(2, f2, 0);
    f1->link(0, f2, 2);
    f1->link(1, f3, 0);
    f2->link(1, f3, 1);
    
    if (num_facets == 0) 
	{
		return false;
    }
    
    // at least one facet on the heap.	
    
    ReplaceMeEdgeBuffer edgeBuffer(20);

    ReplaceMeFacet *facet = 0;
    
    btScalar upper_bound2 = SIMD_INFINITY; 	
    
    do {
        facet = facetHeap[0];
        std::pop_heap(&facetHeap[0], &facetHeap[num_facets], myFacetComp);
        --num_facets;
		
		if (!facet->isObsolete()) 
		{
			assert(facet->getDist2() > btScalar(0.0));
			
			if (num_verts == MaxSupportPoints)
			{
#ifdef DEBUG
				std::cout << "Ouch, no convergence!!!" << std::endl;
#endif 
				ASSERT_MESSAGE(false,"Error: pendepth calc failed");	
				break;
			}
			
			pBuf[num_verts] = transformA(convexA->localGetSupportingVertex((facet->getClosest())*transformA.getBasis()));
			qBuf[num_verts] = transformB(convexB->localGetSupportingVertex((-facet->getClosest())*transformB.getBasis()));
			yBuf[num_verts] = pBuf[num_verts] - qBuf[num_verts];
			

			int index = num_verts++;
			btScalar far_dist2 = yBuf[index].dot(facet->getClosest());
			

			// Make sure the support mapping is OK.
			//assert(far_dist2 > btScalar(0.0));
			
			//
			// this is to avoid problems with implicit-sphere-touching contact
			//
			if (far_dist2 < btScalar(0.0))
			{
				return false;
			}

			GEN_set_min(upper_bound2, (far_dist2 * far_dist2) / facet->getDist2());
			
			if (upper_bound2 <= ReplaceMeAccuracy::depth_tolerance * facet->getDist2()
#define CHECK_NEW_SUPPORT
#ifdef CHECK_NEW_SUPPORT
				|| yBuf[index] == yBuf[(*facet)[0]] 
				|| yBuf[index] == yBuf[(*facet)[1]]
				|| yBuf[index] == yBuf[(*facet)[2]]
#endif
				) 
			{
				break;
			}
			
			// Compute the silhouette cast by the new vertex
			// Note that the new vertex is on the positive side
			// of the current facet, so the current facet is will
			// not be in the convex hull. Start local search
			// from this facet.
			
			facet->silhouette(yBuf[index], edgeBuffer);
			
			if (edgeBuffer.empty()) 
			{
				return false;
			}
			
			ReplaceMeEdgeBuffer::const_iterator it = edgeBuffer.begin();
			ReplaceMeFacet *firstFacet = 
				addFacet((*it).getTarget(), (*it).getSource(),
						 index, facet->getDist2(), upper_bound2);
			
			if (!firstFacet) 
			{
				break;
			}
			
			firstFacet->link(0, (*it).getFacet(), (*it).getIndex());
			ReplaceMeFacet *lastFacet = firstFacet;
			
			++it;
			for (; it != edgeBuffer.end(); ++it) 
			{
				ReplaceMeFacet *newFacet = 
					addFacet((*it).getTarget(), (*it).getSource(),
							 index, facet->getDist2(), upper_bound2);
				
				if (!newFacet) 
				{
					break;
				}
				
				if (!newFacet->link(0, (*it).getFacet(), (*it).getIndex())) 
				{
					break;
				}
				
				if (!newFacet->link(2, lastFacet, 1)) 
				{
					break;
				}
				
				lastFacet = newFacet;				
			}
			if (it != edgeBuffer.end()) 
			{
				break;
			}
			
			firstFacet->link(2, lastFacet, 1);
		}
    }
    while (num_facets > 0 && facetHeap[0]->getDist2() <= upper_bound2);
	
#ifdef DEBUG    
    std::cout << "#facets left = " << num_facets << std::endl;
#endif
    
    v = facet->getClosest();
    pa = facet->getClosestPoint(pBuf);    
    pb = facet->getClosestPoint(qBuf);    
    return true;
}

