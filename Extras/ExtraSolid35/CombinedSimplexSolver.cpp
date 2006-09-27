/*
 * Copyright (c) 2005 Erwin Coumans http://www.erwincoumans.com
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erwin Coumans makes no representations about the suitability 
 * of this software for any purpose.  
 * It is provided "as is" without express or implied warranty.
*/

#include "CombinedSimplexSolver.h"
#include <stdio.h>
//switch off asserts
//#define MY_ASSERT assert
#define MY_ASSERT

bool useVoronoi = true;

CombinedSimplexSolver::CombinedSimplexSolver()
:m_useVoronoiSolver(useVoronoi)
{

}

void CombinedSimplexSolver::reset()
{
	m_voronoiSolver.reset();
	m_johnsonSolver.reset();
}


void CombinedSimplexSolver::addVertex(const btVector3& w, const btPoint3& p, const btPoint3& q)
{
	printf("addVertex (%f %f %f)\n",w[0],w[1],w[2]);
	m_voronoiSolver.addVertex(w,p,q);
	m_johnsonSolver.addVertex(w,p,q);
	int i;
	i=0;

	btPoint3 vp1,vp2;
	btPoint3 jp1,jp2;
/*
	bool isClosest0 = m_voronoiSolver.closest(vp1);
	bool isClosest1 = m_johnsonSolver.closest(vp1);

	m_voronoiSolver.compute_points(vp1, vp2);
	m_johnsonSolver.compute_points(jp1,jp2);
	i=0;
	*/
}



bool CombinedSimplexSolver::closest(btVector3& v)
{
	bool result0 = 0;
	bool result1 = 0;
	
	btVector3 v0,v1;

	result0 = m_voronoiSolver.closest(v0);
	result1 = m_johnsonSolver.closest(v1);

	if (result0 != result1)
	{
		result0 = m_voronoiSolver.closest(v0);
		result1 = m_johnsonSolver.closest(v1);
		int i;
		i=0;
	}
	if (m_useVoronoiSolver)
	{
		v = v0;
		return result0;
	}

	v = v1;
	return result1;
}

btScalar CombinedSimplexSolver::maxVertex()
{
	btScalar maxv0 = m_voronoiSolver.maxVertex();
	btScalar maxv1 = m_johnsonSolver.maxVertex();
	MY_ASSERT(maxv0 = maxv1);
	if (m_useVoronoiSolver)
		return maxv0;

	return maxv1;
}

bool CombinedSimplexSolver::fullSimplex() const
{
	bool fullSimplex0 = m_voronoiSolver.fullSimplex();
	bool fullSimplex1 = m_johnsonSolver.fullSimplex();
	MY_ASSERT(fullSimplex0 == fullSimplex1);

	if (m_useVoronoiSolver)
		return fullSimplex0;

	return fullSimplex1;
}

int CombinedSimplexSolver::getSimplex(btPoint3 *pBuf, btPoint3 *qBuf, btVector3 *yBuf) const
{


	int simplex0 = m_voronoiSolver.getSimplex(pBuf, qBuf, yBuf);
	int simplex1 = m_johnsonSolver.getSimplex(pBuf, qBuf, yBuf);
//	MY_ASSERT(simplex0 == simplex1);
	if (m_useVoronoiSolver)
	{
		return m_voronoiSolver.getSimplex(pBuf, qBuf, yBuf);
	}

	return simplex1;
}

void	CombinedSimplexSolver::debugPrint()
{
	btPoint3 pBuf0[4];
	btPoint3 qBuf0[4];
	btPoint3 yBuf0[4];
	btPoint3 pBuf1[4];
	btPoint3 qBuf1[4];
	btPoint3 yBuf1[4];
	int verts0,verts1;

	verts0 = m_voronoiSolver.getSimplex(&pBuf0[0], &qBuf0[0], &yBuf0[0]);
	verts1 = m_johnsonSolver.getSimplex(&pBuf1[0], &qBuf1[0], &yBuf1[0]);
	printf("numverts0 = %d, numverts1 = %d\n",verts0,verts1);
	for (int i=0;i<verts0;i++)
	{
		printf("vert0 pBuf %d = %f , %f , %f\n",i,pBuf0[i].x(),pBuf0[i].y(),pBuf0[i].z());
		printf("vert0 qBuf %d = %f , %f , %f\n",i,qBuf0[i].x(),qBuf0[i].y(),qBuf0[i].z());
		printf("vert0 yBuf %d = %f , %f , %f\n",i,yBuf0[i].x(),yBuf0[i].y(),yBuf0[i].z());
	}

	for (int i=0;i<verts1;i++)
	{
		printf("vert1 pBuf %d = %f , %f , %f\n",i,pBuf1[i].x(),pBuf1[i].y(),pBuf1[i].z());
		printf("vert1 qBuf %d = %f , %f , %f\n",i,qBuf1[i].x(),qBuf1[i].y(),qBuf1[i].z());
		printf("vert1 yBuf %d = %f , %f , %f\n",i,yBuf1[i].x(),yBuf1[i].y(),yBuf1[i].z());
	}


}
bool CombinedSimplexSolver::inSimplex(const btVector3& w)
{
	bool insimplex0 = m_voronoiSolver.inSimplex(w);
	bool insimplex1 = m_johnsonSolver.inSimplex(w);

	if (insimplex0 != insimplex1)
	{
		debugPrint();

		
	}
	if (m_useVoronoiSolver)
		return insimplex0;

	return insimplex1;

}
void CombinedSimplexSolver::backup_closest(btVector3& v) 
{
	btVector3 v0,v1;

	m_voronoiSolver.backup_closest(v0);
	m_johnsonSolver.backup_closest(v1);
	if (m_useVoronoiSolver)
	{
		v = v0;
	} else
	{
		v = v1;
	}
}

bool CombinedSimplexSolver::emptySimplex() const
{
	bool empty0 = m_voronoiSolver.emptySimplex();
	bool empty1 = m_johnsonSolver.emptySimplex();
	MY_ASSERT(empty0 == empty1);
	if (m_useVoronoiSolver)
		return empty0;

	return empty1;
}

void CombinedSimplexSolver::compute_points(btPoint3& p1, btPoint3& p2)
{
	btPoint3	tmpP1,tmpP2;
	btPoint3	tmpJP1,tmpJP2;

	m_voronoiSolver.compute_points(tmpP1,tmpP2);
	m_johnsonSolver.compute_points(tmpJP1,tmpJP2);

	if (m_useVoronoiSolver)
	{
		p1 = tmpP1;
		p2 = tmpP2;
	}
	else
	{
		p1 = tmpJP1;
		p2 = tmpJP2;
	}
}

int CombinedSimplexSolver::numVertices() const
{
	int numverts0 = m_voronoiSolver.numVertices();
	int numverts1 = m_johnsonSolver.numVertices();
	MY_ASSERT(numverts0==numverts1);
	if (numverts0 != numverts1)
	{
		printf("--------- (numverts0 != numverts1) -----------------------\n");
	}
	printf("numverts0 = %d, numverts1 %d \n",numverts0,numverts1);

	if (m_useVoronoiSolver)
		return numverts0;

	return numverts1;
}
