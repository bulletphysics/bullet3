/* Copyright (c) 2011 Khaled Mamou (kmamou at gmail dot com)
 All rights reserved.
 
 
 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 
 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 
 3. The names of the contributors may not be used to endorse or promote products derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif //_CRT_SECURE_NO_WARNINGS

#include <sstream>
#include "hacdGraph.h"
#include "hacdHACD.h"
#include "hacdICHull.h"
#include <string.h>
#include <algorithm>
#include <iterator>
#include <limits>
#include "assert.h"

bool gCancelRequest=false;
namespace HACD
{ 
	double  HACD::Concavity(ICHull & ch, std::map<long, DPoint> & distPoints)
    {
		double concavity = 0.0;
		double distance = 0.0;       
		std::map<long, DPoint>::iterator itDP(distPoints.begin());
		std::map<long, DPoint>::iterator itDPEnd(distPoints.end());
		for(; itDP != itDPEnd; ++itDP) 
		{
            if (!(itDP->second).m_computed)
            {
                if (itDP->first >= 0)
                {
                    distance = ch.ComputeDistance(itDP->first, m_points[itDP->first], m_normals[itDP->first], (itDP->second).m_computed, true);
                }
                else
                {
                    distance = ch.ComputeDistance(itDP->first, m_facePoints[-itDP->first-1], m_faceNormals[-itDP->first-1], (itDP->second).m_computed, true);
                }
            }
            else
            {
                distance = (itDP->second).m_dist;
            }
			if (concavity < distance) 
			{
				concavity = distance;
			}
		}
		return concavity;
    }

	void HACD::CreateGraph()
    {
		// vertex to triangle adjacency information
		std::vector< std::set<long> >  vertexToTriangles;		
		vertexToTriangles.resize(m_nPoints);
		for(size_t t = 0; t < m_nTriangles; ++t)
		{
			vertexToTriangles[m_triangles[t].X()].insert(static_cast<long>(t));
			vertexToTriangles[m_triangles[t].Y()].insert(static_cast<long>(t));
			vertexToTriangles[m_triangles[t].Z()].insert(static_cast<long>(t));
		}

		m_graph.Clear();
		m_graph.Allocate(m_nTriangles, 5 * m_nTriangles);
		unsigned long long tr1[3];
		unsigned long long tr2[3];
        long i1, j1, k1, i2, j2, k2;
        long t1, t2;
        for (size_t v = 0; v < m_nPoints; v++) 
		{
			std::set<long>::const_iterator it1(vertexToTriangles[v].begin()), itEnd(vertexToTriangles[v].end()); 
			for(; it1 != itEnd; ++it1)
			{
                t1 = *it1;
                i1 = m_triangles[t1].X();
                j1 = m_triangles[t1].Y();
                k1 = m_triangles[t1].Z();
				tr1[0] = GetEdgeIndex(i1, j1);
				tr1[1] = GetEdgeIndex(j1, k1);
				tr1[2] = GetEdgeIndex(k1, i1);
				std::set<long>::const_iterator it2(it1);
				for(++it2; it2 != itEnd; ++it2)
				{
                    t2 = *it2;
                    i2 = m_triangles[t2].X();
                    j2 = m_triangles[t2].Y();
                    k2 = m_triangles[t2].Z();
					tr2[0] = GetEdgeIndex(i2, j2);
					tr2[1] = GetEdgeIndex(j2, k2);
					tr2[2] = GetEdgeIndex(k2, i2);
					int shared = 0;
					for(int i = 0; i < 3; ++i)
					{
						for(int j = 0; j < 3; ++j)
						{
							if (tr1[i] == tr2[j])	
							{
								shared++;
							}
						}
					}
					if (shared == 1) // two triangles are connected if they share exactly one edge
					{
						m_graph.AddEdge(t1, t2);
					}
				}
			}
        }
        if (m_ccConnectDist >= 0.0)
        {
            m_graph.ExtractCCs();
            if (m_graph.m_nCCs > 1) 
            {
                std::vector< std::set<long> > cc2V;
                cc2V.resize(m_graph.m_nCCs);
                long cc;
                for(size_t t = 0; t < m_nTriangles; ++t)
                {
                    cc = m_graph.m_vertices[t].m_cc;
                    cc2V[cc].insert(m_triangles[t].X());
                    cc2V[cc].insert(m_triangles[t].Y());
                    cc2V[cc].insert(m_triangles[t].Z());
                }
                
                for(size_t cc1 = 0; cc1 < m_graph.m_nCCs; ++cc1)
                {
                    for(size_t cc2 = cc1+1; cc2 < m_graph.m_nCCs; ++cc2)
                    {
                        std::set<long>::const_iterator itV1(cc2V[cc1].begin()), itVEnd1(cc2V[cc1].end()); 
                        for(; itV1 != itVEnd1; ++itV1)
                        {
							double distC1C2 = std::numeric_limits<double>::max();
                            double dist;
                            t1 = -1;
                            t2 = -1;
                            std::set<long>::const_iterator itV2(cc2V[cc2].begin()), itVEnd2(cc2V[cc2].end()); 
                            for(; itV2 != itVEnd2; ++itV2)
                            {
                                dist = (m_points[*itV1] - m_points[*itV2]).GetNorm();
                                if (dist < distC1C2)
                                {
                                    distC1C2 = dist;
                                    t1 = *vertexToTriangles[*itV1].begin();
                                    
									std::set<long>::const_iterator it2(vertexToTriangles[*itV2].begin()), 
																   it2End(vertexToTriangles[*itV2].end()); 
									t2 = -1;
									for(; it2 != it2End; ++it2)
									{
										if (*it2 != t1)
										{
											t2 = *it2;
											break;
										}
									}
                                }
                            }
                            if (distC1C2 <= m_ccConnectDist && t1 > 0 && t2 > 0)
                            {
								
                                m_graph.AddEdge(t1, t2);                    
                            }
                        }
                    }
                }
            }
        }
    }
    void HACD::InitializeDualGraph()
    {
        long i, j, k;
        Vec3<Real> u, v, w, normal;
		delete [] m_normals;
		m_normals = new Vec3<Real>[m_nPoints];
        if (m_addFacesPoints)
        {
            delete [] m_facePoints;
            delete [] m_faceNormals;
            m_facePoints = new Vec3<Real>[m_nTriangles];
            m_faceNormals = new Vec3<Real>[m_nTriangles];
        }
		memset(m_normals, 0, sizeof(Vec3<Real>) * m_nPoints);
        for(unsigned long f = 0; f < m_nTriangles; f++)
        {
			if (m_callBack) (*m_callBack)("+ InitializeDualGraph\n", f, m_nTriangles, 0);
			
			if (gCancelRequest)
				return;

            i = m_triangles[f].X();
            j = m_triangles[f].Y();
            k = m_triangles[f].Z();
		
			m_graph.m_vertices[f].m_distPoints[i].m_distOnly = false;
			m_graph.m_vertices[f].m_distPoints[j].m_distOnly = false;
			m_graph.m_vertices[f].m_distPoints[k].m_distOnly = false;
            
            ICHull  * ch = new ICHull;
            m_graph.m_vertices[f].m_convexHull = ch;
            ch->AddPoint(m_points[i], i);
            ch->AddPoint(m_points[j], j);
            ch->AddPoint(m_points[k], k);
			ch->SetDistPoints(&m_graph.m_vertices[f].m_distPoints);

			u = m_points[j] - m_points[i];
			v = m_points[k] - m_points[i];
			w = m_points[k] - m_points[j];
			normal = u ^ v;

			m_normals[i] += normal;
			m_normals[j] += normal;
			m_normals[k] += normal;

			m_graph.m_vertices[f].m_surf = normal.GetNorm();
			m_graph.m_vertices[f].m_perimeter = u.GetNorm() + v.GetNorm() + w.GetNorm();
            
            normal.Normalize();

			m_graph.m_vertices[f].m_boudaryEdges.insert(GetEdgeIndex(i,j));
			m_graph.m_vertices[f].m_boudaryEdges.insert(GetEdgeIndex(j,k));
			m_graph.m_vertices[f].m_boudaryEdges.insert(GetEdgeIndex(k,i));
            if(m_addFacesPoints)
            {
                m_faceNormals[f] = normal;
                m_facePoints[f] = (m_points[i] + m_points[j] + m_points[k]) / 3.0;
                m_graph.m_vertices[f].m_distPoints[-static_cast<long>(f)-1].m_distOnly = true;
            }
            if (m_addExtraDistPoints)	
            {// we need a kd-tree structure to accelerate this part!
                long i1, j1, k1;
                Vec3<Real> u1, v1, normal1;
				normal = -normal;
                double distance = 0.0;
                double distMin = 0.0;
                size_t faceIndex = m_nTriangles;
                Vec3<Real> seedPoint((m_points[i] + m_points[j] + m_points[k]) / 3.0);
                long nhit = 0;
                for(size_t f1 = 0; f1 < m_nTriangles; f1++)
                {
                    i1 = m_triangles[f1].X();
                    j1 = m_triangles[f1].Y();
                    k1 = m_triangles[f1].Z();
                    u1 = m_points[j1] - m_points[i1];
                    v1 = m_points[k1] - m_points[i1];
                    normal1 = (u1 ^ v1);
                    if (normal * normal1 > 0.0)
                    {
                        nhit = IntersectRayTriangle(Vec3<double>(seedPoint.X(), seedPoint.Y(), seedPoint.Z()),
													Vec3<double>(normal.X(), normal.Y(), normal.Z()),
													Vec3<double>(m_points[i1].X(), m_points[i1].Y(), m_points[i1].Z()),
													Vec3<double>(m_points[j1].X(), m_points[j1].Y(), m_points[j1].Z()),
													Vec3<double>(m_points[k1].X(), m_points[k1].Y(), m_points[k1].Z()),
													distance);
                        if ((nhit==1) && ((distMin > distance) || (faceIndex == m_nTriangles)))
                        {
                            distMin = distance;
                            faceIndex = f1;
                        }

                    }
                }
                if (faceIndex < m_nTriangles )
                {
                    i1 = m_triangles[faceIndex].X();
                    j1 = m_triangles[faceIndex].Y();
                    k1 = m_triangles[faceIndex].Z();
                    m_graph.m_vertices[f].m_distPoints[i1].m_distOnly = true;
                    m_graph.m_vertices[f].m_distPoints[j1].m_distOnly = true;
                    m_graph.m_vertices[f].m_distPoints[k1].m_distOnly = true;
					if (m_addFacesPoints)
					{
						m_graph.m_vertices[f].m_distPoints[-static_cast<long>(faceIndex)-1].m_distOnly = true;
					}
				}
            }
        }
        for (size_t v = 0; v < m_nPoints; v++) 
		{
			m_normals[v].Normalize();
		}
    }

	void HACD::NormalizeData()
	{
		if (m_nPoints == 0)
		{
			return;
		}
        m_barycenter = m_points[0];
		Vec3<Real> min = m_points[0];
		Vec3<Real> max = m_points[0];
		Real x, y, z;
        for (size_t v = 1; v < m_nPoints ; v++) 
        {
			m_barycenter += m_points[v];
            x = m_points[v].X();
            y = m_points[v].Y();
            z = m_points[v].Z();
            if ( x < min.X()) min.X() = x;
			else if ( x > max.X()) max.X() = x;
            if ( y < min.Y()) min.Y() = y;
			else if ( y > max.Y()) max.Y() = y;
            if ( z < min.Z()) min.Z() = z;
			else if ( z > max.Z()) max.Z() = z;
        }
		m_barycenter /= static_cast<Real>(m_nPoints);
        m_diag = (max-min).GetNorm();
        const Real invDiag = static_cast<Real>(2.0 * m_scale / m_diag);
		if (m_diag != 0.0)
		{
			for (size_t v = 0; v < m_nPoints ; v++) 
			{
				m_points[v] = (m_points[v] - m_barycenter) * invDiag;
			}
		}
    }
	void HACD::DenormalizeData()
	{
		if (m_nPoints == 0)
		{
			return;
		}
		if (m_diag != 0.0)
		{
			const Real diag = static_cast<Real>(m_diag / (2.0 * m_scale));
			for (size_t v = 0; v < m_nPoints ; v++) 
			{
				m_points[v] = m_points[v] * diag + m_barycenter;
			}
		}
    }
	HACD::HACD(void)
	{
        m_convexHulls = 0;
		m_triangles = 0;
        m_points = 0;
        m_normals = 0;
        m_nTriangles = 0;
        m_nPoints = 0;
        m_nClusters = 0;
        m_concavity = 0.0;
        m_diag = 1.0;
		m_barycenter = Vec3<Real>(0.0, 0.0,0.0);
        m_alpha = 0.1;
        m_beta = 0.1;
        m_nVerticesPerCH = 30;
		m_callBack = 0;
        m_addExtraDistPoints = false;
		m_addNeighboursDistPoints = false;
		m_scale = 1000.0;
		m_partition = 0;
		m_nMinClusters = 3;
        m_facePoints = 0;
        m_faceNormals = 0;
        m_ccConnectDist = 30;
	}																
	HACD::~HACD(void)
	{
		delete [] m_normals;
        delete [] m_convexHulls;
		delete [] m_partition;
        delete [] m_facePoints;
        delete [] m_faceNormals;
	}
	int iteration = 0;
    void HACD::ComputeEdgeCost(size_t e)
    {
		GraphEdge & gE = m_graph.m_edges[e];
        long v1 = gE.m_v1;
        long v2 = gE.m_v2;

        if (m_graph.m_vertices[v2].m_distPoints.size()>m_graph.m_vertices[v1].m_distPoints.size())
        {
            gE.m_v1 = v2;
            gE.m_v2 = v1;
            //std::swap<long>(v1, v2);
			std::swap(v1, v2);
        }
		GraphVertex & gV1 = m_graph.m_vertices[v1];
		GraphVertex & gV2 = m_graph.m_vertices[v2];
	
        // delete old convex-hull
        delete gE.m_convexHull;
        // create the edge's convex-hull
        ICHull  * ch = new ICHull;
        gE.m_convexHull = ch;
        (*ch) = (*gV1.m_convexHull);
        
		// update distPoints
		gE.m_distPoints = gV1.m_distPoints;
		std::map<long, DPoint>::iterator itDP(gV2.m_distPoints.begin());
		std::map<long, DPoint>::iterator itDPEnd(gV2.m_distPoints.end());
		std::map<long, DPoint>::iterator itDP1;
     
		for(; itDP != itDPEnd; ++itDP) 
		{
			itDP1 = gE.m_distPoints.find(itDP->first);
			if (itDP1 == gE.m_distPoints.end())
			{
                gE.m_distPoints[itDP->first].m_distOnly = (itDP->second).m_distOnly;
                if ( !(itDP->second).m_distOnly )
                {
                    ch->AddPoint(m_points[itDP->first], itDP->first);
                }
			}
            else
            {
                if ( (itDP1->second).m_distOnly && !(itDP->second).m_distOnly)
                {
                    gE.m_distPoints[itDP->first].m_distOnly = false;
                    ch->AddPoint(m_points[itDP->first], itDP->first);
                }
            }
		}
		
		ch->SetDistPoints(&gE.m_distPoints);
        // create the convex-hull
        while (ch->Process() == ICHullErrorInconsistent)		// if we face problems when constructing the visual-hull. really ugly!!!!
		{
//			if (m_callBack) (*m_callBack)("\t Problem with convex-hull construction [HACD::ComputeEdgeCost]\n", 0.0, 0.0, 0);
			ch = new ICHull;
			CircularList<TMMVertex> & verticesCH = (gE.m_convexHull)->GetMesh().m_vertices;
			size_t nV = verticesCH.GetSize();
			long ptIndex = 0;
			verticesCH.Next();
			for(size_t v = 1; v < nV; ++v)
			{
				ptIndex = verticesCH.GetHead()->GetData().m_name;
				if (ptIndex != ICHull::sc_dummyIndex/* && ptIndex < m_nPoints*/)
					ch->AddPoint(m_points[ptIndex], ptIndex);
				verticesCH.Next();
			}
			delete gE.m_convexHull;
			gE.m_convexHull = ch;
		}
		double volume = 0.0; 
        double concavity = 0.0;
		if (ch->IsFlat())
		{
			bool insideHull;
            std::map<long, DPoint>::iterator itDP(gE.m_distPoints.begin());
            std::map<long, DPoint>::iterator itDPEnd(gE.m_distPoints.end());
            for(; itDP != itDPEnd; ++itDP) 
            {	
                if (itDP->first >= 0)
                {
                    concavity = std::max<double>(concavity, ch->ComputeDistance(itDP->first, m_points[itDP->first], m_normals[itDP->first], insideHull, false));
                }
			}
		}
        else
        {
            if (m_addNeighboursDistPoints)
            {  // add distance points from adjacent clusters
                std::set<long> eEdges;
                std::set_union(gV1.m_edges.begin(), 
                               gV1.m_edges.end(), 
                               gV2.m_edges.begin(), 
                               gV2.m_edges.end(),
                               std::inserter( eEdges, eEdges.begin() ) );
                
                std::set<long>::const_iterator ed(eEdges.begin());
                std::set<long>::const_iterator itEnd(eEdges.end());
                long a, b, c;
                for(; ed != itEnd; ++ed) 
                {
                    a = m_graph.m_edges[*ed].m_v1;
                    b = m_graph.m_edges[*ed].m_v2;
                    if ( a != v2 && a != v1)
                    {
                        c = a;
                    }
                    else if ( b != v2 && b != v1)
                    {
                        c = b;
                    }
                    else
                    {
                        c = -1;
                    }
                    if ( c > 0)
                    {
                        GraphVertex & gVC = m_graph.m_vertices[c];
                        std::map<long, DPoint>::iterator itDP(gVC.m_distPoints.begin());
                        std::map<long, DPoint>::iterator itDPEnd(gVC.m_distPoints.end());
                        std::map<long, DPoint>::iterator itDP1;
                        for(; itDP != itDPEnd; ++itDP) 
                        {
                            itDP1 = gE.m_distPoints.find(itDP->first);
							if (itDP1 == gE.m_distPoints.end())
							{
								if (itDP->first >= 0 && itDP1 == gE.m_distPoints.end() && ch->IsInside(m_points[itDP->first]))
								{
	                                gE.m_distPoints[itDP->first].m_distOnly = true;
		                        }
								else if (itDP->first < 0 && ch->IsInside(m_facePoints[-itDP->first-1]))
								{
									gE.m_distPoints[itDP->first].m_distOnly = true;
								}
							}
                        }
                    }
                }
            }
            concavity = Concavity(*ch, gE.m_distPoints);
        }
  
		// compute boudary edges
		double perimeter = 0.0;
		double surf    = 1.0;
		if (m_alpha > 0.0)
		{
			gE.m_boudaryEdges.clear();
			std::set_symmetric_difference (gV1.m_boudaryEdges.begin(), 
								  gV1.m_boudaryEdges.end(), 
								  gV2.m_boudaryEdges.begin(), 
								  gV2.m_boudaryEdges.end(),
								  std::inserter( gE.m_boudaryEdges, 
												 gE.m_boudaryEdges.begin() ) );

			std::set<unsigned long long>::const_iterator itBE(gE.m_boudaryEdges.begin());
			std::set<unsigned long long>::const_iterator itBEEnd(gE.m_boudaryEdges.end());
			for(; itBE != itBEEnd; ++itBE)
			{
					perimeter += (m_points[static_cast<long>((*itBE) >> 32)] - 
								   m_points[static_cast<long>((*itBE) & 0xFFFFFFFFULL)]).GetNorm();
			}
			surf    = gV1.m_surf + gV2.m_surf;
		}
        double ratio   = perimeter * perimeter / (4.0 * sc_pi * surf);
		gE.m_volume	   = (m_beta == 0.0)?0.0:ch->ComputeVolume()/pow(m_scale, 3.0);						// cluster's volume
        gE.m_surf      = surf;                          // cluster's area  
        gE.m_perimeter = perimeter;                     // cluster's perimeter
        gE.m_concavity = concavity;                     // cluster's concavity
        gE.m_error     = static_cast<Real>(concavity +  m_alpha * ratio + m_beta * volume);	// cluster's priority
	}
    bool HACD::InitializePriorityQueue()
    {
		m_pqueue.reserve(m_graph.m_nE + 100);
        for (size_t e=0; e < m_graph.m_nE; ++e) 
        {
            ComputeEdgeCost(static_cast<long>(e));
			m_pqueue.push(GraphEdgePriorityQueue(static_cast<long>(e), m_graph.m_edges[e].m_error));
        }
		return true;
    }
	void HACD::Simplify()
	{
		long v1 = -1;
        long v2 = -1;        
        double progressOld = -1.0;
        double progress = 0.0;
        double globalConcavity  = 0.0;     
		char msg[1024];
		double ptgStep = 1.0;
        while ( (globalConcavity < m_concavity) && 
				(m_graph.GetNVertices() > m_nMinClusters) && 
				(m_graph.GetNEdges() > 0)) 
		{
            progress = 100.0-m_graph.GetNVertices() * 100.0 / m_nTriangles;
            if (fabs(progress-progressOld) > ptgStep && m_callBack)
            {
				sprintf(msg, "%3.2f %% V = %lu \t C = %f \t \t \r", progress, static_cast<unsigned long>(m_graph.GetNVertices()), globalConcavity);
				(*m_callBack)(msg, progress, globalConcavity,  m_graph.GetNVertices());
                progressOld = progress;
				if (progress > 99.0)
				{
					ptgStep = 0.01;
				}
				else if (progress > 90.0)
				{
					ptgStep = 0.1;
				}
            }

			GraphEdgePriorityQueue currentEdge(0,0.0);
			bool done = false;
			do
			{
				done = false;
				if (m_pqueue.size() == 0)
				{
                    done = true;
                    break;
				}
                currentEdge = m_pqueue.top();
                m_pqueue.pop();
			}
			while (  m_graph.m_edges[currentEdge.m_name].m_deleted || 
					 m_graph.m_edges[currentEdge.m_name].m_error != currentEdge.m_priority);


			if (m_graph.m_edges[currentEdge.m_name].m_concavity < m_concavity && !done)
			{
                globalConcavity = std::max<double>(globalConcavity ,m_graph.m_edges[currentEdge.m_name].m_concavity);
				v1 = m_graph.m_edges[currentEdge.m_name].m_v1;
				v2 = m_graph.m_edges[currentEdge.m_name].m_v2;	
				// update vertex info
				m_graph.m_vertices[v1].m_error     = m_graph.m_edges[currentEdge.m_name].m_error;
				m_graph.m_vertices[v1].m_surf	   = m_graph.m_edges[currentEdge.m_name].m_surf;
				m_graph.m_vertices[v1].m_volume	   = m_graph.m_edges[currentEdge.m_name].m_volume;
				m_graph.m_vertices[v1].m_concavity = m_graph.m_edges[currentEdge.m_name].m_concavity;
				m_graph.m_vertices[v1].m_perimeter = m_graph.m_edges[currentEdge.m_name].m_perimeter;
                m_graph.m_vertices[v1].m_distPoints   = m_graph.m_edges[currentEdge.m_name].m_distPoints;
                (*m_graph.m_vertices[v1].m_convexHull) = (*m_graph.m_edges[currentEdge.m_name].m_convexHull);
				(m_graph.m_vertices[v1].m_convexHull)->SetDistPoints(&(m_graph.m_vertices[v1].m_distPoints));
				m_graph.m_vertices[v1].m_boudaryEdges   = m_graph.m_edges[currentEdge.m_name].m_boudaryEdges;
				
				// We apply the optimal ecol
//				std::cout << "v1 " << v1 << " v2 " << v2 << std::endl;
				m_graph.EdgeCollapse(v1, v2);
				// recompute the adjacent edges costs
				std::set<long>::const_iterator itE(m_graph.m_vertices[v1].m_edges.begin()), 
								   			   itEEnd(m_graph.m_vertices[v1].m_edges.end());
				for(; itE != itEEnd; ++itE)
				{
					size_t e = *itE;
					ComputeEdgeCost(static_cast<long>(e));
					m_pqueue.push(GraphEdgePriorityQueue(static_cast<long>(e), m_graph.m_edges[e].m_error));
				}
			}
            else
            {
                break;
            }
        }
		while (!m_pqueue.empty())
		{
			m_pqueue.pop();
		}
        
        m_cVertices.clear();
		m_nClusters = m_graph.GetNVertices();
        m_cVertices.reserve(m_nClusters);
		for (size_t p=0, v = 0; v != m_graph.m_vertices.size(); ++v) 
		{
			if (!m_graph.m_vertices[v].m_deleted)
			{
                if (m_callBack) 
                {
                    char msg[1024];
                    sprintf(msg, "\t CH \t %lu \t %lf \t %lf\n", static_cast<unsigned long>(p), m_graph.m_vertices[v].m_concavity, m_graph.m_vertices[v].m_error);
					(*m_callBack)(msg, 0.0, 0.0, m_nClusters);
					p++;
                }
                m_cVertices.push_back(static_cast<long>(v));			
			}
		}
        if (m_callBack)
        {
			sprintf(msg, "# clusters =  %lu \t C = %f\n", static_cast<unsigned long>(m_nClusters), globalConcavity);
			(*m_callBack)(msg, progress, globalConcavity,  m_graph.GetNVertices());
        }

	}
        
    bool HACD::Compute(bool fullCH, bool exportDistPoints)
    {
		gCancelRequest = false;

		if ( !m_points || !m_triangles || !m_nPoints || !m_nTriangles)
		{
			return false;
		}
		size_t nV = m_nTriangles;
		if (m_callBack)
		{
			std::ostringstream msg;
			msg << "+ Mesh" << std::endl;
			msg << "\t # vertices                     \t" << m_nPoints << std::endl;
			msg << "\t # triangles                    \t" << m_nTriangles << std::endl;
			msg << "+ Parameters" << std::endl;
			msg << "\t min # of clusters              \t" << m_nMinClusters << std::endl;
			msg << "\t max concavity                  \t" << m_concavity << std::endl;
			msg << "\t compacity weigth               \t" << m_alpha << std::endl;
            msg << "\t volume weigth                  \t" << m_beta << std::endl;
			msg << "\t # vertices per convex-hull     \t" << m_nVerticesPerCH << std::endl;
			msg << "\t scale                          \t" << m_scale << std::endl;
			msg << "\t add extra distance points      \t" << m_addExtraDistPoints << std::endl;
            msg << "\t add neighbours distance points \t" << m_addNeighboursDistPoints << std::endl;
            msg << "\t add face distance points       \t" << m_addFacesPoints << std::endl;
			msg << "\t produce full convex-hulls      \t" << fullCH << std::endl;	
			msg << "\t max. distance to connect CCs   \t" << m_ccConnectDist << std::endl;
			(*m_callBack)(msg.str().c_str(), 0.0, 0.0, nV);
		}
		if (m_callBack) (*m_callBack)("+ Normalizing Data\n", 0.0, 0.0, nV);
		NormalizeData();
		if (m_callBack) (*m_callBack)("+ Creating Graph\n", 0.0, 0.0, nV);
		CreateGraph();
        // Compute the surfaces and perimeters of all the faces
		if (m_callBack) (*m_callBack)("+ Initializing Dual Graph\n", 0.0, 0.0, nV);
		if (gCancelRequest)
			return false;

		InitializeDualGraph();
		if (m_callBack) (*m_callBack)("+ Initializing Priority Queue\n", 0.0, 0.0, nV);
		if (gCancelRequest)
			return false;

        InitializePriorityQueue();
        // we simplify the graph		
		if (m_callBack) (*m_callBack)("+ Simplification ...\n", 0.0, 0.0, m_nTriangles);
		Simplify();
		if (m_callBack) (*m_callBack)("+ Denormalizing Data\n", 0.0, 0.0, m_nClusters);
		DenormalizeData();
		if (m_callBack) (*m_callBack)("+ Computing final convex-hulls\n", 0.0, 0.0, m_nClusters);
        delete [] m_convexHulls;
        m_convexHulls = new ICHull[m_nClusters];
		delete [] m_partition;
	    m_partition = new long [m_nTriangles];
		for (size_t p = 0; p != m_cVertices.size(); ++p) 
		{
			size_t v = m_cVertices[p];
			m_partition[v] = static_cast<long>(p);
			for(size_t a = 0; a < m_graph.m_vertices[v].m_ancestors.size(); a++)
			{
				m_partition[m_graph.m_vertices[v].m_ancestors[a]] = static_cast<long>(p);
			}
            // compute the convex-hull
            const std::map<long, DPoint> & pointsCH =  m_graph.m_vertices[v].m_distPoints;
            std::map<long, DPoint>::const_iterator itCH(pointsCH.begin());
            std::map<long, DPoint>::const_iterator itCHEnd(pointsCH.end());
            for(; itCH != itCHEnd; ++itCH) 
            {
                if (!(itCH->second).m_distOnly)
                {
                    m_convexHulls[p].AddPoint(m_points[itCH->first], itCH->first);
                }
            }
			m_convexHulls[p].SetDistPoints(&m_graph.m_vertices[v].m_distPoints);
            if (fullCH)
            {
	            m_convexHulls[p].Process();
            }
            else
            {
	            m_convexHulls[p].Process(static_cast<unsigned long>(m_nVerticesPerCH));
            }
            if (exportDistPoints)
            {
                itCH = pointsCH.begin();
                for(; itCH != itCHEnd; ++itCH) 
                {
                    if ((itCH->second).m_distOnly)
                    {
                        if (itCH->first >= 0)
                        {
                            m_convexHulls[p].AddPoint(m_points[itCH->first], itCH->first);
                        }
                        else
                        {
                            m_convexHulls[p].AddPoint(m_facePoints[-itCH->first-1], itCH->first);
                        }
                    }
                }
            }
		}       
        return true;
    }
    
    size_t HACD::GetNTrianglesCH(size_t numCH) const
    {
        if (numCH >= m_nClusters)
        {
            return 0;
        }
        return m_convexHulls[numCH].GetMesh().GetNTriangles();
    }
    size_t HACD::GetNPointsCH(size_t numCH) const
    {
        if (numCH >= m_nClusters)
        {
            return 0;
        }
        return m_convexHulls[numCH].GetMesh().GetNVertices();
    }

    bool HACD::GetCH(size_t numCH, Vec3<Real> * const points, Vec3<long> * const triangles)
    {
        if (numCH >= m_nClusters)
        {
            return false;
        }
        m_convexHulls[numCH].GetMesh().GetIFS(points, triangles);
        return true;
    }

    bool HACD::Save(const char * fileName, bool uniColor, long numCluster) const
    {
        std::ofstream fout(fileName);
        if (fout.is_open())
        {
            if (m_callBack)
            {
                char msg[1024];
                sprintf(msg, "Saving %s\n", fileName);
                (*m_callBack)(msg, 0.0, 0.0, m_graph.GetNVertices());
            }
            Material mat;
            if (numCluster < 0)
            {
                for (size_t p = 0; p != m_nClusters; ++p) 
                {
                    if (!uniColor)
                    {
                        mat.m_diffuseColor.X() = mat.m_diffuseColor.Y() = mat.m_diffuseColor.Z() = 0.0;
                        while (mat.m_diffuseColor.X() == mat.m_diffuseColor.Y() ||
                               mat.m_diffuseColor.Z() == mat.m_diffuseColor.Y() ||
                               mat.m_diffuseColor.Z() == mat.m_diffuseColor.X()  )
                        {
                            mat.m_diffuseColor.X() = (rand()%100) / 100.0;
                            mat.m_diffuseColor.Y() = (rand()%100) / 100.0;
                            mat.m_diffuseColor.Z() = (rand()%100) / 100.0;
                        }
                    }
                    m_convexHulls[p].GetMesh().SaveVRML2(fout, mat);
                }
            }
            else if (numCluster < static_cast<long>(m_cVertices.size()))
            {
                m_convexHulls[numCluster].GetMesh().SaveVRML2(fout, mat);
            }
            fout.close();
            return true;
        }
        else
        {
            if (m_callBack)
            {
                char msg[1024];
                sprintf(msg, "Error saving %s\n", fileName);
                (*m_callBack)(msg, 0.0, 0.0, m_graph.GetNVertices());
            }
            return false;
        }
    }
}


