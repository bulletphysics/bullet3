/* Copyright (c) 2011 Khaled Mamou (kmamou at gmail dot com)
 All rights reserved.
 
 
 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 
 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 
 3. The names of the contributors may not be used to endorse or promote products derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <hacdGraph.h>
namespace HACD
{    
    
    GraphEdge::GraphEdge()
    {
        m_convexHull = 0;
        m_v1 = -1;
        m_v2 = -1;
		m_name = -1;
        m_error = 0;
        m_surf = 0;
        m_perimeter = 0;
        m_concavity = 0;
		m_volume = 0;
        m_deleted = false;
	}
   
    GraphVertex::GraphVertex()
    {
        m_convexHull = 0;
		m_name = -1;
        m_cc = -1;
        m_error = 0;
        m_surf = 0;
        m_perimeter = 0;
        m_concavity = 0;
		m_volume = 0;
        m_deleted = false;
    }
    
    bool GraphVertex::DeleteEdge(long name)
    {
        std::set<long>::iterator it = m_edges.find(name);
        if (it != m_edges.end() )
		{
			m_edges.erase(it);
			return true;
		}
        return false;
    }

    Graph::Graph()
    {
        m_nV = 0;
        m_nE = 0;
        m_nCCs = 0;
    }
    
    Graph::~Graph()
    {
    }
    
	void Graph::Allocate(size_t nV, size_t nE)
	{ 
		m_nV = nV;
		m_edges.reserve(nE);
		m_vertices.resize(nV);
		for(size_t i = 0; i < nV; i++)
		{
			m_vertices[i].m_name = static_cast<long>(i);
		}
	}

    long Graph::AddVertex()
    {
		size_t name = m_vertices.size();
		m_vertices.resize(name+1);
        m_vertices[name].m_name = static_cast<long>(name);
        m_nV++;
        return static_cast<long>(name);
    }
    
    long Graph::AddEdge(long v1, long v2)
    {
		size_t name = m_edges.size();
		m_edges.push_back(GraphEdge());
        m_edges[name].m_name = static_cast<long>(name);
        m_edges[name].m_v1 = v1;
        m_edges[name].m_v2 = v2;
        m_vertices[v1].AddEdge(static_cast<long>(name));
        m_vertices[v2].AddEdge(static_cast<long>(name));
        m_nE++;
		return static_cast<long>(name);
    }

    bool Graph::DeleteEdge(long name)
    {
		if (name < static_cast<long>(m_edges.size()))
		{
            long v1 = m_edges[name].m_v1;
            long v2 = m_edges[name].m_v2;
			m_edges[name].m_deleted = true;
            m_vertices[v1].DeleteEdge(name);
            m_vertices[v2].DeleteEdge(name);
            delete m_edges[name].m_convexHull;
			m_edges[name].m_distPoints.clear();
			m_edges[name].m_boudaryEdges.clear();
            m_edges[name].m_convexHull = 0;
			m_nE--;
			return true;
		}
		return false;
    }
    bool Graph::DeleteVertex(long name)
    {
		if (name < static_cast<long>(m_vertices.size()))
		{
			m_vertices[name].m_deleted = true;
            m_vertices[name].m_edges.clear();
            m_vertices[name].m_ancestors = std::vector<long>();
            delete m_vertices[name].m_convexHull;
			m_vertices[name].m_distPoints.clear();
			m_vertices[name].m_boudaryEdges.clear();
            m_vertices[name].m_convexHull = 0;
			m_nV--;
			return true;
		}
		return false;
    }    
    bool Graph::EdgeCollapse(long v1, long v2)
	{
		long edgeToDelete = GetEdgeID(v1, v2);
        if (edgeToDelete >= 0) 
		{
			// delete the edge (v1, v2)
			DeleteEdge(edgeToDelete);
			// add v2 to v1 ancestors
            m_vertices[v1].m_ancestors.push_back(v2);
			// add v2's ancestors to v1's ancestors
			m_vertices[v1].m_ancestors.insert(m_vertices[v1].m_ancestors.begin(),
											  m_vertices[v2].m_ancestors.begin(), 
											  m_vertices[v2].m_ancestors.end());
			// update adjacency information
			std::set<long> & v1Edges =  m_vertices[v1].m_edges;
			std::set<long>::const_iterator ed(m_vertices[v2].m_edges.begin());
			std::set<long>::const_iterator itEnd(m_vertices[v2].m_edges.end());
			long b = -1;
			for(; ed != itEnd; ++ed) 
			{
				if (m_edges[*ed].m_v1 == v2)
				{
					b = m_edges[*ed].m_v2;
				}
				else
				{
					b = m_edges[*ed].m_v1;
				}
				if (GetEdgeID(v1, b) >= 0)
				{
					m_edges[*ed].m_deleted = true;
					m_vertices[b].DeleteEdge(*ed);
					m_nE--;
				}
				else
				{
					m_edges[*ed].m_v1 = v1;
					m_edges[*ed].m_v2 = b;
					v1Edges.insert(*ed);
				}
			}
			// delete the vertex v2
            DeleteVertex(v2);			
            return true;
        }
		return false;
    }
    
    long Graph::GetEdgeID(long v1, long v2) const
    {
		if (v1 < static_cast<long>(m_vertices.size()) && !m_vertices[v1].m_deleted)
		{
			std::set<long>::const_iterator ed(m_vertices[v1].m_edges.begin());
			std::set<long>::const_iterator itEnd(m_vertices[v1].m_edges.end());
			for(; ed != itEnd; ++ed) 
			{
				if ( (m_edges[*ed].m_v1 == v2) || 
					 (m_edges[*ed].m_v2 == v2)   ) 
				{
					return m_edges[*ed].m_name;
				}
			}
		}
        return -1;
    }
    
	void Graph::Print() const
	{
		std::cout << "-----------------------------" << std::endl;
        std::cout << "vertices (" << m_nV << ")" << std::endl;
        for (size_t v = 0; v < m_vertices.size(); ++v) 
		{
			const GraphVertex & currentVertex = m_vertices[v];
			if (!m_vertices[v].m_deleted)
			{

				std::cout  << currentVertex.m_name	  << "\t";
				std::set<long>::const_iterator ed(currentVertex.m_edges.begin());
				std::set<long>::const_iterator itEnd(currentVertex.m_edges.end());
				for(; ed != itEnd; ++ed) 
				{
					std::cout  << "(" << m_edges[*ed].m_v1 << "," << m_edges[*ed].m_v2 << ") "; 	  
				}
				std::cout << std::endl;
			}			
		}

		std::cout << "vertices (" << m_nE << ")" << std::endl;
		for (size_t e = 0; e < m_edges.size(); ++e) 
		{
			const GraphEdge & currentEdge = m_edges[e];
			if (!m_edges[e].m_deleted)
			{
				std::cout  << currentEdge.m_name	  << "\t(" 
						   << m_edges[e].m_v1		  << "," 
						   << m_edges[e].m_v2		  << ") "<< std::endl;
			}			
		}
	}
	void Graph::Clear()
	{
		m_vertices.clear();
		m_edges.clear();
		m_nV = 0;
        m_nE = 0;
	}
    
    long Graph::ExtractCCs()
	{
        // all CCs to -1
        for (size_t v = 0; v < m_vertices.size(); ++v) 
		{
			if (!m_vertices[v].m_deleted)
			{
				m_vertices[v].m_cc = -1;
			}
        }
        
        // we get the CCs
        m_nCCs = 0;
		long v2 = -1;
		std::vector<long> temp;
        for (size_t v = 0; v < m_vertices.size(); ++v) 
		{
			if (!m_vertices[v].m_deleted && m_vertices[v].m_cc == -1) 
			{
                m_vertices[v].m_cc = static_cast<long>(m_nCCs);
                temp.clear();
                temp.push_back(m_vertices[v].m_name);
                while (temp.size()) 
				{
                    long vertex = temp[temp.size()-1];
                    temp.pop_back();                    
					std::set<long>::const_iterator ed(m_vertices[vertex].m_edges.begin());
					std::set<long>::const_iterator itEnd(m_vertices[vertex].m_edges.end());
					for(; ed != itEnd; ++ed) 
					{
                        if (m_edges[*ed].m_v1 == vertex) 
						{
                            v2 = m_edges[*ed].m_v2;
                        }
                        else 
						{
                            v2 = m_edges[*ed].m_v1;
                        }
                        if ( !m_vertices[v2].m_deleted && m_vertices[v2].m_cc == -1) 
						{
                            m_vertices[v2].m_cc = static_cast<long>(m_nCCs);
                            temp.push_back(v2);
                        }
                    }
                }
                m_nCCs++;
            }
        }        
        return static_cast<long>(m_nCCs);
    }
}