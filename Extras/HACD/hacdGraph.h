/* Copyright (c) 2011 Khaled Mamou (kmamou at gmail dot com)
 All rights reserved.
 
 
 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 
 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 
 3. The names of the contributors may not be used to endorse or promote products derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once
#ifndef HACD_GRAPH_H
#define HACD_GRAPH_H
#include "hacdVersion.h"
#include "hacdVector.h"
#include "hacdICHull.h"
#include <map>
#include <vector>
#include <set>

namespace HACD
{
    class GraphVertex;
    class GraphEdge;
    class Graph;
	class HACD;
       
    class GraphVertex  
    {
    public:
        bool                                    AddEdge(long name) 
												{ 
													m_edges.insert(name); 
													return true; 
												}
        bool                                    DeleteEdge(long name);        
                                                GraphVertex();
                                                ~GraphVertex(){ delete m_convexHull;};      
    private:
        long                                    m_name;
        long                                    m_cc;
        std::set<long>                          m_edges;
        bool                                    m_deleted;
        std::vector<long>	                    m_ancestors;
		std::map<long, DPoint>					m_distPoints;

        Real                                    m_error;
        double                                  m_surf;
		double                                  m_volume;
        double                                  m_perimeter;
        double                                  m_concavity;
        ICHull *                                m_convexHull;
		std::set<unsigned long long>			m_boudaryEdges;
        

        friend class GraphEdge;
        friend class Graph;
		friend class HACD;
    };
    
	class GraphEdge 
    {
    public:
                                                GraphEdge();
                                                ~GraphEdge(){delete m_convexHull;};
    private:
        long                                    m_name;
        long                                    m_v1;
        long                                    m_v2;
		std::map<long, DPoint>					m_distPoints;
        Real                                    m_error;
        double                                  m_surf;
		double                                  m_volume;
        double                                  m_perimeter;
        double                                  m_concavity;
        ICHull *                                m_convexHull;
		std::set<unsigned long long>			m_boudaryEdges;
        bool                                    m_deleted;
		

        
        friend class GraphVertex;
        friend class Graph;
		friend class HACD;
    };
    
    class Graph  
    {
    public:
		size_t									GetNEdges() const { return m_nE;}
		size_t									GetNVertices() const { return m_nV;}
        bool                                    EdgeCollapse(long v1, long v2);
        long                                    AddVertex();
        long                                    AddEdge(long v1, long v2);
        bool                                    DeleteEdge(long name);	
        bool                                    DeleteVertex(long name);
        long                                    GetEdgeID(long v1, long v2) const;
		void									Clear();
        void                                    Print() const;
        long                                    ExtractCCs();
        
                                                Graph();
        virtual                                 ~Graph();      
		void									Allocate(size_t nV, size_t nE);

    private:
        size_t                                  m_nCCs;
        size_t                                  m_nV;
        size_t                                  m_nE;
        std::vector<GraphEdge>                  m_edges;
        std::vector<GraphVertex>                m_vertices;

		friend class HACD;
    };
}
#endif
