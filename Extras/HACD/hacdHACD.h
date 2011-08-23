/* Copyright (c) 2011 Khaled Mamou (kmamou at gmail dot com)
 All rights reserved.
 
 
 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 
 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 
 3. The names of the contributors may not be used to endorse or promote products derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once
#ifndef HACD_HACD_H
#define HACD_HACD_H
#include <hacdVersion.h>
#include <hacdVector.h>
#include <hacdGraph.h>
#include <hacdICHull.h>
#include <set>
#include <vector>
#include <queue>

namespace HACD
{
    const double                                    sc_pi = 3.14159265;
	class HACD;

	// just to be able to set the capcity of the container
	
	template<class _Ty, class _Container = std::vector<_Ty>, class _Pr = std::less<typename _Container::value_type> >
	class reservable_priority_queue: public std::priority_queue<_Ty, _Container, _Pr> 
	{
        typedef typename std::priority_queue<_Ty, _Container, _Pr>::size_type size_type;
	public:
                                                    reservable_priority_queue(size_type capacity = 0) { reserve(capacity); };
		void										reserve(size_type capacity) { this->c.reserve(capacity); } 
        size_type									capacity() const { return this->c.capacity(); } 
	};
	
	//! priority queque element
    class GraphEdgePriorityQueue
    {
		public:
			//! Constructor
			//! @param name edge's id
			//! @param priority edge's priority
													GraphEdgePriorityQueue(long name, Real priority)
													{
														m_name = name;
														m_priority = priority;
													}
			//! Destructor
													~GraphEdgePriorityQueue(void){}
		private:
			long									m_name;						//!< edge name
			Real                                    m_priority;					//!< priority
		//! Operator < for GraphEdgePQ
        friend bool                                 operator<(const GraphEdgePriorityQueue & lhs, const GraphEdgePriorityQueue & rhs);
		//! Operator > for GraphEdgePQ
        friend bool                                 operator>(const GraphEdgePriorityQueue & lhs, const GraphEdgePriorityQueue & rhs);
		friend class HACD;
    };
    inline bool										operator<(const GraphEdgePriorityQueue & lhs, const GraphEdgePriorityQueue & rhs)
													{
														return lhs.m_priority<rhs.m_priority;
													}
    inline bool										operator>(const GraphEdgePriorityQueue & lhs, const GraphEdgePriorityQueue & rhs)
													{
														return lhs.m_priority>rhs.m_priority;
													}
    typedef void (*CallBackFunction)(const char *, double, double, size_t);

	//! Provides an implementation of the Hierarchical Approximate Convex Decomposition (HACD) technique described in "A Simple and Efficient Approach for 3D Mesh Approximate Convex Decomposition" Game Programming Gems 8 - Chapter 2.8, p.202. A short version of the chapter was published in ICIP09 and is available at ftp://ftp.elet.polimi.it/users/Stefano.Tubaro/ICIP_USB_Proceedings_v2/pdfs/0003501.pdf
    class HACD
	{            
    public:

		//! Gives the triangles partitionas an array of size m_nTriangles where the i-th element specifies the cluster to which belong the i-th triangle
		//! @return triangles partition
		const long * const							GetPartition() const { return m_partition;}
        //! Sets the scale factor
		//! @param scale scale factor
		void										SetScaleFactor(double  scale) { m_scale = scale;}
		//! Gives the scale factor
		//! @return scale factor
		const double								GetScaleFactor() const { return m_scale;}
		//! Sets the call-back function
		//! @param callBack pointer to the call-back function
		void										SetCallBack(CallBackFunction  callBack) { m_callBack = callBack;}
		//! Gives the call-back function
		//! @return pointer to the call-back function
		const CallBackFunction                      GetCallBack() const { return m_callBack;}
        
        //! Specifies whether faces points should be added when computing the concavity
		//! @param addFacesPoints true = faces points should be added
		void										SetAddFacesPoints(bool  addFacesPoints) { m_addFacesPoints = addFacesPoints;}
		//! Specifies wheter faces points should be added when computing the concavity
		//! @return true = faces points should be added
		const bool									GetAddFacesPoints() const { return m_addFacesPoints;}
        //! Specifies whether extra points should be added when computing the concavity
		//! @param addExteraDistPoints true = extra points should be added
		void										SetAddExtraDistPoints(bool  addExtraDistPoints) { m_addExtraDistPoints = addExtraDistPoints;}
		//! Specifies wheter extra points should be added when computing the concavity
		//! @return true = extra points should be added
		const bool									GetAddExtraDistPoints() const { return m_addExtraDistPoints;}
        //! Specifies whether extra points should be added when computing the concavity
		//! @param addExteraDistPoints true = extra points should be added
		void										SetAddNeighboursDistPoints(bool  addNeighboursDistPoints) { m_addNeighboursDistPoints = addNeighboursDistPoints;}
		//! Specifies wheter extra points should be added when computing the concavity
		//! @return true = extra points should be added
		const bool									GetAddNeighboursDistPoints() const { return m_addNeighboursDistPoints;}
        //! Sets the points of the input mesh (Remark: the input points will be scaled and shifted. Use DenormalizeData() to invert those operations)
		//! @param points pointer to the input points
		void										SetPoints(Vec3<Real>  * points) { m_points = points;}
		//! Gives the points of the input mesh (Remark: the input points will be scaled and shifted. Use DenormalizeData() to invert those operations)
		//! @return pointer to the input points 
		const Vec3<Real> *                          GetPoints() const { return m_points;}
		//! Sets the triangles of the input mesh.
		//! @param triangles points pointer to the input points
		void										SetTriangles(Vec3<long>  * triangles) { m_triangles = triangles;}
		//! Gives the triangles in the input mesh 
		//! @return pointer to the input triangles 
		const Vec3<long>   *			            GetTriangles() const { return m_triangles;}
		//! Sets the number of points in the input mesh.
		//! @param nPoints number of points the input mesh
		void										SetNPoints(size_t nPoints) { m_nPoints = nPoints;}
		//! Gives the number of points in the input mesh.
		//! @return number of points the input mesh
		const size_t								GetNPoints() const { return m_nPoints;}
		//! Sets the number of triangles in the input mesh.
		//! @param nTriangles number of triangles in the input mesh
		void										SetNTriangles(size_t nTriangles) { m_nTriangles = nTriangles;}
		//! Gives the number of triangles in the input mesh.
		//! @return number of triangles the input mesh
		const size_t								GetNTriangles() const { return m_nTriangles;}
		//! Sets the minimum number of clusters to be generated.
		//! @param nClusters minimum number of clusters
		void										SetNClusters(size_t nClusters) { m_nMinClusters = nClusters;}
		//! Gives the number of generated clusters.
		//! @return number of generated clusters
		const size_t								GetNClusters() const { return m_nClusters;}
		//! Sets the maximum allowed concavity.
		//! @param concavity maximum concavity
		void										SetConcavity(double concavity) { m_concavity = concavity;}
		//! Gives the maximum allowed concavity.
		//! @return maximum concavity
		double                                      GetConcavity() const { return m_concavity;}
		//! Sets the maximum allowed distance to get CCs connected.
		//! @param concavity maximum distance to get CCs connected
		void										SetConnectDist(double ccConnectDist) { m_ccConnectDist = ccConnectDist;}
		//! Gives the maximum allowed distance to get CCs connected.
		//! @return maximum distance to get CCs connected
		double                                      GetConnectDist() const { return m_ccConnectDist;}        
        //! Sets the volume weight.
		//! @param beta volume weight
        void										SetVolumeWeight(double beta) { m_beta = beta;}
		//! Gives the volume weight.
		//! @return volume weight
        double                                      GetVolumeWeight() const { return m_beta;}	
		//! Sets the compacity weight (i.e. parameter alpha in ftp://ftp.elet.polimi.it/users/Stefano.Tubaro/ICIP_USB_Proceedings_v2/pdfs/0003501.pdf).
		//! @param alpha compacity weight
        void										SetCompacityWeight(double alpha) { m_alpha = alpha;}
		//! Gives the compacity weight (i.e. parameter alpha in ftp://ftp.elet.polimi.it/users/Stefano.Tubaro/ICIP_USB_Proceedings_v2/pdfs/0003501.pdf).
		//! @return compacity weight
        double                                      GetCompacityWeight() const { return m_alpha;}	
		//! Sets the maximum number of vertices for each generated convex-hull.
		//! @param nVerticesPerCH maximum # vertices per CH
        void										SetNVerticesPerCH(size_t nVerticesPerCH) { m_nVerticesPerCH = nVerticesPerCH;}
		//! Gives the maximum number of vertices for each generated convex-hull.
		//! @return maximum # vertices per CH
		const size_t								GetNVerticesPerCH() const { return m_nVerticesPerCH;}
		//! Gives the number of vertices for the cluster number numCH.
		//! @return number of vertices
		size_t                                      GetNPointsCH(size_t numCH) const;
		//! Gives the number of triangles for the cluster number numCH.
		//! @param numCH cluster's number
		//! @return number of triangles
        size_t                                      GetNTrianglesCH(size_t numCH) const;
		//! Gives the vertices and the triangles of the cluster number numCH.
		//! @param numCH cluster's number
		//! @param points pointer to the vector of points to be filled
		//! @param triangles pointer to the vector of triangles to be filled
		//! @return true if sucess
        bool                                        GetCH(size_t numCH, Vec3<Real> * const points, Vec3<long> * const triangles);     
		//! Computes the HACD decomposition.
		//! @param fullCH specifies whether to generate convex-hulls with a full or limited (i.e. < m_nVerticesPerCH) number of vertices
        //! @param exportDistPoints specifies wheter distance points should ne exported or not (used only for debugging).
		//! @return true if sucess
        bool										Compute(bool fullCH=false, bool exportDistPoints=false);
		//! Saves the generated convex-hulls in a VRML 2.0 file.
		//! @param fileName the output file name
		//! @param uniColor specifies whether the different convex-hulls should have the same color or not
        //! @param numCluster specifies the cluster to be saved, if numCluster < 0 export all clusters
        //! @return true if sucess
		bool										Save(const char * fileName, bool uniColor, long numCluster=-1) const;
		//! Shifts and scales to the data to have all the coordinates between 0.0 and 1000.0.
		void										NormalizeData();
		//! Inverse the operations applied by NormalizeData().
		void										DenormalizeData();
        //! Constructor.
													HACD(void);
		//! Destructor.
													~HACD(void);

	private:
		//! Gives the edge index.
		//! @param a first vertex id
		//! @param b second vertex id
		//! @return edge's index
		static unsigned long long					GetEdgeIndex(unsigned long long a, unsigned long long b) 
													{ 
														if (a > b) return (a << 32) + b;
														else	   return (b << 32) + a;
													}
		//! Computes the concavity of a cluster.
		//! @param ch the cluster's convex-hull
		//! @param distPoints the cluster's points 
		//! @return cluster's concavity
		double										Concavity(ICHull & ch, std::map<long, DPoint> & distPoints);
		//! Computes the perimeter of a cluster.
		//! @param triIndices the cluster's triangles
		//! @param distPoints the cluster's points 
		//! @return cluster's perimeter
        double										ComputePerimeter(const std::vector<long> & triIndices) const;
		//! Creates the Graph by associating to each mesh triangle a vertex in the graph and to each couple of adjacent triangles an edge in the graph.
        void										CreateGraph();	
		//! Initializes the graph costs and computes the vertices normals
        void										InitializeDualGraph();
		//! Computes the cost of an edge
		//! @param e edge's id
        void                                        ComputeEdgeCost(size_t e);
		//! Initializes the priority queue
		//! @param fast specifies whether fast mode is used
		//! @return true if success
        bool                                        InitializePriorityQueue();
        //! Cleans the intersection between convex-hulls
        void                                        CleanClusters();
        //! Computes convex-hulls from partition information
        //! @param fullCH specifies whether to generate convex-hulls with a full or limited (i.e. < m_nVerticesPerCH) number of vertices
		void										ComputeConvexHulls(bool fullCH);
		//! Simplifies the graph
		//! @param fast specifies whether fast mode is used
		void										Simplify();

	private:
		double										m_scale;					//>! scale factor used for NormalizeData() and DenormalizeData()
        Vec3<long> *								m_triangles;				//>! pointer the triangles array
        Vec3<Real> *                                m_points;					//>! pointer the points array
        Vec3<Real> *                                m_facePoints;               //>! pointer to the faces points array
        Vec3<Real> *                                m_faceNormals;              //>! pointer to the faces normals array
        Vec3<Real> *                                m_normals;					//>! pointer the normals array
        size_t										m_nTriangles;				//>! number of triangles in the original mesh
        size_t										m_nPoints;					//>! number of vertices in the original mesh
        size_t										m_nClusters;				//>! number of clusters
        size_t										m_nMinClusters;				//>! minimum number of clusters
        double										m_ccConnectDist;			//>! maximum allowed distance to connect CCs
        double										m_concavity;				//>! maximum concavity
		double										m_alpha;					//>! compacity weigth
        double                                      m_beta;                     //>! volume weigth
        double										m_diag;						//>! length of the BB diagonal
		Vec3<Real>                                  m_barycenter;				//>! barycenter of the mesh
        std::vector< long >                         m_cVertices;				//>! array of vertices each belonging to a different cluster
        ICHull *                                    m_convexHulls;				//>! convex-hulls associated with the final HACD clusters
		Graph										m_graph;					//>! simplification graph
        size_t                                      m_nVerticesPerCH;			//>! maximum number of vertices per convex-hull
		reservable_priority_queue<GraphEdgePriorityQueue, 
            std::vector<GraphEdgePriorityQueue>,
			std::greater<std::vector<GraphEdgePriorityQueue>::value_type> > m_pqueue;		//!> priority queue
													HACD(const HACD & rhs);
		CallBackFunction							m_callBack;					//>! call-back function
		long *										m_partition;				//>! array of size m_nTriangles where the i-th element specifies the cluster to which belong the i-th triangle
        bool                                        m_addFacesPoints;           //>! specifies whether to add faces points or not
        bool                                        m_addExtraDistPoints;       //>! specifies whether to add extra points for concave shapes or not
		bool										m_addNeighboursDistPoints;  //>! specifies whether to add extra points from adjacent clusters or not

	};
}
#endif