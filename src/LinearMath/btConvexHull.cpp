/*
Stan Melax Convex Hull Computation
Copyright (c) 2003-2006 Stan Melax http://www.melax.com/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include <string.h>

#include "btConvexHull.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btMinMax.h"
#include "LinearMath/btVector3.h"

template <class T>
void Swap(T &a,T &b)
{
	T tmp = a;
	a=b;
	b=tmp;
}


//----------------------------------

class int3  
{
public:
	int x,y,z;
	int3(){};
	int3(int _x,int _y, int _z){x=_x;y=_y;z=_z;}
	const int& operator[](int i) const {return (&x)[i];}
	int& operator[](int i) {return (&x)[i];}
};


//------- Plane ----------

class Plane
{
	public:
	btVector3	normal;
	btScalar	dist;   // distance below origin - the D from plane equasion Ax+By+Cz+D=0
			Plane(const btVector3 &n,btScalar d):normal(n),dist(d){}
			Plane():normal(),dist(0){}
	
};

inline Plane PlaneFlip(const Plane &plane){return Plane(-plane.normal,-plane.dist);}
inline int operator==( const Plane &a, const Plane &b ) { return (a.normal==b.normal && a.dist==b.dist); }
inline int coplanar( const Plane &a, const Plane &b ) { return (a==b || a==PlaneFlip(b)); }


//--------- Utility Functions ------

btVector3  PlaneLineIntersection(const Plane &plane, const btVector3 &p0, const btVector3 &p1);
btVector3  PlaneProject(const Plane &plane, const btVector3 &point);

btVector3  ThreePlaneIntersection(const Plane &p0,const Plane &p1, const Plane &p2)
{
	btVector3 N1 = p0.normal;
	btVector3 N2 = p1.normal;
	btVector3 N3 = p2.normal;

	btVector3 n2n3; n2n3 = N2.cross(N3);
	btVector3 n3n1; n3n1 = N3.cross(N1);
	btVector3 n1n2; n1n2 = N1.cross(N2);

	btScalar quotient = (N1.dot(n2n3));

	btAssert(btFabs(quotient) > btScalar(0.000001));
	
	quotient = btScalar(-1.) / quotient;
	n2n3 *= p0.dist;
	n3n1 *= p1.dist;
	n1n2 *= p2.dist;
	btVector3 potentialVertex = n2n3;
	potentialVertex += n3n1;
	potentialVertex += n1n2;
	potentialVertex *= quotient;

	btVector3 result(potentialVertex.getX(),potentialVertex.getY(),potentialVertex.getZ());
	return result;

}

btScalar   DistanceBetweenLines(const btVector3 &ustart, const btVector3 &udir, const btVector3 &vstart, const btVector3 &vdir, btVector3 *upoint=NULL, btVector3 *vpoint=NULL);
btVector3  TriNormal(const btVector3 &v0, const btVector3 &v1, const btVector3 &v2);
btVector3  NormalOf(const btVector3 *vert, const int n);


btVector3 PlaneLineIntersection(const Plane &plane, const btVector3 &p0, const btVector3 &p1)
{
	// returns the point where the line p0-p1 intersects the plane n&d
				static btVector3 dif;
		dif = p1-p0;
				btScalar dn= dot(plane.normal,dif);
				btScalar t = -(plane.dist+dot(plane.normal,p0) )/dn;
				return p0 + (dif*t);
}

btVector3 PlaneProject(const Plane &plane, const btVector3 &point)
{
	return point - plane.normal * (dot(point,plane.normal)+plane.dist);
}

btVector3 TriNormal(const btVector3 &v0, const btVector3 &v1, const btVector3 &v2)
{
	// return the normal of the triangle
	// inscribed by v0, v1, and v2
	btVector3 cp=cross(v1-v0,v2-v1);
	btScalar m=cp.length();
	if(m==0) return btVector3(1,0,0);
	return cp*(btScalar(1.0)/m);
}


btScalar DistanceBetweenLines(const btVector3 &ustart, const btVector3 &udir, const btVector3 &vstart, const btVector3 &vdir, btVector3 *upoint, btVector3 *vpoint)
{
	static btVector3 cp;
	cp = cross(udir,vdir).normalized();

	btScalar distu = -dot(cp,ustart);
	btScalar distv = -dot(cp,vstart);
	btScalar dist = (btScalar)fabs(distu-distv);
	if(upoint) 
		{
		Plane plane;
		plane.normal = cross(vdir,cp).normalized();
		plane.dist = -dot(plane.normal,vstart);
		*upoint = PlaneLineIntersection(plane,ustart,ustart+udir);
	}
	if(vpoint) 
		{
		Plane plane;
		plane.normal = cross(udir,cp).normalized();
		plane.dist = -dot(plane.normal,ustart);
		*vpoint = PlaneLineIntersection(plane,vstart,vstart+vdir);
	}
	return dist;
}




class PHullResult
{
public:

	PHullResult(void)
	{
		mVcount = 0;
		mIndexCount = 0;
		mFaceCount = 0;
		mVertices = 0;
		mIndices  = 0;
	}

	unsigned int mVcount;
	unsigned int mIndexCount;
	unsigned int mFaceCount;
	btVector3*   mVertices;
	unsigned int *mIndices;
};


#define REAL3 btVector3
#define REAL  btScalar

#define COPLANAR   (0)
#define UNDER      (1)
#define OVER       (2)
#define SPLIT      (OVER|UNDER)
#define PAPERWIDTH (btScalar(0.001))

btScalar planetestepsilon = PAPERWIDTH;


class ConvexH 
{
  public:
	class HalfEdge
	{
	  public:
		short ea;         // the other half of the edge (index into edges list)
		unsigned char v;  // the vertex at the start of this edge (index into vertices list)
		unsigned char p;  // the facet on which this edge lies (index into facets list)
		HalfEdge(){}
		HalfEdge(short _ea,unsigned char _v, unsigned char _p):ea(_ea),v(_v),p(_p){}
	};
	btAlignedObjectArray<REAL3> vertices;
	btAlignedObjectArray<HalfEdge> edges;
	btAlignedObjectArray<Plane>  facets;
	ConvexH(int vertices_size,int edges_size,int facets_size);
};

typedef ConvexH::HalfEdge HalfEdge;

ConvexH::ConvexH(int vertices_size,int edges_size,int facets_size)
{
	vertices.resize(vertices_size);
	edges.resize(edges_size);
	facets.resize(facets_size);
}

ConvexH *ConvexHDup(ConvexH *src) {
	ConvexH *dst = new ConvexH(src->vertices.size(),src->edges.size(),src->facets.size());

	int i;
	for (i=0;i<src->vertices.size();i++)
	{
		dst->vertices[i] = src->vertices[i];
	}

	for (i=0;i<src->edges.size();i++)
	{
		dst->edges[i] = src->edges[i];
	}

	for (i=0;i<src->facets.size();i++)
	{
		dst->facets[i] = src->facets[i];
	}

	return dst;
}


int PlaneTest(const Plane &p, const REAL3 &v) {
	REAL a  = dot(v,p.normal)+p.dist;
	int   flag = (a>planetestepsilon)?OVER:((a<-planetestepsilon)?UNDER:COPLANAR);
	return flag;
}

int SplitTest(ConvexH &convex,const Plane &plane) {
	int flag=0;
	for(int i=0;i<convex.vertices.size();i++) {
		flag |= PlaneTest(plane,convex.vertices[i]);
	}
	return flag;
}

class VertFlag
{
public:
	unsigned char planetest;
	unsigned char junk;
	unsigned char undermap;
	unsigned char overmap;
};
class EdgeFlag 
{
public:
	unsigned char planetest;
	unsigned char fixes;
	short undermap;
	short overmap;
};
class PlaneFlag
{
public:
	unsigned char undermap;
	unsigned char overmap;
};
class Coplanar{
public:
	unsigned short ea;
	unsigned char v0;
	unsigned char v1;
};

int AssertIntact(ConvexH &convex) {
	int i;
	int estart=0;
	for(i=0;i<convex.edges.size();i++) {
		if(convex.edges[estart].p!= convex.edges[i].p) {
			estart=i;
		}
		int inext = i+1;
		if(inext>= convex.edges.size() || convex.edges[inext].p != convex.edges[i].p) {
			inext = estart;
		}
		assert(convex.edges[inext].p == convex.edges[i].p);
		int nb = convex.edges[i].ea;
		assert(nb!=255);
		if(nb==255 || nb==-1) return 0;
		assert(nb!=-1);
		assert(i== convex.edges[nb].ea);
	}
	for(i=0;i<convex.edges.size();i++) {
		assert(COPLANAR==PlaneTest(convex.facets[convex.edges[i].p],convex.vertices[convex.edges[i].v]));
		if(COPLANAR!=PlaneTest(convex.facets[convex.edges[i].p],convex.vertices[convex.edges[i].v])) return 0;
		if(convex.edges[estart].p!= convex.edges[i].p) {
			estart=i;
		}
		int i1 = i+1;
		if(i1>= convex.edges.size() || convex.edges[i1].p != convex.edges[i].p) {
			i1 = estart;
		}
		int i2 = i1+1;
		if(i2>= convex.edges.size() || convex.edges[i2].p != convex.edges[i].p) {
			i2 = estart;
		}
		if(i==i2) continue; // i sliced tangent to an edge and created 2 meaningless edges
		REAL3 localnormal = TriNormal(convex.vertices[convex.edges[i ].v],
			                           convex.vertices[convex.edges[i1].v],
			                           convex.vertices[convex.edges[i2].v]);
		assert(dot(localnormal,convex.facets[convex.edges[i].p].normal)>0);
		if(dot(localnormal,convex.facets[convex.edges[i].p].normal)<=0)return 0;
	}
	return 1;
}

// back to back quads
ConvexH *test_btbq() {
	ConvexH *convex = new ConvexH(4,8,2);
	convex->vertices[0] = REAL3(0,0,0);
	convex->vertices[1] = REAL3(1,0,0);
	convex->vertices[2] = REAL3(1,1,0);
	convex->vertices[3] = REAL3(0,1,0);
	convex->facets[0] = Plane(REAL3(0,0,1),0);
	convex->facets[1] = Plane(REAL3(0,0,-1),0);
	convex->edges[0]  = HalfEdge(7,0,0);
	convex->edges[1]  = HalfEdge(6,1,0);
	convex->edges[2]  = HalfEdge(5,2,0);
	convex->edges[3]  = HalfEdge(4,3,0);

	convex->edges[4]  = HalfEdge(3,0,1);
	convex->edges[5]  = HalfEdge(2,3,1);
	convex->edges[6]  = HalfEdge(1,2,1);
	convex->edges[7]  = HalfEdge(0,1,1);
	AssertIntact(*convex);
	return convex;
}
ConvexH *test_cube() {
	ConvexH *convex = new ConvexH(8,24,6);
	convex->vertices[0] = REAL3(0,0,0);
	convex->vertices[1] = REAL3(0,0,1);
	convex->vertices[2] = REAL3(0,1,0);
	convex->vertices[3] = REAL3(0,1,1);
	convex->vertices[4] = REAL3(1,0,0);
	convex->vertices[5] = REAL3(1,0,1);
	convex->vertices[6] = REAL3(1,1,0);
	convex->vertices[7] = REAL3(1,1,1);

	convex->facets[0] = Plane(REAL3(-1,0,0),0);
	convex->facets[1] = Plane(REAL3(1,0,0),-1);
	convex->facets[2] = Plane(REAL3(0,-1,0),0);
	convex->facets[3] = Plane(REAL3(0,1,0),-1);
	convex->facets[4] = Plane(REAL3(0,0,-1),0);
	convex->facets[5] = Plane(REAL3(0,0,1),-1);

	convex->edges[0 ] = HalfEdge(11,0,0);
	convex->edges[1 ] = HalfEdge(23,1,0);
	convex->edges[2 ] = HalfEdge(15,3,0);
	convex->edges[3 ] = HalfEdge(16,2,0);

	convex->edges[4 ] = HalfEdge(13,6,1);
	convex->edges[5 ] = HalfEdge(21,7,1);
	convex->edges[6 ] = HalfEdge( 9,5,1);
	convex->edges[7 ] = HalfEdge(18,4,1);

	convex->edges[8 ] = HalfEdge(19,0,2);
	convex->edges[9 ] = HalfEdge( 6,4,2);
	convex->edges[10] = HalfEdge(20,5,2);
	convex->edges[11] = HalfEdge( 0,1,2);

	convex->edges[12] = HalfEdge(22,3,3);
	convex->edges[13] = HalfEdge( 4,7,3);
	convex->edges[14] = HalfEdge(17,6,3);
	convex->edges[15] = HalfEdge( 2,2,3);

	convex->edges[16] = HalfEdge( 3,0,4);
	convex->edges[17] = HalfEdge(14,2,4);
	convex->edges[18] = HalfEdge( 7,6,4);
	convex->edges[19] = HalfEdge( 8,4,4);
	
	convex->edges[20] = HalfEdge(10,1,5);
	convex->edges[21] = HalfEdge( 5,5,5);
	convex->edges[22] = HalfEdge(12,7,5);
	convex->edges[23] = HalfEdge( 1,3,5);

	
	return convex;
}
ConvexH *ConvexHMakeCube(const REAL3 &bmin, const REAL3 &bmax) {
	ConvexH *convex = test_cube();
	convex->vertices[0] = REAL3(bmin.getX(),bmin.getY(),bmin.getZ());
	convex->vertices[1] = REAL3(bmin.getX(),bmin.getY(),bmax.getZ());
	convex->vertices[2] = REAL3(bmin.getX(),bmax.getY(),bmin.getZ());
	convex->vertices[3] = REAL3(bmin.getX(),bmax.getY(),bmax.getZ());
	convex->vertices[4] = REAL3(bmax.getX(),bmin.getY(),bmin.getZ());
	convex->vertices[5] = REAL3(bmax.getX(),bmin.getY(),bmax.getZ());
	convex->vertices[6] = REAL3(bmax.getX(),bmax.getY(),bmin.getZ());
	convex->vertices[7] = REAL3(bmax.getX(),bmax.getY(),bmax.getZ());

	convex->facets[0] = Plane(REAL3(-1,0,0), bmin.getX());
	convex->facets[1] = Plane(REAL3(1,0,0), -bmax.getX());
	convex->facets[2] = Plane(REAL3(0,-1,0), bmin.getY());
	convex->facets[3] = Plane(REAL3(0,1,0), -bmax.getY());
	convex->facets[4] = Plane(REAL3(0,0,-1), bmin.getZ());
	convex->facets[5] = Plane(REAL3(0,0,1), -bmax.getZ());
	return convex;
}
ConvexH *ConvexHCrop(ConvexH &convex,const Plane &slice)
{
	int i;
	int vertcountunder=0;
	int vertcountover =0;
	static btAlignedObjectArray<int> vertscoplanar;  // existing vertex members of convex that are coplanar
	vertscoplanar.resize(0);
	static btAlignedObjectArray<int> edgesplit;  // existing edges that members of convex that cross the splitplane
	edgesplit.resize(0);

	assert(convex.edges.size()<480);

	EdgeFlag  edgeflag[512];
	VertFlag  vertflag[256];
	PlaneFlag planeflag[128];
	HalfEdge  tmpunderedges[512];
	Plane	  tmpunderplanes[128];
	Coplanar coplanaredges[512];
	int coplanaredges_num=0;

	btAlignedObjectArray<REAL3> createdverts;
	// do the side-of-plane tests
	for(i=0;i<convex.vertices.size();i++) {
		vertflag[i].planetest = PlaneTest(slice,convex.vertices[i]);
		if(vertflag[i].planetest == COPLANAR) {
			// ? vertscoplanar.Add(i);
			vertflag[i].undermap = vertcountunder++;
			vertflag[i].overmap  = vertcountover++;
		}
		else if(vertflag[i].planetest == UNDER)	{
			vertflag[i].undermap = vertcountunder++;
		}
		else {
			assert(vertflag[i].planetest == OVER);
			vertflag[i].overmap  = vertcountover++;
			vertflag[i].undermap = 255; // for debugging purposes
		}
	}
	int vertcountunderold = vertcountunder; // for debugging only

	int under_edge_count =0;
	int underplanescount=0;
	int e0=0;

	for(int currentplane=0; currentplane<convex.facets.size(); currentplane++) {
		int estart =e0;
		int enextface = 0;
		int planeside = 0;
		int e1 = e0+1;
		int vout=-1;
		int vin =-1;
		int coplanaredge = -1;
		do{

			if(e1 >= convex.edges.size() || convex.edges[e1].p!=currentplane) {
				enextface = e1;
				e1=estart;
			}
			HalfEdge &edge0 = convex.edges[e0];
			HalfEdge &edge1 = convex.edges[e1];
			HalfEdge &edgea = convex.edges[edge0.ea];


			planeside |= vertflag[edge0.v].planetest;
			//if((vertflag[edge0.v].planetest & vertflag[edge1.v].planetest)  == COPLANAR) {
			//	assert(ecop==-1);
			//	ecop=e;
			//}


			if(vertflag[edge0.v].planetest == OVER && vertflag[edge1.v].planetest == OVER){
				// both endpoints over plane
				edgeflag[e0].undermap  = -1;
			}
			else if((vertflag[edge0.v].planetest | vertflag[edge1.v].planetest)  == UNDER) {
				// at least one endpoint under, the other coplanar or under
				
				edgeflag[e0].undermap = under_edge_count;
				tmpunderedges[under_edge_count].v = vertflag[edge0.v].undermap;
				tmpunderedges[under_edge_count].p = underplanescount;
				if(edge0.ea < e0) {
					// connect the neighbors
					assert(edgeflag[edge0.ea].undermap !=-1);
					tmpunderedges[under_edge_count].ea = edgeflag[edge0.ea].undermap;
					tmpunderedges[edgeflag[edge0.ea].undermap].ea = under_edge_count;
				}
				under_edge_count++;
			}
			else if((vertflag[edge0.v].planetest | vertflag[edge1.v].planetest)  == COPLANAR) {
				// both endpoints coplanar 
				// must check a 3rd point to see if UNDER
				int e2 = e1+1;
				if(e2>=convex.edges.size() || convex.edges[e2].p!=currentplane) {
					e2 = estart;
				}
				assert(convex.edges[e2].p==currentplane);
				HalfEdge &edge2 = convex.edges[e2];
				if(vertflag[edge2.v].planetest==UNDER) {
					
					edgeflag[e0].undermap = under_edge_count;
					tmpunderedges[under_edge_count].v = vertflag[edge0.v].undermap;
					tmpunderedges[under_edge_count].p = underplanescount;
					tmpunderedges[under_edge_count].ea = -1;
					// make sure this edge is added to the "coplanar" list
					coplanaredge = under_edge_count;
					vout = vertflag[edge0.v].undermap;
					vin  = vertflag[edge1.v].undermap;
					under_edge_count++;
				}
				else {
					edgeflag[e0].undermap = -1;
				}
			}
			else if(vertflag[edge0.v].planetest == UNDER && vertflag[edge1.v].planetest == OVER) {
				// first is under 2nd is over 
				
				edgeflag[e0].undermap = under_edge_count;
				tmpunderedges[under_edge_count].v = vertflag[edge0.v].undermap;
				tmpunderedges[under_edge_count].p = underplanescount;
				if(edge0.ea < e0) {
					assert(edgeflag[edge0.ea].undermap !=-1);
					// connect the neighbors
					tmpunderedges[under_edge_count].ea = edgeflag[edge0.ea].undermap;
					tmpunderedges[edgeflag[edge0.ea].undermap].ea = under_edge_count;
					vout = tmpunderedges[edgeflag[edge0.ea].undermap].v;
				}
				else {
					Plane &p0 = convex.facets[edge0.p];
					Plane &pa = convex.facets[edgea.p];
					createdverts.push_back(ThreePlaneIntersection(p0,pa,slice));
					//createdverts.push_back(PlaneProject(slice,PlaneLineIntersection(slice,convex.vertices[edge0.v],convex.vertices[edgea.v])));
					//createdverts.push_back(PlaneLineIntersection(slice,convex.vertices[edge0.v],convex.vertices[edgea.v]));
					vout = vertcountunder++;
				}
				under_edge_count++;
				/// hmmm something to think about: i might be able to output this edge regarless of 
				// wheter or not we know v-in yet.  ok i;ll try this now:
				tmpunderedges[under_edge_count].v = vout;
				tmpunderedges[under_edge_count].p = underplanescount;
				tmpunderedges[under_edge_count].ea = -1;
				coplanaredge = under_edge_count;
				under_edge_count++;

				if(vin!=-1) {
					// we previously processed an edge  where we came under
					// now we know about vout as well

					// ADD THIS EDGE TO THE LIST OF EDGES THAT NEED NEIGHBOR ON PARTITION PLANE!!
				}

			}
			else if(vertflag[edge0.v].planetest == COPLANAR && vertflag[edge1.v].planetest == OVER) {
				// first is coplanar 2nd is over 
				
				edgeflag[e0].undermap = -1;
				vout = vertflag[edge0.v].undermap;
				// I hate this but i have to make sure part of this face is UNDER before ouputting this vert
				int k=estart;
				assert(edge0.p == currentplane);
				while(!(planeside&UNDER) && k<convex.edges.size() && convex.edges[k].p==edge0.p) {
					planeside |= vertflag[convex.edges[k].v].planetest;
					k++;
				}
				if(planeside&UNDER){
					tmpunderedges[under_edge_count].v = vout;
					tmpunderedges[under_edge_count].p = underplanescount;
					tmpunderedges[under_edge_count].ea = -1;
					coplanaredge = under_edge_count; // hmmm should make a note of the edge # for later on
					under_edge_count++;
					
				}
			}
			else if(vertflag[edge0.v].planetest == OVER && vertflag[edge1.v].planetest == UNDER) {
				// first is over next is under 
				// new vertex!!!
				assert(vin==-1);
				if(e0<edge0.ea) {
					Plane &p0 = convex.facets[edge0.p];
					Plane &pa = convex.facets[edgea.p];
					createdverts.push_back(ThreePlaneIntersection(p0,pa,slice));
					//createdverts.push_back(PlaneLineIntersection(slice,convex.vertices[edge0.v],convex.vertices[edgea.v]));
					//createdverts.push_back(PlaneProject(slice,PlaneLineIntersection(slice,convex.vertices[edge0.v],convex.vertices[edgea.v])));
					vin = vertcountunder++;
				}
				else {
					// find the new vertex that was created by edge[edge0.ea]
					int nea = edgeflag[edge0.ea].undermap;
					assert(tmpunderedges[nea].p==tmpunderedges[nea+1].p);
					vin = tmpunderedges[nea+1].v;
					assert(vin < vertcountunder);
					assert(vin >= vertcountunderold);   // for debugging only
				}
				if(vout!=-1) {
					// we previously processed an edge  where we went over
					// now we know vin too
					// ADD THIS EDGE TO THE LIST OF EDGES THAT NEED NEIGHBOR ON PARTITION PLANE!!
				}
				// output edge
				tmpunderedges[under_edge_count].v = vin;
				tmpunderedges[under_edge_count].p = underplanescount;
				edgeflag[e0].undermap = under_edge_count;
				if(e0>edge0.ea) {
					assert(edgeflag[edge0.ea].undermap !=-1);
					// connect the neighbors
					tmpunderedges[under_edge_count].ea = edgeflag[edge0.ea].undermap;
					tmpunderedges[edgeflag[edge0.ea].undermap].ea = under_edge_count;
				}
				assert(edgeflag[e0].undermap == under_edge_count);
				under_edge_count++;
			}
			else if(vertflag[edge0.v].planetest == OVER && vertflag[edge1.v].planetest == COPLANAR) {
				// first is over next is coplanar 
				
				edgeflag[e0].undermap = -1;
				vin = vertflag[edge1.v].undermap;
				assert(vin!=-1);
				if(vout!=-1) {
					// we previously processed an edge  where we came under
					// now we know both endpoints
					// ADD THIS EDGE TO THE LIST OF EDGES THAT NEED NEIGHBOR ON PARTITION PLANE!!
				}

			}
			else {
				assert(0);
			}
			

			e0=e1;
			e1++; // do the modulo at the beginning of the loop

		} while(e0!=estart) ;
		e0 = enextface;
		if(planeside&UNDER) {
			planeflag[currentplane].undermap = underplanescount;
			tmpunderplanes[underplanescount] = convex.facets[currentplane];
			underplanescount++;
		}
		else {
			planeflag[currentplane].undermap = 0;
		}
		if(vout>=0 && (planeside&UNDER)) {
			assert(vin>=0);
			assert(coplanaredge>=0);
			assert(coplanaredge!=511);
			coplanaredges[coplanaredges_num].ea = coplanaredge;
			coplanaredges[coplanaredges_num].v0 = vin;
			coplanaredges[coplanaredges_num].v1 = vout;
			coplanaredges_num++;
		}
	}

	// add the new plane to the mix:
	if(coplanaredges_num>0) {
		tmpunderplanes[underplanescount++]=slice;
	}
	for(i=0;i<coplanaredges_num-1;i++) {
		if(coplanaredges[i].v1 != coplanaredges[i+1].v0) {
			int j = 0;
			for(j=i+2;j<coplanaredges_num;j++) {
				if(coplanaredges[i].v1 == coplanaredges[j].v0) {
					Coplanar tmp = coplanaredges[i+1];
					coplanaredges[i+1] = coplanaredges[j];
					coplanaredges[j] = tmp;
					break;
				}
			}
			if(j>=coplanaredges_num)
			{
				assert(j<coplanaredges_num);
				return NULL;
			}
		}
	}
	ConvexH *punder = new ConvexH(vertcountunder,under_edge_count+coplanaredges_num,underplanescount);
	ConvexH &under = *punder;
	int k=0;
	for(i=0;i<convex.vertices.size();i++) {
		if(vertflag[i].planetest != OVER){
			under.vertices[k++] = convex.vertices[i];
		}
	}
	i=0;
	while(k<vertcountunder) {
		under.vertices[k++] = createdverts[i++];
	}
	assert(i==createdverts.size());

	for(i=0;i<coplanaredges_num;i++) {
		under.edges[under_edge_count+i].p  = underplanescount-1;
		under.edges[under_edge_count+i].ea = coplanaredges[i].ea;
		tmpunderedges[coplanaredges[i].ea].ea = under_edge_count+i;
		under.edges[under_edge_count+i].v  = coplanaredges[i].v0;
	}
	
	for (i=0;i<under_edge_count;i++)
	{
		under.edges[i] = tmpunderedges[i];
	}
	for (i=0;i<underplanescount;i++)
	{
		under.facets[i] = tmpunderplanes[i];
	}

	return punder;
}



static int candidateplane(Plane *planes,int planes_count,ConvexH *convex,btScalar epsilon)
{
	int p = 0 ;
	REAL md= 0 ;
	int i;
	for(i=0;i<planes_count;i++)
	{
		REAL d=0;
		for(int j=0;j<convex->vertices.size();j++)
		{
			d = btMax(d,dot(convex->vertices[j],planes[i].normal)+planes[i].dist);
		}
		if(i==0 || d>md)
		{
			p=i;
			md=d;
		}
	}
	return (md>epsilon)?p:-1;
}

template<class T>
inline int maxdir(const T *p,int count,const T &dir)
{
	assert(count);
	int m=0;
	for(int i=1;i<count;i++)
	{
		if(dot(p[i],dir)>dot(p[m],dir)) m=i;
	}
	return m;
}


template<class T>
int maxdirfiltered(const T *p,int count,const T &dir,btAlignedObjectArray<int> &allow)
{
	assert(count);
	int m=-1;
	for(int i=0;i<count;i++) 
		if(allow[i])
		{
			if(m==-1 || dot(p[i],dir)>dot(p[m],dir))
				m=i;
		}
	assert(m!=-1);
	return m;
} 

btVector3 orth(const btVector3 &v)
{
	btVector3 a=cross(v,btVector3(0,0,1));
	btVector3 b=cross(v,btVector3(0,1,0));
	if (a.length() > b.length())
	{
		return a.normalized();
	} else {
		return b.normalized();
	}
}


template<class T>
int maxdirsterid(const T *p,int count,const T &dir,btAlignedObjectArray<int> &allow)
{
	int m=-1;
	while(m==-1)
	{
		m = maxdirfiltered(p,count,dir,allow);
		if(allow[m]==3) return m;
		T u = orth(dir);
		T v = cross(u,dir);
		int ma=-1;
		for(btScalar x = btScalar(0.0) ; x<= btScalar(360.0) ; x+= btScalar(45.0))
		{
			btScalar s = sinf(SIMD_RADS_PER_DEG*(x));
			btScalar c = cosf(SIMD_RADS_PER_DEG*(x));
			int mb = maxdirfiltered(p,count,dir+(u*s+v*c)*btScalar(0.025),allow);
			if(ma==m && mb==m)
			{
				allow[m]=3;
				return m;
			}
			if(ma!=-1 && ma!=mb)  // Yuck - this is really ugly
			{
				int mc = ma;
				for(btScalar xx = x-btScalar(40.0) ; xx <= x ; xx+= btScalar(5.0))
				{
					btScalar s = sinf(SIMD_RADS_PER_DEG*(xx));
					btScalar c = cosf(SIMD_RADS_PER_DEG*(xx));
					int md = maxdirfiltered(p,count,dir+(u*s+v*c)*btScalar(0.025),allow);
					if(mc==m && md==m)
					{
						allow[m]=3;
						return m;
					}
					mc=md;
				}
			}
			ma=mb;
		}
		allow[m]=0;
		m=-1;
	}
	assert(0);
	return m;
} 




int operator ==(const int3 &a,const int3 &b) 
{
	for(int i=0;i<3;i++) 
	{
		if(a[i]!=b[i]) return 0;
	}
	return 1;
}

int3 roll3(int3 a) 
{
	int tmp=a[0];
	a[0]=a[1];
	a[1]=a[2];
	a[2]=tmp;
	return a;
}
int isa(const int3 &a,const int3 &b) 
{
	return ( a==b || roll3(a)==b || a==roll3(b) );
}
int b2b(const int3 &a,const int3 &b) 
{
	return isa(a,int3(b[2],b[1],b[0]));
}
int above(btVector3* vertices,const int3& t, const btVector3 &p, btScalar epsilon) 
{
	btVector3 n=TriNormal(vertices[t[0]],vertices[t[1]],vertices[t[2]]);
	return (dot(n,p-vertices[t[0]]) > epsilon); // EPSILON???
}
int hasedge(const int3 &t, int a,int b)
{
	for(int i=0;i<3;i++)
	{
		int i1= (i+1)%3;
		if(t[i]==a && t[i1]==b) return 1;
	}
	return 0;
}
int hasvert(const int3 &t, int v)
{
	return (t[0]==v || t[1]==v || t[2]==v) ;
}
int shareedge(const int3 &a,const int3 &b)
{
	int i;
	for(i=0;i<3;i++)
	{
		int i1= (i+1)%3;
		if(hasedge(a,b[i1],b[i])) return 1;
	}
	return 0;
}

class Tri;

btAlignedObjectArray<Tri*> tris;

class Tri : public int3
{
public:
	int3 n;
	int id;
	int vmax;
	btScalar rise;
	Tri(int a,int b,int c):int3(a,b,c),n(-1,-1,-1)
	{
		id = tris.size();
		tris.push_back(this);
		vmax=-1;
		rise = btScalar(0.0);
	}
	~Tri()
	{
		assert(tris[id]==this);
		tris[id]=NULL;
	}
	int &neib(int a,int b);
};


int &Tri::neib(int a,int b)
{
	static int er=-1;
	int i;
	for(i=0;i<3;i++) 
	{
		int i1=(i+1)%3;
		int i2=(i+2)%3;
		if((*this)[i]==a && (*this)[i1]==b) return n[i2];
		if((*this)[i]==b && (*this)[i1]==a) return n[i2];
	}
	assert(0);
	return er;
}
void b2bfix(Tri* s,Tri*t)
{
	int i;
	for(i=0;i<3;i++) 
	{
		int i1=(i+1)%3;
		int i2=(i+2)%3;
		int a = (*s)[i1];
		int b = (*s)[i2];
		assert(tris[s->neib(a,b)]->neib(b,a) == s->id);
		assert(tris[t->neib(a,b)]->neib(b,a) == t->id);
		tris[s->neib(a,b)]->neib(b,a) = t->neib(b,a);
		tris[t->neib(b,a)]->neib(a,b) = s->neib(a,b);
	}
}

void removeb2b(Tri* s,Tri*t)
{
	b2bfix(s,t);
	delete s;
	delete t;
}

void checkit(Tri *t)
{
	int i;
	assert(tris[t->id]==t);
	for(i=0;i<3;i++)
	{
		int i1=(i+1)%3;
		int i2=(i+2)%3;
		int a = (*t)[i1];
		int b = (*t)[i2];
		assert(a!=b);
		assert( tris[t->n[i]]->neib(b,a) == t->id);
	}
}
void extrude(Tri *t0,int v)
{
	int3 t= *t0;
	int n = tris.size();
	Tri* ta = new Tri(v,t[1],t[2]);
	ta->n = int3(t0->n[0],n+1,n+2);
	tris[t0->n[0]]->neib(t[1],t[2]) = n+0;
	Tri* tb = new Tri(v,t[2],t[0]);
	tb->n = int3(t0->n[1],n+2,n+0);
	tris[t0->n[1]]->neib(t[2],t[0]) = n+1;
	Tri* tc = new Tri(v,t[0],t[1]);
	tc->n = int3(t0->n[2],n+0,n+1);
	tris[t0->n[2]]->neib(t[0],t[1]) = n+2;
	checkit(ta);
	checkit(tb);
	checkit(tc);
	if(hasvert(*tris[ta->n[0]],v)) removeb2b(ta,tris[ta->n[0]]);
	if(hasvert(*tris[tb->n[0]],v)) removeb2b(tb,tris[tb->n[0]]);
	if(hasvert(*tris[tc->n[0]],v)) removeb2b(tc,tris[tc->n[0]]);
	delete t0;

}

Tri *extrudable(btScalar epsilon)
{
	int i;
	Tri *t=NULL;
	for(i=0;i<tris.size();i++)
	{
		if(!t || (tris[i] && t->rise<tris[i]->rise))
		{
			t = tris[i];
		}
	}
	return (t->rise >epsilon)?t:NULL ;
}

class int4
{
public:
	int x,y,z,w;
	int4(){};
	int4(int _x,int _y, int _z,int _w){x=_x;y=_y;z=_z;w=_w;}
	const int& operator[](int i) const {return (&x)[i];}
	int& operator[](int i) {return (&x)[i];}
};



int4 FindSimplex(btVector3 *verts,int verts_count,btAlignedObjectArray<int> &allow)
{
	btVector3 basis[3];
	basis[0] = btVector3( btScalar(0.01), btScalar(0.02), btScalar(1.0) );      
	int p0 = maxdirsterid(verts,verts_count, basis[0],allow);   
	int	p1 = maxdirsterid(verts,verts_count,-basis[0],allow);
	basis[0] = verts[p0]-verts[p1];
	if(p0==p1 || basis[0]==btVector3(0,0,0)) 
		return int4(-1,-1,-1,-1);
	basis[1] = cross(btVector3(     btScalar(1),btScalar(0.02), btScalar(0)),basis[0]);
	basis[2] = cross(btVector3(btScalar(-0.02),     btScalar(1), btScalar(0)),basis[0]);
	if (basis[1].length() > basis[2].length())
	{
		basis[1].normalize();
	} else {
		basis[1] = basis[2];
		basis[1].normalize ();
	}
	int p2 = maxdirsterid(verts,verts_count,basis[1],allow);
	if(p2 == p0 || p2 == p1)
	{
		p2 = maxdirsterid(verts,verts_count,-basis[1],allow);
	}
	if(p2 == p0 || p2 == p1) 
		return int4(-1,-1,-1,-1);
	basis[1] = verts[p2] - verts[p0];
	basis[2] = cross(basis[1],basis[0]).normalized();
	int p3 = maxdirsterid(verts,verts_count,basis[2],allow);
	if(p3==p0||p3==p1||p3==p2) p3 = maxdirsterid(verts,verts_count,-basis[2],allow);
	if(p3==p0||p3==p1||p3==p2) 
		return int4(-1,-1,-1,-1);
	assert(!(p0==p1||p0==p2||p0==p3||p1==p2||p1==p3||p2==p3));
	if(dot(verts[p3]-verts[p0],cross(verts[p1]-verts[p0],verts[p2]-verts[p0])) <0) {Swap(p2,p3);}
	return int4(p0,p1,p2,p3);
}

int calchullgen(btVector3 *verts,int verts_count, int vlimit)
{
	if(verts_count <4) return 0;
	if(vlimit==0) vlimit=1000000000;
	int j;
	btVector3 bmin(*verts),bmax(*verts);
	btAlignedObjectArray<int> isextreme;
	isextreme.reserve(verts_count);
	btAlignedObjectArray<int> allow;
	allow.reserve(verts_count);

	for(j=0;j<verts_count;j++) 
	{
		allow.push_back(1);
		isextreme.push_back(0);
		bmin.setMin (verts[j]);
		bmax.setMax (verts[j]);
	}
	btScalar epsilon = (bmax-bmin).length() * btScalar(0.001);
	btAssert (epsilon != 0.0);


	int4 p = FindSimplex(verts,verts_count,allow);
	if(p.x==-1) return 0; // simplex failed



	btVector3 center = (verts[p[0]]+verts[p[1]]+verts[p[2]]+verts[p[3]]) / btScalar(4.0);  // a valid interior point
	Tri *t0 = new Tri(p[2],p[3],p[1]); t0->n=int3(2,3,1);
	Tri *t1 = new Tri(p[3],p[2],p[0]); t1->n=int3(3,2,0);
	Tri *t2 = new Tri(p[0],p[1],p[3]); t2->n=int3(0,1,3);
	Tri *t3 = new Tri(p[1],p[0],p[2]); t3->n=int3(1,0,2);
	isextreme[p[0]]=isextreme[p[1]]=isextreme[p[2]]=isextreme[p[3]]=1;
	checkit(t0);checkit(t1);checkit(t2);checkit(t3);

	for(j=0;j<tris.size();j++)
	{
		Tri *t=tris[j];
		assert(t);
		assert(t->vmax<0);
		btVector3 n=TriNormal(verts[(*t)[0]],verts[(*t)[1]],verts[(*t)[2]]);
		t->vmax = maxdirsterid(verts,verts_count,n,allow);
		t->rise = dot(n,verts[t->vmax]-verts[(*t)[0]]);
	}
	Tri *te;
	vlimit-=4;
	while(vlimit >0 && (te=extrudable(epsilon)))
	{
		int3 ti=*te;
		int v=te->vmax;
		assert(v != -1);
		assert(!isextreme[v]);  // wtf we've already done this vertex
		isextreme[v]=1;
		//if(v==p0 || v==p1 || v==p2 || v==p3) continue; // done these already
		j=tris.size();
		while(j--) {
			if(!tris[j]) continue;
			int3 t=*tris[j];
			if(above(verts,t,verts[v],btScalar(0.01)*epsilon)) 
			{
				extrude(tris[j],v);
			}
		}
		// now check for those degenerate cases where we have a flipped triangle or a really skinny triangle
		j=tris.size();
		while(j--)
		{
			if(!tris[j]) continue;
			if(!hasvert(*tris[j],v)) break;
			int3 nt=*tris[j];
			if(above(verts,nt,center,btScalar(0.01)*epsilon)  || cross(verts[nt[1]]-verts[nt[0]],verts[nt[2]]-verts[nt[1]]).length()< epsilon*epsilon*btScalar(0.1) )
			{
				Tri *nb = tris[tris[j]->n[0]];
				assert(nb);assert(!hasvert(*nb,v));assert(nb->id<j);
				extrude(nb,v);
				j=tris.size(); 
			}
		} 
		j=tris.size();
		while(j--)
		{
			Tri *t=tris[j];
			if(!t) continue;
			if(t->vmax>=0) break;
			btVector3 n=TriNormal(verts[(*t)[0]],verts[(*t)[1]],verts[(*t)[2]]);
			t->vmax = maxdirsterid(verts,verts_count,n,allow);
			if(isextreme[t->vmax]) 
			{
				t->vmax=-1; // already done that vertex - algorithm needs to be able to terminate.
			}
			else
			{
				t->rise = dot(n,verts[t->vmax]-verts[(*t)[0]]);
			}
		}
		vlimit --;
	}
	return 1;
}

int calchull(btVector3 *verts,int verts_count, int *&tris_out, int &tris_count,int vlimit) 
{
	int rc=calchullgen(verts,verts_count,  vlimit) ;
	if(!rc) return 0;
	btAlignedObjectArray<int> ts;
	int i;

	for(i=0;i<tris.size();i++)
	{
		if(tris[i])
		{
			for(int j=0;j<3;j++)
				ts.push_back((*tris[i])[j]);
			delete tris[i];
		}
	}
	tris_count = ts.size()/3;
	tris_out = (int*)malloc(sizeof(int)*ts.size());
	
	for (i=0;i<ts.size();i++)
	{
		tris_out[i] = ts[i];
	}
	tris.resize(0);

	return 1;
}



int overhull(Plane *planes,int planes_count,btVector3 *verts, int verts_count,int maxplanes, 
			 btVector3 *&verts_out, int &verts_count_out,  int *&faces_out, int &faces_count_out ,btScalar inflate)
{
	int i,j;
	if(verts_count <4) return 0;
	maxplanes = btMin(maxplanes,planes_count);
	btVector3 bmin(verts[0]),bmax(verts[0]);
	for(i=0;i<verts_count;i++) 
	{
		bmin.setMin(verts[i]);
		bmax.setMax(verts[i]);
	}
//	btScalar diameter = magnitude(bmax-bmin);
//	inflate *=diameter;   // RELATIVE INFLATION
	bmin -= btVector3(inflate,inflate,inflate);
	bmax += btVector3(inflate,inflate,inflate);
	for(i=0;i<planes_count;i++)
	{
		planes[i].dist -= inflate;
	}
	btVector3 emin = bmin; // VectorMin(bmin,btVector3(0,0,0));
	btVector3 emax = bmax; // VectorMax(bmax,btVector3(0,0,0));
	btScalar epsilon  = (emax-emin).length() * btScalar(0.025);
	planetestepsilon = (emax-emin).length() * PAPERWIDTH;
	// todo: add bounding cube planes to force bevel. or try instead not adding the diameter expansion ??? must think.
	// ConvexH *convex = ConvexHMakeCube(bmin - btVector3(diameter,diameter,diameter),bmax+btVector3(diameter,diameter,diameter));
	ConvexH *c = ConvexHMakeCube(REAL3(bmin),REAL3(bmax)); 
	int k;
	while(maxplanes-- && (k=candidateplane(planes,planes_count,c,epsilon))>=0)
	{
		ConvexH *tmp = c;
		c = ConvexHCrop(*tmp,planes[k]);
		if(c==NULL) {c=tmp; break;} // might want to debug this case better!!!
		if(!AssertIntact(*c)) {c=tmp; break;} // might want to debug this case better too!!!
		delete tmp;
	}

	assert(AssertIntact(*c));
	//return c;
	faces_out = (int*)malloc(sizeof(int)*(1+c->facets.size()+c->edges.size()));     // new int[1+c->facets.size()+c->edges.size()];
	faces_count_out=0;
	i=0;
	faces_out[faces_count_out++]=-1;
	k=0;
	while(i<c->edges.size())
	{
		j=1;
		while(j+i<c->edges.size() && c->edges[i].p==c->edges[i+j].p) { j++; }
		faces_out[faces_count_out++]=j;
		while(j--)
		{
			faces_out[faces_count_out++] = c->edges[i].v;
			i++;
		}
		k++;
	}
	faces_out[0]=k; // number of faces.
	assert(k==c->facets.size());
	assert(faces_count_out == 1+c->facets.size()+c->edges.size());
	verts_out = new btVector3[c->vertices.size()]; 
	verts_count_out = c->vertices.size();
	for(i=0;i<c->vertices.size();i++)
	{
		verts_out[i] = btVector3(c->vertices[i]);
	}
	c->vertices.resize(0);
	delete c;
	return 1;
}



bool ComputeHull(unsigned int vcount,const btVector3 *vertices,PHullResult &result,unsigned int vlimit)
{



	
	int  *tris_out;
	int    tris_count;
	int ret = calchull( (btVector3 *) vertices, (int) vcount, tris_out, tris_count, vlimit );
	if(!ret) return false;
	result.mIndexCount = (unsigned int) (tris_count*3);
	result.mFaceCount  = (unsigned int) tris_count;
	result.mVertices   = (btVector3*) vertices;
	result.mVcount     = (unsigned int) vcount;
	result.mIndices    = (unsigned int *) tris_out;
	return true;

}


void ReleaseHull(PHullResult &result)
{
	if ( result.mIndices )
	{
	  free(result.mIndices);
	}

	result.mVcount = 0;
	result.mIndexCount = 0;
	result.mIndices = 0;
	result.mVertices = 0;
	result.mIndices  = 0;
}


//*********************************************************************
//*********************************************************************
//********  HullLib header
//*********************************************************************
//*********************************************************************

//*********************************************************************
//*********************************************************************
//********  HullLib implementation
//*********************************************************************
//*********************************************************************

HullError HullLibrary::CreateConvexHull(const HullDesc       &desc,           // describes the input request
																					HullResult           &result)         // contains the resulst
{
	HullError ret = QE_FAIL;


	PHullResult hr;

	unsigned int vcount = desc.mVcount;
	if ( vcount < 8 ) vcount = 8;

	btVector3* vsource = (btVector3*) malloc (sizeof(btVector3)*vcount);

	btVector3 scale;

	unsigned int ovcount;

	bool ok = CleanupVertices(desc.mVcount,desc.mVertices, desc.mVertexStride, ovcount, vsource, desc.mNormalEpsilon, scale ); // normalize point cloud, remove duplicates!

	if ( ok )
	{


		if ( 1 ) // scale vertices back to their original size.
		{
			for (unsigned int i=0; i<ovcount; i++)
			{
				btVector3& v = vsource[i];
				v[0]*=scale[0];
				v[1]*=scale[1];
				v[2]*=scale[2];
			}
		}

		ok = ComputeHull(ovcount,vsource,hr,desc.mMaxVertices);

		if ( ok )
		{

			// re-index triangle mesh so it refers to only used vertices, rebuild a new vertex table.
			btVector3 *vscratch = (btVector3 *) malloc( sizeof(btVector3)*hr.mVcount);
			BringOutYourDead(hr.mVertices,hr.mVcount, vscratch, ovcount, hr.mIndices, hr.mIndexCount );

			ret = QE_OK;

			if ( desc.HasHullFlag(QF_TRIANGLES) ) // if he wants the results as triangle!
			{
				result.mPolygons          = false;
				result.mNumOutputVertices = ovcount;
				result.mOutputVertices    = (btVector3 *)malloc( sizeof(btVector3)*ovcount);
				result.mNumFaces          = hr.mFaceCount;
				result.mNumIndices        = hr.mIndexCount;

				result.mIndices           = (unsigned int *) malloc( sizeof(unsigned int)*hr.mIndexCount);

				memcpy(result.mOutputVertices, vscratch, sizeof(btVector3)*ovcount );

  			if ( desc.HasHullFlag(QF_REVERSE_ORDER) )
				{

					const unsigned int *source = hr.mIndices;
								unsigned int *dest   = result.mIndices;

					for (unsigned int i=0; i<hr.mFaceCount; i++)
					{
						dest[0] = source[2];
						dest[1] = source[1];
						dest[2] = source[0];
						dest+=3;
						source+=3;
					}

				}
				else
				{
					memcpy(result.mIndices, hr.mIndices, sizeof(unsigned int)*hr.mIndexCount);
				}
			}
			else
			{
				result.mPolygons          = true;
				result.mNumOutputVertices = ovcount;
				result.mOutputVertices    = (btVector3 *)malloc( sizeof(btVector3)*ovcount);
				result.mNumFaces          = hr.mFaceCount;
				result.mNumIndices        = hr.mIndexCount+hr.mFaceCount;
				result.mIndices           = (unsigned int *) malloc( sizeof(unsigned int)*result.mNumIndices);
				memcpy(result.mOutputVertices, vscratch, sizeof(btVector3)*ovcount );

				if ( 1 )
				{
					const unsigned int *source = hr.mIndices;
								unsigned int *dest   = result.mIndices;
					for (unsigned int i=0; i<hr.mFaceCount; i++)
					{
						dest[0] = 3;
						if ( desc.HasHullFlag(QF_REVERSE_ORDER) )
						{
							dest[1] = source[2];
							dest[2] = source[1];
							dest[3] = source[0];
						}
						else
						{
							dest[1] = source[0];
							dest[2] = source[1];
							dest[3] = source[2];
						}

						dest+=4;
						source+=3;
					}
				}
			}
			ReleaseHull(hr);
			if ( vscratch )
			{
				free(vscratch);
			}
		}
	}

	if ( vsource )
	{
		free(vsource);
	}


	return ret;
}



HullError HullLibrary::ReleaseResult(HullResult &result) // release memory allocated for this result, we are done with it.
{
	if ( result.mOutputVertices )
	{
		free(result.mOutputVertices);
		result.mOutputVertices = 0;
	}
	if ( result.mIndices )
	{
		free(result.mIndices);
		result.mIndices = 0;
	}
	return QE_OK;
}


static void addPoint(unsigned int &vcount,btVector3 *p,btScalar x,btScalar y,btScalar z)
{
	// XXX, might be broken
	btVector3& dest = p[vcount];
	dest[0] = x;
	dest[1] = y;
	dest[2] = z;
	vcount++;
}


btScalar GetDist(btScalar px,btScalar py,btScalar pz,const btScalar *p2)
{

	btScalar dx = px - p2[0];
	btScalar dy = py - p2[1];
	btScalar dz = pz - p2[2];

	return dx*dx+dy*dy+dz*dz;
}



bool  HullLibrary::CleanupVertices(unsigned int svcount,
				   const btVector3 *svertices,
				   unsigned int stride,
				   unsigned int &vcount,       // output number of vertices
				   btVector3 *vertices,                 // location to store the results.
				   btScalar  normalepsilon,
				   btVector3& scale)
{
	if ( svcount == 0 ) return false;


#define EPSILON btScalar(0.000001) /* close enough to consider two btScalaring point numbers to be 'the same'. */

	vcount = 0;

	btScalar recip[3];

	if ( scale )
	{
		scale[0] = 1;
		scale[1] = 1;
		scale[2] = 1;
	}

	btScalar bmin[3] = {  FLT_MAX,  FLT_MAX,  FLT_MAX };
	btScalar bmax[3] = { -FLT_MAX, -FLT_MAX, -FLT_MAX };

	const char *vtx = (const char *) svertices;

	if ( 1 )
	{
		for (unsigned int i=0; i<svcount; i++)
		{
			const btScalar *p = (const btScalar *) vtx;

			vtx+=stride;

			for (int j=0; j<3; j++)
			{
				if ( p[j] < bmin[j] ) bmin[j] = p[j];
				if ( p[j] > bmax[j] ) bmax[j] = p[j];
			}
		}
	}

	btScalar dx = bmax[0] - bmin[0];
	btScalar dy = bmax[1] - bmin[1];
	btScalar dz = bmax[2] - bmin[2];

	btVector3 center;

	center[0] = dx*btScalar(0.5) + bmin[0];
	center[1] = dy*btScalar(0.5) + bmin[1];
	center[2] = dz*btScalar(0.5) + bmin[2];

	if ( dx < EPSILON || dy < EPSILON || dz < EPSILON || svcount < 3 )
	{

		btScalar len = FLT_MAX;

		if ( dx > EPSILON && dx < len ) len = dx;
		if ( dy > EPSILON && dy < len ) len = dy;
		if ( dz > EPSILON && dz < len ) len = dz;

		if ( len == FLT_MAX )
		{
			dx = dy = dz = btScalar(0.01); // one centimeter
		}
		else
		{
			if ( dx < EPSILON ) dx = len * btScalar(0.05); // 1/5th the shortest non-zero edge.
			if ( dy < EPSILON ) dy = len * btScalar(0.05);
			if ( dz < EPSILON ) dz = len * btScalar(0.05);
		}

		btScalar x1 = center[0] - dx;
		btScalar x2 = center[0] + dx;

		btScalar y1 = center[1] - dy;
		btScalar y2 = center[1] + dy;

		btScalar z1 = center[2] - dz;
		btScalar z2 = center[2] + dz;

		addPoint(vcount,vertices,x1,y1,z1);
		addPoint(vcount,vertices,x2,y1,z1);
		addPoint(vcount,vertices,x2,y2,z1);
		addPoint(vcount,vertices,x1,y2,z1);
		addPoint(vcount,vertices,x1,y1,z2);
		addPoint(vcount,vertices,x2,y1,z2);
		addPoint(vcount,vertices,x2,y2,z2);
		addPoint(vcount,vertices,x1,y2,z2);

		return true; // return cube


	}
	else
	{
		if ( scale )
		{
			scale[0] = dx;
			scale[1] = dy;
			scale[2] = dz;

			recip[0] = 1 / dx;
			recip[1] = 1 / dy;
			recip[2] = 1 / dz;

			center[0]*=recip[0];
			center[1]*=recip[1];
			center[2]*=recip[2];

		}

	}



	vtx = (const char *) svertices;

	for (unsigned int i=0; i<svcount; i++)
	{
		const btVector3 *p = (const btVector3 *)vtx;
		vtx+=stride;

		btScalar px = p->getX();
		btScalar py = p->getY();
		btScalar pz = p->getZ();

		if ( scale )
		{
			px = px*recip[0]; // normalize
			py = py*recip[1]; // normalize
			pz = pz*recip[2]; // normalize
		}

		if ( 1 )
		{
			unsigned int j;

			for (j=0; j<vcount; j++)
			{
				/// XXX might be broken
				btVector3& v = vertices[j];

				btScalar x = v[0];
				btScalar y = v[1];
				btScalar z = v[2];

				btScalar dx = fabsf(x - px );
				btScalar dy = fabsf(y - py );
				btScalar dz = fabsf(z - pz );

				if ( dx < normalepsilon && dy < normalepsilon && dz < normalepsilon )
				{
					// ok, it is close enough to the old one
					// now let us see if it is further from the center of the point cloud than the one we already recorded.
					// in which case we keep this one instead.

					btScalar dist1 = GetDist(px,py,pz,center);
					btScalar dist2 = GetDist(v[0],v[1],v[2],center);

					if ( dist1 > dist2 )
					{
						v[0] = px;
						v[1] = py;
						v[2] = pz;
					}

					break;
				}
			}

			if ( j == vcount )
			{
				btVector3& dest = vertices[vcount];
				dest[0] = px;
				dest[1] = py;
				dest[2] = pz;
				vcount++;
			}
		}
	}

	// ok..now make sure we didn't prune so many vertices it is now invalid.
	if ( 1 )
	{
		btScalar bmin[3] = {  FLT_MAX,  FLT_MAX,  FLT_MAX };
		btScalar bmax[3] = { -FLT_MAX, -FLT_MAX, -FLT_MAX };

		for (unsigned int i=0; i<vcount; i++)
		{
			const btVector3& p = vertices[i];
			for (int j=0; j<3; j++)
			{
				if ( p[j] < bmin[j] ) bmin[j] = p[j];
				if ( p[j] > bmax[j] ) bmax[j] = p[j];
			}
		}

		btScalar dx = bmax[0] - bmin[0];
		btScalar dy = bmax[1] - bmin[1];
		btScalar dz = bmax[2] - bmin[2];

		if ( dx < EPSILON || dy < EPSILON || dz < EPSILON || vcount < 3)
		{
			btScalar cx = dx*btScalar(0.5) + bmin[0];
			btScalar cy = dy*btScalar(0.5) + bmin[1];
			btScalar cz = dz*btScalar(0.5) + bmin[2];

			btScalar len = FLT_MAX;

			if ( dx >= EPSILON && dx < len ) len = dx;
			if ( dy >= EPSILON && dy < len ) len = dy;
			if ( dz >= EPSILON && dz < len ) len = dz;

			if ( len == FLT_MAX )
			{
				dx = dy = dz = btScalar(0.01); // one centimeter
			}
			else
			{
				if ( dx < EPSILON ) dx = len * btScalar(0.05); // 1/5th the shortest non-zero edge.
				if ( dy < EPSILON ) dy = len * btScalar(0.05);
				if ( dz < EPSILON ) dz = len * btScalar(0.05);
			}

			btScalar x1 = cx - dx;
			btScalar x2 = cx + dx;

			btScalar y1 = cy - dy;
			btScalar y2 = cy + dy;

			btScalar z1 = cz - dz;
			btScalar z2 = cz + dz;

			vcount = 0; // add box

			addPoint(vcount,vertices,x1,y1,z1);
			addPoint(vcount,vertices,x2,y1,z1);
			addPoint(vcount,vertices,x2,y2,z1);
			addPoint(vcount,vertices,x1,y2,z1);
			addPoint(vcount,vertices,x1,y1,z2);
			addPoint(vcount,vertices,x2,y1,z2);
			addPoint(vcount,vertices,x2,y2,z2);
			addPoint(vcount,vertices,x1,y2,z2);

			return true;
		}
	}

	return true;
}

void HullLibrary::BringOutYourDead(const btVector3* verts,unsigned int vcount, btVector3* overts,unsigned int &ocount,unsigned int *indices,unsigned indexcount)
{
	unsigned int *used = (unsigned int *)malloc(sizeof(unsigned int)*vcount);
	memset(used,0,sizeof(unsigned int)*vcount);

	ocount = 0;

	for (unsigned int i=0; i<indexcount; i++)
	{
		unsigned int v = indices[i]; // original array index

		assert( v >= 0 && v < vcount );

		if ( used[v] ) // if already remapped
		{
			indices[i] = used[v]-1; // index to new array
		}
		else
		{

			indices[i] = ocount;      // new index mapping

			overts[ocount][0] = verts[v][0]; // copy old vert to new vert array
			overts[ocount][1] = verts[v][1];
			overts[ocount][2] = verts[v][2];

			ocount++; // increment output vert count

			assert( ocount >=0 && ocount <= vcount );

			used[v] = ocount; // assign new index remapping
		}
	}

	free(used);
}
