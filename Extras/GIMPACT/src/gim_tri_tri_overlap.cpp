
/*
-----------------------------------------------------------------------------
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2006 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com

 This library is free software; you can redistribute it and/or
 modify it under the terms of EITHER:
   (1) The GNU Lesser General Public License as published by the Free
       Software Foundation; either version 2.1 of the License, or (at
       your option) any later version. The text of the GNU Lesser
       General Public License is included with this library in the
       file GIMPACT-LICENSE-LGPL.TXT.
   (2) The BSD-style license that is included with this library in
       the file GIMPACT-LICENSE-BSD.TXT.
   (3) The zlib/libpng license that is included with this library in
       the file GIMPACT-LICENSE-ZLIB.TXT.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
 GIMPACT-LICENSE-LGPL.TXT, GIMPACT-LICENSE-ZLIB.TXT and GIMPACT-LICENSE-BSD.TXT for more details.

-----------------------------------------------------------------------------
*/

#include "GIMPACT/gim_trimesh.h"


#define FABS(x) (float(fabs(x)))        /* implement as is fastest on your machine */

/* some macros */

#define CLASSIFY_TRIPOINTS_BY_FACE(v1,v2,v3,faceplane,out_of_face)\
{   \
    _distances[0] = DISTANCE_PLANE_POINT(faceplane,v1);\
    _distances[1] =  _distances[0] * DISTANCE_PLANE_POINT(faceplane,v2);\
    _distances[2] =  _distances[0] * DISTANCE_PLANE_POINT(faceplane,v3); \
	if(_distances[1]>0.0f && _distances[2]>0.0f)\
	{\
	    out_of_face = 1;\
	}\
	else\
	{\
	    out_of_face = 0;\
	}\
}\

#define CLASSIFY_TRIPOINTS_BY_PLANE(v1,v2,v3,faceplane,out_of_face)\
{   \
    _distances[0] = DISTANCE_PLANE_POINT(faceplane,v1);\
    _distances[1] =  DISTANCE_PLANE_POINT(faceplane,v2);\
    _distances[2] =  DISTANCE_PLANE_POINT(faceplane,v3); \
	if(_distances[0]<=0.0f || _distances[1]<=0.0f || _distances[2]<=0.0f)\
	{\
	    out_of_face = 0;\
	}\
	else\
	{\
	    out_of_face = 1;\
	}\
}\


/* sort so that a<=b */
#define SORT(a,b)       \
             if(a>b)    \
             {          \
               float c; \
               c=a;     \
               a=b;     \
               b=c;     \
             }


/* this edge to edge test is based on Franlin Antonio's gem:
   "Faster Line Segment Intersection", in Graphics Gems III,
   pp. 199-202 */
#define EDGE_EDGE_TEST(V0,U0,U1)                      \
  Bx=U0[i0]-U1[i0];                                   \
  By=U0[i1]-U1[i1];                                   \
  Cx=V0[i0]-U0[i0];                                   \
  Cy=V0[i1]-U0[i1];                                   \
  f=Ay*Bx-Ax*By;                                      \
  d=By*Cx-Bx*Cy;                                      \
  if((f>0 && d>=0 && d<=f) || (f<0 && d<=0 && d>=f))  \
  {                                                   \
    e=Ax*Cy-Ay*Cx;                                    \
    if(f>0)                                           \
    {                                                 \
      if(e>=0 && e<=f) return 1;                      \
    }                                                 \
    else                                              \
    {                                                 \
      if(e<=0 && e>=f) return 1;                      \
    }                                                 \
  }

#define EDGE_AGAINST_TRI_EDGES(V0,V1,U0,U1,U2) \
{                                              \
  float Ax,Ay,Bx,By,Cx,Cy,e,d,f;               \
  Ax=V1[i0]-V0[i0];                            \
  Ay=V1[i1]-V0[i1];                            \
  /* test edge U0,U1 against V0,V1 */          \
  EDGE_EDGE_TEST(V0,U0,U1);                    \
  /* test edge U1,U2 against V0,V1 */          \
  EDGE_EDGE_TEST(V0,U1,U2);                    \
  /* test edge U2,U1 against V0,V1 */          \
  EDGE_EDGE_TEST(V0,U2,U0);                    \
}

#define POINT_IN_TRI(V0,U0,U1,U2)           \
{                                           \
  float a,b,c,d0,d1,d2;                     \
  /* is T1 completly inside T2? */          \
  /* check if V0 is inside tri(U0,U1,U2) */ \
  a=U1[i1]-U0[i1];                          \
  b=-(U1[i0]-U0[i0]);                       \
  c=-a*U0[i0]-b*U0[i1];                     \
  d0=a*V0[i0]+b*V0[i1]+c;                   \
                                            \
  a=U2[i1]-U1[i1];                          \
  b=-(U2[i0]-U1[i0]);                       \
  c=-a*U1[i0]-b*U1[i1];                     \
  d1=a*V0[i0]+b*V0[i1]+c;                   \
                                            \
  a=U0[i1]-U2[i1];                          \
  b=-(U0[i0]-U2[i0]);                       \
  c=-a*U2[i0]-b*U2[i1];                     \
  d2=a*V0[i0]+b*V0[i1]+c;                   \
  if(d0*d1>0.0)                             \
  {                                         \
    if(d0*d2>0.0) return 1;                 \
  }                                         \
}

int coplanar_tri_tri(GIM_TRIANGLE_DATA *tri1,
                    GIM_TRIANGLE_DATA *tri2)
{
   short i0,i1;
   /* first project onto an axis-aligned plane, that maximizes the area */
   /* of the triangles, compute indices: i0,i1. */
   PLANE_MINOR_AXES(tri1->m_planes.m_planes[0], i0, i1);

    /* test all edges of triangle 1 against the edges of triangle 2 */
    EDGE_AGAINST_TRI_EDGES(tri1->m_vertices[0],tri1->m_vertices[1],tri2->m_vertices[0],tri2->m_vertices[1],tri2->m_vertices[2]);
    EDGE_AGAINST_TRI_EDGES(tri1->m_vertices[1],tri1->m_vertices[2],tri2->m_vertices[0],tri2->m_vertices[1],tri2->m_vertices[2]);
    EDGE_AGAINST_TRI_EDGES(tri1->m_vertices[2],tri1->m_vertices[0],tri2->m_vertices[0],tri2->m_vertices[1],tri2->m_vertices[2]);

    /* finally, test if tri1 is totally contained in tri2 or vice versa */
    /*POINT_IN_HULL(tri1->m_vertices[0],(&tri2->m_planes.m_planes[1]),3,i0);
    if(i0==0) return 1;

    POINT_IN_HULL(tri2->m_vertices[0],(&tri1->m_planes.m_planes[1]),3,i0);
    if(i0==0) return 1;*/

	POINT_IN_TRI(tri1->m_vertices[0],tri2->m_vertices[0],tri2->m_vertices[1],tri2->m_vertices[2])
	POINT_IN_TRI(tri2->m_vertices[0],tri1->m_vertices[0],tri1->m_vertices[1],tri1->m_vertices[2])

    return 0;
}



#define NEWCOMPUTE_INTERVALS(VV0,VV1,VV2,D0,D1,D2,D0D1,D0D2,A,B,C,X0,X1) \
{ \
        if(D0D1>0.0f) \
        { \
                /* here we know that D0D2<=0.0 */ \
            /* that is D0, D1 are on the same side, D2 on the other or on the plane */ \
                A=VV2; B=(VV0-VV2)*D2; C=(VV1-VV2)*D2; X0=D2-D0; X1=D2-D1; \
        } \
        else if(D0D2>0.0f)\
        { \
                /* here we know that d0d1<=0.0 */ \
            A=VV1; B=(VV0-VV1)*D1; C=(VV2-VV1)*D1; X0=D1-D0; X1=D1-D2; \
        } \
        else if(D1*D2>0.0f || D0!=0.0f) \
        { \
                /* here we know that d0d1<=0.0 or that D0!=0.0 */ \
                A=VV0; B=(VV1-VV0)*D0; C=(VV2-VV0)*D0; X0=D0-D1; X1=D0-D2; \
        } \
        else if(D1!=0.0f) \
        { \
                A=VV1; B=(VV0-VV1)*D1; C=(VV2-VV1)*D1; X0=D1-D0; X1=D1-D2; \
        } \
        else if(D2!=0.0f) \
        { \
                A=VV2; B=(VV0-VV2)*D2; C=(VV1-VV2)*D2; X0=D2-D0; X1=D2-D1; \
        } \
        else \
        { \
                /* triangles are coplanar */ \
                return coplanar_tri_tri(tri1,tri2); \
        } \
}\


int gim_triangle_triangle_overlap_fast(
							GIM_TRIANGLE_DATA *tri1,
							GIM_TRIANGLE_DATA *tri2)
{
    vec3f _distances;
    char out_of_face;
    CLASSIFY_TRIPOINTS_BY_FACE(tri1->m_vertices[0],tri1->m_vertices[1],tri1->m_vertices[2],tri2->m_planes.m_planes[0],out_of_face);
    if(out_of_face==1) return 0;

	CLASSIFY_TRIPOINTS_BY_PLANE(tri2->m_vertices[0],tri2->m_vertices[1],
		tri2->m_vertices[2],tri1->m_planes.m_planes[0],out_of_face);
    if(out_of_face==1) return 0;


	/*vec3f _points[3];
	VEC_COPY(_points[0],tri2->m_vertices[0]);
	VEC_COPY(_points[1],tri2->m_vertices[1]);
	VEC_COPY(_points[2],tri2->m_vertices[2]);

    CLASSIFY_TRIPOINTS_BY_FACE(_points[0],_points[1],_points[2],tri1->m_planes.m_planes[0],out_of_face);
    if(out_of_face==1) return 0;

	CLASSIFY_TRIPOINTS_BY_PLANE(_points[0],_points[1],_points[2],tri1->m_planes.m_planes[1],out_of_face);
    if(out_of_face==1) return 0;

	CLASSIFY_TRIPOINTS_BY_PLANE(_points[0],_points[1],_points[2],tri1->m_planes.m_planes[2],out_of_face);
    if(out_of_face==1) return 0;

	CLASSIFY_TRIPOINTS_BY_PLANE(_points[0],_points[1],_points[2],tri1->m_planes.m_planes[3],out_of_face);
    if(out_of_face==1) return 0;

	return 1;*/


    short i0,i1;
   /* first project onto an axis-aligned plane, that maximizes the area */
   /* of the triangles, compute indices: i0,i1. */
   PLANE_MINOR_AXES(tri2->m_planes.m_planes[0], i0, i1);
    /* test all edges of triangle 1 against the edges of triangle 2 */
   EDGE_AGAINST_TRI_EDGES(tri1->m_vertices[0],tri1->m_vertices[1],tri2->m_vertices[0],tri2->m_vertices[1],tri2->m_vertices[2]);
   EDGE_AGAINST_TRI_EDGES(tri1->m_vertices[1],tri1->m_vertices[2],tri2->m_vertices[0],tri2->m_vertices[1],tri2->m_vertices[2]);
   EDGE_AGAINST_TRI_EDGES(tri1->m_vertices[2],tri1->m_vertices[0],tri2->m_vertices[0],tri2->m_vertices[1],tri2->m_vertices[2]);

	/* finally, test if tri1 is totally contained in tri2 or vice versa */
    /*POINT_IN_HULL(tri1->m_vertices[0],(&tri2->m_planes.m_planes[1]),3,i0);
    if(i0==0) return 1;

    POINT_IN_HULL(tri2->m_vertices[0],(&tri1->m_planes.m_planes[1]),3,i0);
    if(i0==0) return 1;*/

	POINT_IN_TRI(tri1->m_vertices[0],tri2->m_vertices[0],tri2->m_vertices[1],tri2->m_vertices[2])
	POINT_IN_TRI(tri2->m_vertices[0],tri1->m_vertices[0],tri1->m_vertices[1],tri1->m_vertices[2])

    return 0;
}



int gim_triangle_triangle_overlap(
							GIM_TRIANGLE_DATA *tri1,
							GIM_TRIANGLE_DATA *tri2)
{
    vec3f _distances;
    char out_of_face;
    CLASSIFY_TRIPOINTS_BY_FACE(tri1->m_vertices[0],tri1->m_vertices[1],tri1->m_vertices[2],tri2->m_planes.m_planes[0],out_of_face);
    if(out_of_face==1) return 0;

    CLASSIFY_TRIPOINTS_BY_FACE(tri2->m_vertices[0],tri2->m_vertices[1],tri2->m_vertices[2],tri1->m_planes.m_planes[0],out_of_face);
    if(out_of_face==1) return 0;


    float du0=0,du1=0,du2=0,dv0=0,dv1=0,dv2=0;
    float D[3];
    float isect1[2], isect2[2];
    float du0du1=0,du0du2=0,dv0dv1=0,dv0dv2=0;
    short index;
    float vp0,vp1,vp2;
    float up0,up1,up2;
    float bb,cc,max;

    /* compute direction of intersection line */
    VEC_CROSS(D,tri1->m_planes.m_planes[0],tri2->m_planes.m_planes[0]);

    /* compute and index to the largest component of D */
    max=(float)FABS(D[0]);
    index=0;
    bb=(float)FABS(D[1]);
    cc=(float)FABS(D[2]);
    if(bb>max) max=bb,index=1;
    if(cc>max) max=cc,index=2;

     /* this is the simplified projection onto L*/
     vp0= tri1->m_vertices[0][index];
     vp1= tri1->m_vertices[1][index];
     vp2= tri1->m_vertices[2][index];

     up0= tri2->m_vertices[0][index];
     up1= tri2->m_vertices[1][index];
     up2= tri2->m_vertices[2][index];

    /* compute interval for triangle 1 */
    float a,b,c,x0,x1;
    NEWCOMPUTE_INTERVALS(vp0,vp1,vp2,dv0,dv1,dv2,dv0dv1,dv0dv2,a,b,c,x0,x1);

    /* compute interval for triangle 2 */
    float d,e,f,y0,y1;
    NEWCOMPUTE_INTERVALS(up0,up1,up2,du0,du1,du2,du0du1,du0du2,d,e,f,y0,y1);

    float xx,yy,xxyy,tmp;
    xx=x0*x1;
    yy=y0*y1;
    xxyy=xx*yy;

    tmp=a*xxyy;
    isect1[0]=tmp+b*x1*yy;
    isect1[1]=tmp+c*x0*yy;

    tmp=d*xxyy;
    isect2[0]=tmp+e*xx*y1;
    isect2[1]=tmp+f*xx*y0;

    SORT(isect1[0],isect1[1]);
    SORT(isect2[0],isect2[1]);

    if(isect1[1]<isect2[0] || isect2[1]<isect1[0]) return 0;
    return 1;
}
