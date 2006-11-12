#ifndef GIM_BOX_COLLISION_H_INCLUDED
#define GIM_BOX_COLLISION_H_INCLUDED

/*! \file gim_box_collision.h
\author Francisco León Nájera
*/
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

/*! \defgroup BOUND_AABB_OPERATIONS
*/
//! @{

//!Initializes an AABB
#define INVALIDATE_AABB(aabb) {\
    (aabb).minX = G_REAL_INFINITY;\
    (aabb).maxX = -G_REAL_INFINITY;\
    (aabb).minY = G_REAL_INFINITY;\
    (aabb).maxY = -G_REAL_INFINITY;\
    (aabb).minZ = G_REAL_INFINITY;\
    (aabb).maxZ = -G_REAL_INFINITY;\
}\

#define AABB_GET_MIN(aabb,vmin) {\
    vmin[0] = (aabb).minX;\
    vmin[1] = (aabb).minY;\
    vmin[2] = (aabb).minZ;\
}\

#define AABB_GET_MAX(aabb,vmax) {\
    vmax[0] = (aabb).maxX;\
    vmax[1] = (aabb).maxY;\
    vmax[2] = (aabb).maxZ;\
}\

//!Copy boxes
#define AABB_COPY(dest_aabb,src_aabb)\
{\
    (dest_aabb).minX = (src_aabb).minX;\
    (dest_aabb).maxX = (src_aabb).maxX;\
    (dest_aabb).minY = (src_aabb).minY;\
    (dest_aabb).maxY = (src_aabb).maxY;\
    (dest_aabb).minZ = (src_aabb).minZ;\
    (dest_aabb).maxZ = (src_aabb).maxZ;\
}\

//! Computes an Axis aligned box from  a triangle
#define COMPUTEAABB_FOR_TRIANGLE(aabb,V1,V2,V3) {\
    (aabb).minX = GIM_MIN3(V1[0],V2[0],V3[0]);\
    (aabb).maxX = GIM_MAX3(V1[0],V2[0],V3[0]);\
    (aabb).minY = GIM_MIN3(V1[1],V2[1],V3[1]);\
    (aabb).maxY = GIM_MAX3(V1[1],V2[1],V3[1]);\
    (aabb).minZ = GIM_MIN3(V1[2],V2[2],V3[2]);\
    (aabb).maxZ = GIM_MAX3(V1[2],V2[2],V3[2]);\
}\

//! Apply a transform to an AABB
#define AABB_TRANSFORM(dest_box,source_box,mat4trans)\
{\
    vec3f _vx,_vy,_vz;\
    float _vtemp[] = {(source_box.maxX-source_box.minX),(source_box.maxY-source_box.minY),(source_box.maxZ-source_box.minZ)};\
    MAT_GET_COL(mat4trans,_vx,0);\
    VEC_SCALE(_vx,_vtemp[0],_vx); \
    MAT_GET_COL(mat4trans,_vy,1);\
    VEC_SCALE(_vy,_vtemp[1],_vy);    \
    MAT_GET_COL(mat4trans,_vz,2);\
    VEC_SCALE(_vz,_vtemp[2],_vz);\
    float _vtrans[] = {source_box.minX,source_box.minY,source_box.minZ};    \
    MAT_DOT_VEC_3X4(_vtemp,mat4trans,_vtrans);\
    dest_box.minX = dest_box.maxX = _vtemp[0];\
    dest_box.minY = dest_box.maxY = _vtemp[1];\
    dest_box.minZ = dest_box.maxZ = _vtemp[2]; \
    if(_vx[0]<0.0f)	dest_box.minX += _vx[0]; else dest_box.maxX += _vx[0];\
    if(_vx[1]<0.0f)	dest_box.minY += _vx[1]; else dest_box.maxY += _vx[1];\
    if(_vx[2]<0.0f)	dest_box.minZ += _vx[2]; else dest_box.maxZ += _vx[2];\
    if(_vy[0]<0.0f)	dest_box.minX += _vy[0]; else dest_box.maxX += _vy[0];\
    if(_vy[1]<0.0f)	dest_box.minY += _vy[1]; else dest_box.maxY += _vy[1];\
    if(_vy[2]<0.0f)	dest_box.minZ += _vy[2]; else dest_box.maxZ += _vy[2];\
    if(_vz[0]<0.0f)	dest_box.minX += _vz[0]; else dest_box.maxX += _vz[0];\
    if(_vz[1]<0.0f)	dest_box.minY += _vz[1]; else dest_box.maxY += _vz[1];\
    if(_vz[2]<0.0f)	dest_box.minZ += _vz[2]; else dest_box.maxZ += _vz[2];\
}\


//! Merge two boxes to destaabb
#define MERGEBOXES(destaabb,aabb) {\
    (destaabb).minX = GIM_MIN((aabb).minX,(destaabb).minX);\
    (destaabb).minY = GIM_MIN((aabb).minY,(destaabb).minY);\
    (destaabb).minZ = GIM_MIN((aabb).minZ,(destaabb).minZ);\
    (destaabb).maxX = GIM_MAX((aabb).maxX,(destaabb).maxX);\
    (destaabb).maxY = GIM_MAX((aabb).maxY,(destaabb).maxY);\
    (destaabb).maxZ = GIM_MAX((aabb).maxZ,(destaabb).maxZ);\
}\

//! Extends the box
#define AABB_POINT_EXTEND(destaabb,p) {\
    (destaabb).minX = GIM_MIN(p[0],(destaabb).minX);\
    (destaabb).maxX = GIM_MAX(p[0],(destaabb).maxX);\
    (destaabb).minY = GIM_MIN(p[1],(destaabb).minY);\
    (destaabb).maxY = GIM_MAX(p[1],(destaabb).maxY);\
    (destaabb).minZ = GIM_MIN(p[2],(destaabb).minZ);\
    (destaabb).maxZ = GIM_MAX(p[2],(destaabb).maxZ);\
}\

//! Gets the center and the dimension of the AABB
#define AABB_GET_CENTER_EXTEND(aabb,center,extend)\
{\
    extend[0] =  (aabb.maxX - aabb.minX)*0.5f;\
    extend[1] =  (aabb.maxY - aabb.minY)*0.5f;\
    extend[2] =  (aabb.maxZ - aabb.minZ)*0.5f;\
    center[0] =  aabb.minX + extend[0];\
    center[1] =  aabb.minY + extend[1];\
    center[2] =  aabb.minZ + extend[2];\
}\

//! Finds the intersection box of two boxes
#define BOXINTERSECTION(aabb1, aabb2, iaabb) {\
    (iaabb).minX = GIM_MAX((aabb1).minX,(aabb2).minX);\
    (iaabb).minY = GIM_MAX((aabb1).minY,(aabb2).minY);\
    (iaabb).minZ = GIM_MAX((aabb1).minZ,(aabb2).minZ);\
    (iaabb).maxX = GIM_MIN((aabb1).maxX,(aabb2).maxX);\
    (iaabb).maxY = GIM_MIN((aabb1).maxY,(aabb2).maxY);\
    (iaabb).maxZ = GIM_MIN((aabb1).maxZ,(aabb2).maxZ);\
}\

//! Determines if two aligned boxes do intersect
#define AABBCOLLISION(intersected,aabb1,aabb2) {\
	intersected = 1;\
	if ((aabb1).minX > (aabb2).maxX ||\
        (aabb1).maxX < (aabb2).minX ||\
        (aabb1).minY > (aabb2).maxY ||\
        (aabb1).maxY < (aabb2).minY ||\
        (aabb1).minZ > (aabb2).maxZ ||\
        (aabb1).maxZ < (aabb2).minZ )\
	{\
		intersected = 0;\
	}\
}\

#define AXIS_INTERSECT(min,max, a, d,tfirst, tlast,is_intersected) {\
	if(GIM_IS_ZERO(d))\
	{\
        is_intersected = !(a < min || a > max);\
	}\
	else\
	{\
        GREAL a0, a1;\
        a0 = (min - a) / (d);\
        a1 = (max - a) / (d);\
        if(a0 > a1)   GIM_SWAP_NUMBERS(a0, a1);\
        tfirst = GIM_MAX(a0, tfirst);\
        tlast = GIM_MIN(a1, tlast);\
        if (tlast < tfirst)\
        {\
             is_intersected = 0;\
        }\
        else\
        {\
            is_intersected = 1;\
        }\
	}\
}\

/*! \brief Finds the Ray intersection parameter.

\param aabb Aligned box
\param vorigin A vec3f with the origin of the ray
\param vdir A vec3f with the direction of the ray
\param tparam Output parameter
\param tmax Max lenght of the ray
\param is_intersected 1 if the ray collides the box, else false

*/
#define BOX_INTERSECTS_RAY(aabb, vorigin, vdir, tparam, tmax,is_intersected) { \
	GREAL _tfirst = 0.0f, _tlast = tmax;\
	AXIS_INTERSECT(aabb.minX,aabb.maxX,vorigin[0], vdir[0], _tfirst, _tlast,is_intersected);\
	if(is_intersected)\
	{\
        AXIS_INTERSECT(aabb.minY,aabb.maxY,vorigin[1], vdir[1], _tfirst, _tlast,is_intersected);\
	}\
	if(is_intersected)\
	{\
        AXIS_INTERSECT(aabb.minZ,aabb.maxZ,vorigin[2], vdir[2], _tfirst, _tlast,is_intersected);\
	}\
	tparam = _tfirst;\
}\

/*! \brief Finds the Ray intersection parameter.

\param aabb Aligned box
\param vorigin A vec3f with the origin of the ray
\param vdir A vec3f with the direction of the ray
*/
inline int BOX_INTERSECTS_RAY_FAST(aabb3f & aabb, vec3f vorigin,vec3f  vdir)
{
    vec3f extents,center;
    AABB_GET_CENTER_EXTEND(aabb,center,extents);

	GREAL Dx = vorigin[0] - center[0];
	if(GIM_GREATER(Dx, extents[0]) && Dx*vdir[0]>=0.0f)	return 0;
	GREAL Dy = vorigin[1] - center[1];
	if(GIM_GREATER(Dy, extents[1]) && Dy*vdir[1]>=0.0f)	return 0;
	GREAL Dz = vorigin[2] - center[2];
	if(GIM_GREATER(Dz, extents[2]) && Dz*vdir[2]>=0.0f)	return 0;
	GREAL f;
	f = vdir[1] * Dz - vdir[2] * Dy;
	if(fabsf(f) > extents[1]*fabsf(vdir[2]) + extents[2]*fabsf(vdir[1])) return 0;
	f = vdir[2] * Dx - vdir[0] * Dz;
	if(fabsf(f) > extents[0]*fabsf(vdir[2]) + extents[2]*fabsf(vdir[0]))return 0;
	f = vdir[0] * Dy - vdir[1] * Dx;
	if(fabsf(f) > extents[0]*fabsf(vdir[1]) + extents[1]*fabsf(vdir[0]))return 0;
	return 1;
}

#define AABB_PROJECTION_INTERVAL(aabb,direction, vmin, vmax)\
{\
    GREAL _center[] = {(aabb.minX + aabb.maxX)*0.5f, (aabb.minY + aabb.maxY)*0.5f, (aabb.minZ + aabb.maxZ)*0.5f};\
    \
    GREAL _extend[] = {aabb.maxX-_center[0],aabb.maxY-_center[1],aabb.maxZ-_center[2]};\
    GREAL _fOrigin =  VEC_DOT(direction,_center);\
	GREAL _fMaximumExtent = _extend[0]*fabsf(direction[0]) + \
                            _extend[1]*fabsf(direction[1]) + \
                            _extend[2]*fabsf(direction[2]); \
\
    vmin = _fOrigin - _fMaximumExtent; \
    vmax = _fOrigin + _fMaximumExtent; \
}\

#define BOX_PLANE_EPSILON 0.000001f

/*!
classify values:
<ol>
<li> 0 : In back of plane
<li> 1 : Spanning
<li> 2 : In front of
</ol>
*/
#define PLANE_CLASSIFY_BOX(plane,aabb,classify)\
{\
	GREAL _fmin,_fmax; \
	AABB_PROJECTION_INTERVAL(aabb,plane, _fmin, _fmax); \
	if(plane[3] > _fmax + BOX_PLANE_EPSILON ) \
	{ \
		classify = 0;/*In back of*/ \
	} \
	else \
	{ \
		if(plane[3]+BOX_PLANE_EPSILON >=_fmin) \
		{ \
			classify = 1;/*Spanning*/ \
		} \
		else \
		{ \
			classify = 2;/*In front of*/ \
		} \
	} \
}\

//!  Class for transforming a model1 to the space of model0
class GIM_BOX_BOX_TRANSFORM_CACHE
{
public:
    vec3f  m_T1to0;//!< Transforms translation of model1 to model 0
	mat3f m_R1to0;//!< Transforms Rotation of model1 to model 0, equal  to R0' * R1
	mat3f m_AR;//!< Absolute value of m_R1to0

	GIM_BOX_BOX_TRANSFORM_CACHE(mat4f  trans1_to_0)
	{
		COPY_MATRIX_3X3(m_R1to0,trans1_to_0)
        MAT_GET_TRANSLATION(trans1_to_0,m_T1to0)
		int i,j;

        for(i=0;i<3;i++)
        {
            for(j=0;j<3;j++ )
            {
            	m_AR[i][j] = 1e-6f + fabsf(m_R1to0[i][j]);
            }
        }
	}
};



inline int gim_box_box_overlap_trans_conservative(aabb3f * box0,aabb3f * box1,mat4f trans1_to_0)
{
    aabb3f pbox1;
    AABB_TRANSFORM(pbox1,(*box1),trans1_to_0);
    int intersected;
    AABBCOLLISION(intersected,pbox1,(*box0));
    return intersected;
}

inline int gim_box_box_overlap_cache(aabb3f * box0,aabb3f * box1,GIM_BOX_BOX_TRANSFORM_CACHE * boxcache, bool fulltest)
{
	//Taken from OPCODE
	vec3f ea,eb;//extends
	vec3f ca,cb;//extends
	AABB_GET_CENTER_EXTEND((*box0),ca,ea);
	AABB_GET_CENTER_EXTEND((*box1),cb,eb);


	vec3f T;

    GREAL t,t2;
	int i;

	// Class I : A's basis vectors
	for(i=0;i<3;i++)
	{
		T[i] = MAT_DOT_ROW(boxcache->m_R1to0,cb,i) + boxcache->m_T1to0[i] - ca[i];
		t = MAT_DOT_ROW(boxcache->m_AR,eb,i) + ea[i];
		if(GIM_GREATER(T[i], t))	return 0;
	}
	// Class II : B's basis vectors
	for(i=0;i<3;i++)
	{
		t = MAT_DOT_COL(boxcache->m_R1to0,T,i);
		t2 = MAT_DOT_COL(boxcache->m_AR,ea,i) + eb[i];
		if(GIM_GREATER(t,t2))	return 0;
	}
	// Class III : 9 cross products
	if(fulltest)
	{
		int j,m,n,o,p,q,r;
		for(i=0;i<3;i++)
		{
			m = (i+1)%3;
			n = (i+2)%3;
			o = i==0?1:0;
			p = i==2?1:2;
			for(j=0;j<3;j++)
			{
				q = j==2?1:2;
				r = j==0?1:0;
				t = T[n]*boxcache->m_R1to0[m][j] - T[m]*boxcache->m_R1to0[n][j];
				t2 = ea[o]*boxcache->m_AR[p][j] + ea[p]*boxcache->m_AR[o][j] +
					eb[r]*boxcache->m_AR[i][q] + eb[q]*boxcache->m_AR[i][r];
				if(GIM_GREATER(t,t2))	return 0;
			}
		}

	}
	return 1;
}


/// conservative test for overlap between triangle and aabb
inline int gim_box_collide_triangle(vec3f p1, vec3f p2, vec3f p3,aabb3f & box)
{
    if(GIM_MIN3(p1[0],p2[0],p3[0])> box.maxX) return 0;
    if(GIM_MAX3(p1[0],p2[0],p3[0])< box.minX) return 0;

    if(GIM_MIN3(p1[2],p2[2],p3[2])> box.maxZ) return 0;
    if(GIM_MAX3(p1[2],p2[2],p3[2])< box.minZ) return 0;

    if(GIM_MIN3(p1[1],p2[1],p3[1])> box.maxY) return 0;
    if(GIM_MAX3(p1[1],p2[1],p3[1])< box.minY) return 0;

	vec4f plane;
	TRIANGLE_PLANE_FAST(p1,p2,p3,plane);
	char classify;
	PLANE_CLASSIFY_BOX(plane,box,classify);

	if(classify != 1) return 0;

	return 1;
}


//! @}

#endif // GIM_BOX_COLLISION_H_INCLUDED
