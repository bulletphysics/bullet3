#ifndef GIM_VECTOR_H_INCLUDED
#define GIM_VECTOR_H_INCLUDED

/*! \file gim_geometry.h
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


#include "GIMPACT/gim_math.h"

/*! \defgroup GEOMETRIC_TYPES
\brief
Basic types and constants for geometry
*/
//! @{

//! Short Integer vector 2D
typedef GSHORT vec2s[2];
//! Integer vector 3D
typedef GSHORT vec3s[3];
//! Integer vector 4D
typedef GSHORT vec4s[4];

//! Short Integer vector 2D
typedef GUSHORT vec2us[2];
//! Integer vector 3D
typedef GUSHORT vec3us[3];
//! Integer vector 4D
typedef GUSHORT vec4us[4];

//! Integer vector 2D
typedef GINT vec2i[2];
//! Integer vector 3D
typedef GINT vec3i[3];
//! Integer vector 4D
typedef GINT vec4i[4];

//! Unsigned Integer vector 2D
typedef GUINT vec2ui[2];
//! Unsigned Integer vector 3D
typedef GUINT vec3ui[3];
//! Unsigned Integer vector 4D
typedef GUINT vec4ui[4];

//! Float vector 2D
typedef GREAL vec2f[2];
//! Float vector 3D
typedef GREAL vec3f[3];
//! Float vector 4D
typedef GREAL vec4f[4];

//! Double vector 2D
typedef GREAL2 vec2d[2];
//! Float vector 3D
typedef GREAL2 vec3d[3];
//! Float vector 4D
typedef GREAL2 vec4d[4];

//! Matrix 2D, row ordered
typedef GREAL mat2f[2][2];
//! Matrix 3D, row ordered
typedef GREAL mat3f[3][3];
//! Matrix 4D, row ordered
typedef GREAL mat4f[4][4];

//! Quaternion
typedef GREAL quatf[4];

//! Axis aligned box
struct aabb3f{
    GREAL minX;
    GREAL maxX;
    GREAL minY;
    GREAL maxY;
    GREAL minZ;
    GREAL maxZ;
};
//typedef struct _aabb3f aabb3f;
//! @}


/*! \defgroup VECTOR_OPERATIONS
Operations for vectors : vec2f,vec3f and vec4f
*/
//! @{

//! Zero out a 2D vector
#define VEC_ZERO_2(a)				\
{						\
   (a)[0] = (a)[1] = 0.0f;			\
}\


//! Zero out a 3D vector
#define VEC_ZERO(a)				\
{						\
   (a)[0] = (a)[1] = (a)[2] = 0.0f;		\
}\


/// Zero out a 4D vector
#define VEC_ZERO_4(a)				\
{						\
   (a)[0] = (a)[1] = (a)[2] = (a)[3] = 0.0f;	\
}\


/// Vector copy
#define VEC_COPY_2(b,a)				\
{						\
   (b)[0] = (a)[0];				\
   (b)[1] = (a)[1];				\
}\


/// Copy 3D vector
#define VEC_COPY(b,a)				\
{						\
   (b)[0] = (a)[0];				\
   (b)[1] = (a)[1];				\
   (b)[2] = (a)[2];				\
}\


/// Copy 4D vector
#define VEC_COPY_4(b,a)				\
{						\
   (b)[0] = (a)[0];				\
   (b)[1] = (a)[1];				\
   (b)[2] = (a)[2];				\
   (b)[3] = (a)[3];				\
}\


/// Vector difference
#define VEC_DIFF_2(v21,v2,v1)			\
{						\
   (v21)[0] = (v2)[0] - (v1)[0];		\
   (v21)[1] = (v2)[1] - (v1)[1];		\
}\


/// Vector difference
#define VEC_DIFF(v21,v2,v1)			\
{						\
   (v21)[0] = (v2)[0] - (v1)[0];		\
   (v21)[1] = (v2)[1] - (v1)[1];		\
   (v21)[2] = (v2)[2] - (v1)[2];		\
}\


/// Vector difference
#define VEC_DIFF_4(v21,v2,v1)			\
{						\
   (v21)[0] = (v2)[0] - (v1)[0];		\
   (v21)[1] = (v2)[1] - (v1)[1];		\
   (v21)[2] = (v2)[2] - (v1)[2];		\
   (v21)[3] = (v2)[3] - (v1)[3];		\
}\


/// Vector sum
#define VEC_SUM_2(v21,v2,v1)			\
{						\
   (v21)[0] = (v2)[0] + (v1)[0];		\
   (v21)[1] = (v2)[1] + (v1)[1];		\
}\


/// Vector sum
#define VEC_SUM(v21,v2,v1)			\
{						\
   (v21)[0] = (v2)[0] + (v1)[0];		\
   (v21)[1] = (v2)[1] + (v1)[1];		\
   (v21)[2] = (v2)[2] + (v1)[2];		\
}\


/// Vector sum
#define VEC_SUM_4(v21,v2,v1)			\
{						\
   (v21)[0] = (v2)[0] + (v1)[0];		\
   (v21)[1] = (v2)[1] + (v1)[1];		\
   (v21)[2] = (v2)[2] + (v1)[2];		\
   (v21)[3] = (v2)[3] + (v1)[3];		\
}\


/// scalar times vector
#define VEC_SCALE_2(c,a,b)			\
{						\
   (c)[0] = (a)*(b)[0];				\
   (c)[1] = (a)*(b)[1];				\
}\


/// scalar times vector
#define VEC_SCALE(c,a,b)			\
{						\
   (c)[0] = (a)*(b)[0];				\
   (c)[1] = (a)*(b)[1];				\
   (c)[2] = (a)*(b)[2];				\
}\


/// scalar times vector
#define VEC_SCALE_4(c,a,b)			\
{						\
   (c)[0] = (a)*(b)[0];				\
   (c)[1] = (a)*(b)[1];				\
   (c)[2] = (a)*(b)[2];				\
   (c)[3] = (a)*(b)[3];				\
}\


/// accumulate scaled vector
#define VEC_ACCUM_2(c,a,b)			\
{						\
   (c)[0] += (a)*(b)[0];			\
   (c)[1] += (a)*(b)[1];			\
}\


/// accumulate scaled vector
#define VEC_ACCUM(c,a,b)			\
{						\
   (c)[0] += (a)*(b)[0];			\
   (c)[1] += (a)*(b)[1];			\
   (c)[2] += (a)*(b)[2];			\
}\


/// accumulate scaled vector
#define VEC_ACCUM_4(c,a,b)			\
{						\
   (c)[0] += (a)*(b)[0];			\
   (c)[1] += (a)*(b)[1];			\
   (c)[2] += (a)*(b)[2];			\
   (c)[3] += (a)*(b)[3];			\
}\


/// Vector dot product
#define VEC_DOT_2(a,b) ((a)[0]*(b)[0] + (a)[1]*(b)[1])


/// Vector dot product
#define VEC_DOT(a,b) ((a)[0]*(b)[0] + (a)[1]*(b)[1] + (a)[2]*(b)[2])

/// Vector dot product
#define VEC_DOT_4(a,b)	((a)[0]*(b)[0] + (a)[1]*(b)[1] + (a)[2]*(b)[2] + (a)[3]*(b)[3])

/// vector impact parameter (squared)
#define VEC_IMPACT_SQ(bsq,direction,position) {\
   GREAL _llel_ = VEC_DOT(direction, position);\
   bsq = VEC_DOT(position, position) - _llel_*_llel_;\
}\


/// vector impact parameter
#define VEC_IMPACT(bsq,direction,position)	{\
   VEC_IMPACT_SQ(bsq,direction,position);		\
   GIM_SQRT(bsq,bsq);					\
}\

/// Vector length
#define VEC_LENGTH_2(a,l)\
{\
    GREAL _pp = VEC_DOT_2(a,a);\
    GIM_SQRT(_pp,l);\
}\


/// Vector length
#define VEC_LENGTH(a,l)\
{\
    GREAL _pp = VEC_DOT(a,a);\
    GIM_SQRT(_pp,l);\
}\


/// Vector length
#define VEC_LENGTH_4(a,l)\
{\
    GREAL _pp = VEC_DOT_4(a,a);\
    GIM_SQRT(_pp,l);\
}\

/// Vector inv length
#define VEC_INV_LENGTH_2(a,l)\
{\
    GREAL _pp = VEC_DOT_2(a,a);\
    GIM_INV_SQRT(_pp,l);\
}\


/// Vector inv length
#define VEC_INV_LENGTH(a,l)\
{\
    GREAL _pp = VEC_DOT(a,a);\
    GIM_INV_SQRT(_pp,l);\
}\


/// Vector inv length
#define VEC_INV_LENGTH_4(a,l)\
{\
    GREAL _pp = VEC_DOT_4(a,a);\
    GIM_INV_SQRT(_pp,l);\
}\



/// distance between two points
#define VEC_DISTANCE(_len,_va,_vb) {\
    vec3f _tmp_;				\
    VEC_DIFF(_tmp_, _vb, _va);			\
    VEC_LENGTH(_tmp_,_len);			\
}\


/// Vector length
#define VEC_CONJUGATE_LENGTH(a,l)\
{\
    GREAL _pp = 1.0 - a[0]*a[0] - a[1]*a[1] - a[2]*a[2];\
    GIM_SQRT(_pp,l);\
}\


/// Vector length
#define VEC_NORMALIZE(a) {	\
    GREAL len;\
    VEC_INV_LENGTH(a,len); \
    if(len<G_REAL_INFINITY)\
    {\
        a[0] *= len;				\
        a[1] *= len;				\
        a[2] *= len;				\
    }						\
}\

/// Set Vector size
#define VEC_RENORMALIZE(a,newlen) {	\
    GREAL len;\
    VEC_INV_LENGTH(a,len); \
    if(len<G_REAL_INFINITY)\
    {\
        len *= newlen;\
        a[0] *= len;				\
        a[1] *= len;				\
        a[2] *= len;				\
    }						\
}\

/// Vector cross
#define VEC_CROSS(c,a,b)		\
{						\
   c[0] = (a)[1] * (b)[2] - (a)[2] * (b)[1];	\
   c[1] = (a)[2] * (b)[0] - (a)[0] * (b)[2];	\
   c[2] = (a)[0] * (b)[1] - (a)[1] * (b)[0];	\
}\


/*! Vector perp -- assumes that n is of unit length
 * accepts vector v, subtracts out any component parallel to n */
#define VEC_PERPENDICULAR(vp,v,n)			\
{						\
   GREAL dot = VEC_DOT(v, n);			\
   vp[0] = (v)[0] - dot*(n)[0];		\
   vp[1] = (v)[1] - dot*(n)[1];		\
   vp[2] = (v)[2] - dot*(n)[2];		\
}\


/*! Vector parallel -- assumes that n is of unit length */
#define VEC_PARALLEL(vp,v,n)			\
{						\
   GREAL dot = VEC_DOT(v, n);			\
   vp[0] = (dot) * (n)[0];			\
   vp[1] = (dot) * (n)[1];			\
   vp[2] = (dot) * (n)[2];			\
}\

/*! Same as Vector parallel --  n can have any length
 * accepts vector v, subtracts out any component perpendicular to n */
#define VEC_PROJECT(vp,v,n)			\
{ \
	GREAL scalar = VEC_DOT(v, n);			\
	scalar/= VEC_DOT(n, n); \
	vp[0] = (scalar) * (n)[0];			\
    vp[1] = (scalar) * (n)[1];			\
    vp[2] = (scalar) * (n)[2];			\
}\


/*! accepts vector v*/
#define VEC_UNPROJECT(vp,v,n)			\
{ \
	GREAL scalar = VEC_DOT(v, n);			\
	scalar = VEC_DOT(n, n)/scalar; \
	vp[0] = (scalar) * (n)[0];			\
    vp[1] = (scalar) * (n)[1];			\
    vp[2] = (scalar) * (n)[2];			\
}\


/*! Vector reflection -- assumes n is of unit length
 Takes vector v, reflects it against reflector n, and returns vr */
#define VEC_REFLECT(vr,v,n)			\
{						\
   GREAL dot = VEC_DOT(v, n);			\
   vr[0] = (v)[0] - 2.0 * (dot) * (n)[0];	\
   vr[1] = (v)[1] - 2.0 * (dot) * (n)[1];	\
   vr[2] = (v)[2] - 2.0 * (dot) * (n)[2];	\
}\


/*! Vector blending
Takes two vectors a, b, blends them together with two scalars */
#define VEC_BLEND_AB(vr,sa,a,sb,b)			\
{						\
   vr[0] = (sa) * (a)[0] + (sb) * (b)[0];	\
   vr[1] = (sa) * (a)[1] + (sb) * (b)[1];	\
   vr[2] = (sa) * (a)[2] + (sb) * (b)[2];	\
}\

/*! Vector blending
Takes two vectors a, b, blends them together with s <=1 */
#define VEC_BLEND(vr,a,b,s)	VEC_BLEND_AB(vr,1-s,a,sb,s)

#define VEC_SET3(a,b,op,c) a[0]=b[0] op c[0]; a[1]=b[1] op c[1]; a[2]=b[2] op c[2];


//! Finds the 2 smallest cartesian coordinates from a vector
#define VEC_MINOR_AXES(vec, i0, i1)\
{\
    GREAL A[] = {fabs(vec[0]),fabs(vec[1]),fabs(vec[2])};\
    if(A[0]>A[1])\
	{\
		if(A[0]>A[2])\
		{\
			i0=1;  /* A[0] is greatest */ \
			i1=2;\
		}\
		else \
		{\
			i0=0;      /* A[2] is greatest */ \
			i1=1; \
		}\
	}\
	else   /* A[0]<=A[1] */ \
	{\
		if(A[2]>A[1]) \
		{ \
			i0=0;      /* A[2] is greatest */ \
			i1=1; \
		}\
		else \
		{\
			i0=0;      /* A[1] is greatest */ \
			i1=2; \
		}\
	} \
}\


//! Finds the bigger cartesian coordinate from a vector
#define VEC_MAYOR_COORD(vec, maxc)\
{\
    maxc = fabs(vec[0])>fabs(vec[1])?(fabs(vec[0])>fabs(vec[2])?0:2):(fabs(vec[1])>fabs(vec[2])?1:2);\
}\


//! @}


/*! \defgroup MATRIX_OPERATIONS
Operations for matrices : mat2f, mat3f and mat4f
*/
//! @{

/// initialize matrix
#define IDENTIFY_MATRIX_3X3(m)			\
{						\
   m[0][0] = 1.0;				\
   m[0][1] = 0.0;				\
   m[0][2] = 0.0;				\
						\
   m[1][0] = 0.0;				\
   m[1][1] = 1.0;				\
   m[1][2] = 0.0;				\
						\
   m[2][0] = 0.0;				\
   m[2][1] = 0.0;				\
   m[2][2] = 1.0;				\
}\

/*! initialize matrix */
#define IDENTIFY_MATRIX_4X4(m)			\
{						\
   m[0][0] = 1.0;				\
   m[0][1] = 0.0;				\
   m[0][2] = 0.0;				\
   m[0][3] = 0.0;				\
						\
   m[1][0] = 0.0;				\
   m[1][1] = 1.0;				\
   m[1][2] = 0.0;				\
   m[1][3] = 0.0;				\
						\
   m[2][0] = 0.0;				\
   m[2][1] = 0.0;				\
   m[2][2] = 1.0;				\
   m[2][3] = 0.0;				\
						\
   m[3][0] = 0.0;				\
   m[3][1] = 0.0;				\
   m[3][2] = 0.0;				\
   m[3][3] = 1.0;				\
}\

/*! initialize matrix */
#define ZERO_MATRIX_4X4(m)			\
{						\
   m[0][0] = 0.0;				\
   m[0][1] = 0.0;				\
   m[0][2] = 0.0;				\
   m[0][3] = 0.0;				\
						\
   m[1][0] = 0.0;				\
   m[1][1] = 0.0;				\
   m[1][2] = 0.0;				\
   m[1][3] = 0.0;				\
						\
   m[2][0] = 0.0;				\
   m[2][1] = 0.0;				\
   m[2][2] = 0.0;				\
   m[2][3] = 0.0;				\
						\
   m[3][0] = 0.0;				\
   m[3][1] = 0.0;				\
   m[3][2] = 0.0;				\
   m[3][3] = 0.0;				\
}\

/*! matrix rotation  X */
#define ROTX_CS(m,cosine,sine)		\
{					\
   /* rotation about the x-axis */	\
					\
   m[0][0] = 1.0;			\
   m[0][1] = 0.0;			\
   m[0][2] = 0.0;			\
   m[0][3] = 0.0;			\
					\
   m[1][0] = 0.0;			\
   m[1][1] = (cosine);			\
   m[1][2] = (sine);			\
   m[1][3] = 0.0;			\
					\
   m[2][0] = 0.0;			\
   m[2][1] = -(sine);			\
   m[2][2] = (cosine);			\
   m[2][3] = 0.0;			\
					\
   m[3][0] = 0.0;			\
   m[3][1] = 0.0;			\
   m[3][2] = 0.0;			\
   m[3][3] = 1.0;			\
}\

/*! matrix rotation  Y */
#define ROTY_CS(m,cosine,sine)		\
{					\
   /* rotation about the y-axis */	\
					\
   m[0][0] = (cosine);			\
   m[0][1] = 0.0;			\
   m[0][2] = -(sine);			\
   m[0][3] = 0.0;			\
					\
   m[1][0] = 0.0;			\
   m[1][1] = 1.0;			\
   m[1][2] = 0.0;			\
   m[1][3] = 0.0;			\
					\
   m[2][0] = (sine);			\
   m[2][1] = 0.0;			\
   m[2][2] = (cosine);			\
   m[2][3] = 0.0;			\
					\
   m[3][0] = 0.0;			\
   m[3][1] = 0.0;			\
   m[3][2] = 0.0;			\
   m[3][3] = 1.0;			\
}\

/*! matrix rotation  Z */
#define ROTZ_CS(m,cosine,sine)		\
{					\
   /* rotation about the z-axis */	\
					\
   m[0][0] = (cosine);			\
   m[0][1] = (sine);			\
   m[0][2] = 0.0;			\
   m[0][3] = 0.0;			\
					\
   m[1][0] = -(sine);			\
   m[1][1] = (cosine);			\
   m[1][2] = 0.0;			\
   m[1][3] = 0.0;			\
					\
   m[2][0] = 0.0;			\
   m[2][1] = 0.0;			\
   m[2][2] = 1.0;			\
   m[2][3] = 0.0;			\
					\
   m[3][0] = 0.0;			\
   m[3][1] = 0.0;			\
   m[3][2] = 0.0;			\
   m[3][3] = 1.0;			\
}\

/*! matrix copy */
#define COPY_MATRIX_2X2(b,a)	\
{				\
   b[0][0] = a[0][0];		\
   b[0][1] = a[0][1];		\
				\
   b[1][0] = a[1][0];		\
   b[1][1] = a[1][1];		\
				\
}\


/*! matrix copy */
#define COPY_MATRIX_2X3(b,a)	\
{				\
   b[0][0] = a[0][0];		\
   b[0][1] = a[0][1];		\
   b[0][2] = a[0][2];		\
				\
   b[1][0] = a[1][0];		\
   b[1][1] = a[1][1];		\
   b[1][2] = a[1][2];		\
}\


/*! matrix copy */
#define COPY_MATRIX_3X3(b,a)	\
{				\
   b[0][0] = a[0][0];		\
   b[0][1] = a[0][1];		\
   b[0][2] = a[0][2];		\
				\
   b[1][0] = a[1][0];		\
   b[1][1] = a[1][1];		\
   b[1][2] = a[1][2];		\
				\
   b[2][0] = a[2][0];		\
   b[2][1] = a[2][1];		\
   b[2][2] = a[2][2];		\
}\


/*! matrix copy */
#define COPY_MATRIX_4X4(b,a)	\
{				\
   b[0][0] = a[0][0];		\
   b[0][1] = a[0][1];		\
   b[0][2] = a[0][2];		\
   b[0][3] = a[0][3];		\
				\
   b[1][0] = a[1][0];		\
   b[1][1] = a[1][1];		\
   b[1][2] = a[1][2];		\
   b[1][3] = a[1][3];		\
				\
   b[2][0] = a[2][0];		\
   b[2][1] = a[2][1];		\
   b[2][2] = a[2][2];		\
   b[2][3] = a[2][3];		\
				\
   b[3][0] = a[3][0];		\
   b[3][1] = a[3][1];		\
   b[3][2] = a[3][2];		\
   b[3][3] = a[3][3];		\
}\


/*! matrix transpose */
#define TRANSPOSE_MATRIX_2X2(b,a)	\
{				\
   b[0][0] = a[0][0];		\
   b[0][1] = a[1][0];		\
				\
   b[1][0] = a[0][1];		\
   b[1][1] = a[1][1];		\
}\


/*! matrix transpose */
#define TRANSPOSE_MATRIX_3X3(b,a)	\
{				\
   b[0][0] = a[0][0];		\
   b[0][1] = a[1][0];		\
   b[0][2] = a[2][0];		\
				\
   b[1][0] = a[0][1];		\
   b[1][1] = a[1][1];		\
   b[1][2] = a[2][1];		\
				\
   b[2][0] = a[0][2];		\
   b[2][1] = a[1][2];		\
   b[2][2] = a[2][2];		\
}\


/*! matrix transpose */
#define TRANSPOSE_MATRIX_4X4(b,a)	\
{				\
   b[0][0] = a[0][0];		\
   b[0][1] = a[1][0];		\
   b[0][2] = a[2][0];		\
   b[0][3] = a[3][0];		\
				\
   b[1][0] = a[0][1];		\
   b[1][1] = a[1][1];		\
   b[1][2] = a[2][1];		\
   b[1][3] = a[3][1];		\
				\
   b[2][0] = a[0][2];		\
   b[2][1] = a[1][2];		\
   b[2][2] = a[2][2];		\
   b[2][3] = a[3][2];		\
				\
   b[3][0] = a[0][3];		\
   b[3][1] = a[1][3];		\
   b[3][2] = a[2][3];		\
   b[3][3] = a[3][3];		\
}\


/*! multiply matrix by scalar */
#define SCALE_MATRIX_2X2(b,s,a)		\
{					\
   b[0][0] = (s) * a[0][0];		\
   b[0][1] = (s) * a[0][1];		\
					\
   b[1][0] = (s) * a[1][0];		\
   b[1][1] = (s) * a[1][1];		\
}\


/*! multiply matrix by scalar */
#define SCALE_MATRIX_3X3(b,s,a)		\
{					\
   b[0][0] = (s) * a[0][0];		\
   b[0][1] = (s) * a[0][1];		\
   b[0][2] = (s) * a[0][2];		\
					\
   b[1][0] = (s) * a[1][0];		\
   b[1][1] = (s) * a[1][1];		\
   b[1][2] = (s) * a[1][2];		\
					\
   b[2][0] = (s) * a[2][0];		\
   b[2][1] = (s) * a[2][1];		\
   b[2][2] = (s) * a[2][2];		\
}\


/*! multiply matrix by scalar */
#define SCALE_MATRIX_4X4(b,s,a)		\
{					\
   b[0][0] = (s) * a[0][0];		\
   b[0][1] = (s) * a[0][1];		\
   b[0][2] = (s) * a[0][2];		\
   b[0][3] = (s) * a[0][3];		\
					\
   b[1][0] = (s) * a[1][0];		\
   b[1][1] = (s) * a[1][1];		\
   b[1][2] = (s) * a[1][2];		\
   b[1][3] = (s) * a[1][3];		\
					\
   b[2][0] = (s) * a[2][0];		\
   b[2][1] = (s) * a[2][1];		\
   b[2][2] = (s) * a[2][2];		\
   b[2][3] = (s) * a[2][3];		\
					\
   b[3][0] = s * a[3][0];		\
   b[3][1] = s * a[3][1];		\
   b[3][2] = s * a[3][2];		\
   b[3][3] = s * a[3][3];		\
}\


/*! multiply matrix by scalar */
#define SCALE_VEC_MATRIX_2X2(b,svec,a)		\
{					\
   b[0][0] = svec[0] * a[0][0];		\
   b[0][1] = svec[0] * a[0][1];		\
					\
   b[1][0] = svec[1] * a[1][0];		\
   b[1][1] = svec[1] * a[1][1];		\
}\


/*! multiply matrix by scalar */
#define SCALE_VEC_MATRIX_3X3(b,svec,a)		\
{					\
   b[0][0] = svec[0] * a[0][0];		\
   b[0][1] = svec[0] * a[0][1];		\
   b[0][2] = svec[0] * a[0][2];		\
					\
   b[1][0] = svec[1] * a[1][0];		\
   b[1][1] = svec[1] * a[1][1];		\
   b[1][2] = svec[1] * a[1][2];		\
					\
   b[2][0] = svec[2] * a[2][0];		\
   b[2][1] = svec[2] * a[2][1];		\
   b[2][2] = svec[2] * a[2][2];		\
}\


/*! multiply matrix by scalar */
#define SCALE_VEC_MATRIX_4X4(b,svec,a)		\
{					\
   b[0][0] = svec[0] * a[0][0];		\
   b[0][1] = svec[0] * a[0][1];		\
   b[0][2] = svec[0] * a[0][2];		\
   b[0][3] = svec[0] * a[0][3];		\
					\
   b[1][0] = svec[1] * a[1][0];		\
   b[1][1] = svec[1] * a[1][1];		\
   b[1][2] = svec[1] * a[1][2];		\
   b[1][3] = svec[1] * a[1][3];		\
					\
   b[2][0] = svec[2] * a[2][0];		\
   b[2][1] = svec[2] * a[2][1];		\
   b[2][2] = svec[2] * a[2][2];		\
   b[2][3] = svec[2] * a[2][3];		\
					\
   b[3][0] = svec[3] * a[3][0];		\
   b[3][1] = svec[3] * a[3][1];		\
   b[3][2] = svec[3] * a[3][2];		\
   b[3][3] = svec[3] * a[3][3];		\
}\


/*! multiply matrix by scalar */
#define ACCUM_SCALE_MATRIX_2X2(b,s,a)		\
{					\
   b[0][0] += (s) * a[0][0];		\
   b[0][1] += (s) * a[0][1];		\
					\
   b[1][0] += (s) * a[1][0];		\
   b[1][1] += (s) * a[1][1];		\
}\


/*! multiply matrix by scalar */
#define ACCUM_SCALE_MATRIX_3X3(b,s,a)		\
{					\
   b[0][0] += (s) * a[0][0];		\
   b[0][1] += (s) * a[0][1];		\
   b[0][2] += (s) * a[0][2];		\
					\
   b[1][0] += (s) * a[1][0];		\
   b[1][1] += (s) * a[1][1];		\
   b[1][2] += (s) * a[1][2];		\
					\
   b[2][0] += (s) * a[2][0];		\
   b[2][1] += (s) * a[2][1];		\
   b[2][2] += (s) * a[2][2];		\
}\


/*! multiply matrix by scalar */
#define ACCUM_SCALE_MATRIX_4X4(b,s,a)		\
{					\
   b[0][0] += (s) * a[0][0];		\
   b[0][1] += (s) * a[0][1];		\
   b[0][2] += (s) * a[0][2];		\
   b[0][3] += (s) * a[0][3];		\
					\
   b[1][0] += (s) * a[1][0];		\
   b[1][1] += (s) * a[1][1];		\
   b[1][2] += (s) * a[1][2];		\
   b[1][3] += (s) * a[1][3];		\
					\
   b[2][0] += (s) * a[2][0];		\
   b[2][1] += (s) * a[2][1];		\
   b[2][2] += (s) * a[2][2];		\
   b[2][3] += (s) * a[2][3];		\
					\
   b[3][0] += (s) * a[3][0];		\
   b[3][1] += (s) * a[3][1];		\
   b[3][2] += (s) * a[3][2];		\
   b[3][3] += (s) * a[3][3];		\
}\

/*! matrix product */
/*! c[x][y] = a[x][0]*b[0][y]+a[x][1]*b[1][y]+a[x][2]*b[2][y]+a[x][3]*b[3][y];*/
#define MATRIX_PRODUCT_2X2(c,a,b)		\
{						\
   c[0][0] = a[0][0]*b[0][0]+a[0][1]*b[1][0];	\
   c[0][1] = a[0][0]*b[0][1]+a[0][1]*b[1][1];	\
						\
   c[1][0] = a[1][0]*b[0][0]+a[1][1]*b[1][0];	\
   c[1][1] = a[1][0]*b[0][1]+a[1][1]*b[1][1];	\
						\
}\

/*! matrix product */
/*! c[x][y] = a[x][0]*b[0][y]+a[x][1]*b[1][y]+a[x][2]*b[2][y]+a[x][3]*b[3][y];*/
#define MATRIX_PRODUCT_3X3(c,a,b)				\
{								\
   c[0][0] = a[0][0]*b[0][0]+a[0][1]*b[1][0]+a[0][2]*b[2][0];	\
   c[0][1] = a[0][0]*b[0][1]+a[0][1]*b[1][1]+a[0][2]*b[2][1];	\
   c[0][2] = a[0][0]*b[0][2]+a[0][1]*b[1][2]+a[0][2]*b[2][2];	\
								\
   c[1][0] = a[1][0]*b[0][0]+a[1][1]*b[1][0]+a[1][2]*b[2][0];	\
   c[1][1] = a[1][0]*b[0][1]+a[1][1]*b[1][1]+a[1][2]*b[2][1];	\
   c[1][2] = a[1][0]*b[0][2]+a[1][1]*b[1][2]+a[1][2]*b[2][2];	\
								\
   c[2][0] = a[2][0]*b[0][0]+a[2][1]*b[1][0]+a[2][2]*b[2][0];	\
   c[2][1] = a[2][0]*b[0][1]+a[2][1]*b[1][1]+a[2][2]*b[2][1];	\
   c[2][2] = a[2][0]*b[0][2]+a[2][1]*b[1][2]+a[2][2]*b[2][2];	\
}\


/*! matrix product */
/*! c[x][y] = a[x][0]*b[0][y]+a[x][1]*b[1][y]+a[x][2]*b[2][y]+a[x][3]*b[3][y];*/
#define MATRIX_PRODUCT_4X4(c,a,b)		\
{						\
   c[0][0] = a[0][0]*b[0][0]+a[0][1]*b[1][0]+a[0][2]*b[2][0]+a[0][3]*b[3][0];\
   c[0][1] = a[0][0]*b[0][1]+a[0][1]*b[1][1]+a[0][2]*b[2][1]+a[0][3]*b[3][1];\
   c[0][2] = a[0][0]*b[0][2]+a[0][1]*b[1][2]+a[0][2]*b[2][2]+a[0][3]*b[3][2];\
   c[0][3] = a[0][0]*b[0][3]+a[0][1]*b[1][3]+a[0][2]*b[2][3]+a[0][3]*b[3][3];\
						\
   c[1][0] = a[1][0]*b[0][0]+a[1][1]*b[1][0]+a[1][2]*b[2][0]+a[1][3]*b[3][0];\
   c[1][1] = a[1][0]*b[0][1]+a[1][1]*b[1][1]+a[1][2]*b[2][1]+a[1][3]*b[3][1];\
   c[1][2] = a[1][0]*b[0][2]+a[1][1]*b[1][2]+a[1][2]*b[2][2]+a[1][3]*b[3][2];\
   c[1][3] = a[1][0]*b[0][3]+a[1][1]*b[1][3]+a[1][2]*b[2][3]+a[1][3]*b[3][3];\
						\
   c[2][0] = a[2][0]*b[0][0]+a[2][1]*b[1][0]+a[2][2]*b[2][0]+a[2][3]*b[3][0];\
   c[2][1] = a[2][0]*b[0][1]+a[2][1]*b[1][1]+a[2][2]*b[2][1]+a[2][3]*b[3][1];\
   c[2][2] = a[2][0]*b[0][2]+a[2][1]*b[1][2]+a[2][2]*b[2][2]+a[2][3]*b[3][2];\
   c[2][3] = a[2][0]*b[0][3]+a[2][1]*b[1][3]+a[2][2]*b[2][3]+a[2][3]*b[3][3];\
						\
   c[3][0] = a[3][0]*b[0][0]+a[3][1]*b[1][0]+a[3][2]*b[2][0]+a[3][3]*b[3][0];\
   c[3][1] = a[3][0]*b[0][1]+a[3][1]*b[1][1]+a[3][2]*b[2][1]+a[3][3]*b[3][1];\
   c[3][2] = a[3][0]*b[0][2]+a[3][1]*b[1][2]+a[3][2]*b[2][2]+a[3][3]*b[3][2];\
   c[3][3] = a[3][0]*b[0][3]+a[3][1]*b[1][3]+a[3][2]*b[2][3]+a[3][3]*b[3][3];\
}\


/*! matrix times vector */
#define MAT_DOT_VEC_2X2(p,m,v)					\
{								\
   p[0] = m[0][0]*v[0] + m[0][1]*v[1];				\
   p[1] = m[1][0]*v[0] + m[1][1]*v[1];				\
}\


/*! matrix times vector */
#define MAT_DOT_VEC_3X3(p,m,v)					\
{								\
   p[0] = m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2];		\
   p[1] = m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2];		\
   p[2] = m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2];		\
}\


/*! matrix times vector
v is a vec4f
*/
#define MAT_DOT_VEC_4X4(p,m,v)					\
{								\
   p[0] = m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2] + m[0][3]*v[3];	\
   p[1] = m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2] + m[1][3]*v[3];	\
   p[2] = m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2] + m[2][3]*v[3];	\
   p[3] = m[3][0]*v[0] + m[3][1]*v[1] + m[3][2]*v[2] + m[3][3]*v[3];	\
}\

/*! matrix times vector
v is a vec3f
and m is a mat4f<br>
Last column is added as the position
*/
#define MAT_DOT_VEC_3X4(p,m,v)					\
{								\
   p[0] = m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2] + m[0][3];	\
   p[1] = m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2] + m[1][3];	\
   p[2] = m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2] + m[2][3];	\
}\

/*! vector transpose times matrix */
/*! p[j] = v[0]*m[0][j] + v[1]*m[1][j] + v[2]*m[2][j]; */
#define VEC_DOT_MAT_3X3(p,v,m)					\
{								\
   p[0] = v[0]*m[0][0] + v[1]*m[1][0] + v[2]*m[2][0];		\
   p[1] = v[0]*m[0][1] + v[1]*m[1][1] + v[2]*m[2][1];		\
   p[2] = v[0]*m[0][2] + v[1]*m[1][2] + v[2]*m[2][2];		\
}\


/*! affine matrix times vector */
/** The matrix is assumed to be an affine matrix, with last two
 * entries representing a translation */
#define MAT_DOT_VEC_2X3(p,m,v)					\
{								\
   p[0] = m[0][0]*v[0] + m[0][1]*v[1] + m[0][2];		\
   p[1] = m[1][0]*v[0] + m[1][1]*v[1] + m[1][2];		\
}\

//! Transform a plane
#define MAT_TRANSFORM_PLANE_4X4(pout,m,plane)\
{								\
   pout[0] = m[0][0]*plane[0] + m[0][1]*plane[1]  + m[0][2]*plane[2];\
   pout[1] = m[1][0]*plane[0] + m[1][1]*plane[1]  + m[1][2]*plane[2];\
   pout[2] = m[2][0]*plane[0] + m[2][1]*plane[1]  + m[2][2]*plane[2];\
   pout[3] = m[0][3]*pout[0] + m[1][3]*pout[1]  + m[2][3]*pout[2] + plane[3];\
}\

	

/** inverse transpose of matrix times vector
 *
 * This macro computes inverse transpose of matrix m,
 * and multiplies vector v into it, to yeild vector p
 *
 * DANGER !!! Do Not use this on normal vectors!!!
 * It will leave normals the wrong length !!!
 * See macro below for use on normals.
 */
#define INV_TRANSP_MAT_DOT_VEC_2X2(p,m,v)			\
{								\
   GREAL det;						\
								\
   det = m[0][0]*m[1][1] - m[0][1]*m[1][0];			\
   p[0] = m[1][1]*v[0] - m[1][0]*v[1];				\
   p[1] = - m[0][1]*v[0] + m[0][0]*v[1];			\
								\
   /* if matrix not singular, and not orthonormal, then renormalize */ \
   if ((det!=1.0f) && (det != 0.0f)) {				\
      det = 1.0f / det;						\
      p[0] *= det;						\
      p[1] *= det;						\
   }								\
}\


/** transform normal vector by inverse transpose of matrix
 * and then renormalize the vector
 *
 * This macro computes inverse transpose of matrix m,
 * and multiplies vector v into it, to yeild vector p
 * Vector p is then normalized.
 */
#define NORM_XFORM_2X2(p,m,v)					\
{								\
   GREAL len;							\
								\
   /* do nothing if off-diagonals are zero and diagonals are 	\
    * equal */							\
   if ((m[0][1] != 0.0) || (m[1][0] != 0.0) || (m[0][0] != m[1][1])) { \
      p[0] = m[1][1]*v[0] - m[1][0]*v[1];			\
      p[1] = - m[0][1]*v[0] + m[0][0]*v[1];			\
								\
      len = p[0]*p[0] + p[1]*p[1];				\
      GIM_INV_SQRT(len,len);					\
      p[0] *= len;						\
      p[1] *= len;						\
   } else {							\
      VEC_COPY_2 (p, v);					\
   }								\
}\


/** outer product of vector times vector transpose
 *
 * The outer product of vector v and vector transpose t yeilds
 * dyadic matrix m.
 */
#define OUTER_PRODUCT_2X2(m,v,t)				\
{								\
   m[0][0] = v[0] * t[0];					\
   m[0][1] = v[0] * t[1];					\
								\
   m[1][0] = v[1] * t[0];					\
   m[1][1] = v[1] * t[1];					\
}\


/** outer product of vector times vector transpose
 *
 * The outer product of vector v and vector transpose t yeilds
 * dyadic matrix m.
 */
#define OUTER_PRODUCT_3X3(m,v,t)				\
{								\
   m[0][0] = v[0] * t[0];					\
   m[0][1] = v[0] * t[1];					\
   m[0][2] = v[0] * t[2];					\
								\
   m[1][0] = v[1] * t[0];					\
   m[1][1] = v[1] * t[1];					\
   m[1][2] = v[1] * t[2];					\
								\
   m[2][0] = v[2] * t[0];					\
   m[2][1] = v[2] * t[1];					\
   m[2][2] = v[2] * t[2];					\
}\


/** outer product of vector times vector transpose
 *
 * The outer product of vector v and vector transpose t yeilds
 * dyadic matrix m.
 */
#define OUTER_PRODUCT_4X4(m,v,t)				\
{								\
   m[0][0] = v[0] * t[0];					\
   m[0][1] = v[0] * t[1];					\
   m[0][2] = v[0] * t[2];					\
   m[0][3] = v[0] * t[3];					\
								\
   m[1][0] = v[1] * t[0];					\
   m[1][1] = v[1] * t[1];					\
   m[1][2] = v[1] * t[2];					\
   m[1][3] = v[1] * t[3];					\
								\
   m[2][0] = v[2] * t[0];					\
   m[2][1] = v[2] * t[1];					\
   m[2][2] = v[2] * t[2];					\
   m[2][3] = v[2] * t[3];					\
								\
   m[3][0] = v[3] * t[0];					\
   m[3][1] = v[3] * t[1];					\
   m[3][2] = v[3] * t[2];					\
   m[3][3] = v[3] * t[3];					\
}\


/** outer product of vector times vector transpose
 *
 * The outer product of vector v and vector transpose t yeilds
 * dyadic matrix m.
 */
#define ACCUM_OUTER_PRODUCT_2X2(m,v,t)				\
{								\
   m[0][0] += v[0] * t[0];					\
   m[0][1] += v[0] * t[1];					\
								\
   m[1][0] += v[1] * t[0];					\
   m[1][1] += v[1] * t[1];					\
}\


/** outer product of vector times vector transpose
 *
 * The outer product of vector v and vector transpose t yeilds
 * dyadic matrix m.
 */
#define ACCUM_OUTER_PRODUCT_3X3(m,v,t)				\
{								\
   m[0][0] += v[0] * t[0];					\
   m[0][1] += v[0] * t[1];					\
   m[0][2] += v[0] * t[2];					\
								\
   m[1][0] += v[1] * t[0];					\
   m[1][1] += v[1] * t[1];					\
   m[1][2] += v[1] * t[2];					\
								\
   m[2][0] += v[2] * t[0];					\
   m[2][1] += v[2] * t[1];					\
   m[2][2] += v[2] * t[2];					\
}\


/** outer product of vector times vector transpose
 *
 * The outer product of vector v and vector transpose t yeilds
 * dyadic matrix m.
 */
#define ACCUM_OUTER_PRODUCT_4X4(m,v,t)				\
{								\
   m[0][0] += v[0] * t[0];					\
   m[0][1] += v[0] * t[1];					\
   m[0][2] += v[0] * t[2];					\
   m[0][3] += v[0] * t[3];					\
								\
   m[1][0] += v[1] * t[0];					\
   m[1][1] += v[1] * t[1];					\
   m[1][2] += v[1] * t[2];					\
   m[1][3] += v[1] * t[3];					\
								\
   m[2][0] += v[2] * t[0];					\
   m[2][1] += v[2] * t[1];					\
   m[2][2] += v[2] * t[2];					\
   m[2][3] += v[2] * t[3];					\
								\
   m[3][0] += v[3] * t[0];					\
   m[3][1] += v[3] * t[1];					\
   m[3][2] += v[3] * t[2];					\
   m[3][3] += v[3] * t[3];					\
}\


/** determinant of matrix
 *
 * Computes determinant of matrix m, returning d
 */
#define DETERMINANT_2X2(d,m)					\
{								\
   d = m[0][0] * m[1][1] - m[0][1] * m[1][0];			\
}\


/** determinant of matrix
 *
 * Computes determinant of matrix m, returning d
 */
#define DETERMINANT_3X3(d,m)					\
{								\
   d = m[0][0] * (m[1][1]*m[2][2] - m[1][2] * m[2][1]);		\
   d -= m[0][1] * (m[1][0]*m[2][2] - m[1][2] * m[2][0]);	\
   d += m[0][2] * (m[1][0]*m[2][1] - m[1][1] * m[2][0]);	\
}\


/** i,j,th cofactor of a 4x4 matrix
 *
 */
#define COFACTOR_4X4_IJ(fac,m,i,j) 				\
{								\
   GUINT __ii[4], __jj[4], __k;						\
								\
   for (__k=0; __k<i; __k++) __ii[__k] = __k;				\
   for (__k=i; __k<3; __k++) __ii[__k] = __k+1;				\
   for (__k=0; __k<j; __k++) __jj[__k] = __k;				\
   for (__k=j; __k<3; __k++) __jj[__k] = __k+1;				\
								\
   (fac) = m[__ii[0]][__jj[0]] * (m[__ii[1]][__jj[1]]*m[__ii[2]][__jj[2]] 	\
                            - m[__ii[1]][__jj[2]]*m[__ii[2]][__jj[1]]); \
   (fac) -= m[__ii[0]][__jj[1]] * (m[__ii[1]][__jj[0]]*m[__ii[2]][__jj[2]]	\
                             - m[__ii[1]][__jj[2]]*m[__ii[2]][__jj[0]]);\
   (fac) += m[__ii[0]][__jj[2]] * (m[__ii[1]][__jj[0]]*m[__ii[2]][__jj[1]]	\
                             - m[__ii[1]][__jj[1]]*m[__ii[2]][__jj[0]]);\
								\
   __k = i+j;							\
   if ( __k != (__k/2)*2) {						\
      (fac) = -(fac);						\
   }								\
}\


/** determinant of matrix
 *
 * Computes determinant of matrix m, returning d
 */
#define DETERMINANT_4X4(d,m)					\
{								\
   GREAL cofac;						\
   COFACTOR_4X4_IJ (cofac, m, 0, 0);				\
   d = m[0][0] * cofac;						\
   COFACTOR_4X4_IJ (cofac, m, 0, 1);				\
   d += m[0][1] * cofac;					\
   COFACTOR_4X4_IJ (cofac, m, 0, 2);				\
   d += m[0][2] * cofac;					\
   COFACTOR_4X4_IJ (cofac, m, 0, 3);				\
   d += m[0][3] * cofac;					\
}\


/** cofactor of matrix
 *
 * Computes cofactor of matrix m, returning a
 */
#define COFACTOR_2X2(a,m)					\
{								\
   a[0][0] = (m)[1][1];						\
   a[0][1] = - (m)[1][0];						\
   a[1][0] = - (m)[0][1];						\
   a[1][1] = (m)[0][0];						\
}\


/** cofactor of matrix
 *
 * Computes cofactor of matrix m, returning a
 */
#define COFACTOR_3X3(a,m)					\
{								\
   a[0][0] = m[1][1]*m[2][2] - m[1][2]*m[2][1];			\
   a[0][1] = - (m[1][0]*m[2][2] - m[2][0]*m[1][2]);		\
   a[0][2] = m[1][0]*m[2][1] - m[1][1]*m[2][0];			\
   a[1][0] = - (m[0][1]*m[2][2] - m[0][2]*m[2][1]);		\
   a[1][1] = m[0][0]*m[2][2] - m[0][2]*m[2][0];			\
   a[1][2] = - (m[0][0]*m[2][1] - m[0][1]*m[2][0]);		\
   a[2][0] = m[0][1]*m[1][2] - m[0][2]*m[1][1];			\
   a[2][1] = - (m[0][0]*m[1][2] - m[0][2]*m[1][0]);		\
   a[2][2] = m[0][0]*m[1][1] - m[0][1]*m[1][0]);		\
}\


/** cofactor of matrix
 *
 * Computes cofactor of matrix m, returning a
 */
#define COFACTOR_4X4(a,m)					\
{								\
   int i,j;							\
								\
   for (i=0; i<4; i++) {					\
      for (j=0; j<4; j++) {					\
         COFACTOR_4X4_IJ (a[i][j], m, i, j);			\
      }								\
   }								\
}\


/** adjoint of matrix
 *
 * Computes adjoint of matrix m, returning a
 * (Note that adjoint is just the transpose of the cofactor matrix)
 */
#define ADJOINT_2X2(a,m)					\
{								\
   a[0][0] = (m)[1][1];						\
   a[1][0] = - (m)[1][0];						\
   a[0][1] = - (m)[0][1];						\
   a[1][1] = (m)[0][0];						\
}\


/** adjoint of matrix
 *
 * Computes adjoint of matrix m, returning a
 * (Note that adjoint is just the transpose of the cofactor matrix)
 */
#define ADJOINT_3X3(a,m)					\
{								\
   a[0][0] = m[1][1]*m[2][2] - m[1][2]*m[2][1];			\
   a[1][0] = - (m[1][0]*m[2][2] - m[2][0]*m[1][2]);		\
   a[2][0] = m[1][0]*m[2][1] - m[1][1]*m[2][0];			\
   a[0][1] = - (m[0][1]*m[2][2] - m[0][2]*m[2][1]);		\
   a[1][1] = m[0][0]*m[2][2] - m[0][2]*m[2][0];			\
   a[2][1] = - (m[0][0]*m[2][1] - m[0][1]*m[2][0]);		\
   a[0][2] = m[0][1]*m[1][2] - m[0][2]*m[1][1];			\
   a[1][2] = - (m[0][0]*m[1][2] - m[0][2]*m[1][0]);		\
   a[2][2] = m[0][0]*m[1][1] - m[0][1]*m[1][0]);		\
}\


/** adjoint of matrix
 *
 * Computes adjoint of matrix m, returning a
 * (Note that adjoint is just the transpose of the cofactor matrix)
 */
#define ADJOINT_4X4(a,m)					\
{								\
   char _i_,_j_;							\
								\
   for (_i_=0; _i_<4; _i_++) {					\
      for (_j_=0; _j_<4; _j_++) {					\
         COFACTOR_4X4_IJ (a[_j_][_i_], m, _i_, _j_);			\
      }								\
   }								\
}\


/** compute adjoint of matrix and scale
 *
 * Computes adjoint of matrix m, scales it by s, returning a
 */
#define SCALE_ADJOINT_2X2(a,s,m)				\
{								\
   a[0][0] = (s) * m[1][1];					\
   a[1][0] = - (s) * m[1][0];					\
   a[0][1] = - (s) * m[0][1];					\
   a[1][1] = (s) * m[0][0];					\
}\


/** compute adjoint of matrix and scale
 *
 * Computes adjoint of matrix m, scales it by s, returning a
 */
#define SCALE_ADJOINT_3X3(a,s,m)				\
{								\
   a[0][0] = (s) * (m[1][1] * m[2][2] - m[1][2] * m[2][1]);	\
   a[1][0] = (s) * (m[1][2] * m[2][0] - m[1][0] * m[2][2]);	\
   a[2][0] = (s) * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);	\
								\
   a[0][1] = (s) * (m[0][2] * m[2][1] - m[0][1] * m[2][2]);	\
   a[1][1] = (s) * (m[0][0] * m[2][2] - m[0][2] * m[2][0]);	\
   a[2][1] = (s) * (m[0][1] * m[2][0] - m[0][0] * m[2][1]);	\
								\
   a[0][2] = (s) * (m[0][1] * m[1][2] - m[0][2] * m[1][1]);	\
   a[1][2] = (s) * (m[0][2] * m[1][0] - m[0][0] * m[1][2]);	\
   a[2][2] = (s) * (m[0][0] * m[1][1] - m[0][1] * m[1][0]);	\
}\


/** compute adjoint of matrix and scale
 *
 * Computes adjoint of matrix m, scales it by s, returning a
 */
#define SCALE_ADJOINT_4X4(a,s,m)				\
{								\
   char _i_,_j_; \
   for (_i_=0; _i_<4; _i_++) {					\
      for (_j_=0; _j_<4; _j_++) {					\
         COFACTOR_4X4_IJ (a[_j_][_i_], m, _i_, _j_);			\
         a[_j_][_i_] *= s;						\
      }								\
   }								\
}\

/** inverse of matrix
 *
 * Compute inverse of matrix a, returning determinant m and
 * inverse b
 */
#define INVERT_2X2(b,det,a)			\
{						\
   GREAL _tmp_;					\
   DETERMINANT_2X2 (det, a);			\
   _tmp_ = 1.0 / (det);				\
   SCALE_ADJOINT_2X2 (b, _tmp_, a);		\
}\


/** inverse of matrix
 *
 * Compute inverse of matrix a, returning determinant m and
 * inverse b
 */
#define INVERT_3X3(b,det,a)			\
{						\
   GREAL _tmp_;					\
   DETERMINANT_3X3 (det, a);			\
   _tmp_ = 1.0 / (det);				\
   SCALE_ADJOINT_3X3 (b, _tmp_, a);		\
}\


/** inverse of matrix
 *
 * Compute inverse of matrix a, returning determinant m and
 * inverse b
 */
#define INVERT_4X4(b,det,a)			\
{						\
   GREAL _tmp_;					\
   DETERMINANT_4X4 (det, a);			\
   _tmp_ = 1.0 / (det);				\
   SCALE_ADJOINT_4X4 (b, _tmp_, a);		\
}\

//! Get the triple(3) row of a transform matrix
#define MAT_GET_ROW(mat,vec3,rowindex)\
{\
    vec3[0] = mat[rowindex][0];\
    vec3[1] = mat[rowindex][1];\
    vec3[2] = mat[rowindex][2]; \
}\

//! Get the tuple(2) row of a transform matrix
#define MAT_GET_ROW2(mat,vec2,rowindex)\
{\
    vec2[0] = mat[rowindex][0];\
    vec2[1] = mat[rowindex][1];\
}\


//! Get the quad (4) row of a transform matrix
#define MAT_GET_ROW4(mat,vec4,rowindex)\
{\
    vec4[0] = mat[rowindex][0];\
    vec4[1] = mat[rowindex][1];\
    vec4[2] = mat[rowindex][2];\
    vec4[3] = mat[rowindex][3];\
}\

//! Get the triple(3) col of a transform matrix
#define MAT_GET_COL(mat,vec3,colindex)\
{\
    vec3[0] = mat[0][colindex];\
    vec3[1] = mat[1][colindex];\
    vec3[2] = mat[2][colindex]; \
}\

//! Get the tuple(2) col of a transform matrix
#define MAT_GET_COL2(mat,vec2,colindex)\
{\
    vec2[0] = mat[0][colindex];\
    vec2[1] = mat[1][colindex];\
}\


//! Get the quad (4) col of a transform matrix
#define MAT_GET_COL4(mat,vec4,colindex)\
{\
    vec4[0] = mat[0][colindex];\
    vec4[1] = mat[1][colindex];\
    vec4[2] = mat[2][colindex];\
    vec4[3] = mat[3][colindex];\
}\

//! Get the triple(3) col of a transform matrix
#define MAT_GET_TRANSLATION(mat,vec3)\
{\
    vec3[0] = mat[0][3];\
    vec3[1] = mat[1][3];\
    vec3[2] = mat[2][3]; \
}\



//! Set the triple(3) col of a transform matrix
#define MAT_SET_TRANSLATION(mat,vec3)\
{\
    mat[0][3] = vec3[0];\
    mat[1][3] = vec3[1];\
    mat[2][3] = vec3[2]; \
}\




//! Returns the dot product between a vec3f and the row of a matrix
#define MAT_DOT_ROW(mat,vec3,rowindex) (vec3[0]*mat[rowindex][0] + vec3[1]*mat[rowindex][1] + vec3[2]*mat[rowindex][2])

//! Returns the dot product between a vec2f and the row of a matrix
#define MAT_DOT_ROW2(mat,vec2,rowindex) (vec2[0]*mat[rowindex][0] + vec2[1]*mat[rowindex][1])

//! Returns the dot product between a vec4f and the row of a matrix
#define MAT_DOT_ROW4(mat,vec4,rowindex) (vec4[0]*mat[rowindex][0] + vec4[1]*mat[rowindex][1] + vec4[2]*mat[rowindex][2] + vec4[3]*mat[rowindex][3])


//! Returns the dot product between a vec3f and the col of a matrix
#define MAT_DOT_COL(mat,vec3,colindex) (vec3[0]*mat[0][colindex] + vec3[1]*mat[1][colindex] + vec3[2]*mat[2][colindex])

//! Returns the dot product between a vec2f and the col of a matrix
#define MAT_DOT_COL2(mat,vec2,colindex) (vec2[0]*mat[0][colindex] + vec2[1]*mat[1][colindex])

//! Returns the dot product between a vec4f and the col of a matrix
#define MAT_DOT_COL4(mat,vec4,colindex) (vec4[0]*mat[0][colindex] + vec4[1]*mat[1][colindex] + vec4[2]*mat[2][colindex] + vec4[3]*mat[3][colindex])



//! @}


/*! \defgroup GEOMETRIC_OPERATIONS
*/
//! @{


#define PLANEDIREPSILON 0.0000001f
#define PARALELENORMALS 0.000001f

#define TRIANGLE_NORMAL(v1,v2,v3,n){\
    vec3f _dif1,_dif2; \
    VEC_DIFF(_dif1,v2,v1); \
    VEC_DIFF(_dif2,v3,v1); \
    VEC_CROSS(n,_dif1,_dif2); \
    VEC_NORMALIZE(n); \
}\

#define TRIANGLE_NORMAL_FAST(v1,v2,v3,n){\
    vec3f _dif1,_dif2; \
    VEC_DIFF(_dif1,v2,v1); \
    VEC_DIFF(_dif2,v3,v1); \
    VEC_CROSS(n,_dif1,_dif2); \
}\

/// plane is a vec4f
#define TRIANGLE_PLANE(v1,v2,v3,plane) {\
    TRIANGLE_NORMAL(v1,v2,v3,plane);\
    plane[3] = VEC_DOT(v1,plane);\
}\

/// plane is a vec4f
#define TRIANGLE_PLANE_FAST(v1,v2,v3,plane) {\
    TRIANGLE_NORMAL_FAST(v1,v2,v3,plane);\
    plane[3] = VEC_DOT(v1,plane);\
}\

/// Calc a plane from an edge an a normal. plane is a vec4f
#define EDGE_PLANE(e1,e2,n,plane) {\
    vec3f _dif; \
    VEC_DIFF(_dif,e2,e1); \
    VEC_CROSS(plane,_dif,n); \
    VEC_NORMALIZE(plane); \
    plane[3] = VEC_DOT(e1,plane);\
}\

#define DISTANCE_PLANE_POINT(plane,point) (VEC_DOT(plane,point) - plane[3])

#define PROJECT_POINT_PLANE(point,plane,projected) {\
	GREAL _dis;\
	_dis = DISTANCE_PLANE_POINT(plane,point);\
	VEC_SCALE(projected,-_dis,plane);\
	VEC_SUM(projected,projected,point);	\
}\

#define POINT_IN_HULL(point,planes,plane_count,outside)\
{\
	GREAL _dis;\
	outside = 0;\
	GUINT _i = 0;\
	do\
	{\
	    _dis = DISTANCE_PLANE_POINT(planes[_i],point);\
	    if(_dis>0.0f) outside = 1;\
	    _i++;\
	}while(_i<plane_count&&outside==0);\
}\


#define PLANE_CLIP_SEGMENT(s1,s2,plane,clipped) {\
	GREAL _dis1,_dis2;\
	_dis1 = DISTANCE_PLANE_POINT(plane,s1);\
	VEC_DIFF(clipped,s2,s1);\
	_dis2 = VEC_DOT(clipped,plane);\
	VEC_SCALE(clipped,-_dis1/_dis2,clipped);\
	VEC_SUM(clipped,clipped,s1);	\
}\

//! Confirms if the plane intersect the edge or nor
/*!
intersection type must have the following values
<ul>
<li> 0 : Segment in front of plane, s1 closest
<li> 1 : Segment in front of plane, s2 closest
<li> 2 : Segment in back of plane, s1 closest
<li> 3 : Segment in back of plane, s2 closest
<li> 4 : Segment collides plane, s1 in back
<li> 5 : Segment collides plane, s2 in back
</ul>
*/
#define PLANE_CLIP_SEGMENT2(s1,s2,plane,clipped,intersection_type) \
{\
	GREAL _dis1,_dis2;\
	_dis1 = DISTANCE_PLANE_POINT(plane,s1);\
	_dis2 = DISTANCE_PLANE_POINT(plane,s2);\
	if(_dis1 >-G_EPSILON && _dis2 >-G_EPSILON)\
	{\
	    if(_dis1<_dis2) intersection_type = 0;\
	    else  intersection_type = 1;\
	}\
	else if(_dis1 <G_EPSILON && _dis2 <G_EPSILON)\
	{\
	    if(_dis1>_dis2) intersection_type = 2;\
	    else  intersection_type = 3;	    \
	}\
	else\
	{\
	    if(_dis1<_dis2) intersection_type = 4;\
	    else  intersection_type = 5;\
        VEC_DIFF(clipped,s2,s1);\
        _dis2 = VEC_DOT(clipped,plane);\
        VEC_SCALE(clipped,-_dis1/_dis2,clipped);\
        VEC_SUM(clipped,clipped,s1);	\
	}\
}\

//! Confirms if the plane intersect the edge or not
/*!
clipped1 and clipped2 are the vertices behind the plane.
clipped1 is the closest

intersection_type must have the following values
<ul>
<li> 0 : Segment in front of plane, s1 closest
<li> 1 : Segment in front of plane, s2 closest
<li> 2 : Segment in back of plane, s1 closest
<li> 3 : Segment in back of plane, s2 closest
<li> 4 : Segment collides plane, s1 in back
<li> 5 : Segment collides plane, s2 in back
</ul>
*/
#define PLANE_CLIP_SEGMENT_CLOSEST(s1,s2,plane,clipped1,clipped2,intersection_type)\
{\
	PLANE_CLIP_SEGMENT2(s1,s2,plane,clipped1,intersection_type);\
	if(intersection_type == 0)\
	{\
	    VEC_COPY(clipped1,s1);\
	    VEC_COPY(clipped2,s2);\
	}\
	else if(intersection_type == 1)\
	{\
	    VEC_COPY(clipped1,s2);\
	    VEC_COPY(clipped2,s1);\
	}\
	else if(intersection_type == 2)\
	{\
	    VEC_COPY(clipped1,s1);\
	    VEC_COPY(clipped2,s2);\
	}\
	else if(intersection_type == 3)\
	{\
	    VEC_COPY(clipped1,s2);\
	    VEC_COPY(clipped2,s1);\
	}\
	else if(intersection_type == 4)\
	{	    \
	    VEC_COPY(clipped2,s1);\
	}\
	else if(intersection_type == 5)\
	{	    \
	    VEC_COPY(clipped2,s2);\
	}\
}\


//! Finds the 2 smallest cartesian coordinates of a plane normal
#define PLANE_MINOR_AXES(plane, i0, i1) VEC_MINOR_AXES(plane, i0, i1)

//! Ray plane collision
#define RAY_PLANE_COLLISION(plane,vDir,vPoint,pout,tparam,does_intersect)\
{\
	GREAL _dis,_dotdir;	\
	_dotdir = VEC_DOT(plane,vDir);\
	if(_dotdir<PLANEDIREPSILON)\
	{\
	    does_intersect = 0;\
	}\
	else\
	{\
        _dis = DISTANCE_PLANE_POINT(plane,vPoint);	\
        tparam = -_dis/_dotdir;\
        VEC_SCALE(pout,tparam,vDir);\
        VEC_SUM(pout,vPoint,pout);	\
        does_intersect = 1;\
	}\
}\

//! Bidireccional ray
#define LINE_PLANE_COLLISION(plane,vDir,vPoint,pout,tparam, tmin, tmax)\
{\
    tparam = -DISTANCE_PLANE_POINT(plane,vPoint);\
	tparam /= VEC_DOT(plane,vDir);\
    tparam = GIM_CLAMP(tparam,tmin,tmax);\
    VEC_SCALE(pout,tparam,vDir);\
    VEC_SUM(pout,vPoint,pout);	\
}\

/*! \brief Returns the Ray on which 2 planes intersect if they do.
    Written by Rodrigo Hernandez on ODE convex collision

  \param p1 Plane 1
  \param p2 Plane 2
  \param p Contains the origin of the ray upon returning if planes intersect
  \param d Contains the direction of the ray upon returning if planes intersect
  \param dointersect 1 if the planes intersect, 0 if paralell.

*/
#define INTERSECT_PLANES(p1,p2,p,d,dointersect) \
{ \
  VEC_CROSS(d,p1,p2); \
  GREAL denom = VEC_DOT(d, d);\
  if (IS_ZERO(denom)) \
  { \
      dointersect = 0; \
  } \
  else \
  { \
      vec3f _n;\
      _n[0]=p1[3]*p2[0] - p2[3]*p1[0]; \
      _n[1]=p1[3]*p2[1] - p2[3]*p1[1]; \
      _n[2]=p1[3]*p2[2] - p2[3]*p1[2]; \
      VEC_CROSS(p,_n,d); \
      p[0]/=denom; \
      p[1]/=denom; \
      p[2]/=denom; \
      dointersect = 1; \
  }\
}\

//***************** SEGMENT and LINE FUNCTIONS **********************************///

/*! Finds the closest point(cp) to (v) on a segment (e1,e2)
 */
#define CLOSEST_POINT_ON_SEGMENT(cp,v,e1,e2)			\
{ \
    vec3f _n;\
    VEC_DIFF(_n,e2,e1);\
    VEC_DIFF(cp,v,e1);\
	GREAL _scalar = VEC_DOT(cp, _n);			\
	_scalar/= VEC_DOT(_n, _n); \
	if(_scalar <0.0f)\
	{\
	    VEC_COPY(cp,e1);\
	}\
	else if(_scalar >1.0f)\
	{\
	    VEC_COPY(cp,e2);\
	}\
	else \
	{\
        VEC_SCALE(cp,_scalar,_n);\
        VEC_SUM(cp,cp,e1);\
	}	\
}\


/*! \brief Finds the line params where these lines intersect.

\param dir1 Direction of line 1
\param point1 Point of line 1
\param dir2 Direction of line 2
\param point2 Point of line 2
\param t1 Result Parameter for line 1
\param t2 Result Parameter for line 2
\param dointersect  0  if the lines won't intersect, else 1

*/
#define LINE_INTERSECTION_PARAMS(dir1,point1, dir2, point2,t1,t2,dointersect) {\
    GREAL det;\
	GREAL e1e1 = VEC_DOT(dir1,dir1);\
	GREAL e1e2 = VEC_DOT(dir1,dir2);\
	GREAL e2e2 = VEC_DOT(dir2,dir2);\
	vec3f p1p2;\
    VEC_DIFF(p1p2,point1,point2);\
    GREAL p1p2e1 = VEC_DOT(p1p2,dir1);\
	GREAL p1p2e2 = VEC_DOT(p1p2,dir2);\
	det = e1e2*e1e2 - e1e1*e2e2;\
	if(IS_ZERO(det))\
	{\
	     dointersect = 0;\
	}\
	else\
	{\
        t1 = (e1e2*p1p2e2 - e2e2*p1p2e1)/det;\
        t2 = (e1e1*p1p2e2 - e1e2*p1p2e1)/det;\
        dointersect = 1;\
	}\
}\

//! Find closest points on segments
#define SEGMENT_COLLISION(vA1,vA2,vB1,vB2,vPointA,vPointB)\
{\
    vec3f _AD,_BD,_N;\
    vec4f _M;\
    VEC_DIFF(_AD,vA2,vA1);\
    VEC_DIFF(_BD,vB2,vB1);\
    VEC_CROSS(_N,_AD,_BD);\
    VEC_CROSS(_M,_N,_BD);\
    _M[3] = VEC_DOT(_M,vB1);\
    GREAL _tp; \
    LINE_PLANE_COLLISION(_M,_AD,vA1,vPointA,_tp,0.0f, 1.0f);\
    /*Closest point on segment*/ \
    VEC_DIFF(vPointB,vPointA,vB1);\
	_tp = VEC_DOT(vPointB, _BD);			\
	_tp/= VEC_DOT(_BD, _BD); \
	_tp = GIM_CLAMP(_tp,0.0f,1.0f);	\
    VEC_SCALE(vPointB,_tp,_BD);\
    VEC_SUM(vPointB,vPointB,vB1);\
}\

//! @}

///Additional Headers for Collision
#include "GIMPACT/gim_box_collision.h"
#include "GIMPACT/gim_tri_collision.h"
#include "GIMPACT/gim_tri_sphere_collision.h"
#include "GIMPACT/gim_tri_capsule_collision.h"

#endif // GIM_VECTOR_H_INCLUDED
