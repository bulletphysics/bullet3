/*************************************************************************
*                                                                       *
* Open Dynamics Engine, Copyright (C) 2001-2003 Russell L. Smith.       *
* All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
*                                                                       *
* This library is free software; you can redistribute it and/or         *
* modify it under the terms of EITHER:                                  *
*   (1) The GNU Lesser General Public License as published by the Free  *
*       Software Foundation; either version 2.1 of the License, or (at  *
*       your option) any later version. The text of the GNU Lesser      *
*       General Public License is included with this library in the     *
*       file LICENSE.TXT.                                               *
*   (2) The BSD-style license that is included with this library in     *
*       the file LICENSE-BSD.TXT.                                       *
*                                                                       *
* This library is distributed in the hope that it will be useful,       *
* but WITHOUT ANY WARRANTY; without even the implied warranty of        *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
* LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
*                                                                       *
*************************************************************************/

/*
 *	Triangle-CCylinder(Capsule) collider by Alen Ladavac
 *  Ported to ODE by Nguyen Binh
 */

// NOTES from Nguyen Binh
//	14 Apr : Seem to be robust
//       There is a problem when you use original Step and set contact friction
//		surface.mu = dInfinity;
//		More description : 
//			When I dropped CCylinder over the bunny ears, it seems to stuck
//			there for a while. I think the cause is when you set surface.mu = dInfinity;
//			the friction force is too high so it just hang the capsule there.
//			So the good cure for this is to set mu = around 1.5 (in my case)
//		For StepFast1, this become as solid as rock : StepFast1 just approximate 
//		friction force.

// NOTES from Croteam's Alen
//As a side note... there are some extra contacts that can be generated
//on the edge between two triangles, and if the capsule penetrates deeply into
//the triangle (usually happens with large mass or low FPS), some such
//contacts can in some cases push the capsule away from the edge instead of
//away from the two triangles. This shows up as capsule slowing down a bit
//when hitting an edge while sliding along a flat tesselated grid of
//triangles. This is only if capsule is standing upwards.

//Same thing can appear whenever a smooth object (e.g sphere) hits such an
//edge, and it needs to be solved as a special case probably. This is a
//problem we are looking forward to address soon.

#include <ode/collision.h>
#include <ode/matrix.h>
#include <ode/rotation.h>
#include <ode/odemath.h>
#include "collision_util.h"

#define TRIMESH_INTERNAL
#include "collision_trimesh_internal.h"

// largest number, double or float
#if defined(dSINGLE)
#define MAX_REAL	FLT_MAX
#define MIN_REAL	(-FLT_MAX)
#else
#define MAX_REAL	DBL_MAX
#define MIN_REAL	(-DBL_MAX)
#endif

// To optimize before send contacts to dynamic part
#define OPTIMIZE_CONTACTS

// dVector3
// r=a-b
#define SUBTRACT(a,b,r) \
	(r)[0]=(a)[0] - (b)[0]; \
	(r)[1]=(a)[1] - (b)[1]; \
	(r)[2]=(a)[2] - (b)[2]; 


// dVector3
// a=b
#define SET(a,b) \
	(a)[0]=(b)[0]; \
	(a)[1]=(b)[1]; \
	(a)[2]=(b)[2]; 


// dMatrix3
// a=b
#define SETM(a,b) \
	(a)[0]=(b)[0]; \
	(a)[1]=(b)[1]; \
	(a)[2]=(b)[2]; \
	(a)[3]=(b)[3]; \
	(a)[4]=(b)[4]; \
	(a)[5]=(b)[5]; \
	(a)[6]=(b)[6]; \
	(a)[7]=(b)[7]; \
	(a)[8]=(b)[8]; \
	(a)[9]=(b)[9]; \
	(a)[10]=(b)[10]; \
	(a)[11]=(b)[11];


// dVector3
// r=a+b
#define ADD(a,b,r) \
	(r)[0]=(a)[0] + (b)[0]; \
	(r)[1]=(a)[1] + (b)[1]; \
	(r)[2]=(a)[2] + (b)[2]; 


// dMatrix3, int, dVector3
// v=column a from m
#define GETCOL(m,a,v) \
	(v)[0]=(m)[(a)+0]; \
	(v)[1]=(m)[(a)+4]; \
	(v)[2]=(m)[(a)+8];


// dVector4, dVector3
// distance between plane p and point v
#define POINTDISTANCE(p,v) \
	( p[0]*v[0] + p[1]*v[1] + p[2]*v[2] + p[3] ); \


// dVector4, dVector3, dReal
// construct plane from normal and d
#define CONSTRUCTPLANE(plane,normal,d) \
	plane[0]=normal[0];\
	plane[1]=normal[1];\
	plane[2]=normal[2];\
	plane[3]=d;


// dVector3
// length of vector a
#define LENGTHOF(a) \
	dSqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);\

inline dReal _length2OfVector3(dVector3 v)
{
	return (v[0] * v[0] + v[1] * v[1] + v[2] * v[2] );
}


// Local contacts data
typedef struct _sLocalContactData
{
	dVector3	vPos;
	dVector3	vNormal;
	dReal		fDepth;
	int			nFlags; // 0 = filtered out, 1 = OK
}sLocalContactData;

static sLocalContactData   *gLocalContacts;
static unsigned int			ctContacts = 0;

// capsule data
// real time data
static dMatrix3  mCapsuleRotation;
static dVector3   vCapsulePosition;
static dVector3   vCapsuleAxis;
// static data
static dReal      vCapsuleRadius;
static dReal      fCapsuleSize;

// mesh data
static  dMatrix4  mHullDstPl;
static   dMatrix3  mTriMeshRot;
static dVector3   mTriMeshPos;
static dVector3   vE0, vE1, vE2;

// Two geom
dxGeom*	   gCylinder;
dxGeom*	   gTriMesh;

// global collider data
static dVector3 vNormal;
static dReal    fBestDepth;
static dReal    fBestCenter;
static dReal    fBestrt;
static int		iBestAxis;
static dVector3 vN = {0,0,0,0};

static dVector3 vV0; 
static dVector3 vV1;
static dVector3 vV2;

// ODE contact's specific
static int iFlags;
static dContactGeom *ContactGeoms;
static int iStride;

// Capsule lie on axis number 3 = (Z axis)
static const int nCAPSULE_AXIS = 2;

// Use to classify contacts to be "near" in position
static const dReal fSameContactPositionEpsilon = REAL(0.0001); // 1e-4
// Use to classify contacts to be "near" in normal direction
static const dReal fSameContactNormalEpsilon = REAL(0.0001); // 1e-4


// If this two contact can be classified as "near"
inline int _IsNearContacts(sLocalContactData& c1,sLocalContactData& c2)
{
	int bPosNear = 0;
	int bSameDir = 0;
	dVector3	vDiff;

	// First check if they are "near" in position
	SUBTRACT(c1.vPos,c2.vPos,vDiff);
	if (  (dFabs(vDiff[0]) < fSameContactPositionEpsilon)
		&&(dFabs(vDiff[1]) < fSameContactPositionEpsilon)
		&&(dFabs(vDiff[2]) < fSameContactPositionEpsilon))
	{
		bPosNear = 1;
	}

	// Second check if they are "near" in normal direction
	SUBTRACT(c1.vNormal,c2.vNormal,vDiff);
	if (  (dFabs(vDiff[0]) < fSameContactNormalEpsilon)
		&&(dFabs(vDiff[1]) < fSameContactNormalEpsilon)
		&&(dFabs(vDiff[2]) < fSameContactNormalEpsilon) )
	{
		bSameDir = 1;
	}

	// Will be "near" if position and normal direction are "near"
	return (bPosNear && bSameDir);
}

inline int _IsBetter(sLocalContactData& c1,sLocalContactData& c2)
{
	// The not better will be throw away
	// You can change the selection criteria here
	return (c1.fDepth > c2.fDepth);
}

// iterate through gLocalContacts and filtered out "near contact"
inline void	_OptimizeLocalContacts()
{
	int nContacts = ctContacts;
		
	for (int i = 0; i < nContacts-1; i++)
	{
		for (int j = i+1; j < nContacts; j++)
		{
			if (_IsNearContacts(gLocalContacts[i],gLocalContacts[j]))
			{
				// If they are seem to be the samed then filtered 
				// out the least penetrate one
				if (_IsBetter(gLocalContacts[j],gLocalContacts[i]))
				{
					gLocalContacts[i].nFlags = 0; // filtered 1st contact
				}
				else
				{
					gLocalContacts[j].nFlags = 0; // filtered 2nd contact
				}

				// NOTE
				// There is other way is to add two depth together but
				// it not work so well. Why???
			}
		}
	}
}

inline int	_ProcessLocalContacts()
{
	if (ctContacts == 0)
	{
        delete[] gLocalContacts;
		return 0;
	}

#ifdef OPTIMIZE_CONTACTS
	if (ctContacts > 1)
	{
		// Can be optimized...
		_OptimizeLocalContacts();
	}
#endif		

	unsigned int iContact = 0;
	dContactGeom* Contact = 0;

	int nFinalContact = 0;

	for (iContact = 0; iContact < ctContacts; iContact ++)
	{
        // Ensure that we haven't created too many contacts
        if( nFinalContact >= (iFlags & NUMC_MASK)) 
		{
            break;
        }

		if (1 == gLocalContacts[iContact].nFlags)
		{
				Contact =  SAFECONTACT(iFlags, ContactGeoms, nFinalContact, iStride);
				Contact->depth = gLocalContacts[iContact].fDepth;
				SET(Contact->normal,gLocalContacts[iContact].vNormal);
				SET(Contact->pos,gLocalContacts[iContact].vPos);
				Contact->g1 = gCylinder;
				Contact->g2 = gTriMesh;

				nFinalContact++;
		}
	}
	// debug
	//if (nFinalContact != ctContacts)
	//{
	//	printf("[Info] %d contacts generated,%d  filtered.\n",ctContacts,ctContacts-nFinalContact);
	//}

    delete[] gLocalContacts;
	return nFinalContact;
}

BOOL _cldClipEdgeToPlane( dVector3 &vEpnt0, dVector3 &vEpnt1, const dVector4& plPlane)
{
	// calculate distance of edge points to plane
	dReal fDistance0 = POINTDISTANCE( plPlane, vEpnt0 );
	dReal fDistance1 = POINTDISTANCE( plPlane, vEpnt1 );

	// if both points are behind the plane
	if ( fDistance0 < 0 && fDistance1 < 0 ) 
	{
		// do nothing
		return FALSE;
		// if both points in front of the plane
	} else if ( fDistance0 > 0 && fDistance1 > 0 ) 
	{
		// accept them
		return TRUE;
		// if we have edge/plane intersection
	} else if ((fDistance0 > 0 && fDistance1 < 0) || ( fDistance0 < 0 && fDistance1 > 0)) 
	{

			// find intersection point of edge and plane
			dVector3 vIntersectionPoint;
			vIntersectionPoint[0]= vEpnt0[0]-(vEpnt0[0]-vEpnt1[0])*fDistance0/(fDistance0-fDistance1);
			vIntersectionPoint[1]= vEpnt0[1]-(vEpnt0[1]-vEpnt1[1])*fDistance0/(fDistance0-fDistance1);
			vIntersectionPoint[2]= vEpnt0[2]-(vEpnt0[2]-vEpnt1[2])*fDistance0/(fDistance0-fDistance1);

			// clamp correct edge to intersection point
			if ( fDistance0 < 0 ) 
			{
				SET(vEpnt0,vIntersectionPoint);
			} else 
			{
				SET(vEpnt1,vIntersectionPoint);
			}
			return TRUE;
		}
		return TRUE;
}

static BOOL _cldTestAxis(const dVector3 &v0,
						 const dVector3 &v1,
						 const dVector3 &v2, 
						 dVector3 vAxis, 
						 int iAxis,
						 BOOL bNoFlip = FALSE) 
{

	// calculate length of separating axis vector
	dReal fL = LENGTHOF(vAxis);
	// if not long enough
	// TODO : dReal epsilon please
	if ( fL < 1e-5f ) 
	{
		// do nothing
		//iLastOutAxis = 0;
		return TRUE;
	}

	// otherwise normalize it
	dNormalize3(vAxis);

	// project capsule on vAxis
	dReal frc = dFabs(dDOT(vCapsuleAxis,vAxis))*(fCapsuleSize*REAL(0.5)-vCapsuleRadius) + vCapsuleRadius;

	// project triangle on vAxis
	dReal afv[3];
	afv[0] = dDOT( vV0 , vAxis );
	afv[1] = dDOT( vV1 , vAxis );
	afv[2] = dDOT( vV2 , vAxis );

	dReal fMin = MAX_REAL;
	dReal fMax = MIN_REAL;

	// for each vertex 
	for(int i=0; i<3; i++) 
	{
		// find minimum
		if (afv[i]<fMin) 
		{
			fMin = afv[i];
		}
		// find maximum
		if (afv[i]>fMax) 
		{
			fMax = afv[i];
		}
	}

	// find triangle's center of interval on axis
	dReal fCenter = (fMin+fMax)*REAL(0.5);
	// calculate triangles half interval 
	dReal fTriangleRadius = (fMax-fMin)*REAL(0.5);

	// if they do not overlap, 
	if( dFabs(fCenter) > ( frc + fTriangleRadius ) ) 
	{ 
		// exit, we have no intersection
		return FALSE; 
	}

	// calculate depth 
	dReal fDepth = dFabs(fCenter) - (frc+fTriangleRadius);

	// if greater then best found so far
	if ( fDepth > fBestDepth ) 
	{
		// remember depth
		fBestDepth  = fDepth;
		fBestCenter = fCenter;
		fBestrt     = fTriangleRadius;

		vNormal[0]     = vAxis[0];
		vNormal[1]     = vAxis[1];
		vNormal[2]     = vAxis[2];

		iBestAxis   = iAxis;

		// flip normal if interval is wrong faced
		if (fCenter<0 && !bNoFlip) 
		{ 
			vNormal[0] = -vNormal[0];
			vNormal[1] = -vNormal[1];
			vNormal[2] = -vNormal[2];

			fBestCenter = -fCenter;
		}
	}

	return TRUE;
}

// helper for less key strokes
inline void _CalculateAxis(const dVector3& v1,
						   const dVector3& v2,
						   const dVector3& v3,
						   const dVector3& v4,
						   dVector3& r)
{
	dVector3 t1;
	dVector3 t2;

	SUBTRACT(v1,v2,t1);
	dCROSS(t2,=,t1,v3);
	dCROSS(r,=,t2,v4);
}

static BOOL _cldTestSeparatingAxesOfCapsule(const dVector3 &v0,
											const dVector3 &v1,
											const dVector3 &v2) 
{
	// calculate caps centers in absolute space
	dVector3 vCp0;
	vCp0[0] = vCapsulePosition[0] + vCapsuleAxis[0]*(fCapsuleSize*REAL(0.5)-vCapsuleRadius);
	vCp0[1] = vCapsulePosition[1] + vCapsuleAxis[1]*(fCapsuleSize*REAL(0.5)-vCapsuleRadius);
	vCp0[2] = vCapsulePosition[2] + vCapsuleAxis[2]*(fCapsuleSize*REAL(0.5)-vCapsuleRadius);

	dVector3 vCp1;
	vCp1[0] = vCapsulePosition[0] - vCapsuleAxis[0]*(fCapsuleSize*REAL(0.5)-vCapsuleRadius);
	vCp1[1] = vCapsulePosition[1] - vCapsuleAxis[1]*(fCapsuleSize*REAL(0.5)-vCapsuleRadius);
	vCp1[2] = vCapsulePosition[2] - vCapsuleAxis[2]*(fCapsuleSize*REAL(0.5)-vCapsuleRadius);

	// reset best axis
	iBestAxis = 0;
	// reset best depth
	fBestDepth  = -MAX_REAL;
	// reset separating axis vector
	dVector3 vAxis = {REAL(0.0),REAL(0.0),REAL(0.0),REAL(0.0)};

	// Epsilon value for checking axis vector length 
	const dReal fEpsilon = 1e-6f;

	// Translate triangle to Cc cord.
	SUBTRACT(v0 , vCapsulePosition, vV0);
	SUBTRACT(v1 , vCapsulePosition, vV1);
	SUBTRACT(v2 , vCapsulePosition, vV2);
	
	// We begin to test for 19 separating axis now
	// I wonder does it help if we employ the method like ISA-GJK???
	// Or at least we should do experiment and find what axis will
	// be most likely to be separating axis to check it first.

	// Original
	// axis vN
	//vAxis = -vN;
	vAxis[0] = - vN[0];
	vAxis[1] = - vN[1];
	vAxis[2] = - vN[2];
	if (!_cldTestAxis( v0, v1, v2, vAxis, 1, TRUE)) 
	{ 
		return FALSE; 
	}

	// axis CxE0 - Edge 0
	dCROSS(vAxis,=,vCapsuleAxis,vE0);
	//vAxis = dCROSS( vCapsuleAxis cross vE0 );
	if( _length2OfVector3( vAxis ) > fEpsilon ) {
		if (!_cldTestAxis( v0, v1, v2, vAxis, 2)) { 
			return FALSE; 
		}
	}

	// axis CxE1 - Edge 1
	dCROSS(vAxis,=,vCapsuleAxis,vE1);
	//vAxis = ( vCapsuleAxis cross vE1 );
	if(_length2OfVector3( vAxis ) > fEpsilon ) {
		if (!_cldTestAxis( v0, v1, v2, vAxis, 3)) { 
			return FALSE; 
		}
	}

	// axis CxE2 - Edge 2
	//vAxis = ( vCapsuleAxis cross vE2 );
	dCROSS(vAxis,=,vCapsuleAxis,vE2);
	if(_length2OfVector3( vAxis ) > fEpsilon ) {
		if (!_cldTestAxis( v0, v1, v2, vAxis, 4)) { 
			return FALSE; 
		}
	}

	// first capsule point
	// axis ((Cp0-V0) x E0) x E0
	_CalculateAxis(vCp0,v0,vE0,vE0,vAxis);
//	vAxis = ( ( vCp0-v0) cross vE0 ) cross vE0;
	if(_length2OfVector3( vAxis ) > fEpsilon ) {
		if (!_cldTestAxis( v0, v1, v2, vAxis, 5)) { 
			return FALSE; 
		}
	}

	// axis ((Cp0-V1) x E1) x E1
	_CalculateAxis(vCp0,v1,vE1,vE1,vAxis);
	//vAxis = ( ( vCp0-v1) cross vE1 ) cross vE1;
	if(_length2OfVector3( vAxis ) > fEpsilon ) {
		if (!_cldTestAxis( v0, v1, v2, vAxis, 6)) { 
			return FALSE; 
		}
	}

	// axis ((Cp0-V2) x E2) x E2
	_CalculateAxis(vCp0,v2,vE2,vE2,vAxis);
	//vAxis = ( ( vCp0-v2) cross vE2 ) cross vE2;
	if(_length2OfVector3( vAxis ) > fEpsilon ) {
		if (!_cldTestAxis( v0, v1, v2, vAxis, 7)) { 
			return FALSE; 
		}
	}

	// second capsule point
	// axis ((Cp1-V0) x E0) x E0
	_CalculateAxis(vCp1,v0,vE0,vE0,vAxis);	
	//vAxis = ( ( vCp1-v0 ) cross vE0 ) cross vE0;
	if(_length2OfVector3( vAxis ) > fEpsilon ) {
		if (!_cldTestAxis( v0, v1, v2, vAxis, 8)) { 
			return FALSE; 
		}
	}

	// axis ((Cp1-V1) x E1) x E1
	_CalculateAxis(vCp1,v1,vE1,vE1,vAxis);	
	//vAxis = ( ( vCp1-v1 ) cross vE1 ) cross vE1;
	if(_length2OfVector3( vAxis ) > fEpsilon ) {
		if (!_cldTestAxis( v0, v1, v2, vAxis, 9)) { 
			return FALSE; 
		}
	}

	// axis ((Cp1-V2) x E2) x E2
	_CalculateAxis(vCp1,v2,vE2,vE2,vAxis);	
	//vAxis = ( ( vCp1-v2 ) cross vE2 ) cross vE2;
	if(_length2OfVector3( vAxis ) > fEpsilon ) {
		if (!_cldTestAxis( v0, v1, v2, vAxis, 10)) { 
			return FALSE; 
		}
	}

	// first vertex on triangle
	// axis ((V0-Cp0) x C) x C
	_CalculateAxis(v0,vCp0,vCapsuleAxis,vCapsuleAxis,vAxis);	
	//vAxis = ( ( v0-vCp0 ) cross vCapsuleAxis ) cross vCapsuleAxis;
	if(_length2OfVector3( vAxis ) > fEpsilon ) {
		if (!_cldTestAxis( v0, v1, v2, vAxis, 11)) { 
			return FALSE; 
		}
	}

	// second vertex on triangle
	// axis ((V1-Cp0) x C) x C
	_CalculateAxis(v1,vCp0,vCapsuleAxis,vCapsuleAxis,vAxis);	
	//vAxis = ( ( v1-vCp0 ) cross vCapsuleAxis ) cross vCapsuleAxis;
	if(_length2OfVector3( vAxis ) > fEpsilon ) {
		if (!_cldTestAxis( v0, v1, v2, vAxis, 12)) { 
			return FALSE; 
		}
	}

	// third vertex on triangle
	// axis ((V2-Cp0) x C) x C
	_CalculateAxis(v2,vCp0,vCapsuleAxis,vCapsuleAxis,vAxis);	
	//vAxis = ( ( v2-vCp0 ) cross vCapsuleAxis ) cross vCapsuleAxis;
	if(_length2OfVector3( vAxis ) > fEpsilon ) {
		if (!_cldTestAxis( v0, v1, v2, vAxis, 13)) { 
			return FALSE; 
		}
	}

	// Test as separating axes direction vectors between each triangle
	// edge and each capsule's cap center

	// first triangle vertex and first capsule point
	//vAxis = v0 - vCp0;
	SUBTRACT(v0,vCp0,vAxis);
	if(_length2OfVector3( vAxis ) > fEpsilon ) {
		if (!_cldTestAxis( v0, v1, v2, vAxis, 14)) { 
			return FALSE; 
		}
	}

	// second triangle vertex and first capsule point
	//vAxis = v1 - vCp0;
	SUBTRACT(v1,vCp0,vAxis);
	if(_length2OfVector3( vAxis ) > fEpsilon ) {
		if (!_cldTestAxis( v0, v1, v2, vAxis, 15)) { 
			return FALSE; 
		}
	}

	// third triangle vertex and first capsule point
	//vAxis = v2 - vCp0;
	SUBTRACT(v2,vCp0,vAxis);
	if(_length2OfVector3( vAxis ) > fEpsilon ) {
		if (!_cldTestAxis( v0, v1, v2, vAxis, 16)) { 
			return FALSE; 
		}
	}

	// first triangle vertex and second capsule point
	//vAxis = v0 - vCp1;
	SUBTRACT(v0,vCp1,vAxis);
	if(_length2OfVector3( vAxis ) > fEpsilon ) {
		if (!_cldTestAxis( v0, v1, v2, vAxis, 17)) { 
			return FALSE; 
		}
	}

	// second triangle vertex and second capsule point
	//vAxis = v1 - vCp1;
	SUBTRACT(v1,vCp1,vAxis);
	if(_length2OfVector3( vAxis ) > fEpsilon ) {
		if (!_cldTestAxis( v0, v1, v2, vAxis, 18)) { 
			return FALSE; 
		}
	}

	// third triangle vertex and second capsule point
	//vAxis = v2 - vCp1;
	SUBTRACT(v2,vCp1,vAxis);
	if(_length2OfVector3( vAxis ) > fEpsilon ) {
		if (!_cldTestAxis( v0, v1, v2, vAxis, 19)) { 
			return FALSE; 
		}
	}	

	return TRUE;
}

// test one mesh triangle on intersection with capsule
static void _cldTestOneTriangleVSCCylinder( const dVector3 &v0, 
											const dVector3 &v1, 
											const dVector3 &v2 )
{

	// calculate edges
	SUBTRACT(v1,v0,vE0);
	SUBTRACT(v2,v1,vE1);
	SUBTRACT(v0,v2,vE2);

	dVector3	_minus_vE0;
	SUBTRACT(v0,v1,_minus_vE0);

	// calculate poly normal
	dCROSS(vN,=,vE1,_minus_vE0);
	dNormalize3(vN);
	
	// create plane from triangle
	dReal plDistance = -dDOT(v0,vN);
	dVector4 plTrianglePlane;
	CONSTRUCTPLANE(plTrianglePlane,vN,plDistance);
	
	// calculate capsule distance to plane
	dReal fDistanceCapsuleCenterToPlane = POINTDISTANCE(plTrianglePlane,vCapsulePosition);

	// Capsule must be over positive side of triangle
	if(fDistanceCapsuleCenterToPlane < 0 /* && !bDoubleSided*/) 
	{
		// if not don't generate contacts
		return;
	}

	dVector3 vPnt0;
	SET	(vPnt0,v0);
	dVector3 vPnt1;
	SET	(vPnt1,v1);
	dVector3 vPnt2;
	SET	(vPnt2,v2);

	if (fDistanceCapsuleCenterToPlane < 0 )
	{
		SET	(vPnt0,v0);
		SET	(vPnt1,v2);
		SET	(vPnt2,v1);
	}

	// do intersection test and find best separating axis
	if(!_cldTestSeparatingAxesOfCapsule(vPnt0, vPnt1, vPnt2) ) 
	{
		// if not found do nothing
		return;
	}

	// if best separation axis is not found
	if ( iBestAxis == 0 ) 
	{
		// this should not happen (we should already exit in that case)
		ASSERT(FALSE);
		// do nothing
		return;
	}

	// calculate caps centers in absolute space
	dVector3 vCposTrans;
	vCposTrans[0] = vCapsulePosition[0] + vNormal[0]*vCapsuleRadius;
	vCposTrans[1] = vCapsulePosition[1] + vNormal[1]*vCapsuleRadius;
	vCposTrans[2] = vCapsulePosition[2] + vNormal[2]*vCapsuleRadius;

	dVector3 vCEdgePoint0;
	vCEdgePoint0[0]  = vCposTrans[0] + vCapsuleAxis[0]*(fCapsuleSize*REAL(0.5)-vCapsuleRadius);
	vCEdgePoint0[1]  = vCposTrans[1] + vCapsuleAxis[1]*(fCapsuleSize*REAL(0.5)-vCapsuleRadius);
	vCEdgePoint0[2]  = vCposTrans[2] + vCapsuleAxis[2]*(fCapsuleSize*REAL(0.5)-vCapsuleRadius);
    
	dVector3 vCEdgePoint1;
	vCEdgePoint1[0] = vCposTrans[0] - vCapsuleAxis[0]*(fCapsuleSize*REAL(0.5)-vCapsuleRadius);
	vCEdgePoint1[1] = vCposTrans[1] - vCapsuleAxis[1]*(fCapsuleSize*REAL(0.5)-vCapsuleRadius);
	vCEdgePoint1[2] = vCposTrans[2] - vCapsuleAxis[2]*(fCapsuleSize*REAL(0.5)-vCapsuleRadius);

	// transform capsule edge points into triangle space
	vCEdgePoint0[0] -= vPnt0[0];
	vCEdgePoint0[1] -= vPnt0[1];
	vCEdgePoint0[2] -= vPnt0[2];

	vCEdgePoint1[0] -= vPnt0[0];
	vCEdgePoint1[1] -= vPnt0[1];
	vCEdgePoint1[2] -= vPnt0[2];

	dVector4 plPlane;
	dVector3 _minus_vN;
	_minus_vN[0] = -vN[0];
	_minus_vN[1] = -vN[1];
	_minus_vN[2] = -vN[2];
	// triangle plane
	CONSTRUCTPLANE(plPlane,_minus_vN,0);
	//plPlane = Plane4f( -vN, 0);

	if(!_cldClipEdgeToPlane( vCEdgePoint0, vCEdgePoint1, plPlane )) 
	{ 
		return; 
	}

	// plane with edge 0
	dVector3 vTemp;
	dCROSS(vTemp,=,vN,vE0);
	CONSTRUCTPLANE(plPlane, vTemp, 1e-5f);
	if(!_cldClipEdgeToPlane( vCEdgePoint0, vCEdgePoint1, plPlane ))
	{ 
		return; 
	}

	dCROSS(vTemp,=,vN,vE1);
	CONSTRUCTPLANE(plPlane, vTemp, -(dDOT(vE0,vTemp)-1e-5f));
	if(!_cldClipEdgeToPlane( vCEdgePoint0, vCEdgePoint1, plPlane )) 
	{ 
		return; 
	}

	dCROSS(vTemp,=,vN,vE2);
	CONSTRUCTPLANE(plPlane, vTemp, 1e-5f);
	if(!_cldClipEdgeToPlane( vCEdgePoint0, vCEdgePoint1, plPlane )) { 
		return; 
	}

	// return capsule edge points into absolute space
	vCEdgePoint0[0] += vPnt0[0];
	vCEdgePoint0[1] += vPnt0[1];
	vCEdgePoint0[2] += vPnt0[2];

	vCEdgePoint1[0] += vPnt0[0];
	vCEdgePoint1[1] += vPnt0[1];
	vCEdgePoint1[2] += vPnt0[2];

	// calculate depths for both contact points
	SUBTRACT(vCEdgePoint0,vCapsulePosition,vTemp);
	dReal fDepth0 = dDOT(vTemp,vNormal) - (fBestCenter-fBestrt);
	SUBTRACT(vCEdgePoint1,vCapsulePosition,vTemp);
	dReal fDepth1 = dDOT(vTemp,vNormal) - (fBestCenter-fBestrt);

	// clamp depths to zero
	if(fDepth0 < 0) 
	{
		fDepth0 = 0.0f;
	}

	if(fDepth1 < 0 ) 
	{
		fDepth1 = 0.0f;
	}

	// Cached contacts's data
	// contact 0
    if (ctContacts < (iFlags & NUMC_MASK)) {
	gLocalContacts[ctContacts].fDepth = fDepth0;
	SET(gLocalContacts[ctContacts].vNormal,vNormal);
	SET(gLocalContacts[ctContacts].vPos,vCEdgePoint0);
	gLocalContacts[ctContacts].nFlags = 1;
	ctContacts++;

        if (ctContacts < (iFlags & NUMC_MASK)) {
	// contact 1
	gLocalContacts[ctContacts].fDepth = fDepth1;
	SET(gLocalContacts[ctContacts].vNormal,vNormal);
	SET(gLocalContacts[ctContacts].vPos,vCEdgePoint1);
	gLocalContacts[ctContacts].nFlags = 1;
	ctContacts++;
        }
    }

}

// capsule - trimesh by CroTeam
// Ported by Nguyem Binh
int dCollideCCTL(dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip)
{
	dxTriMesh* TriMesh = (dxTriMesh*)o1;
	gCylinder = o2;
	gTriMesh = o1;

	const dMatrix3* pRot = (const dMatrix3*) dGeomGetRotation(gCylinder);
	memcpy(mCapsuleRotation,pRot,sizeof(dMatrix3));

	const dVector3* pDst = (const dVector3*)dGeomGetPosition(gCylinder);
	memcpy(vCapsulePosition,pDst,sizeof(dVector3));

	vCapsuleAxis[0] = mCapsuleRotation[0*4 + nCAPSULE_AXIS]; 
	vCapsuleAxis[1] = mCapsuleRotation[1*4 + nCAPSULE_AXIS];
	vCapsuleAxis[2] = mCapsuleRotation[2*4 + nCAPSULE_AXIS];

	// Get size of CCylinder
	dGeomCCylinderGetParams(gCylinder,&vCapsuleRadius,&fCapsuleSize);
	fCapsuleSize += 2*vCapsuleRadius;

	const dMatrix3* pTriRot = (const dMatrix3*)dGeomGetRotation(TriMesh);
	memcpy(mTriMeshRot,pTriRot,sizeof(dMatrix3));

	const dVector3* pTriPos = (const dVector3*)dGeomGetPosition(TriMesh);
	memcpy(mTriMeshPos,pTriPos,sizeof(dVector3));	

	// global info for contact creation
	iStride			=skip;
	iFlags			=flags;
	ContactGeoms	=contact;

	// reset contact counter
	ctContacts = 0;	
    // allocat local contact workspace
    gLocalContacts = new sLocalContactData[(iFlags & NUMC_MASK)];

	// reset best depth
	fBestDepth  = - MAX_REAL;
	fBestCenter = 0;
	fBestrt     = 0;

	// reset collision normal
	vNormal[0] = REAL(0.0);
	vNormal[1] = REAL(0.0);
	vNormal[2] = REAL(0.0);

	// Will it better to use LSS here? -> confirm Pierre.
	 OBBCollider& Collider = TriMesh->_OBBCollider;

	 Point cCenter((float) vCapsulePosition[0],(float) vCapsulePosition[1],(float) vCapsulePosition[2]);
	 Point cExtents((float) vCapsuleRadius,(float) vCapsuleRadius,(float) fCapsuleSize/2);

	 Matrix3x3 obbRot;

	 obbRot[0][0] = (float) mCapsuleRotation[0];
	 obbRot[1][0] = (float) mCapsuleRotation[1];
	 obbRot[2][0] = (float) mCapsuleRotation[2];

	 obbRot[0][1] = (float) mCapsuleRotation[4];
	 obbRot[1][1] = (float) mCapsuleRotation[5];
	 obbRot[2][1] = (float) mCapsuleRotation[6];

	 obbRot[0][2] = (float) mCapsuleRotation[8];
	 obbRot[1][2] = (float) mCapsuleRotation[9];
	 obbRot[2][2] = (float) mCapsuleRotation[10];

	 OBB obbCCylinder(cCenter,cExtents,obbRot);

	 Matrix4x4 CCylinderMatrix;
	 MakeMatrix(vCapsulePosition, mCapsuleRotation, CCylinderMatrix);

	 Matrix4x4 MeshMatrix;
	 MakeMatrix(mTriMeshPos, mTriMeshRot, MeshMatrix);

	 // TC results
	 if (TriMesh->doBoxTC) {
		 dxTriMesh::BoxTC* BoxTC = 0;
		 for (int i = 0; i < TriMesh->BoxTCCache.size(); i++){
			 if (TriMesh->BoxTCCache[i].Geom == gCylinder){
				 BoxTC = &TriMesh->BoxTCCache[i];
				 break;
			 }
		 }
		 if (!BoxTC){
			 TriMesh->BoxTCCache.push(dxTriMesh::BoxTC());

			 BoxTC = &TriMesh->BoxTCCache[TriMesh->BoxTCCache.size() - 1];
			 BoxTC->Geom = gCylinder;
			 BoxTC->FatCoeff = 1.0f;
		 }

		 // Intersect
		 Collider.SetTemporalCoherence(true);
		 Collider.Collide(*BoxTC, obbCCylinder, TriMesh->Data->BVTree, null, &MeshMatrix);
	 }
	 else {
		 Collider.SetTemporalCoherence(false);
		 Collider.Collide(dxTriMesh::defaultBoxCache, obbCCylinder, TriMesh->Data->BVTree, null,&MeshMatrix);
	 }

         if (!Collider.GetContactStatus()) {
            /* no collision occurred */
            return 0;
         }

	 // Retrieve data
	 int TriCount = Collider.GetNbTouchedPrimitives();
	 const int* Triangles = (const int*)Collider.GetTouchedPrimitives();

	 if (TriCount != 0)
	 {
		 if (TriMesh->ArrayCallback != null)
		 {
			 TriMesh->ArrayCallback(TriMesh, gCylinder, Triangles, TriCount);
		 }

		int OutTriCount = 0;

		// loop through all intersecting triangles
		for (int i = 0; i < TriCount; i++)
		{
			if(ctContacts>=(iFlags & NUMC_MASK)) 
			{
				break;
			}

			const int& Triint = Triangles[i];
			if (!Callback(TriMesh, gCylinder, Triint)) continue;


			dVector3 dv[3];
			FetchTriangle(TriMesh, Triint, mTriMeshPos, mTriMeshRot, dv);

			// test this triangle
			_cldTestOneTriangleVSCCylinder(dv[0],dv[1],dv[2]);
			
		}
	 }

	return _ProcessLocalContacts();
}

