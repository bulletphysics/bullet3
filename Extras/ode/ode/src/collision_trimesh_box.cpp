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


/*************************************************************************
 *                                                                       *
 * Triangle-box collider by Alen Ladavac and Vedran Klanac.              *
 * Ported to ODE by Oskari Nyman.                                        *
 *                                                                       *
 *************************************************************************/


#include <ode/collision.h>
#include <ode/matrix.h>
#include <ode/rotation.h>
#include <ode/odemath.h>
#include "collision_util.h"

#define TRIMESH_INTERNAL
#include "collision_trimesh_internal.h"

static void
GenerateContact(int in_Flags, dContactGeom* in_Contacts, int in_Stride,  
                dxGeom* in_g1,  dxGeom* in_g2,
                const dVector3 in_ContactPos, const dVector3 in_Normal, dReal in_Depth,
                int& OutTriCount);


// largest number, double or float
#if defined(dSINGLE)
  #define MAXVALUE FLT_MAX
#else
  #define MAXVALUE DBL_MAX
#endif


// dVector3
// r=a-b
#define SUBTRACT(a,b,r) do{ \
  (r)[0]=(a)[0] - (b)[0]; \
  (r)[1]=(a)[1] - (b)[1]; \
  (r)[2]=(a)[2] - (b)[2]; }while(0)


// dVector3
// a=b
#define SET(a,b) do{ \
  (a)[0]=(b)[0]; \
  (a)[1]=(b)[1]; \
  (a)[2]=(b)[2]; }while(0)


// dMatrix3
// a=b
#define SETM(a,b) do{ \
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
                     (a)[11]=(b)[11]; }while(0)


// dVector3
// r=a+b
#define ADD(a,b,r) do{ \
  (r)[0]=(a)[0] + (b)[0]; \
  (r)[1]=(a)[1] + (b)[1]; \
  (r)[2]=(a)[2] + (b)[2]; }while(0)


// dMatrix3, int, dVector3
// v=column a from m
#define GETCOL(m,a,v) do{ \
  (v)[0]=(m)[(a)+0]; \
  (v)[1]=(m)[(a)+4]; \
  (v)[2]=(m)[(a)+8]; }while(0)


// dVector4, dVector3
// distance between plane p and point v
#define POINTDISTANCE(p,v) \
  ( p[0]*v[0] + p[1]*v[1] + p[2]*v[2] + p[3] )


// dVector4, dVector3, dReal
// construct plane from normal and d
#define CONSTRUCTPLANE(plane,normal,d) do{ \
  plane[0]=normal[0];\
  plane[1]=normal[1];\
  plane[2]=normal[2];\
  plane[3]=d; }while(0)


// dVector3
// length of vector a
#define LENGTHOF(a) \
  dSqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2])


// box data
static dMatrix3 mHullBoxRot;
static dVector3 vHullBoxPos;
static dVector3 vBoxHalfSize;

// mesh data
static dVector3   vHullDstPos;

// global collider data
static dVector3 vBestNormal;
static dReal    fBestDepth;
static int    iBestAxis = 0;
static int    iExitAxis = 0;
static dVector3 vE0, vE1, vE2, vN;

// global info for contact creation
static int iFlags;
static dContactGeom *ContactGeoms;
static int iStride;
static dxGeom *Geom1;
static dxGeom *Geom2;
static int ctContacts = 0;



// Test normal of mesh face as separating axis for intersection
static BOOL _cldTestNormal( dReal fp0, dReal fR, dVector3 vNormal, int iAxis ) 
{
  // calculate overlapping interval of box and triangle
  dReal fDepth = fR+fp0;
  
  // if we do not overlap
  if ( fDepth<0 ) { 
    // do nothing
    return FALSE;
  }

  // calculate normal's length
  dReal fLength = LENGTHOF(vNormal);
  // if long enough
  if ( fLength > 0.0f ) {

    dReal fOneOverLength = 1.0f/fLength;
    // normalize depth
    fDepth = fDepth*fOneOverLength;

    // get minimum depth
    if (fDepth<fBestDepth) {
      vBestNormal[0] = -vNormal[0]*fOneOverLength;
      vBestNormal[1] = -vNormal[1]*fOneOverLength;
      vBestNormal[2] = -vNormal[2]*fOneOverLength;
      iBestAxis = iAxis;
      //dAASSERT(fDepth>=0);
      fBestDepth = fDepth;
    }

  }

  return TRUE;
}




// Test box axis as separating axis 
static BOOL _cldTestFace( dReal fp0, dReal fp1, dReal fp2, dReal fR, dReal fD, 
                          dVector3 vNormal, int iAxis ) 
{
  dReal fMin, fMax;

  // find min of triangle interval 
  if ( fp0 < fp1 ) {
    if ( fp0 < fp2 ) {
      fMin = fp0;
    } else {
      fMin = fp2;
    }
  } else {
    if( fp1 < fp2 ) {
      fMin = fp1; 
    } else {
      fMin = fp2;
    }
  }

  // find max of triangle interval 
  if ( fp0 > fp1 ) {
    if ( fp0 > fp2 ) {
      fMax = fp0;
    } else {
      fMax = fp2;
    }
  } else {
    if( fp1 > fp2 ) {
      fMax = fp1; 
    } else {
      fMax = fp2;
    }
  }

  // calculate minimum and maximum depth
  dReal fDepthMin = fR - fMin;
  dReal fDepthMax = fMax + fR;

  // if we dont't have overlapping interval
  if ( fDepthMin < 0 || fDepthMax < 0 ) {
    // do nothing
    return FALSE;
  }

  dReal fDepth = 0;

  // if greater depth is on negative side 
  if ( fDepthMin > fDepthMax ) {
    // use smaller depth (one from positive side)
    fDepth = fDepthMax;
    // flip normal direction
    vNormal[0] = -vNormal[0];
    vNormal[1] = -vNormal[1];
    vNormal[2] = -vNormal[2];
    fD = -fD;
  // if greater depth is on positive side 
  } else {
    // use smaller depth (one from negative side)
    fDepth = fDepthMin;   
  }

  
  // if lower depth than best found so far 
  if (fDepth<fBestDepth) {
    // remember current axis as best axis
    vBestNormal[0]  = vNormal[0];
    vBestNormal[1]  = vNormal[1];
    vBestNormal[2]  = vNormal[2];
    iBestAxis    = iAxis;
    //dAASSERT(fDepth>=0);
    fBestDepth   = fDepth;
  }

  return TRUE;
}





// Test cross products of box axis and triangle edges as separating axis
static BOOL _cldTestEdge( dReal fp0, dReal fp1, dReal fR, dReal fD, 
                          dVector3 vNormal, int iAxis ) 
{
  dReal fMin, fMax;

  // calculate min and max interval values  
  if ( fp0 < fp1 ) {
    fMin = fp0;
    fMax = fp1;
  } else {
    fMin = fp1;
    fMax = fp0;    
  }

  // check if we overlapp
  dReal fDepthMin = fR - fMin;
  dReal fDepthMax = fMax + fR;

  // if we don't overlapp
  if ( fDepthMin < 0 || fDepthMax < 0 ) {
    // do nothing
    return FALSE;
  }

  dReal fDepth;
  

  // if greater depth is on negative side 
  if ( fDepthMin > fDepthMax ) {
    // use smaller depth (one from positive side)
    fDepth = fDepthMax;
    // flip normal direction
    vNormal[0] = -vNormal[0];
    vNormal[1] = -vNormal[1];
    vNormal[2] = -vNormal[2];
    fD = -fD;
  // if greater depth is on positive side 
  } else {
    // use smaller depth (one from negative side)
    fDepth = fDepthMin;   
  }

  // calculate normal's length
  dReal fLength = LENGTHOF(vNormal);

  // if long enough
  if ( fLength > 0.0f ) {

    // normalize depth
    dReal fOneOverLength = 1.0f/fLength;
    fDepth = fDepth*fOneOverLength;
    fD*=fOneOverLength;
    

    // if lower depth than best found so far (favor face over edges)
    if (fDepth*1.5f<fBestDepth) {
      // remember current axis as best axis
      vBestNormal[0]  = vNormal[0]*fOneOverLength;
      vBestNormal[1]  = vNormal[1]*fOneOverLength;
      vBestNormal[2]  = vNormal[2]*fOneOverLength;
      iBestAxis    = iAxis;
      //dAASSERT(fDepth>=0);
      fBestDepth   = fDepth;
    }
  }

  return TRUE;
}





// clip polygon with plane and generate new polygon points
static void _cldClipPolyToPlane( dVector3 avArrayIn[], int ctIn, 
                      dVector3 avArrayOut[], int &ctOut, 
                      const dVector4 &plPlane )
{
  // start with no output points
  ctOut = 0;

  int i0 = ctIn-1;

  // for each edge in input polygon
  for (int i1=0; i1<ctIn; i0=i1, i1++) {
  

    // calculate distance of edge points to plane
    dReal fDistance0 = POINTDISTANCE( plPlane ,avArrayIn[i0] );
    dReal fDistance1 = POINTDISTANCE( plPlane ,avArrayIn[i1] );


    // if first point is in front of plane
    if( fDistance0 >= 0 ) {
      // emit point
      avArrayOut[ctOut][0] = avArrayIn[i0][0];
      avArrayOut[ctOut][1] = avArrayIn[i0][1];
      avArrayOut[ctOut][2] = avArrayIn[i0][2];
      ctOut++;
    }

    // if points are on different sides
    if( (fDistance0 > 0 && fDistance1 < 0) || ( fDistance0 < 0 && fDistance1 > 0) ) {

      // find intersection point of edge and plane
      dVector3 vIntersectionPoint;
      vIntersectionPoint[0]= avArrayIn[i0][0] - (avArrayIn[i0][0]-avArrayIn[i1][0])*fDistance0/(fDistance0-fDistance1);
      vIntersectionPoint[1]= avArrayIn[i0][1] - (avArrayIn[i0][1]-avArrayIn[i1][1])*fDistance0/(fDistance0-fDistance1);
      vIntersectionPoint[2]= avArrayIn[i0][2] - (avArrayIn[i0][2]-avArrayIn[i1][2])*fDistance0/(fDistance0-fDistance1);

      // emit intersection point
      avArrayOut[ctOut][0] = vIntersectionPoint[0];
      avArrayOut[ctOut][1] = vIntersectionPoint[1];
      avArrayOut[ctOut][2] = vIntersectionPoint[2];
      ctOut++;
    }
  }

}




static BOOL _cldTestSeparatingAxes(const dVector3 &v0, const dVector3 &v1, const dVector3 &v2) {
  // reset best axis
  iBestAxis = 0;
  iExitAxis = -1;
  fBestDepth = MAXVALUE;

  // calculate edges
  SUBTRACT(v1,v0,vE0);
  SUBTRACT(v2,v0,vE1);
  SUBTRACT(vE1,vE0,vE2);

  // calculate poly normal
  dCROSS(vN,=,vE0,vE1);

  // extract box axes as vectors
  dVector3 vA0,vA1,vA2;
  GETCOL(mHullBoxRot,0,vA0);
  GETCOL(mHullBoxRot,1,vA1);
  GETCOL(mHullBoxRot,2,vA2);

  // box halfsizes
  dReal fa0 = vBoxHalfSize[0];
  dReal fa1 = vBoxHalfSize[1];
  dReal fa2 = vBoxHalfSize[2];

  // calculate relative position between box and triangle
  dVector3 vD;
  SUBTRACT(v0,vHullBoxPos,vD);

  // calculate length of face normal
  dReal fNLen = LENGTHOF( vN );

  dVector3 vL;
  dReal fp0, fp1, fp2, fR, fD;

  // Test separating axes for intersection
  // ************************************************
  // Axis 1 - Triangle Normal 
  SET(vL,vN);
  fp0  = dDOT(vL,vD);
  fp1  = fp0;
  fp2  = fp0;
  fR=fa0*dFabs( dDOT(vN,vA0) ) + fa1 * dFabs( dDOT(vN,vA1) ) + fa2 * dFabs( dDOT(vN,vA2) );


  if( !_cldTestNormal( fp0, fR, vL, 1) ) { 
    iExitAxis=1;
    return FALSE; 
  } 
 
  // ************************************************

  // Test Faces
  // ************************************************
  // Axis 2 - Box X-Axis
  SET(vL,vA0);
  fD  = dDOT(vL,vN)/fNLen;
  fp0 = dDOT(vL,vD);
  fp1 = fp0 + dDOT(vA0,vE0);
  fp2 = fp0 + dDOT(vA0,vE1);
  fR  = fa0;


  if( !_cldTestFace( fp0, fp1, fp2, fR, fD, vL, 2) ) { 
    iExitAxis=2;
    return FALSE; 
  }
  // ************************************************

  // ************************************************
  // Axis 3 - Box Y-Axis
  SET(vL,vA1);
  fD = dDOT(vL,vN)/fNLen;
  fp0 = dDOT(vL,vD);
  fp1 = fp0 + dDOT(vA1,vE0);
  fp2 = fp0 + dDOT(vA1,vE1);
  fR  = fa1;


  if( !_cldTestFace( fp0, fp1, fp2, fR, fD, vL, 3) ) { 
    iExitAxis=3;
    return FALSE; 
  }

  // ************************************************

  // ************************************************
  // Axis 4 - Box Z-Axis
  SET(vL,vA2);
  fD = dDOT(vL,vN)/fNLen;
  fp0 = dDOT(vL,vD);
  fp1 = fp0 + dDOT(vA2,vE0);
  fp2 = fp0 + dDOT(vA2,vE1);
  fR  = fa2;


  if( !_cldTestFace( fp0, fp1, fp2, fR, fD, vL, 4) ) { 
    iExitAxis=4;
    return FALSE; 
  }

  // ************************************************

  // Test Edges
  // ************************************************
  // Axis 5 - Box X-Axis cross Edge0
  dCROSS(vL,=,vA0,vE0);
  fD  = dDOT(vL,vN)/fNLen;
  fp0 = dDOT(vL,vD);
  fp1 = fp0;
  fp2 = fp0 + dDOT(vA0,vN);
  fR  = fa1 * dFabs(dDOT(vA2,vE0)) + fa2 * dFabs(dDOT(vA1,vE0));


  if( !_cldTestEdge( fp1, fp2, fR, fD, vL, 5) ) { 
    iExitAxis=5;
    return FALSE; 
  }
  // ************************************************

  // ************************************************
  // Axis 6 - Box X-Axis cross Edge1
  dCROSS(vL,=,vA0,vE1);
  fD  = dDOT(vL,vN)/fNLen;
  fp0 = dDOT(vL,vD);
  fp1 = fp0 - dDOT(vA0,vN);
  fp2 = fp0;
  fR  = fa1 * dFabs(dDOT(vA2,vE1)) + fa2 * dFabs(dDOT(vA1,vE1));


  if( !_cldTestEdge( fp0, fp1, fR, fD, vL, 6) ) { 
    iExitAxis=6;
    return FALSE; 
  }
  // ************************************************

  // ************************************************
  // Axis 7 - Box X-Axis cross Edge2
  dCROSS(vL,=,vA0,vE2);
  fD  = dDOT(vL,vN)/fNLen;
  fp0 = dDOT(vL,vD);
  fp1 = fp0 - dDOT(vA0,vN);
  fp2 = fp0 - dDOT(vA0,vN);
  fR  = fa1 * dFabs(dDOT(vA2,vE2)) + fa2 * dFabs(dDOT(vA1,vE2));


  if( !_cldTestEdge( fp0, fp1, fR, fD, vL, 7) ) { 
    iExitAxis=7;
    return FALSE; 
  }

  // ************************************************

  // ************************************************
  // Axis 8 - Box Y-Axis cross Edge0
  dCROSS(vL,=,vA1,vE0);
  fD  = dDOT(vL,vN)/fNLen;
  fp0 = dDOT(vL,vD);
  fp1 = fp0;
  fp2 = fp0 + dDOT(vA1,vN);
  fR  = fa0 * dFabs(dDOT(vA2,vE0)) + fa2 * dFabs(dDOT(vA0,vE0));


  if( !_cldTestEdge( fp0, fp2, fR, fD, vL, 8) ) { 
    iExitAxis=8;
    return FALSE; 
  }

  // ************************************************

  // ************************************************
  // Axis 9 - Box Y-Axis cross Edge1
  dCROSS(vL,=,vA1,vE1);
  fD  = dDOT(vL,vN)/fNLen;
  fp0 = dDOT(vL,vD);
  fp1 = fp0 - dDOT(vA1,vN);
  fp2 = fp0;
  fR  = fa0 * dFabs(dDOT(vA2,vE1)) + fa2 * dFabs(dDOT(vA0,vE1));


  if( !_cldTestEdge( fp0, fp1, fR, fD, vL, 9) ) { 
    iExitAxis=9;
    return FALSE; 
  }

  // ************************************************

  // ************************************************
  // Axis 10 - Box Y-Axis cross Edge2
  dCROSS(vL,=,vA1,vE2);
  fD  = dDOT(vL,vN)/fNLen;
  fp0 = dDOT(vL,vD);
  fp1 = fp0 - dDOT(vA1,vN);
  fp2 = fp0 - dDOT(vA1,vN);
  fR  = fa0 * dFabs(dDOT(vA2,vE2)) + fa2 * dFabs(dDOT(vA0,vE2));


  if( !_cldTestEdge( fp0, fp1, fR, fD, vL, 10) ) { 
    iExitAxis=10;
    return FALSE; 
  }

  // ************************************************

  // ************************************************
  // Axis 11 - Box Z-Axis cross Edge0
  dCROSS(vL,=,vA2,vE0);
  fD  = dDOT(vL,vN)/fNLen;
  fp0 = dDOT(vL,vD);
  fp1 = fp0;
  fp2 = fp0 + dDOT(vA2,vN);
  fR  = fa0 * dFabs(dDOT(vA1,vE0)) + fa1 * dFabs(dDOT(vA0,vE0));


  if( !_cldTestEdge( fp0, fp2, fR, fD, vL, 11) ) { 
    iExitAxis=11;
    return FALSE; 
  }
  // ************************************************

  // ************************************************
  // Axis 12 - Box Z-Axis cross Edge1
  dCROSS(vL,=,vA2,vE1);
  fD  = dDOT(vL,vN)/fNLen;
  fp0 = dDOT(vL,vD);
  fp1 = fp0 - dDOT(vA2,vN);
  fp2 = fp0;
  fR  = fa0 * dFabs(dDOT(vA1,vE1)) + fa1 * dFabs(dDOT(vA0,vE1));


  if( !_cldTestEdge( fp0, fp1, fR, fD, vL, 12) ) { 
    iExitAxis=12;
    return FALSE; 
  }
  // ************************************************

  // ************************************************
  // Axis 13 - Box Z-Axis cross Edge2
  dCROSS(vL,=,vA2,vE2);
  fD  = dDOT(vL,vN)/fNLen;
  fp0 = dDOT(vL,vD);
  fp1 = fp0 - dDOT(vA2,vN);
  fp2 = fp0 - dDOT(vA2,vN);
  fR  = fa0 * dFabs(dDOT(vA1,vE2)) + fa1 * dFabs(dDOT(vA0,vE2));


  if( !_cldTestEdge( fp0, fp1, fR, fD, vL, 13) ) { 
    iExitAxis=13;
    return FALSE; 
  }
 
  // ************************************************
  return TRUE; 
}





// find two closest points on two lines
static BOOL _cldClosestPointOnTwoLines( dVector3 vPoint1, dVector3 vLenVec1, 
                                        dVector3 vPoint2, dVector3 vLenVec2, 
                                        dReal &fvalue1, dReal &fvalue2) 
{
  // calulate denominator
  dVector3 vp;
  SUBTRACT(vPoint2,vPoint1,vp);
  dReal fuaub  = dDOT(vLenVec1,vLenVec2);
  dReal fq1    = dDOT(vLenVec1,vp);
  dReal fq2    = -dDOT(vLenVec2,vp);
  dReal fd     = 1.0f - fuaub * fuaub;
  
  // if denominator is positive
  if (fd > 0.0f) {
    // calculate points of closest approach
    fd = 1.0f/fd;
    fvalue1 = (fq1 + fuaub*fq2)*fd;
    fvalue2 = (fuaub*fq1 + fq2)*fd;
    return TRUE;
  // otherwise  
  } else {
    // lines are parallel
    fvalue1 = 0.0f;
    fvalue2 = 0.0f;
    return FALSE;
  }

}





// clip and generate contacts
static void _cldClipping(const dVector3 &v0, const dVector3 &v1, const dVector3 &v2) {

  // if we have edge/edge intersection
  if ( iBestAxis > 4 ) {

    dVector3 vub,vPb,vPa;

    SET(vPa,vHullBoxPos);

    // calculate point on box edge
    for( int i=0; i<3; i++) {
      dVector3 vRotCol;
      GETCOL(mHullBoxRot,i,vRotCol);
      dReal fSign = dDOT(vBestNormal,vRotCol) > 0 ? 1.0f : -1.0f;

      vPa[0] += fSign * vBoxHalfSize[i] * vRotCol[0];
      vPa[1] += fSign * vBoxHalfSize[i] * vRotCol[1];
      vPa[2] += fSign * vBoxHalfSize[i] * vRotCol[2];
    }

    int iEdge = (iBestAxis-5)%3;

    // decide which edge is on triangle
    if ( iEdge == 0 ) {
      SET(vPb,v0);
      SET(vub,vE0);
    } else if ( iEdge == 1) {
      SET(vPb,v2);
      SET(vub,vE1);
    } else {
      SET(vPb,v1);
      SET(vub,vE2);
    }
    

    // setup direction parameter for face edge
    dNormalize3(vub);

    dReal fParam1, fParam2;

    // setup direction parameter for box edge
    dVector3 vua;
    int col=(iBestAxis-5)/3;
    GETCOL(mHullBoxRot,col,vua);

    // find two closest points on both edges
    _cldClosestPointOnTwoLines( vPa, vua, vPb, vub, fParam1, fParam2 );
    vPa[0] += vua[0]*fParam1;
    vPa[1] += vua[1]*fParam1;
    vPa[2] += vua[2]*fParam1;

    vPb[0] += vub[0]*fParam2; 
    vPb[1] += vub[1]*fParam2; 
    vPb[2] += vub[2]*fParam2; 

    // calculate collision point
    dVector3 vPntTmp;
    ADD(vPa,vPb,vPntTmp);

    vPntTmp[0]*=0.5f;
    vPntTmp[1]*=0.5f;
    vPntTmp[2]*=0.5f;

    // generate contact point between two closest points
#ifdef ORIG
    if (ctContacts < (iFlags & 0x0ffff)) {
    dContactGeom* Contact = SAFECONTACT(iFlags, ContactGeoms, ctContacts, iStride);
    Contact->depth = fBestDepth;
    SET(Contact->normal,vBestNormal);
    SET(Contact->pos,vPntTmp);
    Contact->g1 = Geom1;
    Contact->g2 = Geom2;
    ctContacts++;
    }
#endif
    GenerateContact(iFlags, ContactGeoms, iStride,  Geom1, Geom2,
                    vPntTmp, vBestNormal, fBestDepth, ctContacts);



  // if triangle is the referent face then clip box to triangle face
  } else if ( iBestAxis == 1 ) {
    
    
    dVector3 vNormal2;
    vNormal2[0]=-vBestNormal[0];
    vNormal2[1]=-vBestNormal[1];
    vNormal2[2]=-vBestNormal[2];

    
    // vNr is normal in box frame, pointing from triangle to box
    dMatrix3 mTransposed;
    mTransposed[0*4+0]=mHullBoxRot[0*4+0];
    mTransposed[0*4+1]=mHullBoxRot[1*4+0];
    mTransposed[0*4+2]=mHullBoxRot[2*4+0];

    mTransposed[1*4+0]=mHullBoxRot[0*4+1];
    mTransposed[1*4+1]=mHullBoxRot[1*4+1];
    mTransposed[1*4+2]=mHullBoxRot[2*4+1];

    mTransposed[2*4+0]=mHullBoxRot[0*4+2];
    mTransposed[2*4+1]=mHullBoxRot[1*4+2];
    mTransposed[2*4+2]=mHullBoxRot[2*4+2];

    dVector3 vNr;
    vNr[0]=mTransposed[0*4+0]*vNormal2[0]+  mTransposed[0*4+1]*vNormal2[1]+  mTransposed[0*4+2]*vNormal2[2];
    vNr[1]=mTransposed[1*4+0]*vNormal2[0]+  mTransposed[1*4+1]*vNormal2[1]+  mTransposed[1*4+2]*vNormal2[2];
    vNr[2]=mTransposed[2*4+0]*vNormal2[0]+  mTransposed[2*4+1]*vNormal2[1]+  mTransposed[2*4+2]*vNormal2[2];
  

    dVector3 vAbsNormal;
    vAbsNormal[0] = dFabs( vNr[0] );
    vAbsNormal[1] = dFabs( vNr[1] );
    vAbsNormal[2] = dFabs( vNr[2] );

    // get closest face from box
    int iB0, iB1, iB2;
    if (vAbsNormal[1] > vAbsNormal[0]) {
      if (vAbsNormal[1] > vAbsNormal[2]) {
        iB1 = 0;  iB0 = 1;  iB2 = 2;
      } else {
        iB1 = 0;  iB2 = 1;  iB0 = 2;
      }
    } else {

      if (vAbsNormal[0] > vAbsNormal[2]) {
        iB0 = 0;  iB1 = 1;  iB2 = 2;
      } else {
        iB1 = 0;  iB2 = 1;  iB0 = 2;
      }
    }

    // Here find center of box face we are going to project
    dVector3 vCenter;
    dVector3 vRotCol;
    GETCOL(mHullBoxRot,iB0,vRotCol);
    
    if (vNr[iB0] > 0) {
        vCenter[0] = vHullBoxPos[0] - v0[0] - vBoxHalfSize[iB0] * vRotCol[0];
      vCenter[1] = vHullBoxPos[1] - v0[1] - vBoxHalfSize[iB0] * vRotCol[1];
      vCenter[2] = vHullBoxPos[2] - v0[2] - vBoxHalfSize[iB0] * vRotCol[2];
    } else {
      vCenter[0] = vHullBoxPos[0] - v0[0] + vBoxHalfSize[iB0] * vRotCol[0];
      vCenter[1] = vHullBoxPos[1] - v0[1] + vBoxHalfSize[iB0] * vRotCol[1];
      vCenter[2] = vHullBoxPos[2] - v0[2] + vBoxHalfSize[iB0] * vRotCol[2];
    }  

    // Here find 4 corner points of box
    dVector3 avPoints[4];

    dVector3 vRotCol2;
    GETCOL(mHullBoxRot,iB1,vRotCol);
    GETCOL(mHullBoxRot,iB2,vRotCol2);

    for(int x=0;x<3;x++) {
        avPoints[0][x] = vCenter[x] + (vBoxHalfSize[iB1] * vRotCol[x]) - (vBoxHalfSize[iB2] * vRotCol2[x]);
        avPoints[1][x] = vCenter[x] - (vBoxHalfSize[iB1] * vRotCol[x]) - (vBoxHalfSize[iB2] * vRotCol2[x]);
        avPoints[2][x] = vCenter[x] - (vBoxHalfSize[iB1] * vRotCol[x]) + (vBoxHalfSize[iB2] * vRotCol2[x]);
        avPoints[3][x] = vCenter[x] + (vBoxHalfSize[iB1] * vRotCol[x]) + (vBoxHalfSize[iB2] * vRotCol2[x]);
    }


    // clip Box face with 4 planes of triangle (1 face plane, 3 egde planes)
    dVector3 avTempArray1[9];
    dVector3 avTempArray2[9];
    dVector4 plPlane;

    int iTempCnt1=0;
    int iTempCnt2=0;

    // zeroify vectors - necessary?
    for(int i=0; i<9; i++) {
      avTempArray1[i][0]=0;
      avTempArray1[i][1]=0;
      avTempArray1[i][2]=0;

      avTempArray2[i][0]=0;
      avTempArray2[i][1]=0;
      avTempArray2[i][2]=0;
    }


    // Normal plane
    dVector3 vTemp;
    vTemp[0]=-vN[0];
    vTemp[1]=-vN[1];
    vTemp[2]=-vN[2];
    dNormalize3(vTemp);
    CONSTRUCTPLANE(plPlane,vTemp,0);

    _cldClipPolyToPlane( avPoints, 4, avTempArray1, iTempCnt1, plPlane  );
    

    // Plane p0
    dVector3 vTemp2;
    SUBTRACT(v1,v0,vTemp2);
    dCROSS(vTemp,=,vN,vTemp2);
    dNormalize3(vTemp);
    CONSTRUCTPLANE(plPlane,vTemp,0);

    _cldClipPolyToPlane( avTempArray1, iTempCnt1, avTempArray2, iTempCnt2, plPlane  );


    // Plane p1
    SUBTRACT(v2,v1,vTemp2);
    dCROSS(vTemp,=,vN,vTemp2);
    dNormalize3(vTemp);
    SUBTRACT(v0,v2,vTemp2);
    CONSTRUCTPLANE(plPlane,vTemp,dDOT(vTemp2,vTemp));

    _cldClipPolyToPlane( avTempArray2, iTempCnt2, avTempArray1, iTempCnt1, plPlane  );


    // Plane p2
    SUBTRACT(v0,v2,vTemp2);
    dCROSS(vTemp,=,vN,vTemp2);
    dNormalize3(vTemp);
    CONSTRUCTPLANE(plPlane,vTemp,0);

    _cldClipPolyToPlane( avTempArray1, iTempCnt1, avTempArray2, iTempCnt2, plPlane  );


    // END of clipping polygons



    // for each generated contact point
    for ( int i=0; i<iTempCnt2; i++ ) {
      // calculate depth
      dReal fTempDepth = dDOT(vNormal2,avTempArray2[i]);

      // clamp depth to zero
      if (fTempDepth > 0) {
        fTempDepth = 0;
      }

      dVector3 vPntTmp;
      ADD(avTempArray2[i],v0,vPntTmp);

#ifdef ORIG
    if (ctContacts < (iFlags & 0x0ffff)) {
          dContactGeom* Contact = SAFECONTACT(iFlags, ContactGeoms, ctContacts, iStride);

          Contact->depth = -fTempDepth;
          SET(Contact->normal,vBestNormal);
          SET(Contact->pos,vPntTmp);
          Contact->g1 = Geom1;
          Contact->g2 = Geom2;
          ctContacts++;
    }
#endif
    GenerateContact(iFlags, ContactGeoms, iStride,  Geom1, Geom2,
                    vPntTmp, vBestNormal, -fTempDepth, ctContacts);
    }

    //dAASSERT(ctContacts>0);

  // if box face is the referent face, then clip triangle on box face
  } else { // 2 <= if iBestAxis <= 4
    
    // get normal of box face
    dVector3 vNormal2;
    SET(vNormal2,vBestNormal);

    // get indices of box axes in correct order
    int iA0,iA1,iA2;
    iA0 = iBestAxis-2;
    if ( iA0 == 0 ) {
      iA1 = 1; iA2 = 2;
    } else if ( iA0 == 1 ) {
      iA1 = 0; iA2 = 2;
    } else {
      iA1 = 0; iA2 = 1;
    }

    dVector3 avPoints[3];
    // calculate triangle vertices in box frame
    SUBTRACT(v0,vHullBoxPos,avPoints[0]);
    SUBTRACT(v1,vHullBoxPos,avPoints[1]);
    SUBTRACT(v2,vHullBoxPos,avPoints[2]);

    // CLIP Polygons
    // define temp data for clipping
    dVector3 avTempArray1[9];
    dVector3 avTempArray2[9];
    
    int iTempCnt1, iTempCnt2;

    // zeroify vectors - necessary?
    for(int i=0; i<9; i++) {
      avTempArray1[i][0]=0;
      avTempArray1[i][1]=0;
      avTempArray1[i][2]=0;

      avTempArray2[i][0]=0;
      avTempArray2[i][1]=0;
      avTempArray2[i][2]=0;
    }

    // clip triangle with 5 box planes (1 face plane, 4 edge planes)

    dVector4 plPlane;

    // Normal plane
    dVector3 vTemp;
    vTemp[0]=-vNormal2[0];
    vTemp[1]=-vNormal2[1];
    vTemp[2]=-vNormal2[2];
    CONSTRUCTPLANE(plPlane,vTemp,vBoxHalfSize[iA0]);

    _cldClipPolyToPlane( avPoints, 3, avTempArray1, iTempCnt1, plPlane );
    

    // Plane p0
    GETCOL(mHullBoxRot,iA1,vTemp);
    CONSTRUCTPLANE(plPlane,vTemp,vBoxHalfSize[iA1]);

    _cldClipPolyToPlane( avTempArray1, iTempCnt1, avTempArray2, iTempCnt2, plPlane );


    // Plane p1
    GETCOL(mHullBoxRot,iA1,vTemp);
    vTemp[0]=-vTemp[0];
    vTemp[1]=-vTemp[1];
    vTemp[2]=-vTemp[2];
    CONSTRUCTPLANE(plPlane,vTemp,vBoxHalfSize[iA1]);

    _cldClipPolyToPlane( avTempArray2, iTempCnt2, avTempArray1, iTempCnt1, plPlane );


    // Plane p2
    GETCOL(mHullBoxRot,iA2,vTemp);
    CONSTRUCTPLANE(plPlane,vTemp,vBoxHalfSize[iA2]);

    _cldClipPolyToPlane( avTempArray1, iTempCnt1, avTempArray2, iTempCnt2, plPlane );


    // Plane p3
    GETCOL(mHullBoxRot,iA2,vTemp);
    vTemp[0]=-vTemp[0];
    vTemp[1]=-vTemp[1];
    vTemp[2]=-vTemp[2];
    CONSTRUCTPLANE(plPlane,vTemp,vBoxHalfSize[iA2]);

    _cldClipPolyToPlane( avTempArray2, iTempCnt2, avTempArray1, iTempCnt1, plPlane );


    // for each generated contact point
    for ( int i=0; i<iTempCnt1; i++ ) {
      // calculate depth
      dReal fTempDepth = dDOT(vNormal2,avTempArray1[i])-vBoxHalfSize[iA0];
      
      // clamp depth to zero
      if (fTempDepth > 0) {
        fTempDepth = 0;
      }
    
      // generate contact data
      dVector3 vPntTmp;
      ADD(avTempArray1[i],vHullBoxPos,vPntTmp);

#ifdef ORIG
      if (ctContacts < (iFlags & 0x0ffff)) {
          dContactGeom* Contact = SAFECONTACT(iFlags, ContactGeoms, ctContacts, iStride);

          Contact->depth = -fTempDepth;
          SET(Contact->normal,vBestNormal);
          SET(Contact->pos,vPntTmp);
          Contact->g1 = Geom1;
          Contact->g2 = Geom2;
          ctContacts++;
      }
#endif
      GenerateContact(iFlags, ContactGeoms, iStride,  Geom1, Geom2,
                      vPntTmp, vBestNormal, -fTempDepth, ctContacts);
    }

    //dAASSERT(ctContacts>0);
  }
  
}





// test one mesh triangle on intersection with given box
static void _cldTestOneTriangle(const dVector3 &v0, const dVector3 &v1, const dVector3 &v2)//, void *pvUser)
{
  // do intersection test and find best separating axis
  if(!_cldTestSeparatingAxes(v0, v1, v2) ) {
     // if not found do nothing
    return;
  }

  // if best separation axis is not found
  if ( iBestAxis == 0 ) {
    // this should not happen (we should already exit in that case)
    //dMessage (0, "best separation axis not found");
    // do nothing
    return;
  }

  _cldClipping(v0, v1, v2);
}





// box to mesh collider
int dCollideBTL(dxGeom* g1, dxGeom* BoxGeom, int Flags, dContactGeom* Contacts, int Stride){

  dxTriMesh* TriMesh = (dxTriMesh*)g1;


  // get source hull position, orientation and half size
  const dMatrix3& mRotBox=*(const dMatrix3*)dGeomGetRotation(BoxGeom);
  const dVector3& vPosBox=*(const dVector3*)dGeomGetPosition(BoxGeom);

  // to global
  SETM(mHullBoxRot,mRotBox);
  SET(vHullBoxPos,vPosBox);

  dGeomBoxGetLengths(BoxGeom, vBoxHalfSize);
  vBoxHalfSize[0] *= 0.5f;
  vBoxHalfSize[1] *= 0.5f;
  vBoxHalfSize[2] *= 0.5f;



  // get destination hull position and orientation
  const dMatrix3& mRotMesh=*(const dMatrix3*)dGeomGetRotation(TriMesh);
  const dVector3& vPosMesh=*(const dVector3*)dGeomGetPosition(TriMesh);

  // to global
  SET(vHullDstPos,vPosMesh);



  // global info for contact creation
  ctContacts = 0;
  iStride=Stride;
  iFlags=Flags;
  ContactGeoms=Contacts;
  Geom1=TriMesh;
  Geom2=BoxGeom;

 

  // reset stuff
  fBestDepth = MAXVALUE;
  vBestNormal[0]=0;
  vBestNormal[1]=0;
  vBestNormal[2]=0;

  OBBCollider& Collider = TriMesh->_OBBCollider;




  // Make OBB
  OBB Box;
  Box.mCenter.x = vPosBox[0];
  Box.mCenter.y = vPosBox[1];
  Box.mCenter.z = vPosBox[2];


  Box.mExtents.x = vBoxHalfSize[0];
  Box.mExtents.y = vBoxHalfSize[1];
  Box.mExtents.z = vBoxHalfSize[2];

  Box.mRot.m[0][0] = mRotBox[0];
  Box.mRot.m[1][0] = mRotBox[1];
  Box.mRot.m[2][0] = mRotBox[2];

  Box.mRot.m[0][1] = mRotBox[4];
  Box.mRot.m[1][1] = mRotBox[5];
  Box.mRot.m[2][1] = mRotBox[6];

  Box.mRot.m[0][2] = mRotBox[8];
  Box.mRot.m[1][2] = mRotBox[9];
  Box.mRot.m[2][2] = mRotBox[10];

  Matrix4x4 amatrix;
  Matrix4x4 BoxMatrix = MakeMatrix(vPosBox, mRotBox, amatrix);

  Matrix4x4 InvBoxMatrix;
  InvertPRMatrix(InvBoxMatrix, BoxMatrix);

  // TC results
  if (TriMesh->doBoxTC) {
	dxTriMesh::BoxTC* BoxTC = 0;
	for (int i = 0; i < TriMesh->BoxTCCache.size(); i++){
		if (TriMesh->BoxTCCache[i].Geom == BoxGeom){
			BoxTC = &TriMesh->BoxTCCache[i];
			break;
		}
	}
	if (!BoxTC){
		TriMesh->BoxTCCache.push(dxTriMesh::BoxTC());

		BoxTC = &TriMesh->BoxTCCache[TriMesh->BoxTCCache.size() - 1];
		BoxTC->Geom = BoxGeom;
    BoxTC->FatCoeff = 1.1f; // Pierre recommends this, instead of 1.0
	}

	// Intersect
	Collider.SetTemporalCoherence(true);
	Collider.Collide(*BoxTC, Box, TriMesh->Data->BVTree, null, &MakeMatrix(vPosMesh, mRotMesh, amatrix));
  }
  else {
		Collider.SetTemporalCoherence(false);
		Collider.Collide(dxTriMesh::defaultBoxCache, Box, TriMesh->Data->BVTree, null, 
						 &MakeMatrix(vPosMesh, mRotMesh, amatrix));	
	}

  if (!Collider.GetContactStatus()) {
     /* no collision occurred */
     return 0;
  }
	    
  // Retrieve data
  int TriCount = Collider.GetNbTouchedPrimitives();
  const int* Triangles = (const int*)Collider.GetTouchedPrimitives();

  if (TriCount != 0){
      if (TriMesh->ArrayCallback != null){
         TriMesh->ArrayCallback(TriMesh, BoxGeom, Triangles, TriCount);
    }
    
    int OutTriCount = 0;
    
    // loop through all intersecting triangles
    for (int i = 0; i < TriCount; i++){

        
        const int& Triint = Triangles[i];
        if (!Callback(TriMesh, BoxGeom, Triint)) continue;


        dVector3 dv[3];
        FetchTriangle(TriMesh, Triint, vPosMesh, mRotMesh, dv);


        // test this triangle
        _cldTestOneTriangle(dv[0],dv[1],dv[2]);
    }
  }


  return ctContacts;
}




// GenerateContact - Written by Jeff Smith (jeff@burri.to)
//   Generate a "unique" contact.  A unique contact has a unique
//   position or normal.  If the potential contact has the same
//   position and normal as an existing contact, but a larger
//   penetration depth, this new depth is used instead
//
static void
GenerateContact(int in_Flags, dContactGeom* in_Contacts, int in_Stride,  
                dxGeom* in_g1,  dxGeom* in_g2,
                const dVector3 in_ContactPos, const dVector3 in_Normal, dReal in_Depth,
                int& OutTriCount)
{
    //if (in_Depth < 0.0)
    //return;

    if (OutTriCount == (in_Flags & 0x0ffff))
        return; // contacts are full!

    dContactGeom* Contact;
    dVector3 diff;
    bool duplicate = false;
    for (int i=0; i<OutTriCount; i++) 
    {
        Contact = SAFECONTACT(in_Flags, in_Contacts, i, in_Stride);

        // same position?
        for (int j=0; j<3; j++)
            diff[j] = in_ContactPos[j] - Contact->pos[j];
        if (dDOT(diff, diff) < 0.01) 
        {
            // same normal?
            if (fabs(dDOT(in_Normal, Contact->normal)) > 0.99) 
            {
                if (in_Depth > Contact->depth)
                    Contact->depth = in_Depth;
                duplicate = true;
            }
        }
    }
    
    if (!duplicate) 
    {
        // Add a new contact
        Contact = SAFECONTACT(in_Flags, in_Contacts, OutTriCount, in_Stride);

        Contact->pos[0] = in_ContactPos[0];
        Contact->pos[1] = in_ContactPos[1];
        Contact->pos[2] = in_ContactPos[2];
        Contact->pos[3] = 0.0;
        
        Contact->normal[0] = in_Normal[0];
        Contact->normal[1] = in_Normal[1];
        Contact->normal[2] = in_Normal[2];
        Contact->normal[3] = 0.0;
        
        Contact->depth = in_Depth;
        
        Contact->g1 = in_g1;
        Contact->g2 = in_g2;
        
        OutTriCount++;
    }
}
