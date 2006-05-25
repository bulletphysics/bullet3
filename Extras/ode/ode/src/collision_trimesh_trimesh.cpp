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

// TriMesh/TriMesh collision code by Jeff Smith (c) 2004
//

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

#include <ode/collision.h>
#include <ode/matrix.h>
#include <ode/rotation.h>
#include <ode/odemath.h>
#include "collision_util.h"

#define TRIMESH_INTERNAL
#include "collision_trimesh_internal.h"

#define SMALL_ELT           2.5e-4
#define EXPANDED_ELT_THRESH 1.0e-3
#define DISTANCE_EPSILON    1.0e-8
#define VELOCITY_EPSILON    1.0e-5
#define TINY_PENETRATION    5.0e-6

struct LineContactSet
{
    dVector3 Points[8];
    int      Count;
};


static void GetTriangleGeometryCallback(udword, VertexPointers&, udword);
static void GenerateContact(int, dContactGeom*, int, dxTriMesh*,  dxTriMesh*, 
                            const dVector3, const dVector3, dReal, int&);
static int TriTriIntersectWithIsectLine(dReal V0[3],dReal V1[3],dReal V2[3],
                                        dReal U0[3],dReal U1[3],dReal U2[3],int *coplanar,
                                        dReal isectpt1[3],dReal isectpt2[3]);
inline void dMakeMatrix4(const dVector3 Position, const dMatrix3 Rotation, dMatrix4 &B);
static void dInvertMatrix4( dMatrix4& B, dMatrix4& Binv );
static int IntersectLineSegmentRay(dVector3, dVector3, dVector3, dVector3,  dVector3);
static bool FindTriSolidIntrsection(const dVector3 Tri[3], 
                                    const dVector4 Planes[6], int numSides,
                                    LineContactSet& ClippedPolygon );
static void ClipConvexPolygonAgainstPlane( const dVector3, dReal, LineContactSet& );
static bool SimpleUnclippedTest(dVector3 in_CoplanarPt, dVector3 in_v, dVector3 in_elt,
                                dVector3 in_n, dVector3* in_col_v, dReal &out_depth);
static int ExamineContactPoint(dVector3* v_col, dVector3 in_n, dVector3 in_point);
static int RayTriangleIntersect(const dVector3 orig, const dVector3 dir,
                                const dVector3 vert0, const dVector3 vert1,const dVector3 vert2,
                                dReal *t,dReal *u,dReal *v);




/* some math macros */
#define CROSS(dest,v1,v2) { dest[0]=v1[1]*v2[2]-v1[2]*v2[1]; \
                            dest[1]=v1[2]*v2[0]-v1[0]*v2[2]; \
                            dest[2]=v1[0]*v2[1]-v1[1]*v2[0]; }

#define DOT(v1,v2) (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])

#define SUB(dest,v1,v2) { dest[0]=v1[0]-v2[0]; dest[1]=v1[1]-v2[1]; dest[2]=v1[2]-v2[2]; }

#define ADD(dest,v1,v2) { dest[0]=v1[0]+v2[0]; dest[1]=v1[1]+v2[1]; dest[2]=v1[2]+v2[2]; }

#define MULT(dest,v,factor) { dest[0]=factor*v[0]; dest[1]=factor*v[1]; dest[2]=factor*v[2]; }

#define SET(dest,src) { dest[0]=src[0]; dest[1]=src[1]; dest[2]=src[2]; }

#define SMULT(p,q,s) { p[0]=q[0]*s; p[1]=q[1]*s; p[2]=q[2]*s; }

#define COMBO(combo,p,t,q) { combo[0]=p[0]+t*q[0]; combo[1]=p[1]+t*q[1]; combo[2]=p[2]+t*q[2]; }

#define LENGTH(x)  ((dReal) dSqrt(dDOT(x, x)))

#define DEPTH(d, p, q, n) d = (p[0] - q[0])*n[0] +  (p[1] - q[1])*n[1] +  (p[2] - q[2])*n[2];

inline const dReal dMin(const dReal x, const dReal y)
{
    return x < y ? x : y;
}


inline void
SwapNormals(dVector3 *&pen_v, dVector3 *&col_v, dVector3* v1, dVector3* v2,
            dVector3 *&pen_elt, dVector3 *elt_f1, dVector3 *elt_f2,
            dVector3 n, dVector3 n1, dVector3 n2)
{
    if (pen_v == v1) {
        pen_v = v2;
        pen_elt = elt_f2;
        col_v = v1;
        SET(n, n1);
    }
    else {
        pen_v = v1;
        pen_elt = elt_f1;
        col_v = v2;
        SET(n, n2);
    }
}




int 
dCollideTTL(dxGeom* g1, dxGeom* g2, int Flags, dContactGeom* Contacts, int Stride)
{
    dxTriMesh* TriMesh1 = (dxTriMesh*) g1;
    dxTriMesh* TriMesh2 = (dxTriMesh*) g2;

    dReal * TriNormals1 = (dReal *) TriMesh1->Data->Normals;
    dReal * TriNormals2 = (dReal *) TriMesh2->Data->Normals;

    const dVector3& TLPosition1 = *(const dVector3*) dGeomGetPosition(TriMesh1);
    // TLRotation1 = column-major order
    const dMatrix3& TLRotation1 = *(const dMatrix3*) dGeomGetRotation(TriMesh1);

    const dVector3& TLPosition2 = *(const dVector3*) dGeomGetPosition(TriMesh2);
    // TLRotation2 = column-major order
    const dMatrix3& TLRotation2 = *(const dMatrix3*) dGeomGetRotation(TriMesh2);

    AABBTreeCollider& Collider = TriMesh1->_AABBTreeCollider;

    static BVTCache ColCache;
    ColCache.Model0 = &TriMesh1->Data->BVTree;
    ColCache.Model1 = &TriMesh2->Data->BVTree;

    // Collision query
    Matrix4x4 amatrix, bmatrix;
    BOOL IsOk = Collider.Collide(ColCache,
                                 &MakeMatrix(TLPosition1, TLRotation1, amatrix),
                                 &MakeMatrix(TLPosition2, TLRotation2, bmatrix) );
    

    // Make "double" versions of these matrices, if appropriate
    dMatrix4 A, B;
    dMakeMatrix4(TLPosition1, TLRotation1, A);
    dMakeMatrix4(TLPosition2, TLRotation2, B);


    if (IsOk) {
        // Get collision status => if true, objects overlap
        if ( Collider.GetContactStatus() ) {
            // Number of colliding pairs and list of pairs
            int TriCount = Collider.GetNbPairs();
            const Pair* CollidingPairs = Collider.GetPairs();

            if (TriCount > 0) {
                // step through the pairs, adding contacts
                int             id1, id2;
                int             OutTriCount = 0;
                dVector3        v1[3], v2[3], CoplanarPt;
                dVector3        e1, e2, e3, n1, n2, n, ContactNormal;
                dReal           depth;
                dVector3        orig_pos, old_pos1, old_pos2, elt1, elt2, elt_sum;
                dVector3        elt_f1[3], elt_f2[3];
                dReal          contact_elt_length = SMALL_ELT;
                LineContactSet  firstClippedTri, secondClippedTri;
                dVector3       *firstClippedElt = NULL;
                dVector3       *secondClippedElt = NULL;
                

                // only do these expensive inversions once
                dMatrix4 InvMatrix1, InvMatrix2;
                dInvertMatrix4(A, InvMatrix1);
                dInvertMatrix4(B, InvMatrix2);

                
                for (int i = 0; i < TriCount; i++)
                    if (OutTriCount < (Flags & 0xffff))  {
                        
                        id1 = CollidingPairs[i].id0;
                        id2 = CollidingPairs[i].id1;
                        
                        // grab the colliding triangles
                        FetchTriangle((dxTriMesh*) g1, id1, TLPosition1, TLRotation1, v1);
                        FetchTriangle((dxTriMesh*) g2, id2, TLPosition2, TLRotation2, v2);
                        // Since we'll be doing matrix transfomrations, we need to
                        //  make sure that all vertices have four elements
                        for (int j=0; j<3; j++) {
                            v1[j][3] = 1.0;
                            v2[j][3] = 1.0;
                        }
                            
                        
                        int IsCoplanar = 0;
                        dReal IsectPt1[3], IsectPt2[3];

                        // Sometimes OPCODE makes mistakes, so we look at the return
                        //  value for TriTriIntersectWithIsectLine.  A retcode of "0"
                        //  means no intersection took place
                        if ( TriTriIntersectWithIsectLine( v1[0], v1[1], v1[2], v2[0], v2[1], v2[2],
                                                           &IsCoplanar,
                                                           IsectPt1, IsectPt2) ) {
                            
                            // Compute the normals of the colliding faces
                            //
                            if (TriNormals1 == NULL) {
                                SUB( e1, v1[1], v1[0] );
                                SUB( e2, v1[2], v1[0] );
                                CROSS( n1, e1, e2 );
                                dNormalize3(n1);
                            }
                            else {
                                // If we were passed normals, we need to adjust them to take into
                                //  account the objects' current rotations
                                e1[0] = TriNormals1[id1*3];
                                e1[1] = TriNormals1[id1*3 + 1];
                                e1[2] = TriNormals1[id1*3 + 2];
                                e1[3] = 0.0;
                                
                                //dMultiply1(n1, TLRotation1, e1, 3, 3, 1);
                                dMultiply0(n1, TLRotation1, e1, 3, 3, 1);
                                n1[3] = 1.0;
                            }
                            
                            if (TriNormals2 == NULL)  {
                                SUB( e1, v2[1], v2[0] );
                                SUB( e2, v2[2], v2[0] );
                                CROSS( n2, e1, e2);
                                dNormalize3(n2);
                            }
                            else {
                                // If we were passed normals, we need to adjust them to take into
                                //  account the objects' current rotations
                                e2[0] = TriNormals2[id2*3];
                                e2[1] = TriNormals2[id2*3 + 1];
                                e2[2] = TriNormals2[id2*3 + 2];
                                e2[3] = 0.0;
                                
                                //dMultiply1(n2, TLRotation2, e2, 3, 3, 1);
                                dMultiply0(n2, TLRotation2, e2, 3, 3, 1);
                                n2[3] = 1.0;
                            }
                            

                            if (IsCoplanar) {
                                // We can reach this case if the faces are coplanar, OR
                                //  if they don't actually intersect.  (OPCODE can make
                                //  mistakes)
                                if (fabs(dDOT(n1, n2)) > 0.999) {
                                    // If the faces are coplanar, we declare that the point of
                                    //  contact is at the average location of the vertices of
                                    //  both faces
                                    dVector3 ContactPt;
                                    for (int j=0; j<3; j++) {
                                        ContactPt[j] = 0.0;
                                        for (int k=0; k<3; k++)
                                            ContactPt[j] += v1[k][j] + v2[k][j];
                                        ContactPt[j] /= 6.0;
                                    }
                                    ContactPt[3] = 1.0;
                                    
                                    // and the contact normal is the normal of face 2
                                    //  (could be face 1, because they are the same)
                                    SET(n, n2);
                                    
                                    // and the penetration depth is the co-normal
                                    // distance between any two vertices A and B,
                                    // i.e.  d = DOT(n, (A-B))
                                    DEPTH(depth, v1[1], v2[1], n);
                                    if (depth < 0)
                                        depth *= -1.0;
                                    
                                    GenerateContact(Flags, Contacts, Stride,  TriMesh1,  TriMesh2,
                                                    ContactPt, n, depth, OutTriCount);
                                }
                            }
                            else {
                                // Otherwise (in non-co-planar cases), we create a coplanar 
                                //  point -- the middle of the line of intersection -- that
                                //   will be used for various computations down the road
                                for (int j=0; j<3; j++)
                                    CoplanarPt[j] = (dReal) ( (IsectPt1[j] + IsectPt2[j]) / 2.0 );
                                CoplanarPt[3] = 1.0;
                                
                                // Find the ELT of the coplanar point
                                //
                                dMultiply1(orig_pos, InvMatrix1, CoplanarPt, 4, 4, 1);
                                dMultiply1(old_pos1, TriMesh1->Data->last_trans, orig_pos, 4, 4, 1);
                                SUB(elt1, CoplanarPt, old_pos1);
                                
                                dMultiply1(orig_pos, InvMatrix2, CoplanarPt, 4, 4, 1);
                                dMultiply1(old_pos2, TriMesh2->Data->last_trans, orig_pos, 4, 4, 1);
                                SUB(elt2, CoplanarPt, old_pos2);
                                
                                SUB(elt_sum, elt1, elt2);  // net motion of the coplanar point
                                
                                
                                // Calculate how much the vertices of each face moved in the
                                //  direction of the opposite face's normal
                                //
                                dReal    total_dp1, total_dp2;
                                total_dp1 = 0.0;
                                total_dp2 = 0.0;
                                
                                for (int ii=0; ii<3; ii++) {
                                    // find the estimated linear translation (ELT) of the vertices
                                    //  on face 1, wrt to the center of face 2. 
                                    
                                    // un-transform this vertex by the current transform
                                    dMultiply1(orig_pos, InvMatrix1, v1[ii], 4, 4, 1 );
                                    
                                    // re-transform this vertex by last_trans (to get its old
                                    //  position)
                                    dMultiply1(old_pos1, TriMesh1->Data->last_trans, orig_pos, 4, 4, 1);
                                    
                                    // Then subtract this position from our current one to find
                                    //  the elapsed linear translation (ELT)
                                    for (int k=0; k<3; k++) {
                                        elt_f1[ii][k] = (v1[ii][k] - old_pos1[k]) - elt2[k];
                                    }
                                    
                                    // Take the dot product of the ELT  for each vertex (wrt the
                                    //  center of face2)
                                    total_dp1 += fabs( dDOT(elt_f1[ii], n2) );
                                }
                                
                                for (int ii=0; ii<3; ii++) {
                                    // find the estimated linear translation (ELT) of the vertices
                                    //  on face 2, wrt to the center of face 1. 
                                    dMultiply1(orig_pos, InvMatrix2, v2[ii], 4, 4, 1);
                                    dMultiply1(old_pos2, TriMesh2->Data->last_trans, orig_pos, 4, 4, 1);
                                    for (int k=0; k<3; k++) {
                                        elt_f2[ii][k] = (v2[ii][k] - old_pos2[k]) - elt1[k];
                                    }
                                    
                                    // Take the dot product of the ELT  for each vertex (wrt the
                                    //  center of face2) and add them
                                    total_dp2 += fabs( dDOT(elt_f2[ii], n1) );
                                }
                                
                                
                                ////////
                                // Estimate the penetration depth.  
                                //                            
                                dReal    dp;
                                BOOL      badPen = true;
                                dVector3 *pen_v;   // the "penetrating vertices"
                                dVector3 *pen_elt; // the elt_f of the penetrating face
                                dVector3 *col_v;   // the "collision vertices" (the penetrated face)
                                
                                
                                depth = 0.0;
                                if ((total_dp1 > DISTANCE_EPSILON) || (total_dp2 > DISTANCE_EPSILON)) {
                                    ////////
                                    // Find the collision normal, by finding the face
                                    //  that is pointed "most" in the direction of travel
                                    //  of the two triangles
                                    //
                                    if (total_dp2 > total_dp1) {
                                        pen_v = v2;
                                        pen_elt = elt_f2;
                                        col_v = v1;
                                        SET(n, n1);
                                    }
                                    else {
                                        pen_v = v1;
                                        pen_elt = elt_f1;
                                        col_v = v2;
                                        SET(n, n2);
                                    }
                                }
                                else {
                                    // the total_dp is very small, so let's fall back
                                    //  to a different test
                                    if (LENGTH(elt2) > LENGTH(elt1)) {
                                        pen_v = v2;
                                        pen_elt = elt_f2;
                                        col_v = v1;
                                        SET(n, n1);
                                    }
                                    else {
                                        pen_v = v1;
                                        pen_elt = elt_f1;
                                        col_v = v2;
                                        SET(n, n2);
                                    }
                                }
                                

                                for (int j=0; j<3; j++)
                                    if (SimpleUnclippedTest(CoplanarPt, pen_v[j], pen_elt[j], n, col_v, depth)) {
                                        GenerateContact(Flags, Contacts, Stride,  TriMesh1,  TriMesh2,
                                                        pen_v[j], n, depth, OutTriCount);
                                        badPen = false;
                                    }
                                

                                if (badPen) {
                                    // try the other normal
                                    SwapNormals(pen_v, col_v, v1, v2, pen_elt, elt_f1, elt_f2, n, n1, n2);

                                    for (int j=0; j<3; j++)
                                        if (SimpleUnclippedTest(CoplanarPt, pen_v[j], pen_elt[j], n, col_v, depth)) {
                                            GenerateContact(Flags, Contacts, Stride,  TriMesh1,  TriMesh2,
                                                            pen_v[j], n, depth, OutTriCount);
                                            badPen = false;
                                    }
                                }



                                ////////////////////////////////////////
                                //
                                // If we haven't found a good penetration, then we're probably straddling
                                //  the edge of one of the objects, or the penetraing face is big
                                //  enough that all of its vertices are outside the bounds of the
                                //  penetrated face.
                                // In these cases, we do a more expensive test. We clip the penetrating
                                //  triangle with a solid defined by the penetrated triangle, and repeat
                                //  the tests above on this new polygon
                                if (badPen) {
      
                                    // Switch pen_v and n back again
                                    SwapNormals(pen_v, col_v, v1, v2, pen_elt, elt_f1, elt_f2, n, n1, n2);
                                    
                                    
                                    // Find the three sides (no top or bottom) of the solid defined by 
                                    //  the edges of the penetrated triangle.
                                    
                                    // The dVector4 "plane" structures contain the following information:
                                    //  [0]-[2]: The normal of the face, pointing INWARDS (i.e.
                                    //           the inverse normal
                                    //  [3]: The distance between the face and the center of the
                                    //       solid, along the normal
                                    dVector4 SolidPlanes[3];
                                    dVector3 tmp1;
                                    dVector3 sn;

                                    for (int j=0; j<3; j++) {
                                        e1[j] = col_v[1][j] - col_v[0][j];
                                        e2[j] = col_v[0][j] - col_v[2][j];
                                        e3[j] = col_v[2][j] - col_v[1][j];
                                    }
                                    
                                    // side 1
                                    CROSS(sn, e1, n);
                                    dNormalize3(sn);
                                    SMULT( SolidPlanes[0], sn, -1.0 );
                                    
                                    ADD(tmp1, col_v[0], col_v[1]); 
                                    SMULT(tmp1, tmp1, 0.5); // center of edge
                                    // distance from center to edge along normal
                                    SolidPlanes[0][3] = dDOT(tmp1, SolidPlanes[0]);
                                    
                                    
                                    // side 2
                                    CROSS(sn, e2, n);
                                    dNormalize3(sn);
                                    SMULT( SolidPlanes[1], sn, -1.0 );
                                    
                                    ADD(tmp1, col_v[0], col_v[2]); 
                                    SMULT(tmp1, tmp1, 0.5); // center of edge
                                    // distance from center to edge along normal
                                    SolidPlanes[1][3] = dDOT(tmp1, SolidPlanes[1]);
                                    
                                    
                                    // side 3
                                    CROSS(sn, e3, n);
                                    dNormalize3(sn);
                                    SMULT( SolidPlanes[2], sn, -1.0 );
                                    
                                    ADD(tmp1, col_v[2], col_v[1]); 
                                    SMULT(tmp1, tmp1, 0.5); // center of edge
                                    // distance from center to edge along normal
                                    SolidPlanes[2][3] = dDOT(tmp1, SolidPlanes[2]);
                                    

                                    FindTriSolidIntrsection(pen_v, SolidPlanes, 3, firstClippedTri);

                                    firstClippedElt = new dVector3[firstClippedTri.Count];

                                    for (int j=0; j<firstClippedTri.Count; j++) {
                                        firstClippedTri.Points[j][3] = 1.0; // because we will be doing matrix mults

                                        DEPTH(dp, CoplanarPt, firstClippedTri.Points[j], n);
                                        
                                        // if thepenetration depth (calculated above) is more than the contact
                                        //  point's ELT, then we've chosen the wrong face and should switch faces
                                        if (pen_v == v1) {
                                            dMultiply1(orig_pos, InvMatrix1, firstClippedTri.Points[j], 4, 4, 1);
                                            dMultiply1(old_pos1, TriMesh1->Data->last_trans, orig_pos, 4, 4, 1);
                                            for (int k=0; k<3; k++) {
                                                firstClippedElt[j][k] = (firstClippedTri.Points[j][k] - old_pos1[k]) - elt2[k];
                                            }
                                        }
                                        else {
                                            dMultiply1(orig_pos, InvMatrix2, firstClippedTri.Points[j], 4, 4, 1);
                                            dMultiply1(old_pos2, TriMesh2->Data->last_trans, orig_pos, 4, 4, 1);
                                            for (int k=0; k<3; k++) {
                                                firstClippedElt[j][k] = (firstClippedTri.Points[j][k] - old_pos2[k]) - elt1[k];
                                            }
                                        }

                                        contact_elt_length = fabs(dDOT(firstClippedElt[j], n));
                                        
                                        if (dp >= 0.0) {
                                            depth = dp;
                                            if (depth == 0.0)
                                                depth = dMin(DISTANCE_EPSILON, contact_elt_length);
                                            
                                            if ((contact_elt_length < SMALL_ELT) && (depth < EXPANDED_ELT_THRESH))
                                                depth = contact_elt_length;
                                            
                                            if (depth <= contact_elt_length) {
                                                // Add a contact
                                                GenerateContact(Flags, Contacts, Stride,  TriMesh1,  TriMesh2,
                                                                firstClippedTri.Points[j], n, depth, OutTriCount);
                                                badPen = false;
                                            }
                                        }

                                    }
                                }
                                
                                if (badPen) {
                                    // Switch pen_v and n (again!)
                                    SwapNormals(pen_v, col_v, v1, v2, pen_elt, elt_f1, elt_f2, n, n1, n2);
                                    
                                    
                                    // Find the three sides (no top or bottom) of the solid created by 
                                    //  the penetrated triangle.
                                    // The dVector4 "plane" structures contain the following information:
                                    //  [0]-[2]: The normal of the face, pointing INWARDS (i.e.
                                    //           the inverse normal
                                    //  [3]: The distance between the face and the center of the
                                    //       solid, along the normal
                                    dVector4 SolidPlanes[3];
                                    dVector3 tmp1;
                                
                                    dVector3 sn;
                                    for (int j=0; j<3; j++) {
                                        e1[j] = col_v[1][j] - col_v[0][j];
                                        e2[j] = col_v[0][j] - col_v[2][j];
                                        e3[j] = col_v[2][j] - col_v[1][j];
                                    }
                                    
                                    // side 1
                                    CROSS(sn, e1, n);
                                    dNormalize3(sn);
                                    SMULT( SolidPlanes[0], sn, -1.0 );
                                    
                                    ADD(tmp1, col_v[0], col_v[1]); 
                                    SMULT(tmp1, tmp1, 0.5); // center of edge
                                    // distance from center to edge along normal
                                    SolidPlanes[0][3] = dDOT(tmp1, SolidPlanes[0]);
                                    
                                    
                                    // side 2
                                    CROSS(sn, e2, n);
                                    dNormalize3(sn);
                                    SMULT( SolidPlanes[1], sn, -1.0 );
                                    
                                    ADD(tmp1, col_v[0], col_v[2]); 
                                    SMULT(tmp1, tmp1, 0.5); // center of edge
                                    // distance from center to edge along normal
                                    SolidPlanes[1][3] = dDOT(tmp1, SolidPlanes[1]);
                                    
                                    
                                    // side 3
                                    CROSS(sn, e3, n);
                                    dNormalize3(sn);
                                    SMULT( SolidPlanes[2], sn, -1.0 );
                                    
                                    ADD(tmp1, col_v[2], col_v[1]); 
                                    SMULT(tmp1, tmp1, 0.5); // center of edge
                                    // distance from center to edge along normal
                                    SolidPlanes[2][3] = dDOT(tmp1, SolidPlanes[2]);
                                    
                                    FindTriSolidIntrsection(pen_v, SolidPlanes, 3, secondClippedTri);
                                    
                                    secondClippedElt = new dVector3[secondClippedTri.Count];

                                    for (int j=0; j<secondClippedTri.Count; j++) {
                                        secondClippedTri.Points[j][3] = 1.0; // because we will be doing matrix mults
                                        
                                        DEPTH(dp, CoplanarPt, secondClippedTri.Points[j], n);
                                        
                                        if (pen_v == v1) {
                                            dMultiply1(orig_pos, InvMatrix1, secondClippedTri.Points[j], 4, 4, 1);
                                            dMultiply1(old_pos1, TriMesh1->Data->last_trans, orig_pos, 4, 4, 1);
                                            for (int k=0; k<3; k++) {
                                                secondClippedElt[j][k] = (secondClippedTri.Points[j][k] - old_pos1[k]) - elt2[k];
                                            }
                                        }
                                        else {
                                            dMultiply1(orig_pos, InvMatrix2, secondClippedTri.Points[j], 4, 4, 1);
                                            dMultiply1(old_pos2, TriMesh2->Data->last_trans, orig_pos, 4, 4, 1);
                                            for (int k=0; k<3; k++) {
                                                secondClippedElt[j][k] = (secondClippedTri.Points[j][k] - old_pos2[k]) - elt1[k];
                                            }
                                        }


                                        contact_elt_length = fabs(dDOT(secondClippedElt[j],n));
                                        
                                        if (dp >= 0.0) {
                                            depth = dp;
                                            if (depth == 0.0)
                                                depth = dMin(DISTANCE_EPSILON, contact_elt_length);
                                            
                                            if ((contact_elt_length < SMALL_ELT) && (depth < EXPANDED_ELT_THRESH))
                                                depth = contact_elt_length;
                                            
                                            if (depth <= contact_elt_length) {
                                                // Add a contact
                                                GenerateContact(Flags, Contacts, Stride,  TriMesh1,  TriMesh2,
                                                                secondClippedTri.Points[j], n, depth, OutTriCount);
                                                badPen = false;
                                            }
                                        }
                                        
                                        
                                    }
                                }


                                
                                /////////////////
                                // All conventional tests have failed at this point, so now we deal with
                                //  cases on a more "heuristic" basis
                                //

                                if (badPen) {
                                    // Switch pen_v and n (for the fourth time, so they're
                                    //  what my original guess said they were)
                                    SwapNormals(pen_v, col_v, v1, v2, pen_elt, elt_f1, elt_f2, n, n1, n2);
                                    
									if (fabs(dDOT(n1, n2)) < 0.01) {
                                        // If we reach this point, we have (close to) perpindicular
                                        //  faces, either resting on each other or sliding in a
                                        // direction orthogonal to both surface normals.
                                        if (LENGTH(elt_sum) < DISTANCE_EPSILON) {
                                            depth = (dReal) fabs(dDOT(n, elt_sum));
                                            
                                            if (depth > 1e-12) {
                                                dNormalize3(n);
                                                GenerateContact(Flags, Contacts, Stride,  TriMesh1,  TriMesh2,
                                                                CoplanarPt, n, depth, OutTriCount);
                                                badPen = false;
                                            }
                                            else {
                                                // If the two faces are (nearly) perfectly at rest with
                                                //  respect to each other, then we ignore the contact,
                                                //  allowing the objects to slip a little in the hopes
                                                //  that next frame, they'll give us something to work
                                                //  with.
                                                badPen = false;
                                            }
                                        }
                                        else {
                                            // The faces are perpindicular, but moving significantly
                                            //  This can be sliding, or an unusual edge-straddling 
                                            //  penetration.
                                            dVector3 cn;
                                            
                                            CROSS(cn, n1, n2);
                                            dNormalize3(cn);
                                            SET(n, cn);
                                            
                                            // The shallowest ineterpenetration of the faces
                                            //  is the depth
                                            dVector3 ContactPt;
                                            dVector3 dvTmp;
                                            dReal    rTmp;
                                            depth = dInfinity;
                                            for (int j=0; j<3; j++) {
                                                for (int k=0; k<3; k++) {
                                                    SUB(dvTmp, col_v[k], pen_v[j]);
                                                    
                                                    rTmp = dDOT(dvTmp, n);
                                                    if ( fabs(rTmp) < fabs(depth) ) {
                                                        depth = rTmp;
                                                        SET( ContactPt, pen_v[j] );
                                                        contact_elt_length = fabs(dDOT(pen_elt[j], n));
                                                    }
                                                }
                                            }
                                            if (depth < 0.0) {
                                                SMULT(n, n, -1.0);
                                                depth *= -1.0;
                                            }
                                            
                                            if ((depth > 0.0) && (depth <= contact_elt_length)) {
                                                GenerateContact(Flags, Contacts, Stride,  TriMesh1,  TriMesh2,
                                                                ContactPt, n, depth, OutTriCount);
                                                badPen = false;
                                            }
                                            
                                        }
                                    }
                                }
                                

                                if (badPen) {
                                    // Use as the normal the direction of travel, rather than any particular
                                    //  face normal
                                    //
                                    dVector3 esn;
                                    
                                    if (pen_v == v1) {
                                        SMULT(esn, elt_sum, -1.0);
                                    }
                                    else {
                                        SET(esn, elt_sum);
                                    }
                                    dNormalize3(esn);


                                    // The shallowest ineterpenetration of the faces
                                    //  is the depth
                                    dVector3 ContactPt;
                                    depth = dInfinity;
                                    for (int j=0; j<3; j++) {
                                        for (int k=0; k<3; k++) {
                                            DEPTH(dp, col_v[k], pen_v[j], esn);
                                            if ( (ExamineContactPoint(col_v, esn, pen_v[j])) &&
                                                 ( fabs(dp) < fabs(depth)) ) {
                                                depth = dp;
                                                SET( ContactPt, pen_v[j] );
                                                contact_elt_length = fabs(dDOT(pen_elt[j], esn));
                                            }
                                        }
                                    }
                                    
                                    if ((depth > 0.0) && (depth <= contact_elt_length)) {
                                        GenerateContact(Flags, Contacts, Stride,  TriMesh1,  TriMesh2,
                                                        ContactPt, esn, depth, OutTriCount);
                                        badPen = false;
                                    }
                                }

                                
                                if (badPen) {                                        
                                    // If the direction of motion is perpindicular to both normals
                                    if ( (fabs(dDOT(n1, elt_sum)) < 0.01) && (fabs(dDOT(n2, elt_sum)) < 0.01) ) {
                                        dVector3 esn;
                                        if (pen_v == v1) {
                                            SMULT(esn, elt_sum, -1.0);
                                        }
                                        else {
                                            SET(esn, elt_sum);
                                        }
                                        
                                        dNormalize3(esn);

                                        
                                        // Look at the clipped points again, checking them against this
                                        //  new normal
                                        for (int j=0; j<firstClippedTri.Count; j++) {
                                            DEPTH(dp, CoplanarPt, firstClippedTri.Points[j], esn);
                                            
                                            if (dp >= 0.0) {
                                                contact_elt_length = fabs(dDOT(firstClippedElt[j], esn));
                                                
                                                depth = dp;
                                                //if (depth == 0.0)
                                                //depth = dMin(DISTANCE_EPSILON, contact_elt_length);
                                                
                                                if ((contact_elt_length < SMALL_ELT) && (depth < EXPANDED_ELT_THRESH))
                                                    depth = contact_elt_length;
                                                
                                                if (depth <= contact_elt_length) {
                                                    // Add a contact
                                                    GenerateContact(Flags, Contacts, Stride,  TriMesh1,  TriMesh2,
                                                                    firstClippedTri.Points[j], esn, depth, OutTriCount);
                                                    badPen = false;
                                                }    
                                            }
                                        }
                                        
                                        if (badPen) {
                                            // If this test failed, try it with the second set of clipped faces
                                            for (int j=0; j<secondClippedTri.Count; j++) {
                                                DEPTH(dp, CoplanarPt, secondClippedTri.Points[j], esn);
                                                
                                                if (dp >= 0.0) {
                                                    contact_elt_length = fabs(dDOT(secondClippedElt[j], esn));
                                                    
                                                    depth = dp;
                                                    //if (depth == 0.0)
                                                    //depth = dMin(DISTANCE_EPSILON, contact_elt_length);
                                                    
                                                    if ((contact_elt_length < SMALL_ELT) && (depth < EXPANDED_ELT_THRESH))
                                                        depth = contact_elt_length;
                                                    
                                                    if (depth <= contact_elt_length) {
                                                        // Add a contact
                                                        GenerateContact(Flags, Contacts, Stride,  TriMesh1,  TriMesh2,
                                                                        secondClippedTri.Points[j], esn, depth, OutTriCount);
                                                        badPen = false;
                                                    }    
                                                }
                                            }
                                        }
                                    }
                                }


                                
                                if (badPen) {
                                    // if we have very little motion, we're dealing with resting contact
                                    //  and shouldn't reference the ELTs at all
                                    //
                                    if (LENGTH(elt_sum) < VELOCITY_EPSILON) {
                                        
                                        // instead of a "contact_elt_length" threshhold, we'll use an
                                        //  arbitrary, small one
                                        for (int j=0; j<3; j++) {
                                            DEPTH(dp, CoplanarPt, pen_v[j], n);
                                            
                                            if (dp == 0.0)
                                                dp = TINY_PENETRATION;
                                            
                                            if ( (dp > 0.0) && (dp <= SMALL_ELT)) {
                                                // Add a contact
                                                GenerateContact(Flags, Contacts, Stride,  TriMesh1,  TriMesh2,
                                                                pen_v[j], n, (dReal) dp, OutTriCount);
                                                badPen = false;
                                            }
                                        }
                                        

                                        if (badPen) {
                                            // try the other normal
                                            SwapNormals(pen_v, col_v, v1, v2, pen_elt, elt_f1, elt_f2, n, n1, n2);

                                            for (int j=0; j<3; j++) {
                                                DEPTH(dp, CoplanarPt, pen_v[j], n);
                                                
                                                if (dp == 0.0)
                                                    dp = TINY_PENETRATION;

                                                if ( (dp > 0.0) && (dp <= SMALL_ELT)) {
                                                    GenerateContact(Flags, Contacts, Stride,  TriMesh1,  TriMesh2,
                                                                    pen_v[j], n, (dReal) dp, OutTriCount);
                                                    badPen = false;
                                                }
                                            }
                                        }
                                        
                                        

                                    }
                                }
                                    
                                if (badPen) {
                                    // find the nearest existing contact, and replicate it's
                                    //  normal and depth
                                    //
                                    dContactGeom*  Contact;
                                    dVector3       pos_diff;
                                    dReal          min_dist, dist;

                                    min_dist = dInfinity;
                                    depth = 0.0;
                                    for (int j=0; j<OutTriCount; j++) {
                                        Contact = SAFECONTACT(Flags, Contacts, j, Stride);
                                        
                                        SUB(pos_diff,  Contact->pos, CoplanarPt);

                                        dist = dDOT(pos_diff, pos_diff);
                                        if (dist < min_dist) {
                                            min_dist = dist;
                                            depth = Contact->depth;
                                            SMULT(ContactNormal, Contact->normal, -1.0);
                                        }
                                    }
                                 
                                    if (depth > 0.0) {
                                        // Add a tiny contact at the coplanar point
                                        GenerateContact(Flags, Contacts, Stride,  TriMesh1,  TriMesh2,
                                                        CoplanarPt, ContactNormal, depth, OutTriCount);
                                        badPen = false;
                                    }
                                }


                                if (badPen) {                                        
                                    // Add a tiny contact at the coplanar point                                    
                                    if (-dDOT(elt_sum, n1) > -dDOT(elt_sum, n2)) {
                                        SET(ContactNormal, n1);
                                    }
                                    else {
                                        SET(ContactNormal, n2);
                                    }

                                    GenerateContact(Flags, Contacts, Stride,  TriMesh1,  TriMesh2,
                                                    CoplanarPt, ContactNormal, TINY_PENETRATION, OutTriCount);
                                    badPen = false;
                                }

                                
                            } // not coplanar (main loop)
                        } // TriTriIntersectWithIsectLine
                
                    // Free memory
                    delete[] firstClippedElt;
	            firstClippedElt = NULL;
                    delete[] secondClippedElt;	
        	    secondClippedElt = NULL;

                } // if (OutTriCount < (Flags & 0xffff))

                // Return the number of contacts
                return OutTriCount; 
                
            }
        }
    }

    
    // There was some kind of failure during the Collide call or
    // there are no faces overlapping
    return 0;    
}



static void
GetTriangleGeometryCallback(udword triangleindex, VertexPointers& triangle, udword user_data)
{
    dVector3 Out[3];

    FetchTriangle((dxTriMesh*) user_data, (int) triangleindex, Out);

    for (int i = 0; i < 3; i++)
        triangle.Vertex[i] =  (const Point*) ((dReal*) Out[i]);
}


//
//
//
#define B11   B[0]
#define B12   B[1]
#define B13   B[2]
#define B14   B[3]
#define B21   B[4]
#define B22   B[5]
#define B23   B[6]
#define B24   B[7]
#define B31   B[8]
#define B32   B[9]
#define B33   B[10]
#define B34   B[11]
#define B41   B[12]
#define B42   B[13]
#define B43   B[14]
#define B44   B[15]

#define Binv11   Binv[0]
#define Binv12   Binv[1]
#define Binv13   Binv[2]
#define Binv14   Binv[3]
#define Binv21   Binv[4]
#define Binv22   Binv[5]
#define Binv23   Binv[6]
#define Binv24   Binv[7]
#define Binv31   Binv[8]
#define Binv32   Binv[9]
#define Binv33   Binv[10]
#define Binv34   Binv[11]
#define Binv41   Binv[12]
#define Binv42   Binv[13]
#define Binv43   Binv[14]
#define Binv44   Binv[15]

inline void
dMakeMatrix4(const dVector3 Position, const dMatrix3 Rotation, dMatrix4 &B)
{
	B11 = Rotation[0]; B21 = Rotation[1]; B31 = Rotation[2];    B41 = Position[0]; 
	B12 = Rotation[4]; B22 = Rotation[5]; B32 = Rotation[6];    B42 = Position[1];
	B13 = Rotation[8]; B23 = Rotation[9]; B33 = Rotation[10];   B43 = Position[2];

    B14 = 0.0;         B24 = 0.0;         B34 = 0.0;            B44 = 1.0;
}


static void
dInvertMatrix4( dMatrix4& B, dMatrix4& Binv )
{
    dReal det =  (B11 * B22 - B12 * B21) * (B33 * B44 - B34 * B43)
        -(B11 * B23 - B13 * B21) * (B32 * B44 - B34 * B42)
        +(B11 * B24 - B14 * B21) * (B32 * B43 - B33 * B42)
        +(B12 * B23 - B13 * B22) * (B31 * B44 - B34 * B41)
        -(B12 * B24 - B14 * B22) * (B31 * B43 - B33 * B41)
        +(B13 * B24 - B14 * B23) * (B31 * B42 - B32 * B41);
    
    dAASSERT (det != 0.0);    
    
    det = 1.0 / det;

    Binv11 = (dReal) (det * ((B22 * B33) - (B23 * B32)));
    Binv12 = (dReal) (det * ((B32 * B13) - (B33 * B12)));
    Binv13 = (dReal) (det * ((B12 * B23) - (B13 * B22)));
    Binv14 = 0.0f;
    Binv21 = (dReal) (det * ((B23 * B31) - (B21 * B33)));
    Binv22 = (dReal) (det * ((B33 * B11) - (B31 * B13)));
    Binv23 = (dReal) (det * ((B13 * B21) - (B11 * B23)));
    Binv24 = 0.0f;
    Binv31 = (dReal) (det * ((B21 * B32) - (B22 * B31)));
    Binv32 = (dReal) (det * ((B31 * B12) - (B32 * B11)));
    Binv33 = (dReal) (det * ((B11 * B22) - (B12 * B21)));
    Binv34 = 0.0f;
    Binv41 = (dReal) (det * (B21*(B33*B42 - B32*B43) + B22*(B31*B43 - B33*B41) + B23*(B32*B41 - B31*B42)));
    Binv42 = (dReal) (det * (B31*(B13*B42 - B12*B43) + B32*(B11*B43 - B13*B41) + B33*(B12*B41 - B11*B42)));
    Binv43 = (dReal) (det * (B41*(B13*B22 - B12*B23) + B42*(B11*B23 - B13*B21) + B43*(B12*B21 - B11*B22)));
    Binv44 = 1.0f;
}



/////////////////////////////////////////////////
//
// Triangle/Triangle intersection utilities
//
// From the article "A Fast Triangle-Triangle Intersection Test",
// Journal of Graphics Tools, 2(2), 1997
//
// Some of this functionality is duplicated in OPCODE (see
//  OPC_TriTriOverlap.h) but we have replicated it here so we don't
//  have to mess with the internals of OPCODE, as well as so we can
//  further optimize some of the functions.
// 
//  This version computes the line of intersection as well (if they
//  are not coplanar):
//  int TriTriIntersectWithIsectLine(dReal V0[3],dReal V1[3],dReal V2[3], 
//                                   dReal U0[3],dReal U1[3],dReal U2[3],
//                                   int *coplanar,
//                                   dReal isectpt1[3],dReal isectpt2[3]);
//
//  parameters: vertices of triangle 1: V0,V1,V2
//              vertices of triangle 2: U0,U1,U2
//
//  result    : returns 1 if the triangles intersect, otherwise 0
//              "coplanar" returns whether the tris are coplanar
//              isectpt1, isectpt2 are the endpoints of the line of
//              intersection
// 



#define FABS(x) ((dReal)fabs(x))        /* implement as is fastest on your machine */

/* if USE_EPSILON_TEST is true then we do a check: 
         if |dv|<EPSILON then dv=0.0;
   else no check is done (which is less robust)
*/
#define USE_EPSILON_TEST TRUE  
#define EPSILON 0.000001


/* sort so that a<=b */
#define SORT(a,b)       \
             if(a>b)    \
             {          \
               dReal c; \
               c=a;     \
               a=b;     \
               b=c;     \
             }

#define ISECT(VV0,VV1,VV2,D0,D1,D2,isect0,isect1) \
              isect0=VV0+(VV1-VV0)*D0/(D0-D1);    \
              isect1=VV0+(VV2-VV0)*D0/(D0-D2);


#define COMPUTE_INTERVALS(VV0,VV1,VV2,D0,D1,D2,D0D1,D0D2,isect0,isect1) \
  if(D0D1>0.0f)                                         \
  {                                                     \
    /* here we know that D0D2<=0.0 */                   \
    /* that is D0, D1 are on the same side, D2 on the other or on the plane */ \
    ISECT(VV2,VV0,VV1,D2,D0,D1,isect0,isect1);          \
  }                                                     \
  else if(D0D2>0.0f)                                    \
  {                                                     \
    /* here we know that d0d1<=0.0 */                   \
    ISECT(VV1,VV0,VV2,D1,D0,D2,isect0,isect1);          \
  }                                                     \
  else if(D1*D2>0.0f || D0!=0.0f)                       \
  {                                                     \
    /* here we know that d0d1<=0.0 or that D0!=0.0 */   \
    ISECT(VV0,VV1,VV2,D0,D1,D2,isect0,isect1);          \
  }                                                     \
  else if(D1!=0.0f)                                     \
  {                                                     \
    ISECT(VV1,VV0,VV2,D1,D0,D2,isect0,isect1);          \
  }                                                     \
  else if(D2!=0.0f)                                     \
  {                                                     \
    ISECT(VV2,VV0,VV1,D2,D0,D1,isect0,isect1);          \
  }                                                     \
  else                                                  \
  {                                                     \
    /* triangles are coplanar */                        \
    return coplanar_tri_tri(N1,V0,V1,V2,U0,U1,U2);      \
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
  dReal Ax,Ay,Bx,By,Cx,Cy,e,d,f;               \
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
  dReal a,b,c,d0,d1,d2;                     \
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

int coplanar_tri_tri(dReal N[3],dReal V0[3],dReal V1[3],dReal V2[3],
                     dReal U0[3],dReal U1[3],dReal U2[3])
{
   dReal A[3];
   short i0,i1;
   /* first project onto an axis-aligned plane, that maximizes the area */
   /* of the triangles, compute indices: i0,i1. */
   A[0]= (dReal) fabs(N[0]);
   A[1]= (dReal) fabs(N[1]);
   A[2]= (dReal) fabs(N[2]);
   if(A[0]>A[1])
   {
      if(A[0]>A[2])  
      {
          i0=1;      /* A[0] is greatest */
          i1=2;
      }
      else
      {
          i0=0;      /* A[2] is greatest */
          i1=1;
      }
   }
   else   /* A[0]<=A[1] */
   {
      if(A[2]>A[1])
      {
          i0=0;      /* A[2] is greatest */
          i1=1;                                           
      }
      else
      {
          i0=0;      /* A[1] is greatest */
          i1=2;
      }
    }               
                
    /* test all edges of triangle 1 against the edges of triangle 2 */
    EDGE_AGAINST_TRI_EDGES(V0,V1,U0,U1,U2);
    EDGE_AGAINST_TRI_EDGES(V1,V2,U0,U1,U2);
    EDGE_AGAINST_TRI_EDGES(V2,V0,U0,U1,U2);
                
    /* finally, test if tri1 is totally contained in tri2 or vice versa */
    POINT_IN_TRI(V0,U0,U1,U2);
    POINT_IN_TRI(U0,V0,V1,V2);

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
                return coplanar_tri_tri(N1,V0,V1,V2,U0,U1,U2); \
        } \
}




/* sort so that a<=b */
#define SORT2(a,b,smallest)       \
             if(a>b)       \
             {             \
               dReal c;    \
               c=a;        \
               a=b;        \
               b=c;        \
               smallest=1; \
             }             \
             else smallest=0;


inline void isect2(dReal VTX0[3],dReal VTX1[3],dReal VTX2[3],dReal VV0,dReal VV1,dReal VV2,
        dReal D0,dReal D1,dReal D2,dReal *isect0,dReal *isect1,dReal isectpoint0[3],dReal isectpoint1[3]) 
{
  dReal tmp=D0/(D0-D1);          
  dReal diff[3];
  *isect0=VV0+(VV1-VV0)*tmp;         
  SUB(diff,VTX1,VTX0);              
  MULT(diff,diff,tmp);               
  ADD(isectpoint0,diff,VTX0);        
  tmp=D0/(D0-D2);                    
  *isect1=VV0+(VV2-VV0)*tmp;          
  SUB(diff,VTX2,VTX0);                   
  MULT(diff,diff,tmp);                 
  ADD(isectpoint1,VTX0,diff);          
}


#if 0
#define ISECT2(VTX0,VTX1,VTX2,VV0,VV1,VV2,D0,D1,D2,isect0,isect1,isectpoint0,isectpoint1) \
              tmp=D0/(D0-D1);                \
              isect0=VV0+(VV1-VV0)*tmp;      \
          SUB(diff,VTX1,VTX0);               \
          MULT(diff,diff,tmp);               \
              ADD(isectpoint0,diff,VTX0);    \
              tmp=D0/(D0-D2);
/*              isect1=VV0+(VV2-VV0)*tmp;          \ */
/*              SUB(diff,VTX2,VTX0);               \ */
/*              MULT(diff,diff,tmp);               \ */
/*              ADD(isectpoint1,VTX0,diff);          */
#endif

inline int compute_intervals_isectline(dReal VERT0[3],dReal VERT1[3],dReal VERT2[3],
                       dReal VV0,dReal VV1,dReal VV2,dReal D0,dReal D1,dReal D2,
                       dReal D0D1,dReal D0D2,dReal *isect0,dReal *isect1,
                       dReal isectpoint0[3],dReal isectpoint1[3])
{
  if(D0D1>0.0f)                                        
  {                                                    
    /* here we know that D0D2<=0.0 */                  
    /* that is D0, D1 are on the same side, D2 on the other or on the plane */
    isect2(VERT2,VERT0,VERT1,VV2,VV0,VV1,D2,D0,D1,isect0,isect1,isectpoint0,isectpoint1);
  } 
  else if(D0D2>0.0f)                                   
    {                                                   
    /* here we know that d0d1<=0.0 */             
    isect2(VERT1,VERT0,VERT2,VV1,VV0,VV2,D1,D0,D2,isect0,isect1,isectpoint0,isectpoint1);
  }                                                  
  else if(D1*D2>0.0f || D0!=0.0f)   
  {                                   
    /* here we know that d0d1<=0.0 or that D0!=0.0 */
    isect2(VERT0,VERT1,VERT2,VV0,VV1,VV2,D0,D1,D2,isect0,isect1,isectpoint0,isectpoint1);   
  }                                                  
  else if(D1!=0.0f)                                  
  {                                               
    isect2(VERT1,VERT0,VERT2,VV1,VV0,VV2,D1,D0,D2,isect0,isect1,isectpoint0,isectpoint1); 
  }                                         
  else if(D2!=0.0f)                                  
  {                                                   
    isect2(VERT2,VERT0,VERT1,VV2,VV0,VV1,D2,D0,D1,isect0,isect1,isectpoint0,isectpoint1);     
  }                                                 
  else                                               
  {                                                   
    /* triangles are coplanar */    
    return 1;
  }
  return 0;
}

#define COMPUTE_INTERVALS_ISECTLINE(VERT0,VERT1,VERT2,VV0,VV1,VV2,D0,D1,D2,D0D1,D0D2,isect0,isect1,isectpoint0,isectpoint1) \
  if(D0D1>0.0f)                                         \
  {                                                     \
    /* here we know that D0D2<=0.0 */                   \
    /* that is D0, D1 are on the same side, D2 on the other or on the plane */ \
    isect2(VERT2,VERT0,VERT1,VV2,VV0,VV1,D2,D0,D1,&isect0,&isect1,isectpoint0,isectpoint1);          \
  }                                                     
#if 0
  else if(D0D2>0.0f)                                    \
  {                                                     \
    /* here we know that d0d1<=0.0 */                   \
    isect2(VERT1,VERT0,VERT2,VV1,VV0,VV2,D1,D0,D2,&isect0,&isect1,isectpoint0,isectpoint1);          \
  }                                                     \
  else if(D1*D2>0.0f || D0!=0.0f)                       \
  {                                                     \
    /* here we know that d0d1<=0.0 or that D0!=0.0 */   \
    isect2(VERT0,VERT1,VERT2,VV0,VV1,VV2,D0,D1,D2,&isect0,&isect1,isectpoint0,isectpoint1);          \
  }                                                     \
  else if(D1!=0.0f)                                     \
  {                                                     \
    isect2(VERT1,VERT0,VERT2,VV1,VV0,VV2,D1,D0,D2,&isect0,&isect1,isectpoint0,isectpoint1);          \
  }                                                     \
  else if(D2!=0.0f)                                     \
  {                                                     \
    isect2(VERT2,VERT0,VERT1,VV2,VV0,VV1,D2,D0,D1,&isect0,&isect1,isectpoint0,isectpoint1);          \
  }                                                     \
  else                                                  \
  {                                                     \
    /* triangles are coplanar */                        \
    coplanar=1;                                         \
    return coplanar_tri_tri(N1,V0,V1,V2,U0,U1,U2);      \
  }
#endif



static int TriTriIntersectWithIsectLine(dReal V0[3],dReal V1[3],dReal V2[3],
                                        dReal U0[3],dReal U1[3],dReal U2[3],int *coplanar,
                                        dReal isectpt1[3],dReal isectpt2[3])
{
  dReal E1[3],E2[3];
  dReal N1[3],N2[3],d1,d2;
  dReal du0,du1,du2,dv0,dv1,dv2;
  dReal D[3];
  dReal isect1[2], isect2[2];
  dReal isectpointA1[3],isectpointA2[3];
  dReal isectpointB1[3],isectpointB2[3];
  dReal du0du1,du0du2,dv0dv1,dv0dv2;
  short index;
  dReal vp0,vp1,vp2;
  dReal up0,up1,up2;
  dReal b,c,max;
  int smallest1,smallest2;
  
  /* compute plane equation of triangle(V0,V1,V2) */
  SUB(E1,V1,V0);
  SUB(E2,V2,V0);
  CROSS(N1,E1,E2);
  d1=-DOT(N1,V0);
  /* plane equation 1: N1.X+d1=0 */

  /* put U0,U1,U2 into plane equation 1 to compute signed distances to the plane*/
  du0=DOT(N1,U0)+d1;
  du1=DOT(N1,U1)+d1;
  du2=DOT(N1,U2)+d1;

  /* coplanarity robustness check */
#if USE_EPSILON_TEST==TRUE
  if(fabs(du0)<EPSILON) du0=0.0;
  if(fabs(du1)<EPSILON) du1=0.0;
  if(fabs(du2)<EPSILON) du2=0.0;
#endif
  du0du1=du0*du1;
  du0du2=du0*du2;

  if(du0du1>0.0f && du0du2>0.0f) /* same sign on all of them + not equal 0 ? */
    return 0;                    /* no intersection occurs */

  /* compute plane of triangle (U0,U1,U2) */
  SUB(E1,U1,U0);
  SUB(E2,U2,U0);
  CROSS(N2,E1,E2);
  d2=-DOT(N2,U0);
  /* plane equation 2: N2.X+d2=0 */

  /* put V0,V1,V2 into plane equation 2 */
  dv0=DOT(N2,V0)+d2;
  dv1=DOT(N2,V1)+d2;
  dv2=DOT(N2,V2)+d2;

#if USE_EPSILON_TEST==TRUE
  if(fabs(dv0)<EPSILON) dv0=0.0;
  if(fabs(dv1)<EPSILON) dv1=0.0;
  if(fabs(dv2)<EPSILON) dv2=0.0;
#endif

  dv0dv1=dv0*dv1;
  dv0dv2=dv0*dv2;
        
  if(dv0dv1>0.0f && dv0dv2>0.0f) /* same sign on all of them + not equal 0 ? */
    return 0;                    /* no intersection occurs */

  /* compute direction of intersection line */
  CROSS(D,N1,N2);

  /* compute and index to the largest component of D */
  max= (dReal) fabs(D[0]);
  index=0;
  b= (dReal) fabs(D[1]);
  c= (dReal) fabs(D[2]);
  if(b>max) max=b,index=1;
  if(c>max) max=c,index=2;

  /* this is the simplified projection onto L*/
  vp0=V0[index];
  vp1=V1[index];
  vp2=V2[index];
  
  up0=U0[index];
  up1=U1[index];
  up2=U2[index];

  /* compute interval for triangle 1 */
  *coplanar=compute_intervals_isectline(V0,V1,V2,vp0,vp1,vp2,dv0,dv1,dv2,
                                        dv0dv1,dv0dv2,&isect1[0],&isect1[1],isectpointA1,isectpointA2);
  if(*coplanar) return coplanar_tri_tri(N1,V0,V1,V2,U0,U1,U2);     


  /* compute interval for triangle 2 */
  compute_intervals_isectline(U0,U1,U2,up0,up1,up2,du0,du1,du2,
                              du0du1,du0du2,&isect2[0],&isect2[1],isectpointB1,isectpointB2);

  SORT2(isect1[0],isect1[1],smallest1);
  SORT2(isect2[0],isect2[1],smallest2);

  if(isect1[1]<isect2[0] || isect2[1]<isect1[0]) return 0;

  /* at this point, we know that the triangles intersect */

  if(isect2[0]<isect1[0])
  {
    if(smallest1==0) { SET(isectpt1,isectpointA1); }
    else { SET(isectpt1,isectpointA2); }

    if(isect2[1]<isect1[1])
    {
      if(smallest2==0) { SET(isectpt2,isectpointB2); }
      else { SET(isectpt2,isectpointB1); }
    }
    else
    {
      if(smallest1==0) { SET(isectpt2,isectpointA2); }
      else { SET(isectpt2,isectpointA1); }
    }
  }
  else
  {
    if(smallest2==0) { SET(isectpt1,isectpointB1); }
    else { SET(isectpt1,isectpointB2); }

    if(isect2[1]>isect1[1])
    {
      if(smallest1==0) { SET(isectpt2,isectpointA2); }
      else { SET(isectpt2,isectpointA1); }      
    }
    else
    {
      if(smallest2==0) { SET(isectpt2,isectpointB2); }
      else { SET(isectpt2,isectpointB1); } 
    }
  }
  return 1;
}





// Find the intersectiojn point between a coplanar line segement,
// defined by X1 and X2, and a ray defined by X3 and direction N.
//
// This forumla for this calculation is:
//               (c x b) . (a x b)
//   Q = x1 + a -------------------
//                  | a x b | ^2
//
// where a = x2 - x1
//       b = x4 - x3
//       c = x3 - x1
// x1 and x2 are the edges of the triangle, and x3 is CoplanarPt
//  and x4 is (CoplanarPt - n)
static int
IntersectLineSegmentRay(dVector3 x1, dVector3 x2, dVector3 x3, dVector3 n, 
                        dVector3 out_pt)
{
    dVector3 a, b, c, x4;

    ADD(x4, x3, n);  // x4 = x3 + n
    
    SUB(a, x2, x1);  // a = x2 - x1
    SUB(b, x4, x3);
    SUB(c, x3, x1);
    
    dVector3 tmp1, tmp2;
    CROSS(tmp1, c, b);
    CROSS(tmp2, a, b);

    dReal num, denom;
    num = dDOT(tmp1, tmp2);
    denom = LENGTH( tmp2 ); 

    dReal s;
    s = num /(denom*denom);
    
    for (int i=0; i<3; i++)
        out_pt[i] = x1[i] + a[i]*s;

    // Test if this intersection is "behind" x3, w.r.t. n
    SUB(a, x3, out_pt);
    if (dDOT(a, n) > 0.0)
        return 0;

    // Test if this intersection point is outside the edge limits,
    //  if (dot( (out_pt-x1), (out_pt-x2) ) < 0) it's inside
    //  else outside
    SUB(a, out_pt, x1);
    SUB(b, out_pt, x2);
    if (dDOT(a,b) < 0.0)
        return 1;
    else
        return 0;
}


// FindTriSolidIntersection - Clips the input trinagle TRI with the 
//  sides of a convex bounding solid, described by PLANES, returning
//  the (convex) clipped polygon in CLIPPEDPOLYGON
//
static bool
FindTriSolidIntrsection(const dVector3 Tri[3], 
                        const dVector4 Planes[6], int numSides,
                        LineContactSet& ClippedPolygon )
{ 
    // Set up the LineContactSet structure
    for (int k=0; k<3; k++) {
        SET(ClippedPolygon.Points[k], Tri[k]);
    }
    ClippedPolygon.Count = 3;

    // Clip wrt the sides
    for ( int i = 0; i < numSides; i++ )
        ClipConvexPolygonAgainstPlane( Planes[i], Planes[i][3], ClippedPolygon );
    
    return (ClippedPolygon.Count > 0);
}




// ClipConvexPolygonAgainstPlane - Clip a a convex polygon, described by
//  CONTACTS, with a plane (described by N and C).  Note:  the input 
//  vertices are assumed to be in counterclockwise order.  
//
// This code is taken from The Nebula Device:
//  http://nebuladevice.sourceforge.net/cgi-bin/twiki/view/Nebula/WebHome
// and is licensed under the following license:
//  http://nebuladevice.sourceforge.net/doc/source/license.txt
//
static void
ClipConvexPolygonAgainstPlane( const dVector3 N, dReal C, 
                               LineContactSet& Contacts )
{
    // test on which side of line are the vertices
    int Positive = 0, Negative = 0, PIndex = -1;
    int Quantity = Contacts.Count;
    
    dReal Test[8];
    for ( int i = 0; i < Contacts.Count; i++ ) {
        // An epsilon is used here because it is possible for the dot product
        // and C to be exactly equal to each other (in theory), but differ
        // slightly because of floating point problems.  Thus, add a little
        // to the test number to push actually equal numbers over the edge
        // towards the positive.  This should probably be somehow a relative
        // tolerance, and I don't think multiplying by the constant is the best
        // way to do this.
        Test[i] = dDOT(N, Contacts.Points[i]) - C + dFabs(C)*1e-08;
            
        if (Test[i] >= REAL(0.0)) {
            Positive++;
            if (PIndex < 0) {
                PIndex = i;
            }
        }
        else Negative++;
    }
    
    if (Positive > 0) {
        if (Negative > 0) {
            // plane transversely intersects polygon
            dVector3 CV[8];
            int CQuantity = 0, Cur, Prv;
            dReal T;
            
            if (PIndex > 0) {
                // first clip vertex on line
                Cur = PIndex;
                Prv = Cur - 1;
                T = Test[Cur] / (Test[Cur] - Test[Prv]);
                CV[CQuantity][0] = Contacts.Points[Cur][0] 
                    + T * (Contacts.Points[Prv][0] - Contacts.Points[Cur][0]);
                CV[CQuantity][1] = Contacts.Points[Cur][1] 
                    + T * (Contacts.Points[Prv][1] - Contacts.Points[Cur][1]);
                CV[CQuantity][2] = Contacts.Points[Cur][2] 
                    + T * (Contacts.Points[Prv][2] - Contacts.Points[Cur][2]);
                CV[CQuantity][3] = Contacts.Points[Cur][3] 
                    + T * (Contacts.Points[Prv][3] - Contacts.Points[Cur][3]);
                CQuantity++;
                
                // vertices on positive side of line
                while (Cur < Quantity && Test[Cur] >= REAL(0.0)) {
                    CV[CQuantity][0] = Contacts.Points[Cur][0];
                    CV[CQuantity][1] = Contacts.Points[Cur][1];
                    CV[CQuantity][2] = Contacts.Points[Cur][2];
                    CV[CQuantity][3] = Contacts.Points[Cur][3];
                    CQuantity++;
                    Cur++;
                }
                
                // last clip vertex on line
                if (Cur < Quantity) {
                    Prv = Cur - 1;
                }
                else {
                    Cur = 0;
                    Prv = Quantity - 1;
                }
                
                T = Test[Cur] / (Test[Cur] - Test[Prv]);
                CV[CQuantity][0] = Contacts.Points[Cur][0] 
                    + T * (Contacts.Points[Prv][0] - Contacts.Points[Cur][0]);
                CV[CQuantity][1] = Contacts.Points[Cur][1] 
                    + T * (Contacts.Points[Prv][1] - Contacts.Points[Cur][1]);
                CV[CQuantity][2] = Contacts.Points[Cur][2] 
                    + T * (Contacts.Points[Prv][2] - Contacts.Points[Cur][2]);
                CV[CQuantity][3] = Contacts.Points[Cur][3] 
                    + T * (Contacts.Points[Prv][3] - Contacts.Points[Cur][3]);
                CQuantity++;
            }
            else {
                // iPIndex is 0
                // vertices on positive side of line
                Cur = 0;
                while (Cur < Quantity && Test[Cur] >= REAL(0.0)) {
                    CV[CQuantity][0] = Contacts.Points[Cur][0];
                    CV[CQuantity][1] = Contacts.Points[Cur][1];
                    CV[CQuantity][2] = Contacts.Points[Cur][2];
                    CV[CQuantity][3] = Contacts.Points[Cur][3];
                    CQuantity++;
                    Cur++;
                }
                
                // last clip vertex on line
                Prv = Cur - 1;
                T = Test[Cur] / (Test[Cur] - Test[Prv]);
                CV[CQuantity][0] = Contacts.Points[Cur][0] 
                    + T * (Contacts.Points[Prv][0] - Contacts.Points[Cur][0]);
                CV[CQuantity][1] = Contacts.Points[Cur][1] 
                    + T * (Contacts.Points[Prv][1] - Contacts.Points[Cur][1]);
                CV[CQuantity][2] = Contacts.Points[Cur][2] 
                    + T * (Contacts.Points[Prv][2] - Contacts.Points[Cur][2]);
                CV[CQuantity][3] = Contacts.Points[Cur][3] 
                    + T * (Contacts.Points[Prv][3] - Contacts.Points[Cur][3]);
                CQuantity++;
                
                // skip vertices on negative side
                while (Cur < Quantity && Test[Cur] < REAL(0.0)) {
                    Cur++;
                }
          
                // first clip vertex on line
                if (Cur < Quantity) {
                    Prv = Cur - 1;
                    T = Test[Cur] / (Test[Cur] - Test[Prv]);
                    CV[CQuantity][0] = Contacts.Points[Cur][0] 
                        + T * (Contacts.Points[Prv][0] - Contacts.Points[Cur][0]);
                    CV[CQuantity][1] = Contacts.Points[Cur][1] 
                              + T * (Contacts.Points[Prv][1] - Contacts.Points[Cur][1]);
                    CV[CQuantity][2] = Contacts.Points[Cur][2] 
                        + T * (Contacts.Points[Prv][2] - Contacts.Points[Cur][2]);
                    CV[CQuantity][3] = Contacts.Points[Cur][3] 
                        + T * (Contacts.Points[Prv][3] - Contacts.Points[Cur][3]);
                    CQuantity++;
            
                    // vertices on positive side of line
                    while (Cur < Quantity && Test[Cur] >= REAL(0.0)) {
                        CV[CQuantity][0] = Contacts.Points[Cur][0];
                        CV[CQuantity][1] = Contacts.Points[Cur][1];
                        CV[CQuantity][2] = Contacts.Points[Cur][2];
                        CV[CQuantity][3] = Contacts.Points[Cur][3];
                        CQuantity++;
                        Cur++;
                    }
                }
                else {
                    // iCur = 0
                    Prv = Quantity - 1;
                    T = Test[0] / (Test[0] - Test[Prv]);
                    CV[CQuantity][0] = Contacts.Points[0][0] 
                        + T * (Contacts.Points[Prv][0] - Contacts.Points[0][0]);
                    CV[CQuantity][1] = Contacts.Points[0][1] 
                              + T * (Contacts.Points[Prv][1] - Contacts.Points[0][1]);
                    CV[CQuantity][2] = Contacts.Points[0][2] 
                        + T * (Contacts.Points[Prv][2] - Contacts.Points[0][2]);
                    CV[CQuantity][3] = Contacts.Points[0][3] 
                        + T * (Contacts.Points[Prv][3] - Contacts.Points[0][3]);
                    CQuantity++;
                }
            }
            Quantity = CQuantity;
            memcpy( Contacts.Points, CV, CQuantity * sizeof(dVector3) );
        }
        // else polygon fully on positive side of plane, nothing to do    
        Contacts.Count = Quantity;
    }
    else {
        Contacts.Count = 0; // This should not happen, but for safety
    }

}



// Determine if a potential collision point is 
//
//
static int
ExamineContactPoint(dVector3* v_col, dVector3 in_n, dVector3 in_point)
{
    // Cast a ray from in_point, along the collison normal. Does it intersect the
    //  collision face.
    dReal t, u, v;
    
    if (!RayTriangleIntersect(in_point, in_n, v_col[0], v_col[1], v_col[2],
                              &t, &u, &v))
        return 0;
    else
    return 1;
}



// RayTriangleIntersect - If an intersection is found, t contains the
//   distance along the ray (dir) and u/v contain u/v coordinates into
//   the triangle.  Returns 0 if no hit is found
//   From "Real-Time Rendering," page 305
//
static int
RayTriangleIntersect(const dVector3 orig, const dVector3 dir,
                     const dVector3 vert0, const dVector3 vert1,const dVector3 vert2,
                     dReal *t,dReal *u,dReal *v)

{
    dReal edge1[3], edge2[3], tvec[3], pvec[3], qvec[3];
    dReal det,inv_det;
    
    // find vectors for two edges sharing vert0
    SUB(edge1, vert1, vert0);
    SUB(edge2, vert2, vert0);
    
    // begin calculating determinant - also used to calculate U parameter
    CROSS(pvec, dir, edge2);

    // if determinant is near zero, ray lies in plane of triangle
    det = DOT(edge1, pvec);

    if ((det > -0.001) && (det < 0.001))
        return 0;
    inv_det = 1.0 / det;

    // calculate distance from vert0 to ray origin 
    SUB(tvec, orig, vert0);

    // calculate U parameter and test bounds
    *u = DOT(tvec, pvec) * inv_det;
    if ((*u < 0.0) || (*u > 1.0))
        return 0;

    // prepare to test V parameter
    CROSS(qvec, tvec, edge1);

    // calculate V parameter and test bounds
    *v = DOT(dir, qvec) * inv_det;
    if ((*v < 0.0) || ((*u + *v) > 1.0))
        return 0;

    // calculate t, ray intersects triangle
    *t = DOT(edge2, qvec) * inv_det;

    return 1;
}



static bool
SimpleUnclippedTest(dVector3 in_CoplanarPt, dVector3 in_v, dVector3 in_elt,
                    dVector3 in_n, dVector3* in_col_v, dReal &out_depth)
{
    dReal dp = 0.0;
    dReal contact_elt_length;

    DEPTH(dp, in_CoplanarPt, in_v, in_n);
    
    if (dp >= 0.0) {
        // if the penetration depth (calculated above) is more than
        //  the contact point's ELT, then we've chosen the wrong face
        //  and should switch faces
        contact_elt_length = fabs(dDOT(in_elt, in_n));
        
        if (dp == 0.0)
            dp = dMin(DISTANCE_EPSILON, contact_elt_length);
        
        if ((contact_elt_length < SMALL_ELT) && (dp < EXPANDED_ELT_THRESH))
            dp = contact_elt_length;
        
        if ( (dp > 0.0) && (dp <= contact_elt_length)) {
            // Add a contact
            
            if ( ExamineContactPoint(in_col_v, in_n, in_v) ) {
                out_depth = dp;
                return true;
            }
        }
    }

    return false;
}




// Generate a "unique" contact.  A unique contact has a unique
//   position or normal.  If the potential contact has the same
//   position and normal as an existing contact, but a larger
//   penetration depth, this new depth is used instead
//
static void
GenerateContact(int in_Flags, dContactGeom* in_Contacts, int in_Stride,  
                dxTriMesh* in_TriMesh1,  dxTriMesh* in_TriMesh2,
                const dVector3 in_ContactPos, const dVector3 in_Normal, dReal in_Depth,
                int& OutTriCount)
{
    if (in_Depth < 0.0)
        return;

    if (OutTriCount == (in_Flags & 0x0ffff))
        return; // contacts are full!


    dContactGeom* Contact;
    dVector3 diff;
    bool duplicate = false;

    for (int i=0; i<OutTriCount; i++) 
    {
        Contact = SAFECONTACT(in_Flags, in_Contacts, i, in_Stride);

        // same position?
        SUB(diff, in_ContactPos, Contact->pos);
        if (dDOT(diff, diff) < 0.01) 
        {
            // same normal?
            if (fabs(dDOT(in_Normal, Contact->normal)) > 0.99 )
            {
                if (in_Depth > Contact->depth) {
                    Contact->depth = in_Depth;
                    SMULT( Contact->normal, in_Normal, -1.0);
                    Contact->normal[3] = 0.0;
                }
                duplicate = true;
            }
        }
    }

    
    if (!duplicate) 
    {
        // Add a new contact
        Contact = SAFECONTACT(in_Flags, in_Contacts, OutTriCount, in_Stride);

        SET( Contact->pos, in_ContactPos );
        Contact->pos[3] = 0.0;
        
        SMULT( Contact->normal, in_Normal, -1.0);
        Contact->normal[3] = 0.0;
        
        Contact->depth = in_Depth;

        Contact->g1 = in_TriMesh1;
        Contact->g2 = in_TriMesh2;
        
        OutTriCount++;
    }


}
