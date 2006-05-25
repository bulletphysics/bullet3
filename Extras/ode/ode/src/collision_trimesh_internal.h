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

// TriMesh code by Erwin de Vries.

#ifndef _ODE_COLLISION_TRIMESH_INTERNAL_H_
#define _ODE_COLLISION_TRIMESH_INTERNAL_H_

int dCollideTrimeshConvex(dxGeom* g1, dxGeom* cnvx, int Flags, dContactGeom* Contacts, int Stride);
int dCollideSTL(dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip);
int dCollideBTL(dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip);
int dCollideRTL(dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip);
int dCollideTTL(dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip);
int dCollideCCTL(dxGeom *o1, dxGeom *o2, int flags, dContactGeom *contact, int skip);

//****************************************************************************
// dxTriMesh class

#ifdef TRIMESH_INTERNAL

#include "collision_kernel.h"
#include <ode/collision_trimesh.h>

#define BAN_OPCODE_AUTOLINK
#include "Opcode.h"
using namespace Opcode;

struct dxTriMeshData  : public dBase {
	Model BVTree;
	MeshInterface Mesh;

    dxTriMeshData();
    ~dxTriMeshData();
    
    void Build(const void* Vertices, int VertexStide, int VertexCount, 
	       const void* Indices, int IndexCount, int TriStride, 
	       const void* Normals, 
	       bool Single);
    
        /* aabb in model space */
        dVector3 AABBCenter;
        dVector3 AABBExtents;

    /* data for use in collison resolution */
    const void* Normals;
    //Matrix4x4   last_trans;
    dMatrix4    last_trans;
};


struct dxTriMesh : public dxGeom{
	// Callbacks
	dTriCallback* Callback;
	dTriArrayCallback* ArrayCallback;
	dTriRayCallback* RayCallback;

	// Data types
	dxTriMeshData* Data;


	// Colliders
	static PlanesCollider _PlanesCollider;
	static SphereCollider _SphereCollider;
	static OBBCollider _OBBCollider;
	static RayCollider _RayCollider;
	static AABBTreeCollider _AABBTreeCollider;
	static LSSCollider _LSSCollider;

	// Some constants
	static CollisionFaces Faces;

	// Temporal coherence
	struct SphereTC : public SphereCache{
		dxGeom* Geom;
	};
	dArray<SphereTC> SphereTCCache;
	static SphereCache defaultSphereCache;

	struct BoxTC : public OBBCache{
		dxGeom* Geom;
	};
	dArray<BoxTC> BoxTCCache;
	static OBBCache defaultBoxCache;
	
	struct CCylinderTC : public LSSCache{
		dxGeom* Geom;
	};
	dArray<CCylinderTC> CCylinderTCCache;
	static LSSCache defaultCCylinderCache;

	bool doSphereTC;
	bool doBoxTC;
	bool doCCylinderTC;

	// Functions
	dxTriMesh(dSpaceID Space, dTriMeshDataID Data);
	~dxTriMesh();

	void ClearTCCache();

	int AABBTest(dxGeom* g, dReal aabb[6]);
	void computeAABB();
};

// Fetches a contact
inline dContactGeom* SAFECONTACT(int Flags, dContactGeom* Contacts, int Index, int Stride){
	dIASSERT(Index >= 0 && Index < (Flags & 0x0ffff));
	return ((dContactGeom*)(((char*)Contacts) + (Index * Stride)));
}

// Fetches a triangle
inline void FetchTriangle(dxTriMesh* TriMesh, int Index, dVector3 Out[3]){
	VertexPointers VP;
	TriMesh->Data->Mesh.GetTriangle(VP, Index);
	for (int i = 0; i < 3; i++){
		Out[i][0] = VP.Vertex[i]->x;
		Out[i][1] = VP.Vertex[i]->y;
		Out[i][2] = VP.Vertex[i]->z;
		Out[i][3] = 0;
	}
}

// Fetches a triangle
inline void FetchTriangle(dxTriMesh* TriMesh, int Index, const dVector3 Position, const dMatrix3 Rotation, dVector3 Out[3]){
	VertexPointers VP;
	TriMesh->Data->Mesh.GetTriangle(VP, Index);
	for (int i = 0; i < 3; i++){
		dVector3 v;
		v[0] = VP.Vertex[i]->x;
		v[1] = VP.Vertex[i]->y;
		v[2] = VP.Vertex[i]->z;
		v[3] = 0;

		dMULTIPLY0_331(Out[i], Rotation, v);
		Out[i][0] += Position[0];
		Out[i][1] += Position[1];
		Out[i][2] += Position[2];
		Out[i][3] = 0;
	}
}

// Creates an OPCODE matrix from an ODE matrix
inline Matrix4x4& MakeMatrix(const dVector3 Position, const dMatrix3 Rotation, Matrix4x4& Out){
	Out.m[0][0] = (float) Rotation[0];
	Out.m[1][0] = (float) Rotation[1];
	Out.m[2][0] = (float) Rotation[2];

	Out.m[0][1] = (float) Rotation[4];
	Out.m[1][1] = (float) Rotation[5];
	Out.m[2][1] = (float) Rotation[6];

	Out.m[0][2] = (float) Rotation[8];
	Out.m[1][2] = (float) Rotation[9];
	Out.m[2][2] = (float) Rotation[10];

	Out.m[3][0] = (float) Position[0];
	Out.m[3][1] = (float) Position[1];
	Out.m[3][2] = (float) Position[2];

	Out.m[0][3] = 0.0f;
	Out.m[1][3] = 0.0f;
	Out.m[2][3] = 0.0f;
	Out.m[3][3] = 1.0f;

	return Out;
}

// Outputs a matrix to 3 vectors
inline void Decompose(const dMatrix3 Matrix, dVector3 Right, dVector3 Up, dVector3 Direction){
	Right[0] = Matrix[0 * 4 + 0];
	Right[1] = Matrix[1 * 4 + 0];
	Right[2] = Matrix[2 * 4 + 0];
	Right[3] = REAL(0.0);
	Up[0] = Matrix[0 * 4 + 1];
	Up[1] = Matrix[1 * 4 + 1];
	Up[2] = Matrix[2 * 4 + 1];
	Up[3] = REAL(0.0);
	Direction[0] = Matrix[0 * 4 + 2];
	Direction[1] = Matrix[1 * 4 + 2];
	Direction[2] = Matrix[2 * 4 + 2];
	Direction[3] = REAL(0.0);
}

// Outputs a matrix to 3 vectors
inline void Decompose(const dMatrix3 Matrix, dVector3 Vectors[3]){
	Decompose(Matrix, Vectors[0], Vectors[1], Vectors[2]);
}

// Creates an OPCODE matrix from an ODE matrix
inline Matrix4x4& MakeMatrix(dxGeom* g, Matrix4x4& Out){
	const dVector3& Position = *(const dVector3*)dGeomGetPosition(g);
	const dMatrix3& Rotation = *(const dMatrix3*)dGeomGetRotation(g);
	return MakeMatrix(Position, Rotation, Out);
}

// Finds barycentric
inline void GetPointFromBarycentric(const dVector3 dv[3], dReal u, dReal v, dVector3 Out){
	dReal w = REAL(1.0) - u - v;

	Out[0] = (dv[0][0] * w) + (dv[1][0] * u) + (dv[2][0] * v);
	Out[1] = (dv[0][1] * w) + (dv[1][1] * u) + (dv[2][1] * v);
	Out[2] = (dv[0][2] * w) + (dv[1][2] * u) + (dv[2][2] * v);
	Out[3] = (dv[0][3] * w) + (dv[1][3] * u) + (dv[2][3] * v);
}

// Performs a callback
inline bool Callback(dxTriMesh* TriMesh, dxGeom* Object, int TriIndex){
	if (TriMesh->Callback != null){
		return TriMesh->Callback(TriMesh, Object, TriIndex);
	}
	else return true;
}

// Some utilities
template<class T> const T& dcMAX(const T& x, const T& y){
	return x > y ? x : y;
}

template<class T> const T& dcMIN(const T& x, const T& y){
	return x < y ? x : y;
}

dReal SqrDistancePointTri( const dVector3 p, const dVector3 triOrigin, 
                           const dVector3 triEdge1, const dVector3 triEdge2,
                           dReal* pfSParam = 0, dReal* pfTParam = 0 );

dReal SqrDistanceSegments( const dVector3 seg1Origin, const dVector3 seg1Direction, 
                           const dVector3 seg2Origin, const dVector3 seg2Direction,
                           dReal* pfSegP0 = 0, dReal* pfSegP1 = 0 );

dReal SqrDistanceSegTri( const dVector3 segOrigin, const dVector3 segEnd, 
                         const dVector3 triOrigin, 
                         const dVector3 triEdge1, const dVector3 triEdge2,
                         dReal* t = 0, dReal* u = 0, dReal* v = 0 );

inline
void Vector3Subtract( const dVector3 left, const dVector3 right, dVector3 result )
{
    result[0] = left[0] - right[0];
    result[1] = left[1] - right[1];
    result[2] = left[2] - right[2];
    result[3] = REAL(0.0);
}

inline
void Vector3Add( const dVector3 left, const dVector3 right, dVector3 result )
{
    result[0] = left[0] + right[0];
    result[1] = left[1] + right[1];
    result[2] = left[2] + right[2];
    result[3] = REAL(0.0);
}

inline
void Vector3Negate( const dVector3 in, dVector3 out )
{
    out[0] = -in[0];
    out[1] = -in[1];
    out[2] = -in[2];
    out[3] = REAL(0.0);
}

inline
void Vector3Copy( const dVector3 in, dVector3 out )
{
    out[0] = in[0];
    out[1] = in[1];
    out[2] = in[2];
    out[3] = REAL(0.0);
}

inline
void Vector3Multiply( const dVector3 in, dReal scalar, dVector3 out )
{
    out[0] = in[0] * scalar;
    out[1] = in[1] * scalar;
    out[2] = in[2] * scalar;
    out[3] = REAL(0.0);
}

inline
void TransformVector3( const dVector3 in, 
                       const dMatrix3 orientation, const dVector3 position, 
                       dVector3 out )
{
    dMULTIPLY0_331( out, orientation, in );
    out[0] += position[0];
    out[1] += position[1];
    out[2] += position[2];
}

//------------------------------------------------------------------------------
/**
  @brief Check for intersection between triangle and capsule.
  
  @param dist [out] Shortest distance squared between the triangle and 
                    the capsule segment (central axis).
  @param t    [out] t value of point on segment that's the shortest distance 
                    away from the triangle, the coordinates of this point 
                    can be found by (cap.seg.end - cap.seg.start) * t,
                    or cap.seg.ipol(t).
  @param u    [out] Barycentric coord on triangle.
  @param v    [out] Barycentric coord on triangle.
  @return True if intersection exists.
  
  The third Barycentric coord is implicit, ie. w = 1.0 - u - v
  The Barycentric coords give the location of the point on the triangle
  closest to the capsule (where the distance between the two shapes
  is the shortest).
*/
inline
bool IntersectCapsuleTri( const dVector3 segOrigin, const dVector3 segEnd, 
                          const dReal radius, const dVector3 triOrigin, 
                          const dVector3 triEdge0, const dVector3 triEdge1,
                          dReal* dist, dReal* t, dReal* u, dReal* v )
{
    dReal sqrDist = SqrDistanceSegTri( segOrigin, segEnd, triOrigin, triEdge0, triEdge1, 
                                       t, u, v );
  
    if ( dist )
      *dist = sqrDist;
    
    return ( sqrDist <= (radius * radius) );
}

#endif	//TRIMESH_INTERNAL

#endif	//_ODE_COLLISION_TRIMESH_INTERNAL_H_
