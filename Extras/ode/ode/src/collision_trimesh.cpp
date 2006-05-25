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

#include <ode/collision.h>
#include <ode/matrix.h>
#include <ode/rotation.h>
#include <ode/odemath.h>
#include "collision_util.h"
#define TRIMESH_INTERNAL
#include "collision_trimesh_internal.h"

// Trimesh data
dxTriMeshData::dxTriMeshData(){
#ifndef dTRIMESH_ENABLED
  dUASSERT(g, "dTRIMESH_ENABLED is not defined. Trimesh geoms will not work");
#endif
}

dxTriMeshData::~dxTriMeshData(){
    //
}

void 
dxTriMeshData::Build(const void* Vertices, int VertexStide, int VertexCount,
		     const void* Indices, int IndexCount, int TriStride,
		     const void* in_Normals,
		     bool Single){
    Mesh.SetNbTriangles(IndexCount / 3);
    Mesh.SetNbVertices(VertexCount);
    Mesh.SetPointers((IndexedTriangle*)Indices, (Point*)Vertices);
    Mesh.SetStrides(TriStride, VertexStide);
    Mesh.Single = Single;
    
    // Build tree
    BuildSettings Settings;
    // recommended in Opcode User Manual
    //Settings.mRules = SPLIT_COMPLETE | SPLIT_SPLATTERPOINTS | SPLIT_GEOMCENTER;
    // used in ODE, why?
    //Settings.mRules = SPLIT_BEST_AXIS;

    // best compromise?
    Settings.mRules = SPLIT_BEST_AXIS | SPLIT_SPLATTER_POINTS | SPLIT_GEOM_CENTER;


    OPCODECREATE TreeBuilder;
    TreeBuilder.mIMesh = &Mesh;

    TreeBuilder.mSettings = Settings;
    TreeBuilder.mNoLeaf = true;
    TreeBuilder.mQuantized = false;

    TreeBuilder.mKeepOriginal = false;
    TreeBuilder.mCanRemap = false;



    BVTree.Build(TreeBuilder);

    // compute model space AABB
    dVector3 AABBMax, AABBMin;
    AABBMax[0] = AABBMax[1] = AABBMax[2] = (dReal) -dInfinity;
    AABBMin[0] = AABBMin[1] = AABBMin[2] = (dReal) dInfinity;
    if( Single ) {
        const char* verts = (const char*)Vertices;
        for( int i = 0; i < VertexCount; ++i ) {
        const float* v = (const float*)verts;
        if( v[0] > AABBMax[0] ) AABBMax[0] = v[0];
        if( v[1] > AABBMax[1] ) AABBMax[1] = v[1];
        if( v[2] > AABBMax[2] ) AABBMax[2] = v[2];
        if( v[0] < AABBMin[0] ) AABBMin[0] = v[0];
        if( v[1] < AABBMin[1] ) AABBMin[1] = v[1];
        if( v[2] < AABBMin[2] ) AABBMin[2] = v[2];
        verts += VertexStide;
        }
    } else {
        const char* verts = (const char*)Vertices;
        for( int i = 0; i < VertexCount; ++i ) {
        const double* v = (const double*)verts;
        if( v[0] > AABBMax[0] ) AABBMax[0] = (dReal) v[0];
        if( v[1] > AABBMax[1] ) AABBMax[1] = (dReal) v[1];
        if( v[2] > AABBMax[2] ) AABBMax[2] = (dReal) v[2];
        if( v[0] < AABBMin[0] ) AABBMin[0] = (dReal) v[0];
        if( v[1] < AABBMin[1] ) AABBMin[1] = (dReal) v[1];
        if( v[2] < AABBMin[2] ) AABBMin[2] = (dReal) v[2];
        verts += VertexStide;
        }
    }
    AABBCenter[0] = (AABBMin[0] + AABBMax[0]) * REAL(0.5);
    AABBCenter[1] = (AABBMin[1] + AABBMax[1]) * REAL(0.5);
    AABBCenter[2] = (AABBMin[2] + AABBMax[2]) * REAL(0.5);
    AABBExtents[0] = AABBMax[0] - AABBCenter[0];
    AABBExtents[1] = AABBMax[1] - AABBCenter[1];
    AABBExtents[2] = AABBMax[2] - AABBCenter[2];

    // user data (not used by OPCODE)
    for (int i=0; i<16; i++)
        last_trans[i] = 0.0;

    Normals = (dReal *) in_Normals;
}

dTriMeshDataID dGeomTriMeshDataCreate(){
    return new dxTriMeshData();
}

void dGeomTriMeshDataDestroy(dTriMeshDataID g){
    delete g;
}

void dGeomTriMeshDataSet(dTriMeshDataID g, int data_id, void* in_data)
{
    dUASSERT(g, "argument not trimesh data");

    double *elem;
    
    switch (data_id) {
    case TRIMESH_FACE_NORMALS:
	g->Normals = (dReal *) in_data;
	break;

    case TRIMESH_LAST_TRANSFORMATION:
	elem = (double *) in_data;
    for (int i=0; i<16; i++)
        g->last_trans[i] = (dReal) elem[i];
	
	break;
    default:
	dUASSERT(data_id, "invalid data type");
	break;
    }

    return;
}



void*  dGeomTriMeshDataGet(dTriMeshDataID g, int data_id)
{
    dUASSERT(g, "argument not trimesh data");

    switch (data_id) {
    case TRIMESH_FACE_NORMALS:
        return (void *) g->Normals;
        break;
        
    case TRIMESH_LAST_TRANSFORMATION:
        return (void *) g->last_trans;
        break;
    default:
        dUASSERT(data_id, "invalid data type");
        break;
    }
    
    return NULL;
}


void dGeomTriMeshDataBuildSingle1(dTriMeshDataID g,
                                  const void* Vertices, int VertexStride, int VertexCount, 
                                  const void* Indices, int IndexCount, int TriStride,
                                  const void* Normals)
{
    dUASSERT(g, "argument not trimesh data");
    
    g->Build(Vertices, VertexStride, VertexCount, 
             Indices, IndexCount, TriStride, 
             Normals, 
             true);
}


void dGeomTriMeshDataBuildSingle(dTriMeshDataID g,
                                 const void* Vertices, int VertexStride, int VertexCount, 
                                 const void* Indices, int IndexCount, int TriStride)
{
    dGeomTriMeshDataBuildSingle1(g, Vertices, VertexStride, VertexCount,
                                 Indices, IndexCount, TriStride, (void*)NULL);
}


void dGeomTriMeshDataBuildDouble1(dTriMeshDataID g,
                                  const void* Vertices, int VertexStride, int VertexCount, 
                                 const void* Indices, int IndexCount, int TriStride,
				 const void* Normals)
{
    dUASSERT(g, "argument not trimesh data");
    
    g->Build(Vertices, VertexStride, VertexCount, 
             Indices, IndexCount, TriStride, 
             Normals, 
             false);
}


void dGeomTriMeshDataBuildDouble(dTriMeshDataID g,
				 const void* Vertices, int VertexStride, int VertexCount, 
                                 const void* Indices, int IndexCount, int TriStride) {
    dGeomTriMeshDataBuildDouble1(g, Vertices, VertexStride, VertexCount,
                                 Indices, IndexCount, TriStride, NULL);
}


void dGeomTriMeshDataBuildSimple1(dTriMeshDataID g,
                                  const dReal* Vertices, int VertexCount, 
                                 const int* Indices, int IndexCount,
                                 const int* Normals){
#ifdef dSINGLE
    dGeomTriMeshDataBuildSingle1(g,
				Vertices, 4 * sizeof(dReal), VertexCount, 
				Indices, IndexCount, 3 * sizeof(unsigned int),
				Normals);
#else
    dGeomTriMeshDataBuildDouble1(g, Vertices, 4 * sizeof(dReal), VertexCount, 
				Indices, IndexCount, 3 * sizeof(unsigned int),
				Normals);
#endif
}


void dGeomTriMeshDataBuildSimple(dTriMeshDataID g,
                                 const dReal* Vertices, int VertexCount, 
                                 const int* Indices, int IndexCount) {
    dGeomTriMeshDataBuildSimple1(g,
                                 Vertices, VertexCount, Indices, IndexCount,
                                 (const int*)NULL);
}


// Trimesh
PlanesCollider dxTriMesh::_PlanesCollider;
SphereCollider dxTriMesh::_SphereCollider;
OBBCollider dxTriMesh::_OBBCollider;
RayCollider dxTriMesh::_RayCollider;
AABBTreeCollider dxTriMesh::_AABBTreeCollider;
LSSCollider dxTriMesh::_LSSCollider;

SphereCache dxTriMesh::defaultSphereCache;
OBBCache dxTriMesh::defaultBoxCache;
LSSCache dxTriMesh::defaultCCylinderCache;

CollisionFaces dxTriMesh::Faces;

dxTriMesh::dxTriMesh(dSpaceID Space, dTriMeshDataID Data) : dxGeom(Space, 1){
    type = dTriMeshClass;

    this->Data = Data;

	_RayCollider.SetDestination(&Faces);

    _PlanesCollider.SetTemporalCoherence(true);

	_SphereCollider.SetTemporalCoherence(true);
        _SphereCollider.SetPrimitiveTests(false);


    _OBBCollider.SetTemporalCoherence(true);

    // no first-contact test (i.e. return full contact info)
	_AABBTreeCollider.SetFirstContact( false );     
    // temporal coherence only works with "first conact" tests
    _AABBTreeCollider.SetTemporalCoherence(false);
    // Perform full BV-BV tests (true) or SAT-lite tests (false)
	_AABBTreeCollider.SetFullBoxBoxTest( true );
    // Perform full Primitive-BV tests (true) or SAT-lite tests (false)
	_AABBTreeCollider.SetFullPrimBoxTest( true );
	_LSSCollider.SetTemporalCoherence(false);

	/* TC has speed/space 'issues' that don't make it a clear
	   win by default on spheres/boxes. */
	this->doSphereTC = false;
	this->doBoxTC = false;
	this->doCCylinderTC = false;

    const char* msg;
    if ((msg =_AABBTreeCollider.ValidateSettings()))
        dDebug (d_ERR_UASSERT, msg, " (%s:%d)", __FILE__,__LINE__);
	_LSSCollider.SetPrimitiveTests(false);
	_LSSCollider.SetFirstContact(false);
}

dxTriMesh::~dxTriMesh(){
    //
}


void dxTriMesh::ClearTCCache(){
  /* dxTriMesh::ClearTCCache uses dArray's setSize(0) to clear the caches -
     but the destructor isn't called when doing this, so we would leak.
     So, call the previous caches' containers' destructors by hand first. */
    int i, n;
    n = SphereTCCache.size();
    for( i = 0; i < n; ++i ) {
        SphereTCCache[i].~SphereTC();
    }
    SphereTCCache.setSize(0);
    n = BoxTCCache.size();
    for( i = 0; i < n; ++i ) {
        BoxTCCache[i].~BoxTC();
    }
    BoxTCCache.setSize(0);
	n = CCylinderTCCache.size();
	for( i = 0; i < n; ++i ) {
	  CCylinderTCCache[i].~CCylinderTC();
	}
	CCylinderTCCache.setSize(0);
}


int dxTriMesh::AABBTest(dxGeom* g, dReal aabb[6]){
    return 1;
}


void dxTriMesh::computeAABB() {
    const dxTriMeshData* d = Data;
    dVector3 c;
    
    dMULTIPLY0_331( c, R, d->AABBCenter );
    
    dReal xrange = dFabs(R[0] * Data->AABBExtents[0]) +
        dFabs(R[1] * Data->AABBExtents[1]) + 
        dFabs(R[2] * Data->AABBExtents[2]);
    dReal yrange = dFabs(R[4] * Data->AABBExtents[0]) +
        dFabs(R[5] * Data->AABBExtents[1]) + 
        dFabs(R[6] * Data->AABBExtents[2]);
    dReal zrange = dFabs(R[8] * Data->AABBExtents[0]) +
        dFabs(R[9] * Data->AABBExtents[1]) + 
        dFabs(R[10] * Data->AABBExtents[2]);

    aabb[0] = c[0] + pos[0] - xrange;
    aabb[1] = c[0] + pos[0] + xrange;
    aabb[2] = c[1] + pos[1] - yrange;
    aabb[3] = c[1] + pos[1] + yrange;
    aabb[4] = c[2] + pos[2] - zrange;
    aabb[5] = c[2] + pos[2] + zrange;
}

dGeomID dCreateTriMesh(dSpaceID space, 
		       dTriMeshDataID Data,
		       dTriCallback* Callback,
		       dTriArrayCallback* ArrayCallback,
		       dTriRayCallback* RayCallback)
{
    dxTriMesh* Geom = new dxTriMesh(space, Data);
    Geom->Callback = Callback;
    Geom->ArrayCallback = ArrayCallback;
    Geom->RayCallback = RayCallback;

    return Geom;
}

void dGeomTriMeshSetCallback(dGeomID g, dTriCallback* Callback)
{
	dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");
	((dxTriMesh*)g)->Callback = Callback;
}

dTriCallback* dGeomTriMeshGetCallback(dGeomID g)
{
	dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");
	return ((dxTriMesh*)g)->Callback;
}

void dGeomTriMeshSetArrayCallback(dGeomID g, dTriArrayCallback* ArrayCallback)
{
	dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");
	((dxTriMesh*)g)->ArrayCallback = ArrayCallback;
}

dTriArrayCallback* dGeomTriMeshGetArrayCallback(dGeomID g)
{
	dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");
	return ((dxTriMesh*)g)->ArrayCallback;
}

void dGeomTriMeshSetRayCallback(dGeomID g, dTriRayCallback* Callback)
{
	dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");
	((dxTriMesh*)g)->RayCallback = Callback;
}

dTriRayCallback* dGeomTriMeshGetRayCallback(dGeomID g)
{
	dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");	
	return ((dxTriMesh*)g)->RayCallback;
}

void dGeomTriMeshSetData(dGeomID g, dTriMeshDataID Data)
{
	dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");
	((dxTriMesh*)g)->Data = Data;
}

dTriMeshDataID dGeomTriMeshGetData(dGeomID g)
{
  dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");
  return ((dxTriMesh*)g)->Data;
}



void dGeomTriMeshEnableTC(dGeomID g, int geomClass, int enable)
{
	dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");
  
	switch (geomClass)
	{
		case dSphereClass: 
			((dxTriMesh*)g)->doSphereTC = (1 == enable);
			break;
		case dBoxClass:
			((dxTriMesh*)g)->doBoxTC = (1 == enable);
			break;
		case dCCylinderClass:
			((dxTriMesh*)g)->doCCylinderTC = (1 == enable);
			break;
	}
}

int dGeomTriMeshIsTCEnabled(dGeomID g, int geomClass)
{
	dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");
  
	switch (geomClass)
	{
		case dSphereClass:
			if (((dxTriMesh*)g)->doSphereTC)
				return 1;
			break;
		case dBoxClass:
			if (((dxTriMesh*)g)->doBoxTC)
				return 1;
			break;
		case dCCylinderClass:
			if (((dxTriMesh*)g)->doCCylinderTC)
				return 1;
			break;
	}
	return 0;
}

void dGeomTriMeshClearTCCache(dGeomID g){
    dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");

    dxTriMesh* Geom = (dxTriMesh*)g;
    Geom->ClearTCCache();
}

/*
 * returns the TriMeshDataID
 */
dTriMeshDataID
dGeomTriMeshGetTriMeshDataID(dGeomID g)
{
    dxTriMesh* Geom = (dxTriMesh*) g;
    return Geom->Data;
}

// Getting data
void dGeomTriMeshGetTriangle(dGeomID g, int Index, dVector3* v0, dVector3* v1, dVector3* v2){
    dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");

    dxTriMesh* Geom = (dxTriMesh*)g;

    const dVector3& Position = *(const dVector3*)dGeomGetPosition(g);
    const dMatrix3& Rotation = *(const dMatrix3*)dGeomGetRotation(g);

    dVector3 v[3];
	FetchTriangle(Geom, Index, Position, Rotation, v);

    if (v0){
        (*v0)[0] = v[0][0];
        (*v0)[1] = v[0][1];
        (*v0)[2] = v[0][2];
        (*v0)[3] = v[0][3];
    }
    if (v1){
        (*v1)[0] = v[1][0];
        (*v1)[1] = v[1][1];
        (*v1)[2] = v[1][2];
        (*v1)[3] = v[1][3];
    }
    if (v2){
        (*v2)[0] = v[2][0];
        (*v2)[1] = v[2][1];
        (*v2)[2] = v[2][2];
        (*v2)[3] = v[2][3];
    }
}

void dGeomTriMeshGetPoint(dGeomID g, int Index, dReal u, dReal v, dVector3 Out){
    dUASSERT(g && g->type == dTriMeshClass, "argument not a trimesh");

    dxTriMesh* Geom = (dxTriMesh*)g;

    const dVector3& Position = *(const dVector3*)dGeomGetPosition(g);
    const dMatrix3& Rotation = *(const dMatrix3*)dGeomGetRotation(g);

    dVector3 dv[3];
    FetchTriangle(Geom, Index, Position, Rotation, dv);

    GetPointFromBarycentric(dv, u, v, Out);
}

int dGeomTriMeshGetTriangleCount (dGeomID g)	 	
{	 	
    dxTriMesh* Geom = (dxTriMesh*)g;	 	
    return Geom->Data->Mesh.GetNbTriangles();	 	
}
