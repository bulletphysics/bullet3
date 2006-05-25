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
 * TriMesh code by Erwin de Vries.
 *
 * Trimesh data.
 * This is where the actual vertexdata (pointers), and BV tree is stored.
 * Vertices should be single precision!
 * This should be more sophisticated, so that the user can easyly implement
 * another collision library, but this is a lot of work, and also costs some
 * performance because some data has to be copied.
 */

#ifndef _ODE_COLLISION_TRIMESH_H_
#define _ODE_COLLISION_TRIMESH_H_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Data storage for triangle meshes.
 */
struct dxTriMeshData;
typedef struct dxTriMeshData* dTriMeshDataID;

/*
 * These dont make much sense now, but they will later when we add more
 * features.
 */
dTriMeshDataID dGeomTriMeshDataCreate(void);
void dGeomTriMeshDataDestroy(dTriMeshDataID g);

enum { TRIMESH_FACE_NORMALS, TRIMESH_LAST_TRANSFORMATION };
void dGeomTriMeshDataSet(dTriMeshDataID g, int data_id, void* in_data);
void* dGeomTriMeshDataGet(dTriMeshDataID g, int data_id);

/*
 * Build TriMesh data with single pricision used in vertex data .
 */
void dGeomTriMeshDataBuildSingle(dTriMeshDataID g,
                                 const void* Vertices, int VertexStride, int VertexCount, 
                                 const void* Indices, int IndexCount, int TriStride);
/* same again with a normals array (used as trimesh-trimesh optimization) */
void dGeomTriMeshDataBuildSingle1(dTriMeshDataID g,
                                  const void* Vertices, int VertexStride, int VertexCount, 
                                  const void* Indices, int IndexCount, int TriStride,
                                  const void* Normals);
/*
* Build TriMesh data with double pricision used in vertex data .
*/
void dGeomTriMeshDataBuildDouble(dTriMeshDataID g, 
                                 const void* Vertices,  int VertexStride, int VertexCount, 
                                 const void* Indices, int IndexCount, int TriStride);
/* same again with a normals array (used as trimesh-trimesh optimization) */
void dGeomTriMeshDataBuildDouble1(dTriMeshDataID g, 
                                  const void* Vertices,  int VertexStride, int VertexCount, 
                                  const void* Indices, int IndexCount, int TriStride,
                                  const void* Normals);

/*
 * Simple build. Single/double precision based on dSINGLE/dDOUBLE!
 */
void dGeomTriMeshDataBuildSimple(dTriMeshDataID g,
                                 const dReal* Vertices, int VertexCount,
                                 const int* Indices, int IndexCount);
/* same again with a normals array (used as trimesh-trimesh optimization) */
void dGeomTriMeshDataBuildSimple1(dTriMeshDataID g,
                                  const dReal* Vertices, int VertexCount,
                                  const int* Indices, int IndexCount,
                                  const int* Normals);
/*
 * Per triangle callback. Allows the user to say if he wants a collision with
 * a particular triangle.
 */
typedef int dTriCallback(dGeomID TriMesh, dGeomID RefObject, int TriangleIndex);
void dGeomTriMeshSetCallback(dGeomID g, dTriCallback* Callback);
dTriCallback* dGeomTriMeshGetCallback(dGeomID g);

/*
 * Per object callback. Allows the user to get the list of triangles in 1
 * shot. Maybe we should remove this one.
 */
typedef void dTriArrayCallback(dGeomID TriMesh, dGeomID RefObject, const int* TriIndices, int TriCount);
void dGeomTriMeshSetArrayCallback(dGeomID g, dTriArrayCallback* ArrayCallback);
dTriArrayCallback* dGeomTriMeshGetArrayCallback(dGeomID g);

/*
 * Ray callback.
 * Allows the user to say if a ray collides with a triangle on barycentric
 * coords. The user can for example sample a texture with alpha transparency
 * to determine if a collision should occur.
 */
typedef int dTriRayCallback(dGeomID TriMesh, dGeomID Ray, int TriangleIndex, dReal u, dReal v);
void dGeomTriMeshSetRayCallback(dGeomID g, dTriRayCallback* Callback);
dTriRayCallback* dGeomTriMeshGetRayCallback(dGeomID g);

/*
 * Trimesh class
 * Construction. Callbacks are optional.
 */
dGeomID dCreateTriMesh(dSpaceID space, dTriMeshDataID Data, dTriCallback* Callback, dTriArrayCallback* ArrayCallback, dTriRayCallback* RayCallback);

void dGeomTriMeshSetData(dGeomID g, dTriMeshDataID Data);
dTriMeshDataID dGeomTriMeshGetData(dGeomID g);


// enable/disable/check temporal coherence
void dGeomTriMeshEnableTC(dGeomID g, int geomClass, int enable);
int dGeomTriMeshIsTCEnabled(dGeomID g, int geomClass);

/*
 * Clears the internal temporal coherence caches. When a geom has its
 * collision checked with a trimesh once, data is stored inside the trimesh.
 * With large worlds with lots of seperate objects this list could get huge.
 * We should be able to do this automagically.
 */
void dGeomTriMeshClearTCCache(dGeomID g);


/*
 * returns the TriMeshDataID
 */
dTriMeshDataID dGeomTriMeshGetTriMeshDataID(dGeomID g);

/*
 * Gets a triangle.
 */
void dGeomTriMeshGetTriangle(dGeomID g, int Index, dVector3* v0, dVector3* v1, dVector3* v2);

/*
 * Gets the point on the requested triangle and the given barycentric
 * coordinates.
 */
void dGeomTriMeshGetPoint(dGeomID g, int Index, dReal u, dReal v, dVector3 Out);

/*

This is how the strided data works:

struct StridedVertex{
	dVector3 Vertex;
	// Userdata
};
int VertexStride = sizeof(StridedVertex);

struct StridedTri{
	int Indices[3];
	// Userdata
};
int TriStride = sizeof(StridedTri);

*/


int dGeomTriMeshGetTriangleCount (dGeomID g);


#ifdef __cplusplus
}
#endif

#endif	/* _ODE_COLLISION_TRIMESH_H_ */

