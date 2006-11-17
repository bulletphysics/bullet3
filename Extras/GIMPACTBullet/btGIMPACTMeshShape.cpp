/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

//#define DISABLE_BVH


#include "btGIMPACTMeshShape.h"
#include "GIMPACT/gimpact.h"

int g_gimpact_references = 0;
//Mechanism for initialize and terminate GIMPACT structures
void increase_gimpact_reference()
{
	g_gimpact_references++;

	if(g_gimpact_references >1 ) return;

	gimpact_init();

}

void decrease_gimpact_reference()
{
	if(g_gimpact_references <=0 ) return;
	g_gimpact_references--;
	if(g_gimpact_references >0 ) return;
	gimpact_terminate();
}

/////////////////////////// btGIMPACTMeshData class/////////////////////////////////////////////////////////////

void btGIMPACTMeshData::clearMeshParts()
{
	for(size_t i = 0;i<m_meshes.size();i++)
	{
		gim_trimesh_data_dec_ref(m_meshes[i]);
	}
	m_meshes.clear();
}

void btGIMPACTMeshData::addMeshPart(btStridingMeshInterface* meshInterface, int part)
{
		//Construct the trimesh
        // The buffer configuration

	const unsigned char *vertexbase;
	int numverts;
	PHY_ScalarType stype;
	int vertexStride;
	const unsigned char *indexbase;
	int indexstride;
	int numfaces;
	PHY_ScalarType indicestype;

	meshInterface->getLockedReadOnlyVertexIndexBase(&vertexbase, numverts,stype,vertexStride,&indexbase,
		indexstride,numfaces,indicestype,part);

	GUINT int_type;
	switch (indicestype)
	{
		case PHY_INTEGER:
			int_type = G_STYPE_INT;
			break;
		case PHY_SHORT:
			int_type = G_STYPE_SHORT;
			break;
	}

	GUINT vert_type;
	switch (stype)
	{
		case PHY_FLOAT:
			vert_type = G_STYPE_REAL;
			break;
		case PHY_DOUBLE:
			vert_type = G_STYPE_REAL2;
			break;
	}

    GBUFFER_ARRAY buffer_vertex_array;
    GBUFFER_ARRAY buffer_triindex_array;
	GUINT trimesh_data_handle;

	//Create shared buffer for indices

    gim_create_shared_buffer_from_data(
    vertexbase, numverts*vertexStride,
    &buffer_vertex_array.m_buffer_id);

    GIM_BUFFER_ARRAY_INIT_OFFSET_STRIDE(
    buffer_vertex_array,buffer_vertex_array.m_buffer_id,
    numverts,0,vertexStride);

	//Create shared buffer for vertices
    gim_create_shared_buffer_from_data(indexbase,
    numfaces*indexstride, &buffer_triindex_array.m_buffer_id);

    GIM_BUFFER_ARRAY_INIT_OFFSET_STRIDE(
    buffer_triindex_array,buffer_triindex_array.m_buffer_id,
    numfaces,0,indexstride);

	//Create the trimesh data
	gim_trimesh_data_create_from_arrays(
            &trimesh_data_handle,
            &buffer_vertex_array,vert_type,
            &buffer_triindex_array,int_type);

    //always call this after create a buffer_array
    GIM_BUFFER_ARRAY_DESTROY(buffer_vertex_array);
    GIM_BUFFER_ARRAY_DESTROY(buffer_triindex_array);

    //Build Bounding volume tree
    gim_trimesh_data_build_aabbtree(trimesh_data_handle);

	gim_trimesh_data_inc_ref(trimesh_data_handle);

	m_meshes.push_back(trimesh_data_handle);

}

void btGIMPACTMeshData::processMeshParts(btStridingMeshInterface* meshInterface)
{
	clearMeshParts();
	this->m_meshInterface = meshInterface;
	int meshcount = meshInterface->getNumSubParts();
	for(int i = 0;i<meshcount;i++)
	{
		addMeshPart(meshInterface,i);
	}
}


btGIMPACTMeshData::btGIMPACTMeshData(btStridingMeshInterface* meshInterface)
{
	increase_gimpact_reference();
	processMeshParts(meshInterface);
}

btGIMPACTMeshData::~btGIMPACTMeshData()
{
	clearMeshParts();
	decrease_gimpact_reference();
}

/////////////////////////// btGIMPACTMeshShape class/////////////////////////////////////////////////////////////

void btGIMPACTMeshShape::clearMeshParts()
{
	GIM_TRIMESH * ptrimesh;
	for(size_t i = 0;i<m_gim_trimesh_parts.size();i++)
	{
		ptrimesh = (GIM_TRIMESH * )m_gim_trimesh_parts[i];
		gim_trimesh_destroy(ptrimesh);
		gim_free(ptrimesh,0);
	}

	m_gim_trimesh_parts.clear();
}

void btGIMPACTMeshShape::processMeshParts(btGIMPACTMeshData * meshdata)
{
	clearMeshParts();
	this->m_meshdata = meshdata;

	BT_GIMPACT_TRIMESH_HANDLE gimhandle;
	GIM_TRIMESH * ptrimesh;

	for(size_t i = 0;i<m_meshdata->m_meshes.size();i++)
	{
		ptrimesh = (GIM_TRIMESH *)gim_alloc(sizeof(GIM_TRIMESH));
		gim_trimesh_create(ptrimesh,m_meshdata->m_meshes[i],1,0);
		gimhandle = (BT_GIMPACT_TRIMESH_HANDLE) ptrimesh;
		m_gim_trimesh_parts.push_back(gimhandle);
	}
}

btGIMPACTMeshShape::btGIMPACTMeshShape(btGIMPACTMeshData * meshdata)
{
	m_scale.setValue(1.0f,1.0f,1.0f);
	processMeshParts(meshdata);

}

btGIMPACTMeshShape::~btGIMPACTMeshShape()
{
	clearMeshParts();
}

void btGIMPACTMeshShape::prepareMeshes(const btTransform & trans) const
{
	mat4f gim_trans;
	IDENTIFY_MATRIX_4X4(gim_trans);
	COPY_MATRIX_3X3(gim_trans,trans.getBasis());

	btVector3 scaling = getLocalScaling();

	SCALE_VEC_MATRIX_3X3(gim_trans,scaling,gim_trans);
	MAT_SET_TRANSLATION(gim_trans,trans.getOrigin());

	GIM_TRIMESH * ptrimesh;

	for(size_t i = 0;i<m_gim_trimesh_parts.size();i++)
	{
		ptrimesh = (GIM_TRIMESH * )m_gim_trimesh_parts[i];
		gim_trimesh_set_tranform(ptrimesh, gim_trans);
	}
}



///getAabb returns the axis aligned bounding box in the coordinate frame of the given transform t.
void btGIMPACTMeshShape::getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const
{
	prepareMeshes(t);

	aabb3f meshbox, global_box;

	INVALIDATE_AABB(global_box);

	GIM_TRIMESH * ptrimesh;

	for(size_t i = 0;i<m_gim_trimesh_parts.size();i++)
	{
		ptrimesh = (GIM_TRIMESH * )m_gim_trimesh_parts[i];
		gim_trimesh_get_aabb(ptrimesh,&meshbox);
		MERGEBOXES(global_box,meshbox);
	}

	aabbMin[0] = global_box.minX;
	aabbMin[1] = global_box.minY;
	aabbMin[2] = global_box.minZ;

	aabbMax[0] = global_box.maxX;
	aabbMax[1] = global_box.maxY;
	aabbMax[2] = global_box.maxZ;

}

void btGIMPACTMeshShape::setLocalScaling(const btVector3& scaling)
{
	m_scale = scaling;
}

const btVector3& btGIMPACTMeshShape::getLocalScaling() const
{
	return m_scale ;
}


void btGIMPACTMeshShape::calculateLocalInertia(btScalar mass,btVector3& inertia)
{
	btTransform t;
	t.setIdentity();

	btVector3 aabbMin;
	btVector3 aabbMax;

	getAabb(t,aabbMin,aabbMax);

	//not yet, return box inertia

	btVector3 halfExtents = (aabbMax-aabbMin)*0.5f;

	btScalar lx=2.f*(halfExtents.x());
	btScalar ly=2.f*(halfExtents.y());
	btScalar lz=2.f*(halfExtents.z());
	const btScalar x2 = lx*lx;
	const btScalar y2 = ly*ly;
	const btScalar z2 = lz*lz;
	const btScalar scaledmass = mass * 0.08333333f;

	inertia = scaledmass * (btVector3(y2+z2,x2+z2,x2+y2));

}



void	btGIMPACTMeshShape::processAllTriangles(btTriangleCallback* callback,const btVector3& aabbMin,const btVector3& aabbMax) const
{

	/*btTransform t;
	t.setIdentity();
	prepareMeshes(t);*/

	GDYNAMIC_ARRAY collision_result;

	GIM_TRIMESH * ptrimesh;
	size_t i,j;
	vec3f trivec[3];
	btVector3 btrivec[3];
	GUINT * boxesresult;
	btVector3 scalevec = this->getLocalScaling();

	aabb3f test_aabb;
	test_aabb.minX = aabbMin[0]/scalevec[0];
	test_aabb.minY = aabbMin[1]/scalevec[1];
	test_aabb.minZ = aabbMin[2]/scalevec[2];

	test_aabb.maxX = aabbMax[0]/scalevec[0];
	test_aabb.maxY = aabbMax[1]/scalevec[1];
	test_aabb.maxZ = aabbMax[2]/scalevec[2];

	for(i = 0;i<m_gim_trimesh_parts.size();i++)
	{
		ptrimesh = (GIM_TRIMESH * )m_gim_trimesh_parts[i];

		GIM_CREATE_BOXQUERY_LIST(collision_result);

		gim_trimesh_midphase_box_collision_local(ptrimesh,&test_aabb,&collision_result);

		boxesresult = GIM_DYNARRAY_POINTER(GUINT,collision_result);

		//collide triangles
		//Locks trimesh
		gim_trimesh_locks_work_data(ptrimesh);

		for(j=0;j<collision_result.m_size;j++)
		{
			gim_trimesh_get_triangle_vertices_local(ptrimesh,boxesresult[j],trivec[0],trivec[1],trivec[2]);

			btrivec[0].setValue(trivec[0][0]*scalevec[0],trivec[0][1]*scalevec[1],trivec[0][2]*scalevec[2]);
			btrivec[1].setValue(trivec[1][0]*scalevec[0],trivec[1][1]*scalevec[1],trivec[1][2]*scalevec[2]);
			btrivec[2].setValue(trivec[2][0]*scalevec[0],trivec[2][1]*scalevec[1],trivec[2][2]*scalevec[2]);

			callback->processTriangle(btrivec,(int)i,(int)boxesresult[j]);
		}

		///unlocks
		gim_trimesh_unlocks_work_data(ptrimesh);


	    GIM_DYNARRAY_DESTROY(collision_result);
	}

}


