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

/*
Author: Francisco León Nájera
Concave-Concave Collision

*/

#include "BulletCollision/CollisionDispatch/btManifoldResult.h"
#include "LinearMath/btIDebugDraw.h"
#include "btConcaveConcaveCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "btGIMPACTMeshShape.h"
#include "BulletCollision/CollisionShapes/btStaticPlaneShape.h"
#include "GIMPACT/gimpact.h"



//! Class for accessing the plane ecuation
class btPlaneShape : public btStaticPlaneShape
{
public:
	void get_plane_equation(float equation[4])
	{
		equation[0] = m_planeNormal[0];
		equation[1] = m_planeNormal[1];
		equation[2] = m_planeNormal[2];
		equation[3] = m_planeConstant;
	}


	void get_plane_equation_transformed(const btTransform & trans,float equation[4])
	{
		/*mat4f plane_trans;
		IDENTIFY_MATRIX_4X4(plane_trans);
		COPY_MATRIX_3X3(plane_trans,trans.getBasis());
		MAT_SET_TRANSLATION(plane_trans,trans.getOrigin());

		float ptemp[4]
		//vec4f ptemp;
		get_plane_equation(ptemp);

		MAT_TRANSFORM_PLANE_4X4(equation,plane_trans,ptemp);*/

		equation[0] = trans.getBasis().getRow(0).dot(m_planeNormal);
		equation[1] = trans.getBasis().getRow(1).dot(m_planeNormal);
		equation[2] = trans.getBasis().getRow(2).dot(m_planeNormal);
		equation[3] = trans.getOrigin().dot(m_planeNormal) + m_planeConstant;
	}
};

btConcaveConcaveCollisionAlgorithm::btConcaveConcaveCollisionAlgorithm( const btCollisionAlgorithmConstructionInfo& ci, btCollisionObject* body0,btCollisionObject* body1)
: btCollisionAlgorithm(ci)
{
}

btConcaveConcaveCollisionAlgorithm::~btConcaveConcaveCollisionAlgorithm()
{
    clearCache();
}

void btConcaveConcaveCollisionAlgorithm::clearCache()
{
    btPersistentManifold* mainfold;
    for (size_t i=0;i<this->m_mainfoldsPtr.size() ; i++)
    {
    	mainfold = m_mainfoldsPtr[i];
    	m_dispatcher->releaseManifold(mainfold);
    }
	m_mainfoldsPtr.clear();
}

btPersistentManifold* btConcaveConcaveCollisionAlgorithm::newContactMainfold(btCollisionObject* body0,btCollisionObject* body1)
{
    btPersistentManifold* newmainfold;
    newmainfold = m_dispatcher->getNewManifold(body0,body1);
    m_mainfoldsPtr.push_back(newmainfold);
    return newmainfold;
}

void process_gimpact_contacts(GIM_CONTACT * pcontacts,
					  int contact_count,
					  btConcaveConcaveCollisionAlgorithm * algorithm,
					  btCollisionObject* body0,
					  btCollisionObject* body1,
					  btManifoldResult* resultOut, bool swapped = false)
{
		int i, ci = MANIFOLD_CACHE_SIZE;//Max point size
		btPersistentManifold * current_mainfold = 0;

		btCollisionObject* pbody0 = swapped?body1:body0;
		btCollisionObject* pbody1 = swapped?body0:body1;

		float csign = swapped?-1.f:1.f;

		btVector3 cpoint;
		btVector3 cnormal;

		for(i=0;i<contact_count;i++)
		{
			if(ci>=MANIFOLD_CACHE_SIZE)
			{
				current_mainfold = algorithm->newContactMainfold(pbody0,pbody1);
				resultOut->setPersistentManifold(current_mainfold);
				ci=0;
			}

			cpoint.setValue(pcontacts->m_point[0],pcontacts->m_point[1],pcontacts->m_point[2]);
			//Normal points to body0
			cnormal.setValue(csign*pcontacts->m_normal[0],csign*pcontacts->m_normal[1],csign*pcontacts->m_normal[2]);

			resultOut->addContactPoint(cnormal,cpoint,-pcontacts->m_depth);

			pcontacts++;
			ci++;
		}
}


void process_gimpact_plane_contacts(vec4f * pcontacts,
									vec4f planenormal,
					  int contact_count,
					  btConcaveConcaveCollisionAlgorithm * algorithm,
					  btCollisionObject* body0,
					  btCollisionObject* body1,
					  btManifoldResult* resultOut, bool swapped = false)
{
		int i, ci = MANIFOLD_CACHE_SIZE;//Max point size
		btPersistentManifold * current_mainfold = 0;

		btCollisionObject* pbody0 = swapped?body1:body0;
		btCollisionObject* pbody1 = swapped?body0:body1;

		float csign = swapped?-1.f:1.f;

		btVector3 cpoint;
		btVector3 cnormal;

		for(i=0;i<contact_count;i++)
		{
			if(ci>=MANIFOLD_CACHE_SIZE)
			{
				current_mainfold = algorithm->newContactMainfold(pbody0,pbody1);
				resultOut->setPersistentManifold(current_mainfold);
				ci=0;
			}

			cpoint.setValue(pcontacts[i][0],pcontacts[i][1],pcontacts[i][2]);
			//Normal points to body0
			cnormal.setValue(csign*planenormal[0],csign*planenormal[1],csign*planenormal[2]);

			resultOut->addContactPoint(cnormal,cpoint,-pcontacts[i][3]);

			ci++;
		}
}

class CONCAVE_TRIANGLE_TOKEN
{
public:
	GIM_TRIANGLE_DATA m_tridata;
	int partId;
	int	triangleIndex;

	CONCAVE_TRIANGLE_TOKEN()
	{
		m_tridata.m_has_planes = 0;
		partId = 0;
		triangleIndex = 0;
	}

	CONCAVE_TRIANGLE_TOKEN(const CONCAVE_TRIANGLE_TOKEN & token)
	{
		m_tridata.m_has_planes = 0;
		VEC_COPY(m_tridata.m_vertices[0],token.m_tridata.m_vertices[0]);
		VEC_COPY(m_tridata.m_vertices[1],token.m_tridata.m_vertices[1]);
		VEC_COPY(m_tridata.m_vertices[2],token.m_tridata.m_vertices[2]);
		partId = token.partId;
		triangleIndex = token.triangleIndex;
	}

};




void bt_gimpact_gimpact_collision(btConcaveConcaveCollisionAlgorithm * algorithm,
					  btCollisionObject* body0,
					  btCollisionObject* body1,
					  btManifoldResult* resultOut)
{
	btGIMPACTMeshShape*	tri0b = static_cast<btGIMPACTMeshShape*>( body0->getCollisionShape());
    btGIMPACTMeshShape*	tri1b = static_cast<btGIMPACTMeshShape*>( body1->getCollisionShape());

	tri0b->prepareMeshes(body0->getWorldTransform());
	tri1b->prepareMeshes(body1->getWorldTransform());

	size_t i,j;
	size_t parts0 = tri0b->m_gim_trimesh_parts.size();
	size_t parts1 = tri1b->m_gim_trimesh_parts.size();

	GIM_TRIMESH * trimesh0;
	GIM_TRIMESH * trimesh1;
	GDYNAMIC_ARRAY contacts;
	GIM_CONTACT * pcontacts;
	for(i=0;i<parts0;i++)
	{
		for(j=0;j<parts1;j++)
		{
			trimesh0 = (GIM_TRIMESH * )tri0b->m_gim_trimesh_parts[i];
			trimesh1 = (GIM_TRIMESH * )tri1b->m_gim_trimesh_parts[j];

			GIM_CREATE_CONTACT_LIST(contacts);

			gim_trimesh_trimesh_collision(trimesh0,trimesh1,&contacts);

			if(contacts.m_size>0)
			{
				pcontacts = GIM_DYNARRAY_POINTER(GIM_CONTACT,contacts);
				process_gimpact_contacts(pcontacts,contacts.m_size,algorithm,body0,body1,resultOut);
			}
			GIM_DYNARRAY_DESTROY(contacts);
		}
	}
}

void bt_gimpact_plane_collision(btConcaveConcaveCollisionAlgorithm * algorithm,
					  btCollisionObject* tribody0,
					  btCollisionObject* planebody1,
					  btManifoldResult* resultOut,bool swapped)
{
	btGIMPACTMeshShape*	tri0b = static_cast<btGIMPACTMeshShape*>( tribody0->getCollisionShape());
    btPlaneShape *	plane1b = static_cast<btPlaneShape *>( planebody1->getCollisionShape());

	tri0b->prepareMeshes(tribody0->getWorldTransform());

	////////////////////////////////Getting plane////////////////////////////////////

	vec4f pnormal;
	plane1b->get_plane_equation_transformed(planebody1->getWorldTransform(),pnormal);

	////////////////////////////////End Getting plane////////////////////////////////////

	size_t i;
	size_t parts0 = tri0b->m_gim_trimesh_parts.size();


	GIM_TRIMESH * trimesh0;

	GDYNAMIC_ARRAY contacts;
	vec4f * pcontacts;
	for(i=0;i<parts0;i++)
	{
		trimesh0 = (GIM_TRIMESH * )tri0b->m_gim_trimesh_parts[i];

		GIM_CREATE_TRIMESHPLANE_CONTACTS(contacts);

		gim_trimesh_plane_collision(trimesh0,pnormal,&contacts);

		if(contacts.m_size>0)
		{
			pcontacts = GIM_DYNARRAY_POINTER(vec4f,contacts);

			process_gimpact_plane_contacts(pcontacts,pnormal,
				contacts.m_size,algorithm,tribody0,planebody1,resultOut,swapped);
		}
		GIM_DYNARRAY_DESTROY(contacts);
	}
}



///For each triangle in the concave mesh that overlaps with the AABB of a convex (m_convexProxy), processTriangle is called.
class btConcaveTriangleCallback : public btTriangleCallback
{

public:
	btCollisionObject* m_body;
	mat4f m_transform;
	std::vector<CONCAVE_TRIANGLE_TOKEN> m_triangles;

	btConcaveTriangleCallback(btCollisionObject* body)
	{
		m_body = body;
		IDENTIFY_MATRIX_4X4(m_transform);
		COPY_MATRIX_3X3(m_transform,body->getWorldTransform().getBasis());
		MAT_SET_TRANSLATION(m_transform,body->getWorldTransform().getOrigin());
		m_triangles.reserve(100);
	}

	void	setTimeStepAndCounters(float collisionMarginTriangle,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
	{}

	virtual ~btConcaveTriangleCallback(){
	}

	virtual void processTriangle(btVector3* triangle, int partId, int triangleIndex)
	{
		CONCAVE_TRIANGLE_TOKEN token;

		token.m_tridata.m_has_planes = 0;
		token.partId = partId;
		token.triangleIndex = triangleIndex;
		//Copy vertices
		MAT_DOT_VEC_3X4(token.m_tridata.m_vertices[0],m_transform,triangle[0]);
		MAT_DOT_VEC_3X4(token.m_tridata.m_vertices[1],m_transform,triangle[1]);
		MAT_DOT_VEC_3X4(token.m_tridata.m_vertices[2],m_transform,triangle[2]);

		m_triangles.push_back(token);
	}

	void clearCache(){}
};



void bt_concave_concave_collision(btConcaveConcaveCollisionAlgorithm * algorithm,
					  btCollisionObject* tribody0,
					  btCollisionObject* tribody1,
					  btManifoldResult* resultOut)
{
	btConcaveShape*	tri0b = static_cast<btConcaveShape*>( tribody0->getCollisionShape());
	btConcaveShape*	tri1b = static_cast<btConcaveShape*>( tribody1->getCollisionShape());

	//Get First AABB
	btVector3 aabbMin0,aabbMax0;
	tri0b->getAabb(tribody0->getWorldTransform(),aabbMin0,aabbMax0);

	//Get Second AABB
	btVector3 aabbMin1,aabbMax1;
	tri1b->getAabb(tribody1->getWorldTransform(),aabbMin1,aabbMax1);

	//Transform boxes to local spaces
	aabb3f aabb0 = {
		aabbMin0[0],aabbMax0[0],
		aabbMin0[1],aabbMax0[1],
		aabbMin0[2],aabbMax0[2],
	};

	aabb3f aabb1 = {
		aabbMin1[0],aabbMax1[0],
		aabbMin1[1],aabbMax1[1],
		aabbMin1[2],aabbMax1[2],
	};

	mat4f transform;
	IDENTIFY_MATRIX_4X4(transform);

	// body0 inverse transform
	btTransform transinv = tribody0->getWorldTransform().inverse();
	COPY_MATRIX_3X3(transform,transinv.getBasis());
	MAT_SET_TRANSLATION(transform,transinv.getOrigin());

	//Transform box1 to body0 space
	AABB_TRANSFORM(aabb1,aabb1,transform);

	AABB_GET_MIN(aabb1,aabbMin1);
	AABB_GET_MAX(aabb1,aabbMax1);

	btConcaveTriangleCallback callback0(tribody0);
	tri0b->processAllTriangles(&callback0,aabbMin1,aabbMax1);

	if(callback0.m_triangles.size()==0) return;
	// body1 inverse transform
	transinv = tribody1->getWorldTransform().inverse();
	COPY_MATRIX_3X3(transform,transinv.getBasis());
	MAT_SET_TRANSLATION(transform,transinv.getOrigin());

	//Transform box0 to body1 space
	AABB_TRANSFORM(aabb0,aabb0,transform);

	AABB_GET_MIN(aabb0,aabbMin0);
	AABB_GET_MAX(aabb0,aabbMax0);

	btConcaveTriangleCallback callback1(tribody1);
	tri1b->processAllTriangles(&callback1,aabbMin0,aabbMax0);
	if(callback1.m_triangles.size()==0) return;

	////////////////////////////////Collide triangles////////////////////////////////////

	//dummy contacts
    GDYNAMIC_ARRAY dummycontacts;
    GIM_CREATE_CONTACT_LIST(dummycontacts);

    //Auxiliary triangle data
    GIM_TRIANGLE_CONTACT_DATA tri_contact_data;

	size_t i,j,ci;
	int colresult;

	for(i=0;i<callback0.m_triangles.size();i++)
	{
		for(j=0;j<callback1.m_triangles.size();j++)
		{

			//collide triangles
			colresult = gim_triangle_triangle_collision(
				&callback0.m_triangles[i].m_tridata,
				&callback1.m_triangles[j].m_tridata,&tri_contact_data);
			if(colresult == 1)
			{
				//Add contacts
				for (ci=0;ci<tri_contact_data.m_point_count ;ci++ )
				{
					GIM_PUSH_CONTACT(dummycontacts, tri_contact_data.m_points[ci],tri_contact_data.m_separating_normal ,tri_contact_data.m_penetration_depth,tribody0, tribody1, callback0.m_triangles[i].triangleIndex, callback1.m_triangles[j].triangleIndex);
				}
			}
		}
	}

	if(dummycontacts.m_size == 0) //reject
    {
        GIM_DYNARRAY_DESTROY(dummycontacts);
        return;
    }

	//dummy contacts
    GDYNAMIC_ARRAY contacts;
    GIM_CREATE_CONTACT_LIST(contacts);

    //merge contacts
    gim_merge_contacts(&dummycontacts,&contacts);

	GIM_CONTACT * pcontacts = GIM_DYNARRAY_POINTER(GIM_CONTACT,contacts);
	process_gimpact_contacts(pcontacts,contacts.m_size,algorithm,tribody0,tribody1,resultOut);

    //Terminate
    GIM_DYNARRAY_DESTROY(dummycontacts);
    GIM_DYNARRAY_DESTROY(contacts);
}


void btConcaveConcaveCollisionAlgorithm::processCollision (btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
{
    clearCache();
	if (body0->getCollisionShape()->getShapeType()==GIMPACT_SHAPE_PROXYTYPE && body1->getCollisionShape()->getShapeType()==GIMPACT_SHAPE_PROXYTYPE )
	{
		bt_gimpact_gimpact_collision(this,body0,body1,resultOut);
	}
	else if (body0->getCollisionShape()->getShapeType()==STATIC_PLANE_PROXYTYPE&& body1->getCollisionShape()->getShapeType()==GIMPACT_SHAPE_PROXYTYPE )
	{
		bt_gimpact_plane_collision(this,body1,body0,resultOut,true);
	}
	else if (body0->getCollisionShape()->getShapeType()==GIMPACT_SHAPE_PROXYTYPE&& body1->getCollisionShape()->getShapeType()==STATIC_PLANE_PROXYTYPE)
	{
		bt_gimpact_plane_collision(this,body0,body1,resultOut,false);
	}
	else if(body0->getCollisionShape()->isConcave() && body1->getCollisionShape()->isConcave() )
	{
		bt_concave_concave_collision(this,body0,body1,resultOut);
	}
}


float btConcaveConcaveCollisionAlgorithm::calculateTimeOfImpact(btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
{
	return 1.f;

}

///////////////////////////////////// REGISTERING ALGORITHM //////////////////////////////////////////////


//! Use this function for register the algorithm externally
void btConcaveConcaveCollisionAlgorithm::registerAlgorithm(btCollisionDispatcher * dispatcher)
{
	dispatcher->registerCollisionCreateFunc(GIMPACT_SHAPE_PROXYTYPE,GIMPACT_SHAPE_PROXYTYPE ,new btConcaveConcaveCollisionAlgorithm::CreateFunc);
	dispatcher->registerCollisionCreateFunc(GIMPACT_SHAPE_PROXYTYPE,STATIC_PLANE_PROXYTYPE ,new btConcaveConcaveCollisionAlgorithm::CreateFunc);
	dispatcher->registerCollisionCreateFunc(STATIC_PLANE_PROXYTYPE,GIMPACT_SHAPE_PROXYTYPE ,new btConcaveConcaveCollisionAlgorithm::CreateFunc);
	dispatcher->registerCollisionCreateFunc(GIMPACT_SHAPE_PROXYTYPE,TRIANGLE_MESH_SHAPE_PROXYTYPE,new btConcaveConcaveCollisionAlgorithm::CreateFunc);
	dispatcher->registerCollisionCreateFunc(TRIANGLE_MESH_SHAPE_PROXYTYPE,GIMPACT_SHAPE_PROXYTYPE,new btConcaveConcaveCollisionAlgorithm::CreateFunc);
	dispatcher->registerCollisionCreateFunc(STATIC_PLANE_PROXYTYPE,TRIANGLE_MESH_SHAPE_PROXYTYPE,new btConcaveConcaveCollisionAlgorithm::CreateFunc);
	dispatcher->registerCollisionCreateFunc(TRIANGLE_MESH_SHAPE_PROXYTYPE,STATIC_PLANE_PROXYTYPE,new btConcaveConcaveCollisionAlgorithm::CreateFunc);
	dispatcher->registerCollisionCreateFunc(TRIANGLE_MESH_SHAPE_PROXYTYPE,TRIANGLE_MESH_SHAPE_PROXYTYPE,new btConcaveConcaveCollisionAlgorithm::CreateFunc);
}
