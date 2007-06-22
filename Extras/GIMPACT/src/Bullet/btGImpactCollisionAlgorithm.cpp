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
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "GIMPACT/Bullet/btGImpactCollisionAlgorithm.h"
#include "GIMPACT/core/gim_contact.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"

#define BULLET_TRIANGLE_COLLISION 1
#define TREE_PRIMITIVE_VS_BOX true
#define GIMPACT_VS_PLANE_COLLISION 1


//! Class for accessing the plane equation
class btPlaneShape : public btStaticPlaneShape
{
public:

	btPlaneShape(const btVector3& v, float f)
		:btStaticPlaneShape(v,f)
	{
	}

	void get_plane_equation(btVector4 &equation)
	{
		equation[0] = m_planeNormal[0];
		equation[1] = m_planeNormal[1];
		equation[2] = m_planeNormal[2];
		equation[3] = m_planeConstant;
	}


	void get_plane_equation_transformed(const btTransform & trans,btVector4 &equation)
	{
		equation[0] = trans.getBasis().getRow(0).dot(m_planeNormal);
		equation[1] = trans.getBasis().getRow(1).dot(m_planeNormal);
		equation[2] = trans.getBasis().getRow(2).dot(m_planeNormal);
		equation[3] = trans.getOrigin().dot(m_planeNormal) + m_planeConstant;
	}
};

btGImpactCollisionAlgorithm::btGImpactCollisionAlgorithm( const btCollisionAlgorithmConstructionInfo& ci, btCollisionObject* body0,btCollisionObject* body1)
: btCollisionAlgorithm(ci)
{
	m_manifoldPtr = NULL;
	m_convex_algorithm = NULL;
}

btGImpactCollisionAlgorithm::~btGImpactCollisionAlgorithm()
{
	clearCache();
}



//////////////////////////////////////////////////////////////////////////////////////////////


void btGImpactCollisionAlgorithm::gimpactcompound_vs_gimpactcompound_find_pairs(
					  const btTransform & trans0,
					  const btTransform & trans1,
					  btGImpactCompoundShape * shape0,
					  btGImpactCompoundShape * shape1,gim_pair_set & pairset) const
{
	GIM_TREE_TREE_COLLIDER<btGImpactCompoundShape::BoxSetClass,btGImpactCompoundShape::BoxSetClass> collider;
	collider.find_collision(shape0->getBoxSet(),trans0,shape1->getBoxSet(),trans1,pairset,TREE_PRIMITIVE_VS_BOX);

}

void btGImpactCollisionAlgorithm::gimpacttrimeshpart_vs_gimpacttrimeshpart_find_pairs(
					  const btTransform & trans0,
					  const btTransform & trans1,
					  btGImpactMeshShapePart * shape0,
					  btGImpactMeshShapePart * shape1,gim_pair_set & pairset) const
{
	GIM_TREE_TREE_COLLIDER<btGImpactMeshShapePart::BoxSetClass,btGImpactMeshShapePart::BoxSetClass> collider;

	collider.find_collision(shape0->getBoxSet(),trans0,shape1->getBoxSet(),trans1,pairset,TREE_PRIMITIVE_VS_BOX);

}

void btGImpactCollisionAlgorithm::gimpactcompound_vs_gimpacttrimeshpart_find_pairs(
					  const btTransform & trans0,
					  const btTransform & trans1,
					  btGImpactCompoundShape * shape0,
					  btGImpactMeshShapePart * shape1,gim_pair_set & pairset) const
{
	GIM_TREE_TREE_COLLIDER<btGImpactCompoundShape::BoxSetClass,btGImpactMeshShapePart::BoxSetClass> collider;
	collider.find_collision(shape0->getBoxSet(),trans0,shape1->getBoxSet(),trans1,pairset,TREE_PRIMITIVE_VS_BOX);
}




void btGImpactCollisionAlgorithm::shape_vs_shape_collision(
					  btCollisionObject * body0,
					  btCollisionObject * body1,
					  btCollisionShape * shape0,
					  btCollisionShape * shape1,bool swapped)
{

	btCollisionShape * orgshape0 = body0->getCollisionShape();
	btCollisionShape * orgshape1 = body1->getCollisionShape();

	body0->setCollisionShape(shape0);
	body1->setCollisionShape(shape1);


	if(swapped)
	{
		btCollisionAlgorithm* algorswapped = newAlgorithm(body1,body0);

		m_resultOut->setPersistentManifold(m_manifoldPtr);
		m_resultOut->setShapeIdentifiers(m_part1,m_triface1,m_part0,m_triface0);

		algorswapped->processCollision(body1,body0,*m_dispatchInfo,m_resultOut);

		delete algorswapped;
	}
	else
	{
		btCollisionAlgorithm* algor = newAlgorithm(body0,body1);

		m_resultOut->setPersistentManifold(m_manifoldPtr);
		m_resultOut->setShapeIdentifiers(m_part0,m_triface0,m_part1,m_triface1);

		algor->processCollision(body0,body1,*m_dispatchInfo,m_resultOut);

		delete algor;
	}

	body0->setCollisionShape(orgshape0);
	body1->setCollisionShape(orgshape1);
}

void btGImpactCollisionAlgorithm::convex_vs_convex_collision(
					  btCollisionObject * body0,
					  btCollisionObject * body1,
					  btCollisionShape * shape0,
					  btCollisionShape * shape1)
{
	//shape_vs_shape_collision(body0,body1,shape0,shape1,false);
	//return;

	btCollisionShape * orgshape0 = body0->getCollisionShape();
	btCollisionShape * orgshape1 = body1->getCollisionShape();

	body0->setCollisionShape(shape0);
	body1->setCollisionShape(shape1);


	m_resultOut->setShapeIdentifiers(m_part0,m_triface0,m_part1,m_triface1);

	checkConvexAlgorithm(body0,body1);
	m_convex_algorithm->processCollision(body0,body1,*m_dispatchInfo,m_resultOut);

	body0->setCollisionShape(orgshape0);
	body1->setCollisionShape(orgshape1);

}



void btGImpactCollisionAlgorithm::gimpacttrimesh_vs_shape_collision(
					  btCollisionObject * body0,
					  btCollisionObject * body1,
					  btGImpactMeshShape * shape0,
					  btCollisionShape * shape1,bool swapped)
{
	GUINT i = shape0->getMeshPartCount();
	while(i--)
	{
		btGImpactMeshShapePart * part = shape0->getMeshPart(i);
		gimpacttrimeshpart_vs_shape_collision(body0,body1,part,shape1,swapped);
	}
}

void btGImpactCollisionAlgorithm::gimpacttrimesh_vs_gimpacttrimesh(
					  btCollisionObject * body0,
					  btCollisionObject * body1,
					  btGImpactMeshShape * shape0,
					  btGImpactMeshShape  * shape1)
{
	GUINT i = shape0->getMeshPartCount();
	while(i--)
	{
		btGImpactMeshShapePart * part0 = shape0->getMeshPart(i);

		GUINT j = shape1->getMeshPartCount();
		while(j--)
		{
			btGImpactMeshShapePart * part1 = shape1->getMeshPart(j);
			gimpacttrimeshpart_vs_gimpacttrimeshpart_collision(body0,body1,part0,part1,false);
		}
	}
}

void btGImpactCollisionAlgorithm::gimpacttrimesh_vs_gimpactcompound(
					  btCollisionObject * body0,
					  btCollisionObject * body1,
					  btGImpactMeshShape * shape0,
					  btGImpactCompoundShape * shape1,bool swapped)
{
	GUINT i = shape0->getMeshPartCount();
	while(i--)
	{
		btGImpactMeshShapePart * part = shape0->getMeshPart(i);
		gimpactcompound_vs_gimpacttrimeshpart_collision(body1,body0,shape1,part,!swapped);
	}
}



void btGImpactCollisionAlgorithm::gimpacttrimesh_vs_trimeshpart(
					  btCollisionObject * body0,
					  btCollisionObject * body1,
					  btGImpactMeshShape * shape0,
					  btGImpactMeshShapePart * shape1,bool swapped)
{
	GUINT i = shape0->getMeshPartCount();
	while(i--)
	{
		btGImpactMeshShapePart * part = shape0->getMeshPart(i);
		gimpacttrimeshpart_vs_gimpacttrimeshpart_collision(body0,body1,part,shape1,swapped);
	}
}


void btGImpactCollisionAlgorithm::gimpactcompound_vs_gimpactcompound_collision(
					  btCollisionObject * body0,
					  btCollisionObject * body1,
					  btGImpactCompoundShape * shape0,
					  btGImpactCompoundShape * shape1)
{
	btTransform orgtrans0 = body0->getWorldTransform();
	btTransform orgtrans1 = body1->getWorldTransform();

	gim_pair_set pairset;

	gimpactcompound_vs_gimpactcompound_find_pairs(orgtrans0,orgtrans1,shape0,shape1,pairset);	

	if(pairset.size()== 0) return;

//	btCollisionShape * orgshape0 = body0->getCollisionShape();
//	btCollisionShape * orgshape1 = body1->getCollisionShape();

	GUINT i = pairset.size();
	while(i--)
	{
		const GIM_PAIR & pair = pairset[i];
		btCollisionShape * colshape0 = shape0->getChildShape(pair.m_index1);
		btCollisionShape * colshape1 = shape1->getChildShape(pair.m_index2);

		btTransform childtrans0 = orgtrans0*shape0->getChildTransform(pair.m_index1);
		btTransform childtrans1 = orgtrans1*shape1->getChildTransform(pair.m_index2);

		body0->setWorldTransform(childtrans0);
		body1->setWorldTransform(childtrans1);


		//collide two shapes
		shape_vs_shape_collision(body0,body1,colshape0,colshape1,false);

		//restore transforms
//		body0->setCollisionShape(orgshape0);
//		body1->setCollisionShape(orgshape1);

		body0->setWorldTransform(orgtrans0);
		body1->setWorldTransform(orgtrans1);
	}
}


void btGImpactCollisionAlgorithm::gimpactcompound_vs_gimpacttrimeshpart_collision(
					  btCollisionObject * body0,
					  btCollisionObject * body1,
					  btGImpactCompoundShape * shape0,
					  btGImpactMeshShapePart * shape1,bool swapped)
{
	//lock trimesh
	shape1->lock();

	btTransform orgtrans0 = body0->getWorldTransform();
	btTransform orgtrans1 = body1->getWorldTransform();

	gim_pair_set pairset;

	gimpactcompound_vs_gimpacttrimeshpart_find_pairs(orgtrans0,orgtrans1,shape0,shape1,pairset);	

	if(pairset.size()== 0)
	{
		//unlock trimesh
		shape1->unlock();
		return;
	}

//	btCollisionShape * orgshape0 = body0->getCollisionShape();
//	btCollisionShape * orgshape1 = body1->getCollisionShape();


	m_part1 = shape1->getPart();
	m_part0 = -1;
	m_triface0 = -1;

	btTriangleShapeEx bullet_triangle(btVector3(0.f,0.f,0.f),btVector3(0.f,0.f,0.f),btVector3(0.f,0.f,0.f));


	GUINT i = pairset.size();

	while(i--)
	{
		const GIM_PAIR & pair = pairset[i];
		btCollisionShape * colshape0 = shape0->getChildShape(pair.m_index1);
		btTransform childtrans0 = orgtrans0*shape0->getChildTransform(pair.m_index1);
		body0->setWorldTransform(childtrans0);

		shape1->getBulletTriangle(pair.m_index2,bullet_triangle);
		m_triface1 = pair.m_index2;

		//collide two shapes
		shape_vs_shape_collision(body0,body1,colshape0,&bullet_triangle,swapped);

		//restore transforms
//		body0->setCollisionShape(orgshape0);
//		body1->setCollisionShape(orgshape1);
		body1->setWorldTransform(orgtrans1);
	}

	//unlock trimesh
	shape1->unlock();
}



void btGImpactCollisionAlgorithm::gimpactcompound_vs_shape_collision(
					  btCollisionObject * body0,
					  btCollisionObject * body1,
					  btGImpactCompoundShape * shape0,
					  btCollisionShape * shape1,bool swapped)
{
	btTransform orgtrans0 = body0->getWorldTransform();

	btTransform trans1to0 = orgtrans0.inverse();
	trans1to0 *= body1->getWorldTransform();

	GIM_AABB boxshape;
	shape1->getAabb(trans1to0,boxshape.m_min,boxshape.m_max);
	gim_array<GUINT> collided_results;
	shape0->getBoxSet()->boxQuery(boxshape, collided_results);

	if(collided_results.size() == 0) return;



	GUINT i = collided_results.size();

	while(i--)
	{
		btCollisionShape * colshape0 = shape0->getChildShape(collided_results[i]);
		btTransform childtrans0 = orgtrans0*shape0->getChildTransform(collided_results[i]);

		body0->setWorldTransform(childtrans0);

		//collide two shapes
		shape_vs_shape_collision(body0,body1,colshape0,shape1,swapped);

		//restore transforms
//		body0->setCollisionShape(orgshape0);
		body0->setWorldTransform(orgtrans0);
	}

}




void btGImpactCollisionAlgorithm::gimpacttrimeshpart_vs_gimpacttrimeshpart_collision(
					  btCollisionObject * body0,
					  btCollisionObject * body1,
					  btGImpactMeshShapePart * shape0,
					  btGImpactMeshShapePart * shape1,bool swapped)
{
	shape0->lock();
	shape1->lock();

	btGImpactMeshShapePart * trishape0;
	btGImpactMeshShapePart * trishape1;
	btCollisionObject * tribody0;
	btCollisionObject * tribody1;

	if(swapped)
	{
		trishape0 = shape1;
		trishape1 = shape0;
		tribody0 = body1;
		tribody1 = body0;
	}
	else
	{
		trishape0 = shape0;
		trishape1 = shape1;
		tribody0 = body0;
		tribody1 = body1;
	}

	btTransform orgtrans0 = tribody0->getWorldTransform();
	btTransform orgtrans1 = tribody1->getWorldTransform();

	gim_pair_set pairset;

	gimpacttrimeshpart_vs_gimpacttrimeshpart_find_pairs(orgtrans0,orgtrans1,trishape0,trishape1,pairset);

	if(pairset.size()== 0)
	{
		shape0->unlock();
		shape1->unlock();
		return;
	}

	m_part0 = trishape0->getPart();
	m_part1 = trishape1->getPart();

#ifdef BULLET_TRIANGLE_COLLISION

	btTriangleShapeEx tri0(btVector3(0.f,0.f,0.f),btVector3(0.f,0.f,0.f),btVector3(0.f,0.f,0.f));
	btTriangleShapeEx tri1(btVector3(0.f,0.f,0.f),btVector3(0.f,0.f,0.f),btVector3(0.f,0.f,0.f));
	GUINT i = pairset.size();
	while(i--)
	{
		const GIM_PAIR & pair = pairset[i];
		m_triface0 = pair.m_index1;
		m_triface1 = pair.m_index2;

		trishape0->getBulletTriangle(pair.m_index1,tri0);
		trishape1->getBulletTriangle(pair.m_index2,tri1);


		convex_vs_convex_collision(
					  tribody0,
					  tribody1,
					  &tri0,
					  &tri1);

	}
	//unlock
	shape0->unlock();
	shape1->unlock();

#else
	gim_contact_array tempcontacts;
	GIM_TRIANGLE tri0;
	GIM_TRIANGLE tri1;
	GIM_TRIANGLE_CONTACT_DATA contact_data;
	GUINT i = pairset.size();
	while(i--)
	{
		const GIM_PAIR & pair = pairset[i];		

		trishape0->getTriangle(pair.m_index1,tri0);
		trishape1->getTriangle(pair.m_index2,tri1);

		tri0.apply_transform(orgtrans0);
		tri1.apply_transform(orgtrans1);

		if(tri0.collide_triangle(tri1,contact_data))
		{
			tempcontacts.push_triangle_contacts(contact_data,pair.m_index1,pair.m_index2);
		}
	}
	//unlock
	shape0->unlock();
	shape1->unlock();

	if(tempcontacts.size()==0) return;

	//sort contacts
	gim_contact_array contacts;
	contacts.merge_contacts(tempcontacts,true);
	// put contacts
	m_part0 = trishape0->getPart();
	m_part1 = trishape1->getPart();
	i = contacts.size();
	while(i--)
	{
		GIM_CONTACT  * pcontact = &contacts[i];

		m_triface0 = pcontact->m_feature1;
		m_triface1 = pcontact->m_feature2;

		addContactPoint(tribody0, tribody1,
					pcontact->m_point,
					pcontact->m_normal,
					-pcontact->m_depth);
	}
#endif
}

void btGImpactCollisionAlgorithm::gimpacttrimeshpart_vs_plane_collision(
					  btCollisionObject * body0,
					  btCollisionObject * body1,
					  btGImpactMeshShapePart * shape0,
					  btStaticPlaneShape * shape1,bool swapped)
{


	btTransform orgtrans0 = body0->getWorldTransform();
	btTransform orgtrans1 = body1->getWorldTransform();

	btPlaneShape * planeshape = static_cast<btPlaneShape *>(shape1);
	btVector4 plane;
	planeshape->get_plane_equation_transformed(orgtrans1,plane);

	//test box against plane

	GIM_AABB tribox;
	shape0->getAabb(orgtrans0,tribox.m_min,tribox.m_max);
	tribox.increment_margin(planeshape->getMargin());

	if( tribox.plane_classify(plane)!= G_COLLIDE_PLANE) return;

	shape0->lock();

	GREAL margin = shape0->getMargin() + planeshape->getMargin();

	btVector3 vertex;
	GUINT vi = shape0->getVertexCount();
	while(vi--)
	{
		shape0->getVertex(vi,vertex);
		vertex = orgtrans0(vertex);

		GREAL distance = vertex.dot(plane) - plane[3] - margin;

		if(distance<0.0)//add contact
		{
			if(swapped)
			{
				addContactPoint(body1, body0,
					vertex,
					-plane,
					distance);
			}
			else
			{
				addContactPoint(body0, body1,
					vertex,
					plane,
					distance);
			}
		}
	}

	shape0->unlock();
}


class btGImpactTriangleCallback: public btTriangleCallback
{
public:
	btGImpactCollisionAlgorithm * algorithm;
	btCollisionObject * body0;
	btCollisionObject * body1;
	btGImpactMeshShapePart * gimpactshape0;
	bool swapped;
	btScalar margin;


	virtual void processTriangle(btVector3* triangle, int partId, int triangleIndex)
	{
		btTriangleShapeEx tri1(triangle[0],triangle[1],triangle[2]);
		tri1.setMargin(margin);
		algorithm->gimpacttrimeshpart_vs_shape_collision(
							body0,body1,gimpactshape0,&tri1,swapped);
	}
};



void btGImpactCollisionAlgorithm::gimpacttrimeshpart_vs_concave_collision(
					  btCollisionObject * body0,
					  btCollisionObject * body1,
					  btGImpactMeshShapePart * shape0,
					  btConcaveShape * shape1,bool swapped)
{

	//create the callback
	btGImpactTriangleCallback tricallback;
	tricallback.algorithm = this;
	tricallback.body0 = body0;
	tricallback.body1 = body1;
	tricallback.gimpactshape0 = shape0;
	tricallback.swapped = swapped;
	tricallback.margin = shape1->getMargin();



	//getting the trimesh AABB
	btTransform gimpactInConcaveSpace;
	gimpactInConcaveSpace = body1->getWorldTransform().inverse() * body0->getWorldTransform();
	btVector3 minAABB,maxAABB;
	shape0->getAabb(gimpactInConcaveSpace,minAABB,maxAABB);

	shape1->processAllTriangles(&tricallback,minAABB,maxAABB);



}


void btGImpactCollisionAlgorithm::gimpacttrimeshpart_vs_shape_collision(
					  btCollisionObject * body0,
					  btCollisionObject * body1,
					  btGImpactMeshShapePart * shape0,
					  btCollisionShape * shape1,bool swapped)
{
#ifdef GIMPACT_VS_PLANE_COLLISION
	if(shape1->getShapeType() == STATIC_PLANE_PROXYTYPE)
	{
		btStaticPlaneShape *  plane1 = static_cast<btStaticPlaneShape * >(shape1);
		gimpacttrimeshpart_vs_plane_collision(body0,body1,shape0,plane1,swapped);
		return;
	}
#endif
	if(shape1->isConcave())
	{
		btConcaveShape *  concave1 = static_cast<btConcaveShape * >(shape1);
		gimpacttrimeshpart_vs_concave_collision(body0,body1,shape0,concave1,swapped);
		return;
	}

	btTransform trans1to0 = body0->getWorldTransform().inverse();
	trans1to0 *= body1->getWorldTransform();

	//lock
	shape0->lock();

	GIM_AABB boxshape;
	shape1->getAabb(trans1to0,boxshape.m_min,boxshape.m_max);
	gim_array<GUINT> collided_results(32);
	shape0->getBoxSet()->boxQuery(boxshape, collided_results);

	if(collided_results.size() == 0)
	{
		shape0->unlock();
		return;
	}

	btTriangleShapeEx bullet_triangle(btVector3(0.f,0.f,0.f),btVector3(0.f,0.f,0.f),btVector3(0.f,0.f,0.f));


	m_part0 = shape0->getPart();
	m_part1 = -1;
	m_triface1 = -1;

	GUINT i = collided_results.size();

	if(shape1->isConvex())
	{
		if(swapped)
		{
			while(i--)
			{
				m_triface0 = collided_results[i];
				shape0->getBulletTriangle(collided_results[i],bullet_triangle);
				//collide two shapes
				convex_vs_convex_collision(body1,body0,shape1,&bullet_triangle);
			}
		}
		else
		{
			while(i--)
			{
				m_triface0 = collided_results[i];
				shape0->getBulletTriangle(collided_results[i],bullet_triangle);
				//collide two shapes
				convex_vs_convex_collision(body0,body1,&bullet_triangle,shape1);
			}
		}
	}
	else
	{
		while(i--)
		{
			m_triface0 = collided_results[i];
			shape0->getBulletTriangle(collided_results[i],bullet_triangle);
			//collide two shapes
			shape_vs_shape_collision(body0,body1,&bullet_triangle,shape1,swapped);
		}
	}



	shape0->unlock();
}


void btGImpactCollisionAlgorithm::gimpact_vs_compoundshape(btCollisionObject * body0,
					  btCollisionObject * body1,
					  btGImpactShapeInterface * shape0,
					  btCompoundShape * shape1,bool swapped)
{
	btTransform orgtrans1 = body1->getWorldTransform();

	int i = shape1->getNumChildShapes();
	while(i--)
	{

		btCollisionShape * colshape1 = shape1->getChildShape(i);
		btTransform childtrans1 = orgtrans1*shape1->getChildTransform(i);

		body1->setWorldTransform(childtrans1);

		//collide child shape
		gimpact_vs_shape(body0, body1,
					  shape0,colshape1,swapped);


		//restore transforms
		body1->setWorldTransform(orgtrans1);
	}
}


void btGImpactCollisionAlgorithm::gimpact_vs_shape(btCollisionObject * body0,
					  btCollisionObject * body1,
					  btGImpactShapeInterface * shape0,
					  btCollisionShape * shape1,bool swapped)
{
	if(shape1->getShapeType() == COMPOUND_SHAPE_PROXYTYPE)
	{
		btCompoundShape * compoundshape = static_cast<btCompoundShape *>(shape1);
		gimpact_vs_compoundshape(body0,body1,shape0,compoundshape,swapped);
		return;
	}

	eGIMPACT_SHAPE_TYPE shapetype0 = shape0->getGImpactShapeType();
	if(shapetype0 == CONST_GIMPACT_TRIMESH_SHAPE)
	{
		btGImpactMeshShape * trimesh0 = static_cast<btGImpactMeshShape *>(shape0);
		gimpacttrimesh_vs_shape_collision(body0,body1,trimesh0,shape1,swapped);
	}
	else if(shapetype0 == CONST_GIMPACT_TRIMESH_SHAPE_PART)
	{
		btGImpactMeshShapePart * trimeshpart0 = static_cast<btGImpactMeshShapePart *>(shape0);
		gimpacttrimeshpart_vs_shape_collision(body0,body1,trimeshpart0,shape1,swapped);
	}
	else// compound
	{
		btGImpactCompoundShape * compound0 = static_cast<btGImpactCompoundShape*>(shape0);
		gimpactcompound_vs_shape_collision(body0,body1,compound0,shape1,swapped);
	}
}

void btGImpactCollisionAlgorithm::gimpact_vs_gimpact(btCollisionObject * body0,
					  btCollisionObject * body1,
					  btGImpactShapeInterface * shape0,
					  btGImpactShapeInterface * shape1)
{

	eGIMPACT_SHAPE_TYPE shapetype0 = shape0->getGImpactShapeType();
	eGIMPACT_SHAPE_TYPE shapetype1 = shape1->getGImpactShapeType();

	btGImpactMeshShape * trimesh0;
	btGImpactMeshShape * trimesh1;
	btGImpactMeshShapePart * trimeshpart0;
	btGImpactMeshShapePart * trimeshpart1;
	btGImpactCompoundShape * compound0;
	btGImpactCompoundShape * compound1;


	if(shapetype0 == CONST_GIMPACT_TRIMESH_SHAPE)
	{
		if(shapetype1 == CONST_GIMPACT_TRIMESH_SHAPE) // trimesh vs trimesh
		{
			trimesh0 = static_cast<btGImpactMeshShape *>(shape0);
			trimesh1 = static_cast<btGImpactMeshShape *>(shape1);
			gimpacttrimesh_vs_gimpacttrimesh(body0,body1,trimesh0,trimesh1);
		}
		else
		{
			if(shapetype1 == CONST_GIMPACT_COMPOUND_SHAPE) // trimesh vs compound
			{
				trimesh0 = static_cast<btGImpactMeshShape *>(shape0);
				compound1 = static_cast<btGImpactCompoundShape *>(shape1);
				gimpacttrimesh_vs_gimpactcompound(body0,body1,trimesh0,compound1,false);
			}
			else // trimesh vs trimesh part
			{
				trimesh0 = static_cast<btGImpactMeshShape *>(shape0);
				trimeshpart1 = static_cast<btGImpactMeshShapePart *>(shape1);
				gimpacttrimesh_vs_trimeshpart(body0,body1,trimesh0,trimeshpart1,false);
			}
		}
	}
	else if(shapetype1 == CONST_GIMPACT_TRIMESH_SHAPE)
	{
		if(shapetype0 == CONST_GIMPACT_COMPOUND_SHAPE) // compound vs trimesh
		{
			compound0 = static_cast<btGImpactCompoundShape *>(shape0);
			trimesh1 = static_cast<btGImpactMeshShape *>(shape1);
			gimpacttrimesh_vs_gimpactcompound(body1,body0,trimesh1,compound0,true);
		}
		else // trimesh part vs trimesh
		{
			trimeshpart0 = static_cast<btGImpactMeshShapePart *>(shape0);
			trimesh1 = static_cast<btGImpactMeshShape *>(shape1);
			gimpacttrimesh_vs_trimeshpart(body1,body0,trimesh1,trimeshpart0,true);
		}
	}
	else
	{
		if(shapetype0  == CONST_GIMPACT_COMPOUND_SHAPE)
		{
			if(shapetype1 == CONST_GIMPACT_COMPOUND_SHAPE) // compound vs compound
			{
				compound0 = static_cast<btGImpactCompoundShape *>(shape0);
				compound1 = static_cast<btGImpactCompoundShape *>(shape1);
				gimpactcompound_vs_gimpactcompound_collision(body0,body1,compound0,compound1);
			}
			else // compound vs trimesh part
			{
				compound0 = static_cast<btGImpactCompoundShape *>(shape0);
				trimeshpart1 = static_cast<btGImpactMeshShapePart *>(shape1);
				gimpactcompound_vs_gimpacttrimeshpart_collision(body0,body1,compound0,trimeshpart1,false);
			}
		}
		else if(shapetype1 == CONST_GIMPACT_COMPOUND_SHAPE) // trimesh part vs compound
		{
			compound1 = static_cast<btGImpactCompoundShape *>(shape1);
			trimeshpart0 = static_cast<btGImpactMeshShapePart *>(shape0);
			gimpactcompound_vs_gimpacttrimeshpart_collision(body1,body0,compound1,trimeshpart0,true);
		}
		else // trimesh part vs trimesh part
		{
			trimeshpart0 = static_cast<btGImpactMeshShapePart *>(shape0);
			trimeshpart1 = static_cast<btGImpactMeshShapePart *>(shape1);

			gimpacttrimeshpart_vs_gimpacttrimeshpart_collision(body0,body1,trimeshpart0,trimeshpart1,false);
		}
	}
}

void btGImpactCollisionAlgorithm::processCollision (btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
{
    clearCache();

    m_resultOut = resultOut;
	m_dispatchInfo = &dispatchInfo;


    btGImpactShapeInterface * gimpactshape0;
    btGImpactShapeInterface * gimpactshape1;
	if (body0->getCollisionShape()->getShapeType()==GIMPACT_SHAPE_PROXYTYPE)
	{
		if( body1->getCollisionShape()->getShapeType()==GIMPACT_SHAPE_PROXYTYPE )
		{
			gimpactshape0 = static_cast<btGImpactShapeInterface *>(body0->getCollisionShape());
			gimpactshape1 = static_cast<btGImpactShapeInterface *>(body1->getCollisionShape());
			gimpact_vs_gimpact(body0,body1,gimpactshape0,gimpactshape1);
		}
		else
		{
			gimpactshape0 = static_cast<btGImpactShapeInterface *>(body0->getCollisionShape());
			gimpact_vs_shape(body0,body1,gimpactshape0,body1->getCollisionShape(),false);
		}
	}
	else if (body1->getCollisionShape()->getShapeType()==GIMPACT_SHAPE_PROXYTYPE )
	{
		gimpactshape1 = static_cast<btGImpactShapeInterface *>(body1->getCollisionShape());
		gimpact_vs_shape(body1,body0,gimpactshape1,body0->getCollisionShape(),true);
	}
}


btScalar btGImpactCollisionAlgorithm::calculateTimeOfImpact(btCollisionObject* body0,btCollisionObject* body1,const btDispatcherInfo& dispatchInfo,btManifoldResult* resultOut)
{
	return 1.f;

}

///////////////////////////////////// REGISTERING ALGORITHM //////////////////////////////////////////////


//! Use this function for register the algorithm externally
void btGImpactCollisionAlgorithm::registerAlgorithm(btCollisionDispatcher * dispatcher)
{

	for (GUINT i = 0;i < MAX_BROADPHASE_COLLISION_TYPES ;i++ )
	{
		dispatcher->registerCollisionCreateFunc(GIMPACT_SHAPE_PROXYTYPE,i ,new btGImpactCollisionAlgorithm::CreateFunc);
	}

	for (GUINT i = 0;i < MAX_BROADPHASE_COLLISION_TYPES ;i++ )
	{
		dispatcher->registerCollisionCreateFunc(i,GIMPACT_SHAPE_PROXYTYPE ,new btGImpactCollisionAlgorithm::CreateFunc);
	}

	/*
	dispatcher->registerCollisionCreateFunc(GIMPACT_SHAPE_PROXYTYPE,GIMPACT_SHAPE_PROXYTYPE,new btGImpactCollisionAlgorithm::CreateFunc);
	dispatcher->registerCollisionCreateFunc(STATIC_PLANE_PROXYTYPE ,GIMPACT_SHAPE_PROXYTYPE,new btGImpactCollisionAlgorithm::CreateFunc);
	dispatcher->registerCollisionCreateFunc(GIMPACT_SHAPE_PROXYTYPE,STATIC_PLANE_PROXYTYPE ,new btGImpactCollisionAlgorithm::CreateFunc);*/


}
