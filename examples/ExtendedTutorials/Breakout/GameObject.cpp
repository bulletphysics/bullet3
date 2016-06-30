/*
 * GameObject.cpp
 *
 *  Created on: Jun 28, 2016
 *      Author: leviathan
 */

#include "GameObject.h"

#include "btBulletDynamicsCommon.h"

GameObject::GameObject():m_position(0,0,0),m_rotation(0,0,0,1),m_body(0),m_shape(0),m_tag(PADDLE) {
	// TODO Auto-generated constructor stub

}

GameObject::~GameObject() {
	// TODO Auto-generated destructor stub
}

void GameObject::createShapeWithVertices(const Vertex* vertices, unsigned int vertexCount,bool isConvex)
{
    //1
    if (isConvex)
    {
        //2
        m_shape = new btConvexHullShape();
        for (int i = 0; i < vertexCount; i++)
        {
            Vertex v = vertices[i];
            btVector3 btv = btVector3(v.position.p[0], v.position.p[1], v.position.p[2]);
            ((btConvexHullShape*)m_shape)->addPoint(btv);
        }
    }
    else
    {
        //3
        btTriangleMesh* mesh = new btTriangleMesh();
        for (int i=0; i < vertexCount; i += 3)
        {
            Vertex v1 = vertices[i];
            Vertex v2 = vertices[i+1];
            Vertex v3 = vertices[i+2];

            btVector3 bv1 = btVector3(v1.position.p[0], v1.position.p[1], v1.position.p[2]);
            btVector3 bv2 = btVector3(v2.position.p[0], v2.position.p[1], v2.position.p[2]);
            btVector3 bv3 = btVector3(v3.position.p[0], v3.position.p[1], v3.position.p[2]);

            mesh->addTriangle(bv1, bv2, bv3);
        }
		m_shape = new btBvhTriangleMeshShape(mesh, true);
    }
}

void GameObject::createBodyWithMass(float mass)
{
	btAssert((!m_shape || m_shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
			m_shape->calculateLocalInertia(mass, localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

	#define USE_MOTIONSTATE 1
	#ifdef USE_MOTIONSTATE
		btDefaultMotionState* motionState = new btDefaultMotionState(btTransform(m_rotation, m_position));

		btRigidBody::btRigidBodyConstructionInfo cInfo(mass, motionState, m_shape, localInertia);

		m_body = new btRigidBody(cInfo);
		//body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

	#else
		m_body = new btRigidBody(mass, 0, shape, localInertia);
		m_body->setWorldTransform(btTransform(m_rotation, m_position));
	#endif

    m_body->setRestitution(1.0f);
    m_body->setFriction(0.5f);

    m_body->setLinearFactor(btVector3(1,1,0));

    m_body->setUserPointer(this);
}

void GameObject::setPosition(btVector3 position){
	m_position = position;
	m_body->getWorldTransform().setOrigin(position);
}

void GameObject::setOrientation(btQuaternion orientation){
	m_rotation = orientation;
	m_body->getWorldTransform().setRotation(orientation);
}
