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
#ifndef CHARACTER_DEMO_H
#define CHARACTER_DEMO_H


///DYNAMIC_CHARACTER_CONTROLLER is not fully implemented yet at the moment
//#define DYNAMIC_CHARACTER_CONTROLLER 1

#include "BulletCollision/CollisionShapes/btConvexHullShape.h"

class btCharacterControllerInterface;
class btDynamicCharacterController;
class btKinematicCharacterController;

class btCollisionShape;


#include "GlutDemoApplication.h"

///CharacterDemo shows how to setup and use the built-in raycast vehicle
class CharacterDemo : public GlutDemoApplication
{
	public:

#ifdef DYNAMIC_CHARACTER_CONTROLLER
	btCharacterControllerInterface* m_character;
#else
	btKinematicCharacterController* m_character;
	class	btPairCachingGhostObject* m_ghostObject;
#endif


	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

	class btBroadphaseInterface*	m_overlappingPairCache;

	class btCollisionDispatcher*	m_dispatcher;

	class btConstraintSolver*	m_constraintSolver;

	class btDefaultCollisionConfiguration* m_collisionConfiguration;

	class btTriangleIndexVertexArray*	m_indexVertexArrays;

	btVector3*	m_vertices;

	void	debugDrawContacts();
	
	float		m_cameraHeight;

	float	m_minCameraDistance;
	float	m_maxCameraDistance;


	CharacterDemo();

	virtual ~CharacterDemo();

	virtual void clientMoveAndDisplay();

	virtual void	clientResetScene();

	virtual void displayCallback();
	
	///a very basic camera following the character
	virtual void updateCamera();

	virtual void specialKeyboard(int key, int x, int y);

	virtual void specialKeyboardUp(int key, int x, int y);

	void renderme();

	void initPhysics();

	static DemoApplication* Create()
	{
		CharacterDemo* demo = new CharacterDemo();
		demo->myinit();
		demo->initPhysics();
		return demo;
	}
};



#define QUAKE_BSP_IMPORTING 1
#ifdef QUAKE_BSP_IMPORTING
#include "../BspDemo/BspLoader.h"
#include "../BspDemo/BspConverter.h"




class BspToBulletConverter : public BspConverter
{
	CharacterDemo* m_demoApp;

public:

	BspToBulletConverter(CharacterDemo*	demoApp)
		:m_demoApp(demoApp)
	{
	}

		virtual void	addConvexVerticesCollider(btAlignedObjectArray<btVector3>& vertices, bool isEntity, const btVector3& entityTargetLocation)
		{
			///perhaps we can do something special with entities (isEntity)
			///like adding a collision Triggering (as example)

			if (vertices.size() > 0)
			{
				float mass = 0.f;
				btTransform startTransform;
				//can use a shift
				startTransform.setIdentity();
				startTransform.setOrigin(btVector3(0,-10.0f,0.0f));
				//this create an internal copy of the vertices
				for (int i = 0; i < vertices.size(); i++)
				{
					vertices[i] *= btScalar(0.5);
					float t = vertices[i].getZ() * btScalar(0.75);
					vertices[i].setZ(-vertices[i].getY());
					vertices[i].setY(t);
				}

				btCollisionShape* shape = new btConvexHullShape(&(vertices[0].getX()),vertices.size());
				m_demoApp->m_collisionShapes.push_back(shape);

				//btRigidBody* body = m_demoApp->localCreateRigidBody(mass, startTransform,shape);
				m_demoApp->localCreateRigidBody(mass, startTransform,shape);
			}
		}
};
#endif //QUAKE_BSP_IMPORTING


#endif //CHARACTER_DEMO_H


