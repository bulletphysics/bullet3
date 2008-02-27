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

class CharacterController;

class btCollisionShape;


#include "DemoApplication.h"

///CharacterDemo shows how to setup and use the built-in raycast vehicle
class CharacterDemo : public DemoApplication
{
	public:

	CharacterController* m_character;

	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

	class btBroadphaseInterface*	m_overlappingPairCache;

	class btCollisionDispatcher*	m_dispatcher;

	class btConstraintSolver*	m_constraintSolver;

	class btDefaultCollisionConfiguration* m_collisionConfiguration;

	class btTriangleIndexVertexArray*	m_indexVertexArrays;

	btVector3*	m_vertices;

	
	float		m_cameraHeight;

	float	m_minCameraDistance;
	float	m_maxCameraDistance;


	CharacterDemo();

	virtual ~CharacterDemo();

	virtual void clientMoveAndDisplay();

	virtual void	clientResetScene();

	virtual void displayCallback();
	
	///a very basic camera following the vehicle
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

#endif //CHARACTER_DEMO_H


