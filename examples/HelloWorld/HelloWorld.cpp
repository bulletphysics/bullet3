/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///-----includes_start-----
#include "btBulletDynamicsCommon.h"
#include <stdio.h>

/// This is a Hello World program for running a basic Bullet physics simulation

int main(int argc, char** argv)
{
	///-----includes_end-----

	int i;
	///-----initialization_start-----

	///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

	///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
	btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

	btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);

	dynamicsWorld->setGravity(btVector3(0, -10, 0));

	///-----initialization_end-----

	//keep track of the shapes, we release memory at exit.
	//make sure to re-use collision shapes among rigid bodies whenever possible!
	btAlignedObjectArray<btCollisionShape*> collisionShapes;

  ///create a few basic rigid bodies

  //the ground is a cube of side 100 at position y = -50.
  {
    btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));

    collisionShapes.push_back(groundShape);

    btTransform groundTransform;
    groundTransform.setIdentity();
    groundTransform.setOrigin(btVector3(0, -50, 0));

    btScalar mass(0.);

    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
      groundShape->calculateLocalInertia(mass, localInertia);

    //using motionstate is optional, it provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
    btRigidBody* body = new btRigidBody(rbInfo);

    //add the body to the dynamics world
    dynamicsWorld->addRigidBody(body);
  }

  // Here's the start of the loop where we resimulate and evaluate determinism
  constexpr int numReps = 10;
  for (int k = 0; k < numReps; k++) {

    assert(dynamicsWorld->getNumCollisionObjects() == 1); // we don't recreate the ground; it persists across reps

    {
      //create a stack of spheres

      const float ySpacing = 1.5f;
      const float xzSpacing = 0.01;

      constexpr int numObjects = 20;
      for (int j = 0; j < numObjects; j++) {

        //btCollisionShape* colShape = new btBoxShape(btVector3(1,1,1));
        btCollisionShape* colShape = new btSphereShape(btScalar(1.));
        collisionShapes.push_back(colShape);

        /// Create Dynamic Objects
        btTransform startTransform;
        startTransform.setIdentity();
        startTransform.getOrigin().setY((j + 1) * ySpacing);

        // Interesting note: if I use 0 offset for x and y, behavior is 100% 
        // deterministic
        startTransform.getOrigin().setX((j) * xzSpacing);
        startTransform.getOrigin().setZ((j) * xzSpacing);

        btScalar mass(1.f);

        //rigidbody is dynamic if and only if mass is non zero, otherwise static
        bool isDynamic = (mass != 0.f);

        btVector3 localInertia(0, 0, 0);
        if (isDynamic)
          colShape->calculateLocalInertia(mass, localInertia);

        //startTransform.setOrigin(btVector3(2, 10, 0));

        //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
        btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);

        dynamicsWorld->addRigidBody(body);
      }
    }

    /// Do some simulation

    ///-----stepsimulation_start-----
    for (i = 0; i < 600; i++)
    {
      dynamicsWorld->stepSimulation(1.f / 60.f, 10);
    }

    // print end state
    for (int j = dynamicsWorld->getNumCollisionObjects() - 1; j >= 0; j--)
    {
      btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[j];
      btRigidBody* body = btRigidBody::upcast(obj);
      btTransform trans;
      if (body && body->getMotionState())
      {
        body->getMotionState()->getWorldTransform(trans);
      }
      else
      {
        trans = obj->getWorldTransform();
      }
      printf("%d,%f,%f,%f,", j, float(trans.getOrigin().getX()), float(trans.getOrigin().getY()), float(trans.getOrigin().getZ()));
    }
    printf("\n");



    ///-----stepsimulation_end-----

    //cleanup in the reverse order of creation/initialization

    ///-----cleanup_start-----

    //remove the rigidbodies from the dynamics world and delete them, except for the ground plane
    for (i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 1; i--)
    {
      btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
      btRigidBody* body = btRigidBody::upcast(obj);
      if (body && body->getMotionState())
      {
        delete body->getMotionState();
      }
      dynamicsWorld->removeCollisionObject(obj);
      delete obj;
    }

    // Let's not delete the shapes until after all the reps
  }

  //delete collision shapes
  for (int j = 0; j < collisionShapes.size(); j++)
  {
    btCollisionShape* shape = collisionShapes[j];
    collisionShapes[j] = 0;
    delete shape;
  }

	//delete dynamics world
	delete dynamicsWorld;

	//delete solver
	delete solver;

	//delete broadphase
	delete overlappingPairCache;

	//delete dispatcher
	delete dispatcher;

	delete collisionConfiguration;

	//next line is optional: it will be cleared by the destructor when the array goes out of scope
	collisionShapes.clear();
}
