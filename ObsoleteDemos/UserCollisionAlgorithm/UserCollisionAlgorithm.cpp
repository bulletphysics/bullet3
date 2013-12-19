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

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btIDebugDraw.h"
#include "GLDebugDrawer.h"
#include "UserCollisionAlgorithm.h"
#include "GL_ShapeDrawer.h"
#include "GlutStuff.h"

//The user defined collision algorithm
#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"

GLDebugDrawer	debugDrawer;

static const int NUM_VERTICES = 5;
static const int NUM_TRIANGLES=4;

btVector3	gVertices[NUM_VERTICES];
int	gIndices[NUM_TRIANGLES*3];
const float TRIANGLE_SIZE=10.f;


///User can override this material combiner by implementing gContactAddedCallback and setting body0->m_collisionFlags |= btCollisionObject::customMaterialCallback;
inline btScalar	calculateCombinedFriction(float friction0,float friction1)
{
	btScalar friction = friction0 * friction1;

	const btScalar MAX_FRICTION  = 10.f;
	if (friction < -MAX_FRICTION)
		friction = -MAX_FRICTION;
	if (friction > MAX_FRICTION)
		friction = MAX_FRICTION;
	return friction;

}

inline btScalar	calculateCombinedRestitution(float restitution0,float restitution1)
{
	return restitution0 * restitution1;
}






int main(int argc,char** argv)
{

	UserCollisionAlgorithm* userCollisionAlgorithm = new UserCollisionAlgorithm;
	userCollisionAlgorithm->initPhysics();
	userCollisionAlgorithm->setCameraDistance(30.f);

	return glutmain(argc, argv,640,480,"Static Concave Mesh Demo",userCollisionAlgorithm);
}

void	UserCollisionAlgorithm::initPhysics()
{
	#define TRISIZE 5.f


	const int NUM_VERTS_X = 50;
	const int NUM_VERTS_Y = 50;
	const int totalVerts = NUM_VERTS_X*NUM_VERTS_Y;

	btVector3*	gVertices = new btVector3[totalVerts];
	
	int i;
	for ( i=0;i<NUM_VERTS_X;i++)
	{
		for (int j=0;j<NUM_VERTS_Y;j++)
		{
			gVertices[i+j*NUM_VERTS_X].setValue((i-NUM_VERTS_X*0.5f)*TRIANGLE_SIZE,2.f*sinf((float)i)*cosf((float)j),(j-NUM_VERTS_Y*0.5f)*TRIANGLE_SIZE);
		}
	}

	btTriangleMesh* trimesh = new btTriangleMesh();

	for ( i=0;i<NUM_VERTS_X-1;i++)
	{
		for (int j=0;j<NUM_VERTS_Y-1;j++)
		{
			trimesh->addTriangle(gVertices[j*NUM_VERTS_X+i],gVertices[j*NUM_VERTS_X+i+1],gVertices[(j+1)*NUM_VERTS_X+i+1]);
			trimesh->addTriangle(gVertices[j*NUM_VERTS_X+i],gVertices[(j+1)*NUM_VERTS_X+i+1],gVertices[(j+1)*NUM_VERTS_X+i]);
		}
	}
	
	delete[] gVertices;

	bool useQuantizedBvhTree = true;
	btCollisionShape* trimeshShape  = new btBvhTriangleMeshShape(trimesh,useQuantizedBvhTree);
		
	//ConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new	btCollisionDispatcher(collisionConfiguration);
		
	btVector3 maxAabb(10000,10000,10000);
	btBroadphaseInterface* broadphase = new btAxisSweep3(-maxAabb,maxAabb);//SimpleBroadphase();
	dispatcher->registerCollisionCreateFunc(GIMPACT_SHAPE_PROXYTYPE,GIMPACT_SHAPE_PROXYTYPE,new btSphereSphereCollisionAlgorithm::CreateFunc);
	
	btConstraintSolver* constraintSolver = new btSequentialImpulseConstraintSolver();
	m_dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,constraintSolver,collisionConfiguration);
	
	float mass = 0.f;
	btTransform	startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(0,-2,0));

	btRigidBody* staticBody= localCreateRigidBody(mass, startTransform,trimeshShape);
	//enable custom material callback
	staticBody->setCollisionFlags(staticBody->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);

	{
		for (int i=0;i<10;i++)
		{
			btCollisionShape* sphereShape = new btSphereShape(1);
			startTransform.setOrigin(btVector3(1,2*i,1));
			//btRigidBody* body = localCreateRigidBody(1, startTransform,sphereShape);
			localCreateRigidBody(1, startTransform,sphereShape);
		}
	}
	
	

	m_dynamicsWorld->setDebugDrawer(&debugDrawer);
}

void UserCollisionAlgorithm::clientMoveAndDisplay()
{
	 glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	float dt = getDeltaTimeMicroseconds() * 0.000001f;
	

	m_dynamicsWorld->stepSimulation(dt);
	
	//optional but useful: debug drawing
	m_dynamicsWorld->debugDrawWorld();

	renderme();

    glFlush();
    glutSwapBuffers();

}






void UserCollisionAlgorithm::displayCallback(void) {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

    glFlush();
    glutSwapBuffers();
}


