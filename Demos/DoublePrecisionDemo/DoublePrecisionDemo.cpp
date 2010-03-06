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


///
/// DoublePrecisionDemo shows high level usage of the Collision Detection.
///

#include "GL_Simplex1to4.h"

//include common Bullet Collision Detection headerfiles
#include "btBulletCollisionCommon.h"

#include "LinearMath/btIDebugDraw.h"
#include "GLDebugFont.h"


#include "GL_ShapeDrawer.h"
#include "DoublePrecisionDemo.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"

btScalar yaw=btScalar(0.);
btScalar pitch=btScalar(0.);
btScalar roll=btScalar(0.);
const int maxNumObjects = 4;
const int numObjects = 2;

GL_Simplex1to4 simplex;


btCollisionObject	objects[maxNumObjects];
btCollisionWorld*	collisionWorld = 0;

// so pixel ratio is 1:1
int screenWidth = 640;
int screenHeight = 640;
GLDebugDrawer debugDrawer;

const btScalar LARGE_DISTANCE_FROM_ORIGIN = btScalar(999999.0);
const btScalar VERY_SMALL_INCREMENT = btScalar(0.000009);

int main(int argc,char** argv)
{
	DoublePrecisionDemo* doublePrecisionDemo = new DoublePrecisionDemo();

	doublePrecisionDemo->initPhysics();
	doublePrecisionDemo->setCameraDistance(btScalar(2.0));

	doublePrecisionDemo->clientResetScene();

	return glutmain(argc, argv,screenWidth,screenHeight,"Double Precision Demo",doublePrecisionDemo);
}

void	DoublePrecisionDemo::initPhysics()
{
  m_debugMode |= btIDebugDraw::DBG_DrawWireframe;
	
	btMatrix3x3 basisA;
	basisA.setIdentity();

	btMatrix3x3 basisB;
	basisB.setIdentity();

	objects[0].getWorldTransform().setBasis(basisA);
	objects[1].getWorldTransform().setBasis(basisB);
	

	btBoxShape* boxA = new btBoxShape(btVector3(0.5,0.5,0.5));
	btBoxShape* boxB = new btBoxShape(btVector3(0.5,0.5,0.5));

	objects[0].setCollisionShape(boxA);//&hullA;
	objects[1].setCollisionShape(boxB);//&hullB;

	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
	btVector3	worldAabbMin(80000,80000,80000);
	btVector3	worldAabbMax(120000,120000,120000);

	btAxisSweep3*	broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax);
	
	collisionWorld = new btCollisionWorld(dispatcher,broadphase,collisionConfiguration);
		
	collisionWorld->addCollisionObject(&objects[0]);
	collisionWorld->addCollisionObject(&objects[1]);

}


//to be implemented by the demo

void DoublePrecisionDemo::clientMoveAndDisplay()
{
	
	displayCallback();
}


static btVoronoiSimplexSolver sGjkSimplexSolver;
btSimplexSolverInterface& gGjkSimplexSolver = sGjkSimplexSolver;

void DoublePrecisionDemo::displayCallback(void) 
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	glDisable(GL_LIGHTING);

	collisionWorld->getDispatchInfo().m_debugDraw = &debugDrawer;
	
  if (collisionWorld)
    collisionWorld->performDiscreteCollisionDetection();

  int i;

	btVector3	worldBoundsMin,worldBoundsMax;
	collisionWorld->getBroadphase()->getBroadphaseAabb(worldBoundsMin,worldBoundsMax);


  ///one way to draw all the contact points is iterating over contact manifolds / points:
  int numManifolds = collisionWorld->getDispatcher()->getNumManifolds();
  for (i=0;i<numManifolds;i++)
  {
    btPersistentManifold* contactManifold = collisionWorld->getDispatcher()->getManifoldByIndexInternal(i);
    btCollisionObject* obA = static_cast<btCollisionObject*>(contactManifold->getBody0());
    btCollisionObject* obB = static_cast<btCollisionObject*>(contactManifold->getBody1());
    contactManifold->refreshContactPoints(obA->getWorldTransform(),obB->getWorldTransform());

    int numContacts = contactManifold->getNumContacts();
    for (int j=0;j<numContacts;j++)
    {
      btManifoldPoint& pt = contactManifold->getContactPoint(j);

      glBegin(GL_LINES);
      glColor3f(1, 1, 1);

      btVector3 ptA = pt.getPositionWorldOnA() - m_cameraPosition;
      btVector3 ptB = pt.getPositionWorldOnB() - m_cameraPosition;

      glVertex3d(ptA.x(),ptA.y(),ptA.z());
      glVertex3d(ptB.x(),ptB.y(),ptB.z());
      glEnd();
    }

    //you can un-comment out this line, and then all points are removed
    //contactManifold->clearManifold();	
  }

	btScalar m[16];
	btTransform temp;
	

  btVector3 color;
  //int i;
	for (i=0;i<numObjects;i++)
	{
	  if (i % 2)
	  {
	    color = btVector3(1,0,0);
	  }
	  else
	  {
      color = btVector3(0,0,1);
	  }
	  temp = objects[i].getWorldTransform();
	  temp.setOrigin(temp.getOrigin() - m_cameraPosition);
		temp.getOpenGLMatrix( m );
		m_shapeDrawer->drawOpenGL(m,objects[i].getCollisionShape(),color,getDebugMode(),worldBoundsMin,worldBoundsMax);
	}

  objects[1].getWorldTransform().setOrigin(objects[1].getWorldTransform().getOrigin()+btVector3(-VERY_SMALL_INCREMENT,-VERY_SMALL_INCREMENT,0));
  objects[0].getWorldTransform().setOrigin(objects[0].getWorldTransform().getOrigin()+btVector3(VERY_SMALL_INCREMENT,VERY_SMALL_INCREMENT,0));
  
  float yStart = 20.f;
  float yIncr = 20.f;
  char buf[124];

  glColor3f(0, 0, 0);

  setOrthographicProjection();

  glRasterPos3f(10.0f,yStart,0);
  #ifdef BT_USE_DOUBLE_PRECISION
  GLDebugDrawString(10.f,yStart,"Double Precision Mode");
  #else
  GLDebugDrawString(10.f,yStart,"Single Precision Mode");
  #endif
  yStart += yIncr;
  
  glRasterPos3f(10.0f,yStart,0);
  sprintf(buf,"Movement distance in x and y axis = %lf", VERY_SMALL_INCREMENT);

  GLDebugDrawString(10.f,yStart,buf);
  yStart += yIncr;
  
  glRasterPos3f(10.0f,yStart,0);
  btScalar xValue = objects[0].getWorldTransform().getOrigin().x();
  btScalar yValue = objects[0].getWorldTransform().getOrigin().y();
  btScalar zValue = objects[0].getWorldTransform().getOrigin().z();
  sprintf(buf,"Cube 0 location = ( %lf, %lf, %lf )", xValue, yValue, zValue);
  GLDebugDrawString(10.f,yStart,buf);
  yStart += yIncr;

  xValue = objects[1].getWorldTransform().getOrigin().x();
  yValue = objects[1].getWorldTransform().getOrigin().y();
  zValue = objects[1].getWorldTransform().getOrigin().z();
  glRasterPos3f(10.0f,yStart,0);
  sprintf(buf,"Cube 1 location = ( %lf, %lf, %lf )", xValue, yValue, zValue);
  GLDebugDrawString(10.f,yStart,buf);
  yStart += yIncr;

  glRasterPos3f(10.0f,yStart,0);
  GLDebugDrawString(10.f,yStart,"w=toggle wireframe/solid");

  resetPerspectiveProjection();

	glFlush();
  glutSwapBuffers();
}

void DoublePrecisionDemo::clientResetScene()
{
	objects[0].getWorldTransform().setOrigin(btVector3(LARGE_DISTANCE_FROM_ORIGIN,LARGE_DISTANCE_FROM_ORIGIN,LARGE_DISTANCE_FROM_ORIGIN));
	objects[1].getWorldTransform().setOrigin(btVector3(LARGE_DISTANCE_FROM_ORIGIN-VERY_SMALL_INCREMENT,LARGE_DISTANCE_FROM_ORIGIN-VERY_SMALL_INCREMENT,LARGE_DISTANCE_FROM_ORIGIN));
}



void DoublePrecisionDemo::updateCamera() 
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  // look at the stationary cube
  m_cameraTargetPosition = objects[0].getWorldTransform().getOrigin();

  m_cameraPosition = m_cameraTargetPosition;
  m_cameraPosition[2] = m_cameraTargetPosition[2] - m_cameraDistance;

  //update OpenGL camera settings
  glFrustum(-1.0, 1.0, -1.0, 1.0, 1.0, 10000.0);

  // To not loose precision in the rendering process, we shift the object to the origin
  gluLookAt(0.0, 0.0, 0.0,
    m_cameraTargetPosition[0]-m_cameraPosition[0],m_cameraTargetPosition[1]-m_cameraPosition[1], m_cameraTargetPosition[2]-m_cameraPosition[2],
    m_cameraUp.getX(),m_cameraUp.getY(),m_cameraUp.getZ());
  glMatrixMode(GL_MODELVIEW);
}

void DoublePrecisionDemo::keyboardCallback(unsigned char key, int x, int y)
{
  if (key == 'w')
  {
    if (m_debugMode & btIDebugDraw::DBG_DrawWireframe)
    {
      m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawWireframe);
      m_debugMode |= btIDebugDraw::DBG_DrawAabb;
    }
    else
    {
      m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawAabb);
      m_debugMode |= btIDebugDraw::DBG_DrawWireframe;
    }
    return;
  }
  
  DemoApplication::keyboardCallback(key, x, y);
}


