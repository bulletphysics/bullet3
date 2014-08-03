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
/// Convex Hull Distance Demo shows distance calculation between two convex hulls of points.
/// GJK with the btVoronoiSimplexSolver is used.
///

#include "GL_Simplex1to4.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btTransform.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/CollisionShapes/btConvexHullShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"

#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btPointCollector.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h"
#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btConvexHullComputer.h"




#define TaruVtxCount 43

static btScalar TaruVtx[] = {
1.08664f,-1.99237f,0.0f,
0.768369f,-1.99237f,-0.768369f,
1.28852f,1.34412e-007f,-1.28852f,
1.82224f,1.90735e-007f,0.0f,
0.0f,-1.99237f,-1.08664f,
0.0f,0.0f,-1.82224f,
0.0f,-1.99237f,-1.08664f,
-0.768369f,-1.99237f,-0.768369f,
-1.28852f,1.34412e-007f,-1.28852f,
0.0f,0.0f,-1.82224f,
-1.08664f,-1.99237f,1.82086e-007f,
-1.82224f,1.90735e-007f,1.59305e-007f,
-0.768369f,-1.99237f,0.76837f,
-1.28852f,2.47058e-007f,1.28852f,
1.42495e-007f,-1.99237f,1.08664f,
2.38958e-007f,2.70388e-007f,1.82224f,
0.768369f,-1.99237f,0.768369f,
1.28852f,2.47058e-007f,1.28852f,
0.768369f,1.99237f,-0.768369f,
1.08664f,1.99237f,0.0f,
0.0f,1.99237f,-1.08664f,
-0.768369f,1.99237f,-0.768369f,
0.0f,1.99237f,-1.08664f,
-1.08664f,1.99237f,0.0f,
-0.768369f,1.99237f,0.768369f,
1.42495e-007f,1.99237f,1.08664f,
0.768369f,1.99237f,0.768369f,
1.42495e-007f,-1.99237f,1.08664f,
-0.768369f,-1.99237f,0.76837f,
-1.08664f,-1.99237f,1.82086e-007f,
-0.768369f,-1.99237f,-0.768369f,
0.0f,-1.99237f,-1.08664f,
0.768369f,-1.99237f,-0.768369f,
1.08664f,-1.99237f,0.0f,
0.768369f,-1.99237f,0.768369f,
0.768369f,1.99237f,-0.768369f,
0.0f,1.99237f,-1.08664f,
-0.768369f,1.99237f,-0.768369f,
-1.08664f,1.99237f,0.0f,
-0.768369f,1.99237f,0.768369f,
1.42495e-007f,1.99237f,1.08664f,
0.768369f,1.99237f,0.768369f,
1.08664f,1.99237f,0.0f,
};




#define USE_GJK

#ifndef USE_GJK
#include "btBulletCollisionCommon.h"

#endif //USE_GJK
#include "GL_ShapeDrawer.h"
#ifdef WIN32 //needed for glut.h
#include <windows.h>
#endif
//think different
#if defined(__APPLE__) && !defined (VMDMESA)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include "GlutStuff.h"


float yaw=0.f,pitch=0.f,roll=0.f;
const int maxNumObjects = 4;
const int numObjects = 2;

GL_Simplex1to4 simplex;

btConvexShape*	shapePtr[maxNumObjects];

btTransform tr[numObjects];
int screenWidth = 640.f;
int screenHeight = 480.f;

void clientResetScene()
{
	tr[0].setOrigin(btVector3(0.0f,3.f,7.f));
	tr[1].setOrigin(btVector3(0.0f,9.f,2.f));
}

int debugMode = btIDebugDraw::DBG_DrawWireframe;
GL_ShapeDrawer shapeDrawer;
int m_glutScreenWidth=0;
int m_glutScreenHeight=0;
float m_frustumZNear = 1.f;
float m_frustumZFar = 10000.f;
bool m_ortho = false;

int myglutmain(int argc, char **argv,int width,int height,const char* title);


void updateCamera() {


	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	btScalar rele = 0;
	btScalar razi = 0;
	btVector3 m_cameraUp(0,1,0);
	btScalar m_cameraDistance = 10.f;
	btVector3 m_cameraPosition;
	

	btVector3 m_cameraTargetPosition = (tr[0].getOrigin()+tr[1].getOrigin())*0.5;


	btQuaternion rot(m_cameraUp,razi);


	int m_forwardAxis = 2;

	btVector3 eyePos(0,0,0);
	eyePos[m_forwardAxis] = -m_cameraDistance;

	btVector3 forward(eyePos[0],eyePos[1],eyePos[2]);
	if (forward.length2() < SIMD_EPSILON)
	{
		forward.setValue(1.f,0.f,0.f);
	}
	btVector3 right = m_cameraUp.cross(forward);
	btQuaternion roll(right,-rele);

	eyePos = btMatrix3x3(rot) * btMatrix3x3(roll) * eyePos;

	m_cameraPosition[0] = eyePos.getX();
	m_cameraPosition[1] = eyePos.getY();
	m_cameraPosition[2] = eyePos.getZ();
	m_cameraPosition += m_cameraTargetPosition;

	if (m_glutScreenWidth == 0 && m_glutScreenHeight == 0)
		return;

	btScalar aspect;
	btVector3 extents;

	if (m_glutScreenWidth > m_glutScreenHeight) 
	{
		aspect = m_glutScreenWidth / (btScalar)m_glutScreenHeight;
		extents.setValue(aspect * 1.0f, 1.0f,0);
	} else 
	{
		aspect = m_glutScreenHeight / (btScalar)m_glutScreenWidth;
		extents.setValue(1.0f, aspect*1.f,0);
	}

	
	if (m_ortho)
	{
		// reset matrix
		glLoadIdentity();
		
		
		extents *= m_cameraDistance;
		btVector3 lower = m_cameraTargetPosition - extents;
		btVector3 upper = m_cameraTargetPosition + extents;
		//gluOrtho2D(lower.x, upper.x, lower.y, upper.y);
		glOrtho(lower.getX(), upper.getX(), lower.getY(), upper.getY(),-1000,1000);
		
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		//glTranslatef(100,210,0);
	} else
	{
		if (m_glutScreenWidth > m_glutScreenHeight) 
		{
//			glFrustum (-aspect, aspect, -1.0, 1.0, 1.0, 10000.0);
			glFrustum (-aspect * m_frustumZNear, aspect * m_frustumZNear, -m_frustumZNear, m_frustumZNear, m_frustumZNear, m_frustumZFar);
		} else 
		{
//			glFrustum (-1.0, 1.0, -aspect, aspect, 1.0, 10000.0);
			glFrustum (-aspect * m_frustumZNear, aspect * m_frustumZNear, -m_frustumZNear, m_frustumZNear, m_frustumZNear, m_frustumZFar);
		}
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		gluLookAt(m_cameraPosition[0], m_cameraPosition[1], m_cameraPosition[2], 
			m_cameraTargetPosition[0], m_cameraTargetPosition[1], m_cameraTargetPosition[2], 
			m_cameraUp.getX(),m_cameraUp.getY(),m_cameraUp.getZ());
	}

}


int main(int argc,char** argv)
{
	clientResetScene();

	btMatrix3x3 basisA;
	basisA.setIdentity();

	btMatrix3x3 basisB;
	basisB.setIdentity();

	tr[0].setBasis(basisA);
	tr[1].setBasis(basisB);

	btVector3	points0[3]={btVector3(1,0,0),btVector3(0,1,0),btVector3(0,0,1)};
	//btVector3	points1[5]={btVector3(1,0,0),btVector3(0,1,0),btVector3(0,0,1),btVector3(0,0,-1),btVector3(-1,-1,0)};
	
	btConvexHullShape	hullA(&points0[0].getX(),3);
	btConvexHullShape	hullB(TaruVtx,TaruVtxCount,3*sizeof(btScalar));
	
	shapePtr[0] = &hullA;
	shapePtr[1] = &hullB;
	

	btTransform tr;
	tr.setIdentity();


	return myglutmain(argc, argv,screenWidth,screenHeight,"Convex Hull Distance Demo");
}


static btVoronoiSimplexSolver sGjkSimplexSolver;
btSimplexSolverInterface& gGjkSimplexSolver = sGjkSimplexSolver;

#include <stdio.h>


void clientDisplay(void) {


	updateCamera();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	glDisable(GL_LIGHTING);

	//GL_ShapeDrawer::drawCoordSystem();

	ATTRIBUTE_ALIGNED16(btScalar) m[16];
	int i;
#ifdef USE_GJK
	btGjkEpaPenetrationDepthSolver epa;
	btGjkPairDetector	convexConvex(shapePtr[0],shapePtr[1],&sGjkSimplexSolver,&epa);

	btVector3 seperatingAxis(0.00000000f,0.059727669f,0.29259586f);
	convexConvex.setCachedSeperatingAxis(seperatingAxis);

	btPointCollector gjkOutput;
	btGjkPairDetector::ClosestPointInput input;
	input.m_transformA = tr[0];
	input.m_transformB = tr[1];

	convexConvex.getClosestPoints(input ,gjkOutput,0);

	if (gjkOutput.m_hasResult)
	{
		btVector3 endPt = gjkOutput.m_pointInWorld +
			gjkOutput.m_normalOnBInWorld*gjkOutput.m_distance;

		 glBegin(GL_LINES);
		glColor3f(1, 0, 0);
		glVertex3d(gjkOutput.m_pointInWorld.x(), gjkOutput.m_pointInWorld.y(),gjkOutput.m_pointInWorld.z());
		glVertex3d(endPt.x(),endPt.y(),endPt.z());
		glEnd();

	}
#else //USE_GJK

	
	struct	MyContactResultCallback : public btCollisionWorld::ContactResultCallback
	{
		virtual	btScalar	addSingleResult(btManifoldPoint& cp,	const btCollisionObjectWrapper* colObj0Wrap,int partId0,int index0,const btCollisionObjectWrapper* colObj1Wrap,int partId1,int index1)
		{
			 glBegin(GL_LINES);
			glColor3f(1, 0, 0);
			
			glVertex3d(cp.m_positionWorldOnA.getX(),cp.m_positionWorldOnA.getY(),cp.m_positionWorldOnA.getZ());
			glVertex3d(cp.m_positionWorldOnB.getX(),cp.m_positionWorldOnB.getY(),cp.m_positionWorldOnB.getZ());
			glEnd();

			return 1.f;
		}
	};

	btDefaultCollisionConfiguration collisionConfiguration;
	btCollisionDispatcher				dispatcher(&collisionConfiguration);
	btDbvtBroadphase pairCache;
	btCollisionWorld world (&dispatcher,&pairCache,&collisionConfiguration);
	gContactBreakingThreshold=1e10f;
	
	MyContactResultCallback result;
	btCollisionObject obA;
	obA.setCollisionShape(shapePtr[0]);
	obA.setWorldTransform(tr[0]);
	btCollisionObject obB;
	obB.setCollisionShape(shapePtr[1]);
	obB.setWorldTransform(tr[1]);
	world.contactPairTest(&obA,&obB,result);

#endif//USE_GJK

	btVector3 worldMin(-1000,-1000,-1000);
	btVector3 worldMax(1000,1000,1000);

	for (i=0;i<numObjects;i++)
	{
		
		tr[i].getOpenGLMatrix( m );

		if (debugMode)
		{
			/// for polyhedral shapes
				if (shapePtr[i]->isPolyhedral())
				{
					if (!shapePtr[i]->getUserPointer())
					{
						btConvexHullComputer* convexUtil = new btConvexHullComputer();
						shapePtr[i]->setUserPointer(convexUtil);

						btPolyhedralConvexShape* polyshape = (btPolyhedralConvexShape*) shapePtr[i];

						btAlignedObjectArray<btVector3> vertices;
						vertices.resize(polyshape->getNumVertices());
						for (int i=0;i<polyshape->getNumVertices();i++)
						{
							polyshape->getVertex(i,vertices[i]);
						}
						
						bool useDoublePrecision = false;
						convexUtil->compute(&vertices[0].getX(),sizeof(btVector3), polyshape->getNumVertices(),0,0);
					} 

					if (shapePtr[i]->getUserPointer())
					{
						btConvexHullComputer* convexUtil = (btConvexHullComputer*)shapePtr[i]->getUserPointer();
						//printf("num faces = %d\n",convexUtil->faces.size());
						for (int j=0;j<convexUtil->faces.size();j++)
						{
							int face = convexUtil->faces[j];
							//printf("face=%d\n",face);
							const btConvexHullComputer::Edge*  firstEdge = &convexUtil->edges[face];
							const btConvexHullComputer::Edge*  edge = firstEdge;

							do
							{
								int src = edge->getSourceVertex();
								int targ = edge->getTargetVertex();
								//printf("src=%d target = %d\n", src,targ);
								
								btVector3 wa = tr[i] * convexUtil->vertices[src];
								btVector3 wb = tr[i] * convexUtil->vertices[targ];

								glBegin(GL_LINES);
								glColor3f(1, 1, 1);
								glVertex3f(wa.getX(),wa.getY(),wa.getZ());
								glVertex3f(wb.getX(),wb.getY(),wb.getZ());
								glEnd();

								edge = edge->getNextEdgeOfFace();
							} while (edge!=firstEdge);

						}
					}
				}
		} else
		{
			shapeDrawer.drawOpenGL(m,shapePtr[i],btVector3(1,1,1),debugMode, worldMin, worldMax);
		}


	}

	simplex.setSimplexSolver(&sGjkSimplexSolver);
	btVector3 ybuf[4],pbuf[4],qbuf[4];
	int numpoints = sGjkSimplexSolver.getSimplex(pbuf,qbuf,ybuf);
	simplex.reset();
	
	for (i=0;i<numpoints;i++)
		simplex.addVertex(ybuf[i]);

	btTransform ident;
	ident.setIdentity();
	ident.getOpenGLMatrix(m);
	shapeDrawer.drawOpenGL(m,&simplex,btVector3(1,1,1),debugMode, worldMin,worldMax);


	btQuaternion orn;
	orn.setEuler(yaw,pitch,roll);
	tr[0].setRotation(orn);
	tr[1].setRotation(orn);

	pitch += 0.005f;
	yaw += 0.01f;

	glFlush();
    glutSwapBuffers();
}

void clientMoveAndDisplay()
{
	clientDisplay();
}

static void glutReshapeCallback(int w, int h)
{
	m_glutScreenWidth=w;
	m_glutScreenHeight=h;
}


int myglutmain(int argc, char **argv,int width,int height,const char* title) {
    

	glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_STENCIL);
    glutInitWindowPosition(0, 0);
    glutInitWindowSize(width, height);
	
    glutCreateWindow(title);
#ifdef BT_USE_FREEGLUT
	glutSetOption (GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
#endif


	glutIdleFunc( clientDisplay );
	glutDisplayFunc( clientDisplay );
	glutReshapeFunc(glutReshapeCallback);

//enable vsync to avoid tearing on Apple (todo: for Windows)

#if defined(__APPLE__) && !defined (VMDMESA)
int swap_interval = 1;
CGLContextObj cgl_context = CGLGetCurrentContext();
CGLSetParameter(cgl_context, kCGLCPSwapInterval, &swap_interval);
#endif

	GLfloat light_ambient[] = { btScalar(0.2), btScalar(0.2), btScalar(0.2), btScalar(1.0) };
	GLfloat light_diffuse[] = { btScalar(1.0), btScalar(1.0), btScalar(1.0), btScalar(1.0) };
	GLfloat light_specular[] = { btScalar(1.0), btScalar(1.0), btScalar(1.0), btScalar(1.0 )};
	/*	light_position is NOT default value	*/
	GLfloat light_position0[] = { btScalar(1.0), btScalar(10.0), btScalar(1.0), btScalar(0.0 )};
	GLfloat light_position1[] = { btScalar(-1.0), btScalar(-10.0), btScalar(-1.0), btScalar(0.0) };

	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position0);

	glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT1, GL_POSITION, light_position1);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);


	glShadeModel(GL_SMOOTH);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	glClearColor(btScalar(0.7),btScalar(0.7),btScalar(0.7),btScalar(0));


	
    glutMainLoop();
    return 0;
}