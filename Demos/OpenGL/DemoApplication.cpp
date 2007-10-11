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


#include "DemoApplication.h"
#include "LinearMath/btIDebugDraw.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"

#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"//picking
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btUniformScalingShape.h"

#include "GL_ShapeDrawer.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btDefaultMotionState.h"

#include "BMF_Api.h"

extern bool gDisableDeactivation;
int numObjects = 0;
const int maxNumObjects = 16384;
btTransform startTransforms[maxNumObjects];
btCollisionShape* gShapePtr[maxNumObjects];//1 rigidbody has 1 shape (no re-use of shapes)
#define SHOW_NUM_DEEP_PENETRATIONS 1

#ifdef SHOW_NUM_DEEP_PENETRATIONS 
extern int gNumDeepPenetrationChecks;
extern int gNumGjkChecks;
#endif //


DemoApplication::DemoApplication()
		//see btIDebugDraw.h for modes
:
m_dynamicsWorld(0),
m_pickConstraint(0),
	m_cameraDistance(15.0),
	m_debugMode(0),
	m_ele(20.f),
	m_azi(0.f),
	m_cameraPosition(0.f,0.f,0.f),
	m_cameraTargetPosition(0.f,0.f,0.f),
	m_scaleBottom(0.5f),
	m_scaleFactor(2.f),
	m_cameraUp(0,1,0),
	m_forwardAxis(2),	
	m_glutScreenWidth(0),
	m_glutScreenHeight(0),
	m_ShootBoxInitialSpeed(40.f),
	m_stepping(true),
	m_singleStep(false),
	m_idle(false)
{
}



DemoApplication::~DemoApplication()
{

}


void DemoApplication::myinit(void)
{

    GLfloat light_ambient[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
    GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
    /*	light_position is NOT default value	*/
    GLfloat light_position0[] = { 1.0, 1.0, 1.0, 0.0 };
    GLfloat light_position1[] = { -1.0, -1.0, -1.0, 0.0 };
  
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

		glClearColor(0.8,0.8,0.8,0);

    //  glEnable(GL_CULL_FACE);
    //  glCullFace(GL_BACK);
}


void	DemoApplication::setCameraDistance(float dist)
{
	m_cameraDistance  = dist;
}

float	DemoApplication::getCameraDistance()
{
	return m_cameraDistance;
}



void DemoApplication::toggleIdle() {
    if (m_idle) {
        m_idle = false;
    }
    else {
        m_idle = true;
    }
}




void DemoApplication::updateCamera() {

	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	float rele = m_ele * 0.01745329251994329547;// rads per deg
	float razi = m_azi * 0.01745329251994329547;// rads per deg
	

	btQuaternion rot(m_cameraUp,razi);


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
 
    glFrustum(-1.0, 1.0, -1.0, 1.0, 1.0, 10000.0);
    gluLookAt(m_cameraPosition[0], m_cameraPosition[1], m_cameraPosition[2], 
              m_cameraTargetPosition[0], m_cameraTargetPosition[1], m_cameraTargetPosition[2], 
			  m_cameraUp.getX(),m_cameraUp.getY(),m_cameraUp.getZ());
    glMatrixMode(GL_MODELVIEW);
}



const float STEPSIZE = 5;

void DemoApplication::stepLeft() 
{ 
	m_azi -= STEPSIZE; if (m_azi < 0) m_azi += 360; updateCamera(); 
}
void DemoApplication::stepRight() 
{ 
	m_azi += STEPSIZE; if (m_azi >= 360) m_azi -= 360; updateCamera(); 
}
void DemoApplication::stepFront() 
{ 
	m_ele += STEPSIZE; if (m_ele >= 360) m_ele -= 360; updateCamera(); 
}
void DemoApplication::stepBack() 
{ 
	m_ele -= STEPSIZE; if (m_ele < 0) m_ele += 360; updateCamera(); 
}
void DemoApplication::zoomIn() 
{ 
	m_cameraDistance -= 0.4; updateCamera(); 
	if (m_cameraDistance < 0.1)
		m_cameraDistance = 0.1;

}
void DemoApplication::zoomOut() 
{ 
	m_cameraDistance += 0.4; updateCamera(); 
	
}









	
void DemoApplication::reshape(int w, int h) 
{
	m_glutScreenWidth = w;
	m_glutScreenHeight = h;
	
	glViewport(0, 0, w, h);
	updateCamera();
}




void DemoApplication::keyboardCallback(unsigned char key, int x, int y)
{
	(void)x;
	(void)y;

		m_lastKey = 0;

    switch (key) 
    {
    case 'q' : exit(0); break;

    case 'l' : stepLeft(); break;
    case 'r' : stepRight(); break;
    case 'f' : stepFront(); break;
    case 'b' : stepBack(); break;
    case 'z' : zoomIn(); break;
    case 'x' : zoomOut(); break;
    case 'i' : toggleIdle(); break;
	case 'h':
			if (m_debugMode & btIDebugDraw::DBG_NoHelpText)
				m_debugMode = m_debugMode & (~btIDebugDraw::DBG_NoHelpText);
			else
				m_debugMode |= btIDebugDraw::DBG_NoHelpText;
			break;

	case 'w':
			if (m_debugMode & btIDebugDraw::DBG_DrawWireframe)
				m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawWireframe);
			else
				m_debugMode |= btIDebugDraw::DBG_DrawWireframe;
		   break;

   case 'p':
	   if (m_debugMode & btIDebugDraw::DBG_ProfileTimings)
		m_debugMode = m_debugMode & (~btIDebugDraw::DBG_ProfileTimings);
	else
		m_debugMode |= btIDebugDraw::DBG_ProfileTimings;
   break;

   case 'm':
	   if (m_debugMode & btIDebugDraw::DBG_EnableSatComparison)
		m_debugMode = m_debugMode & (~btIDebugDraw::DBG_EnableSatComparison);
	else
		m_debugMode |= btIDebugDraw::DBG_EnableSatComparison;
   break;

   case 'n':
	   if (m_debugMode & btIDebugDraw::DBG_DisableBulletLCP)
		m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DisableBulletLCP);
	else
		m_debugMode |= btIDebugDraw::DBG_DisableBulletLCP;
   break;

	case 't' : 
			if (m_debugMode & btIDebugDraw::DBG_DrawText)
				m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawText);
			else
				m_debugMode |= btIDebugDraw::DBG_DrawText;
		   break;
	case 'y':		
			if (m_debugMode & btIDebugDraw::DBG_DrawFeaturesText)
				m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawFeaturesText);
			else
				m_debugMode |= btIDebugDraw::DBG_DrawFeaturesText;
		break;
	case 'a':	
		if (m_debugMode & btIDebugDraw::DBG_DrawAabb)
				m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawAabb);
			else
				m_debugMode |= btIDebugDraw::DBG_DrawAabb;
			break;
		case 'c' : 
			if (m_debugMode & btIDebugDraw::DBG_DrawContactPoints)
				m_debugMode = m_debugMode & (~btIDebugDraw::DBG_DrawContactPoints);
			else
				m_debugMode |= btIDebugDraw::DBG_DrawContactPoints;
			break;

		case 'd' : 
			if (m_debugMode & btIDebugDraw::DBG_NoDeactivation)
				m_debugMode = m_debugMode & (~btIDebugDraw::DBG_NoDeactivation);
			else
				m_debugMode |= btIDebugDraw::DBG_NoDeactivation;
			if (m_debugMode | btIDebugDraw::DBG_NoDeactivation)
			{
				gDisableDeactivation = true;
			} else
			{
				gDisableDeactivation = false;
			}
			break;
			

		

	case 'o' :
		{
			m_stepping = !m_stepping;
			break;
		}
	case 's' : clientMoveAndDisplay(); break;
//    case ' ' : newRandom(); break;
	case ' ':
		clientResetScene();
			break;
	case '1':
		{
			if (m_debugMode & btIDebugDraw::DBG_EnableCCD)
				m_debugMode = m_debugMode & (~btIDebugDraw::DBG_EnableCCD);
			else
				m_debugMode |= btIDebugDraw::DBG_EnableCCD;
			break;
		}

		case '.':
		{
			shootBox(getCameraTargetPosition());
			break;
		}

		case '+':
		{
			m_ShootBoxInitialSpeed += 10.f;
			break;
		}
		case '-':
		{
			m_ShootBoxInitialSpeed -= 10.f;
			break;
		}

    default:
//        std::cout << "unused key : " << key << std::endl;
        break;
    }

	if (getDynamicsWorld() && getDynamicsWorld()->getDebugDrawer())
		getDynamicsWorld()->getDebugDrawer()->setDebugMode(m_debugMode);

	glutPostRedisplay();

}
	
void DemoApplication::specialKeyboardUp(int key, int x, int y)
{

		glutPostRedisplay();

}

void DemoApplication::specialKeyboard(int key, int x, int y)	
{
	(void)x;
	(void)y;

    switch (key) 
    {
	case GLUT_KEY_F1:
		{

			break;
		}

	case GLUT_KEY_F2:
	{

		break;
	}


	case GLUT_KEY_END:
		{
			int numObj = getDynamicsWorld()->getNumCollisionObjects();
			if (numObj)
			{
				btCollisionObject* obj = getDynamicsWorld()->getCollisionObjectArray()[numObj-1];
				getDynamicsWorld()->removeCollisionObject(obj);
			}
			break;
		}
    case GLUT_KEY_LEFT : stepLeft(); break;
    case GLUT_KEY_RIGHT : stepRight(); break;
    case GLUT_KEY_UP : stepFront(); break;
    case GLUT_KEY_DOWN : stepBack(); break;
    case GLUT_KEY_PAGE_UP : zoomIn(); break;
    case GLUT_KEY_PAGE_DOWN : zoomOut(); break;
    case GLUT_KEY_HOME : toggleIdle(); break;
    default:
//        std::cout << "unused (special) key : " << key << std::endl;
        break;
    }

	glutPostRedisplay();

}



void DemoApplication::moveAndDisplay()
{
	if (!m_idle)
		clientMoveAndDisplay();
}



	
void DemoApplication::displayCallback()
{
}




void	DemoApplication::shootBox(const btVector3& destination)
{

	if (m_dynamicsWorld)
	{
		float mass = 100.f;
		btTransform startTransform;
		startTransform.setIdentity();
		btVector3 camPos = getCameraPosition();
		startTransform.setOrigin(camPos);
//#define TEST_UNIFORM_SCALING_SHAPE 1
#ifdef TEST_UNIFORM_SCALING_SHAPE
		btConvexShape* childShape = new btBoxShape(btVector3(1.f,1.f,1.f));
		btUniformScalingShape* boxShape = new btUniformScalingShape(childShape,0.5f);
#else
		btCollisionShape* boxShape = new btBoxShape(btVector3(0.5f,0.5f,0.5f));
#endif//
		btRigidBody* body = this->localCreateRigidBody(mass, startTransform,boxShape);

		btVector3 linVel(destination[0]-camPos[0],destination[1]-camPos[1],destination[2]-camPos[2]);
		linVel.normalize();
		linVel*=m_ShootBoxInitialSpeed;

		body->getWorldTransform().setOrigin(camPos);
		body->getWorldTransform().setRotation(btQuaternion(0,0,0,1));
		body->setLinearVelocity(linVel);
		body->setAngularVelocity(btVector3(0,0,0));
	}

}


int gPickingConstraintId = 0;
btVector3 gOldPickingPos;
float gOldPickingDist  = 0.f;
btRigidBody* pickedBody = 0;//for deactivation state


btVector3	DemoApplication::getRayTo(int x,int y)
{

		float top = 1.f;
	float bottom = -1.f;
	float nearPlane = 1.f;
	float tanFov = (top-bottom)*0.5f / nearPlane;
	float fov = 2.0 * atanf (tanFov);

	btVector3	rayFrom = getCameraPosition();
	btVector3 rayForward = (getCameraTargetPosition()-getCameraPosition());
	rayForward.normalize();
	float farPlane = 600.f;
	rayForward*= farPlane;

	btVector3 rightOffset;
	btVector3 vertical = m_cameraUp;

	btVector3 hor;
	hor = rayForward.cross(vertical);
	hor.normalize();
	vertical = hor.cross(rayForward);
	vertical.normalize();

	float tanfov = tanf(0.5f*fov);
	hor *= 2.f * farPlane * tanfov;
	vertical *= 2.f * farPlane * tanfov;
	btVector3 rayToCenter = rayFrom + rayForward;
	btVector3 dHor = hor * 1.f/float(m_glutScreenWidth);
	btVector3 dVert = vertical * 1.f/float(m_glutScreenHeight);
	btVector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
	rayTo += x * dHor;
	rayTo -= y * dVert;
	return rayTo;
}


void DemoApplication::mouseFunc(int button, int state, int x, int y)
{
	//printf("button %i, state %i, x=%i,y=%i\n",button,state,x,y);
	//button 0, state 0 means left mouse down

	btVector3 rayTo = getRayTo(x,y);

	switch (button)
	{
	case 2:
		{
			if (state==0)
			{

				shootBox(rayTo);
			}
			break;
		};
	case 1:
		{


			if (state==0)
			{
				

				//apply an impulse
				if (m_dynamicsWorld)
				{
					btCollisionWorld::ClosestRayResultCallback rayCallback(m_cameraPosition,rayTo);
					m_dynamicsWorld->rayTest(m_cameraPosition,rayTo,rayCallback);
					if (rayCallback.HasHit())
					{
						
						btRigidBody* body = btRigidBody::upcast(rayCallback.m_collisionObject);
						if (body)
						{
								body->setActivationState(ACTIVE_TAG);
								btVector3 impulse = rayTo;
								impulse.normalize();
								float impulseStrength = 10.f;
								impulse *= impulseStrength;
								btVector3 relPos = rayCallback.m_hitPointWorld - body->getCenterOfMassPosition();
								body->applyImpulse(impulse,relPos);
							}
					}
				}

				

			} else
			{

			}
			break;	
		}
	case 0:
		{
			if (state==0)
			{
		

				//add a point to point constraint for picking
				if (m_dynamicsWorld)
				{
					btCollisionWorld::ClosestRayResultCallback rayCallback(m_cameraPosition,rayTo);
					m_dynamicsWorld->rayTest(m_cameraPosition,rayTo,rayCallback);
					if (rayCallback.HasHit())
					{
						
						
						btRigidBody* body = btRigidBody::upcast(rayCallback.m_collisionObject);
						if (body)
						{
							//other exclusions?
							if (!(body->isStaticObject() || body->isKinematicObject()))
							{
								pickedBody = body;
								pickedBody->setActivationState(DISABLE_DEACTIVATION);

								
								btVector3 pickPos = rayCallback.m_hitPointWorld;

								btVector3 localPivot = body->getCenterOfMassTransform().inverse() * pickPos;

								btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*body,localPivot);
								m_dynamicsWorld->addConstraint(p2p);
								m_pickConstraint = p2p;
								
								//save mouse position for dragging
								gOldPickingPos = rayTo;

								btVector3 eyePos(m_cameraPosition[0],m_cameraPosition[1],m_cameraPosition[2]);

								gOldPickingDist  = (pickPos-eyePos).length();

								//very weak constraint for picking
								p2p->m_setting.m_tau = 0.1f;
							}
						}
					}
				}

			} else
			{

				if (m_pickConstraint && m_dynamicsWorld)
				{
					m_dynamicsWorld->removeConstraint(m_pickConstraint);
					delete m_pickConstraint;
					//printf("removed constraint %i",gPickingConstraintId);
					m_pickConstraint = 0;
					pickedBody->forceActivationState(ACTIVE_TAG);
					pickedBody->setDeactivationTime( 0.f );
					pickedBody = 0;
				}

				
			}

			break;

		}
	default:
		{
		}
	}

}

void	DemoApplication::mouseMotionFunc(int x,int y)
{

	if (m_pickConstraint)
	{
		//move the constraint pivot
		btPoint2PointConstraint* p2p = static_cast<btPoint2PointConstraint*>(m_pickConstraint);
		if (p2p)
		{
			//keep it at the same picking distance

			btVector3 newRayTo = getRayTo(x,y);
			btVector3 eyePos(m_cameraPosition[0],m_cameraPosition[1],m_cameraPosition[2]);
			btVector3 dir = newRayTo-eyePos;
			dir.normalize();
			dir *= gOldPickingDist;

			btVector3 newPos = eyePos + dir;
			p2p->setPivotB(newPos);
		}

	}

	
}



btRigidBody*	DemoApplication::localCreateRigidBody(float mass, const btTransform& startTransform,btCollisionShape* shape)
{
	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
	if (isDynamic)
		shape->calculateLocalInertia(mass,localInertia);

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody* body = new btRigidBody(mass,myMotionState,shape,localInertia);

#else
	btRigidBody* body = new btRigidBody(mass,startTransform,shape,localInertia);	
#endif//
	m_dynamicsWorld->addRigidBody(body);
	
	return body;
}

//See http://www.lighthouse3d.com/opengl/glut/index.php?bmpfontortho
void DemoApplication::setOrthographicProjection() 
{

	// switch to projection mode
	glMatrixMode(GL_PROJECTION);
	// save previous matrix which contains the 
	//settings for the perspective projection
	glPushMatrix();
	// reset matrix
	glLoadIdentity();
	// set a 2D orthographic projection
	gluOrtho2D(0, m_glutScreenWidth, 0, m_glutScreenHeight);
	// invert the y axis, down is positive
	glScalef(1, -1, 1);
	// mover the origin from the bottom left corner
	// to the upper left corner
	glTranslatef(0, -m_glutScreenHeight, 0);
	glMatrixMode(GL_MODELVIEW);
}

void DemoApplication::resetPerspectiveProjection() 
{
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}


void DemoApplication::renderme()
{
	updateCamera();

	btScalar m[16];

	if (m_dynamicsWorld)
	{
		int numObjects = m_dynamicsWorld->getNumCollisionObjects();
		btVector3 wireColor(1,0,0);
		for (int i=0;i<numObjects;i++)
		{
			btCollisionObject* colObj = m_dynamicsWorld->getCollisionObjectArray()[i];
			btRigidBody* body = btRigidBody::upcast(colObj);

			if (body && body->getMotionState())
			{
				btDefaultMotionState* myMotionState = (btDefaultMotionState*)body->getMotionState();
				myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);
			} else
			{
				colObj->getWorldTransform().getOpenGLMatrix(m);
			}

			btVector3 wireColor(1.f,1.0f,0.5f); //wants deactivation
			if (i & 1)
			{
				wireColor = btVector3(0.f,0.0f,1.f);
			}
			///color differently for active, sleeping, wantsdeactivation states
			if (colObj->getActivationState() == 1) //active
			{
				if (i & 1)
				{
					wireColor += btVector3 (1.f,0.f,0.f);
				} else
				{			
					wireColor += btVector3 (.5f,0.f,0.f);
				}
			}
			if (colObj->getActivationState() == 2) //ISLAND_SLEEPING
			{
				if (i & 1)
				{
					wireColor += btVector3 (0.f,1.f, 0.f);
				} else
				{
					wireColor += btVector3 (0.f,0.5f,0.f);
				}
			}

			GL_ShapeDrawer::drawOpenGL(m,colObj->getCollisionShape(),wireColor,getDebugMode());
		}


			float xOffset = 10.f;
			float yStart = 20.f;
			float yIncr = 20.f;
			char buf[124];

			glColor3f(0, 0, 0);

			if ((m_debugMode & btIDebugDraw::DBG_NoHelpText)==0)
				{
				setOrthographicProjection();

	#ifdef USE_QUICKPROF


			if ( getDebugMode() & btIDebugDraw::DBG_ProfileTimings)
			{
				static int counter = 0;
				counter++;
				std::map<std::string, hidden::ProfileBlock*>::iterator iter;
				for (iter = btProfiler::mProfileBlocks.begin(); iter != btProfiler::mProfileBlocks.end(); ++iter)
				{
					char blockTime[128];
					sprintf(blockTime, "%s: %lf",&((*iter).first[0]),btProfiler::getBlockTime((*iter).first, btProfiler::BLOCK_CYCLE_SECONDS));//BLOCK_TOTAL_PERCENT));
					glRasterPos3f(xOffset,yStart,0);
					BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),blockTime);
					yStart += yIncr;

				}

			}
	#endif //USE_QUICKPROF

			
				glRasterPos3f(xOffset,yStart,0);
				sprintf(buf,"mouse to interact");
				BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
				yStart += yIncr;

				glRasterPos3f(xOffset,yStart,0);
				sprintf(buf,"space to reset");
				BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
				yStart += yIncr;
			
				glRasterPos3f(xOffset,yStart,0);
				sprintf(buf,"cursor keys and z,x to navigate");
				BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
				yStart += yIncr;

				glRasterPos3f(xOffset,yStart,0);
				sprintf(buf,"i to toggle simulation, s single step");
				BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
				yStart += yIncr;

				glRasterPos3f(xOffset,yStart,0);
				sprintf(buf,"q to quit");
				BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
				yStart += yIncr;

				glRasterPos3f(xOffset,yStart,0);
				sprintf(buf,". to shoot box");
				BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
				yStart += yIncr;

				// not yet hooked up again after refactoring...

				glRasterPos3f(xOffset,yStart,0);
				sprintf(buf,"d to toggle deactivation");
				BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
				yStart += yIncr;

				
				glRasterPos3f(xOffset,yStart,0);
				sprintf(buf,"g to toggle mesh animation (ConcaveDemo)");
				BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
				yStart += yIncr;
			

				glRasterPos3f(xOffset,yStart,0);
				sprintf(buf,"h to toggle help text");
				BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
				yStart += yIncr;

				
				glRasterPos3f(xOffset,yStart,0);
				sprintf(buf,"p to toggle profiling (+results to file)");
				BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
				yStart += yIncr;


				//bool useBulletLCP = !(getDebugMode() & btIDebugDraw::DBG_DisableBulletLCP);
				//bool useCCD = (getDebugMode() & btIDebugDraw::DBG_EnableCCD);
				//glRasterPos3f(xOffset,yStart,0);
				//sprintf(buf,"1 CCD mode (adhoc) = %i",useCCD);
				//BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
				//yStart += yIncr;


				glRasterPos3f(xOffset,yStart,0);
				sprintf(buf,"+- shooting speed = %10.2f",m_ShootBoxInitialSpeed);
				BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
				yStart += yIncr;

#ifdef SHOW_NUM_DEEP_PENETRATIONS
				
				glRasterPos3f(xOffset,yStart,0);
				sprintf(buf,"gNumDeepPenetrationChecks = %d",gNumDeepPenetrationChecks);
				BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
				yStart += yIncr;

				glRasterPos3f(xOffset,yStart,0);
				sprintf(buf,"gNumGjkChecks= %d",gNumGjkChecks);
				BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
				yStart += yIncr;

#endif //SHOW_NUM_DEEP_PENETRATIONS

				resetPerspectiveProjection();
			}
		
	}

}

void	DemoApplication::clientResetScene()
{
#ifdef SHOW_NUM_DEEP_PENETRATIONS
	gNumDeepPenetrationChecks = 0;
	gNumGjkChecks = 0;
#endif //SHOW_NUM_DEEP_PENETRATIONS

	int numObjects = 0;
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(1.f/60.f,0);
		numObjects = m_dynamicsWorld->getNumCollisionObjects();
	}
	
	for (int i=0;i<numObjects;i++)
	{
		btCollisionObject* colObj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(colObj);
		if (body)
		{
			if (body->getMotionState())
			{
				btDefaultMotionState* myMotionState = (btDefaultMotionState*)body->getMotionState();
				myMotionState->m_graphicsWorldTrans = myMotionState->m_startWorldTrans;
				colObj->setWorldTransform( myMotionState->m_graphicsWorldTrans );
				colObj->setInterpolationWorldTransform( myMotionState->m_startWorldTrans );
				colObj->activate();
			}
			//removed cached contact points
			m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(colObj->getBroadphaseHandle(),getDynamicsWorld()->getDispatcher());

			btRigidBody* body = btRigidBody::upcast(colObj);
			if (body && !body->isStaticObject())
			{
				btRigidBody::upcast(colObj)->setLinearVelocity(btVector3(0,0,0));
				btRigidBody::upcast(colObj)->setAngularVelocity(btVector3(0,0,0));
			}
		}

	/*
	//quickly search some issue at a certain simulation frame, pressing space to reset
		int fixed=18;
		for (int i=0;i<fixed;i++)
		{
			getDynamicsWorld()->stepSimulation(1./60.f,1);
		}
	*/
	}
}
