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
#include "LinearMath/GenIDebugDraw.h"

#include "CcdPhysicsEnvironment.h"
#include "CcdPhysicsController.h"
#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"//picking
#include "PHY_Pro.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "GL_ShapeDrawer.h"
#include "LinearMath/GenQuickprof.h"
#include "BMF_Api.h"

int numObjects = 0;
const int maxNumObjects = 16384;
DefaultMotionState ms[maxNumObjects];
CcdPhysicsController* physObjects[maxNumObjects];
SimdTransform startTransforms[maxNumObjects];
CollisionShape* gShapePtr[maxNumObjects];//1 rigidbody has 1 shape (no re-use of shapes)


DemoApplication::DemoApplication()
		//see IDebugDraw.h for modes
:
m_physicsEnvironmentPtr(0),
	m_cameraDistance(15.0),
	m_debugMode(0),
	m_ele(0.f),
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
	

	SimdQuaternion rot(m_cameraUp,razi);


	SimdVector3 eyePos(0,0,0);
	eyePos[m_forwardAxis] = -m_cameraDistance;

	SimdVector3 forward(eyePos[0],eyePos[1],eyePos[2]);
	if (forward.length2() < SIMD_EPSILON)
	{
		forward.setValue(1.f,0.f,0.f);
	}
	SimdVector3 right = m_cameraUp.cross(forward);
	SimdQuaternion roll(right,-rele);

	eyePos = SimdMatrix3x3(rot) * SimdMatrix3x3(roll) * eyePos;

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
	m_ele += STEPSIZE; if (m_azi >= 360) m_azi -= 360; updateCamera(); 
}
void DemoApplication::stepBack() 
{ 
	m_ele -= STEPSIZE; if (m_azi < 0) m_azi += 360; updateCamera(); 
}
void DemoApplication::zoomIn() 
{ 
	m_cameraDistance -= 1; updateCamera(); 
}
void DemoApplication::zoomOut() 
{ 
	m_cameraDistance += 1; updateCamera(); 
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
			if (m_debugMode & IDebugDraw::DBG_NoHelpText)
				m_debugMode = m_debugMode & (~IDebugDraw::DBG_NoHelpText);
			else
				m_debugMode |= IDebugDraw::DBG_NoHelpText;
			break;

	case 'w':
			if (m_debugMode & IDebugDraw::DBG_DrawWireframe)
				m_debugMode = m_debugMode & (~IDebugDraw::DBG_DrawWireframe);
			else
				m_debugMode |= IDebugDraw::DBG_DrawWireframe;
		   break;

   case 'p':
	   if (m_debugMode & IDebugDraw::DBG_ProfileTimings)
		m_debugMode = m_debugMode & (~IDebugDraw::DBG_ProfileTimings);
	else
		m_debugMode |= IDebugDraw::DBG_ProfileTimings;
   break;

   case 'm':
	   if (m_debugMode & IDebugDraw::DBG_EnableSatComparison)
		m_debugMode = m_debugMode & (~IDebugDraw::DBG_EnableSatComparison);
	else
		m_debugMode |= IDebugDraw::DBG_EnableSatComparison;
   break;

   case 'n':
	   if (m_debugMode & IDebugDraw::DBG_DisableBulletLCP)
		m_debugMode = m_debugMode & (~IDebugDraw::DBG_DisableBulletLCP);
	else
		m_debugMode |= IDebugDraw::DBG_DisableBulletLCP;
   break;

	case 't' : 
			if (m_debugMode & IDebugDraw::DBG_DrawText)
				m_debugMode = m_debugMode & (~IDebugDraw::DBG_DrawText);
			else
				m_debugMode |= IDebugDraw::DBG_DrawText;
		   break;
	case 'y':		
			if (m_debugMode & IDebugDraw::DBG_DrawFeaturesText)
				m_debugMode = m_debugMode & (~IDebugDraw::DBG_DrawFeaturesText);
			else
				m_debugMode |= IDebugDraw::DBG_DrawFeaturesText;
		break;
	case 'a':	
		if (m_debugMode & IDebugDraw::DBG_DrawAabb)
				m_debugMode = m_debugMode & (~IDebugDraw::DBG_DrawAabb);
			else
				m_debugMode |= IDebugDraw::DBG_DrawAabb;
			break;
		case 'c' : 
			if (m_debugMode & IDebugDraw::DBG_DrawContactPoints)
				m_debugMode = m_debugMode & (~IDebugDraw::DBG_DrawContactPoints);
			else
				m_debugMode |= IDebugDraw::DBG_DrawContactPoints;
			break;

		case 'd' : 
			if (m_debugMode & IDebugDraw::DBG_NoDeactivation)
				m_debugMode = m_debugMode & (~IDebugDraw::DBG_NoDeactivation);
			else
				m_debugMode |= IDebugDraw::DBG_NoDeactivation;
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
			if (m_debugMode & IDebugDraw::DBG_EnableCCD)
				m_debugMode = m_debugMode & (~IDebugDraw::DBG_EnableCCD);
			else
				m_debugMode |= IDebugDraw::DBG_EnableCCD;
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

	if (m_physicsEnvironmentPtr)
		m_physicsEnvironmentPtr->setDebugMode(m_debugMode);

	glutPostRedisplay();

}
	
void DemoApplication::specialKeyboard(int key, int x, int y)	
{
    switch (key) 
    {
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




void	DemoApplication::shootBox(const SimdVector3& destination)
{
	if (m_physicsEnvironmentPtr)
	{
		bool isDynamic = true;
		float mass = 1.f;
		SimdTransform startTransform;
		startTransform.setIdentity();
		SimdVector3 camPos = getCameraPosition();
		startTransform.setOrigin(camPos);
		CollisionShape* boxShape = new BoxShape(SimdVector3(1.f,1.f,1.f));

		CcdPhysicsController* newBox = LocalCreatePhysicsObject(isDynamic, mass, startTransform,boxShape);

		SimdVector3 linVel(destination[0]-camPos[0],destination[1]-camPos[1],destination[2]-camPos[2]);
		linVel.normalize();
		linVel*=m_ShootBoxInitialSpeed;

		newBox->setPosition(camPos[0],camPos[1],camPos[2]);
		newBox->setOrientation(0,0,0,1);
		newBox->SetLinearVelocity(linVel[0],linVel[1],linVel[2],false);
		newBox->SetAngularVelocity(0,0,0,false);
	}
}


int gPickingConstraintId = 0;
SimdVector3 gOldPickingPos;
float gOldPickingDist  = 0.f;
RigidBody* pickedBody = 0;//for deactivation state


SimdVector3	DemoApplication::GetRayTo(int x,int y)
{

		float top = 1.f;
	float bottom = -1.f;
	float nearPlane = 1.f;
	float tanFov = (top-bottom)*0.5f / nearPlane;
	float fov = 2.0 * atanf (tanFov);

	SimdVector3	rayFrom = getCameraPosition();
	SimdVector3 rayForward = (getCameraTargetPosition()-getCameraPosition());
	rayForward.normalize();
	float farPlane = 600.f;
	rayForward*= farPlane;

	SimdVector3 rightOffset;
	SimdVector3 vertical = m_cameraUp;

	SimdVector3 hor;
	hor = rayForward.cross(vertical);
	hor.normalize();
	vertical = hor.cross(rayForward);
	vertical.normalize();

	float tanfov = tanf(0.5f*fov);
	hor *= 2.f * farPlane * tanfov;
	vertical *= 2.f * farPlane * tanfov;
	SimdVector3 rayToCenter = rayFrom + rayForward;
	SimdVector3 dHor = hor * 1.f/float(m_glutScreenWidth);
	SimdVector3 dVert = vertical * 1.f/float(m_glutScreenHeight);
	SimdVector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
	rayTo += x * dHor;
	rayTo -= y * dVert;
	return rayTo;
}


void DemoApplication::mouseFunc(int button, int state, int x, int y)
{
	//printf("button %i, state %i, x=%i,y=%i\n",button,state,x,y);
	//button 0, state 0 means left mouse down

	SimdVector3 rayTo = GetRayTo(x,y);

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
				if (m_physicsEnvironmentPtr)
				{
					float hit[3];
					float normal[3];
					PHY_IPhysicsController* hitObj = m_physicsEnvironmentPtr->rayTest(0,m_cameraPosition[0],m_cameraPosition[1],m_cameraPosition[2],rayTo.getX(),rayTo.getY(),rayTo.getZ(),hit[0],hit[1],hit[2],normal[0],normal[1],normal[2]);
					if (hitObj)
					{
						CcdPhysicsController* physCtrl = static_cast<CcdPhysicsController*>(hitObj);
						RigidBody* body = physCtrl->GetRigidBody();
						if (body)
						{
							body->SetActivationState(ACTIVE_TAG);
							SimdVector3 impulse = rayTo;
							impulse.normalize();
							float impulseStrength = 10.f;
							impulse *= impulseStrength;
							SimdVector3 relPos(
								hit[0] - body->getCenterOfMassPosition().getX(),						
								hit[1] - body->getCenterOfMassPosition().getY(),
								hit[2] - body->getCenterOfMassPosition().getZ());

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
				if (m_physicsEnvironmentPtr)
				{
					float hit[3];
					float normal[3];
					PHY_IPhysicsController* hitObj = m_physicsEnvironmentPtr->rayTest(0,m_cameraPosition[0],m_cameraPosition[1],m_cameraPosition[2],rayTo.getX(),rayTo.getY(),rayTo.getZ(),hit[0],hit[1],hit[2],normal[0],normal[1],normal[2]);
					if (hitObj)
					{

						CcdPhysicsController* physCtrl = static_cast<CcdPhysicsController*>(hitObj);
						RigidBody* body = physCtrl->GetRigidBody();

						if (body && !body->IsStatic())
						{
							pickedBody = body;
							pickedBody->SetActivationState(DISABLE_DEACTIVATION);

							SimdVector3 pickPos(hit[0],hit[1],hit[2]);

							SimdVector3 localPivot = body->getCenterOfMassTransform().inverse() * pickPos;

							gPickingConstraintId = m_physicsEnvironmentPtr->createConstraint(physCtrl,0,PHY_POINT2POINT_CONSTRAINT,
								localPivot.getX(),
								localPivot.getY(),
								localPivot.getZ(),
								0,0,0);
							//printf("created constraint %i",gPickingConstraintId);

							//save mouse position for dragging
							gOldPickingPos = rayTo;


							SimdVector3 eyePos(m_cameraPosition[0],m_cameraPosition[1],m_cameraPosition[2]);

							gOldPickingDist  = (pickPos-eyePos).length();

							Point2PointConstraint* p2p = static_cast<Point2PointConstraint*>(m_physicsEnvironmentPtr->getConstraintById(gPickingConstraintId));
							if (p2p)
							{
								//very weak constraint for picking
								p2p->m_setting.m_tau = 0.1f;
							}
						}
					}
				}
			} else
			{
				if (gPickingConstraintId && m_physicsEnvironmentPtr)
				{
					m_physicsEnvironmentPtr->removeConstraint(gPickingConstraintId);
					//printf("removed constraint %i",gPickingConstraintId);
					gPickingConstraintId = 0;
					pickedBody->ForceActivationState(ACTIVE_TAG);
					pickedBody->m_deactivationTime = 0.f;
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

	if (gPickingConstraintId && m_physicsEnvironmentPtr)
	{

		//move the constraint pivot

		Point2PointConstraint* p2p = static_cast<Point2PointConstraint*>(m_physicsEnvironmentPtr->getConstraintById(gPickingConstraintId));
		if (p2p)
		{
			//keep it at the same picking distance

			SimdVector3 newRayTo = GetRayTo(x,y);
			SimdVector3 eyePos(m_cameraPosition[0],m_cameraPosition[1],m_cameraPosition[2]);
			SimdVector3 dir = newRayTo-eyePos;
			dir.normalize();
			dir *= gOldPickingDist;

			SimdVector3 newPos = eyePos + dir;
			p2p->SetPivotB(newPos);
		}

	}
}


///Very basic import
CcdPhysicsController*  DemoApplication::LocalCreatePhysicsObject(bool isDynamic, float mass, const SimdTransform& startTransform,CollisionShape* shape)
{

	startTransforms[numObjects] = startTransform;

	PHY_ShapeProps shapeProps;

	shapeProps.m_do_anisotropic = false;
	shapeProps.m_do_fh = false;
	shapeProps.m_do_rot_fh = false;
	shapeProps.m_friction_scaling[0] = 1.;
	shapeProps.m_friction_scaling[1] = 1.;
	shapeProps.m_friction_scaling[2] = 1.;

	shapeProps.m_inertia = 1.f;
	shapeProps.m_lin_drag = 0.2f;
	shapeProps.m_ang_drag = 0.1f;
	shapeProps.m_mass = 10.0f;

	PHY_MaterialProps materialProps;
	materialProps.m_friction = 10.5f;
	materialProps.m_restitution = 0.0f;

	CcdConstructionInfo ccdObjectCi;
	ccdObjectCi.m_friction = 0.5f;

	ccdObjectCi.m_linearDamping = shapeProps.m_lin_drag;
	ccdObjectCi.m_angularDamping = shapeProps.m_ang_drag;

	SimdTransform tr;
	tr.setIdentity();

	int i = numObjects;
	{
		gShapePtr[i] = shape;

		shapeProps.m_shape = gShapePtr[i];
		shapeProps.m_shape->SetMargin(0.05f);

		SimdQuaternion orn = startTransform.getRotation();


		ms[i].setWorldOrientation(orn[0],orn[1],orn[2],orn[3]);
		ms[i].setWorldPosition(startTransform.getOrigin().getX(),startTransform.getOrigin().getY(),startTransform.getOrigin().getZ());

		ccdObjectCi.m_MotionState = &ms[i];
		ccdObjectCi.m_gravity = SimdVector3(0,-9.8,0);
		ccdObjectCi.m_localInertiaTensor =SimdVector3(0,0,0);
		if (!isDynamic)
		{
			shapeProps.m_mass = 0.f;
			ccdObjectCi.m_mass = shapeProps.m_mass;
			ccdObjectCi.m_collisionFlags = CollisionObject::isStatic;
			ccdObjectCi.m_collisionFilterGroup = CcdConstructionInfo::StaticFilter;
			ccdObjectCi.m_collisionFilterMask = CcdConstructionInfo::AllFilter ^ CcdConstructionInfo::StaticFilter;
		}
		else
		{
			shapeProps.m_mass = mass;
			ccdObjectCi.m_mass = shapeProps.m_mass;
			ccdObjectCi.m_collisionFlags = 0;
		}


		SimdVector3 localInertia(0.f,0.f,0.f);

		if (isDynamic)
		{
			gShapePtr[i]->CalculateLocalInertia(shapeProps.m_mass,localInertia);
		}

		ccdObjectCi.m_localInertiaTensor = localInertia;
		ccdObjectCi.m_collisionShape = gShapePtr[i];


		physObjects[i]= new CcdPhysicsController( ccdObjectCi);

		// Only do CCD if  motion in one timestep (1.f/60.f) exceeds CUBE_HALF_EXTENTS
		physObjects[i]->GetRigidBody()->m_ccdSquareMotionTreshold = 0.f; 

		//Experimental: better estimation of CCD Time of Impact:
		//physObjects[i]->GetRigidBody()->m_ccdSweptShereRadius = 0.5*CUBE_HALF_EXTENTS;

		m_physicsEnvironmentPtr->addCcdPhysicsController( physObjects[i]);

	}

	//return newly created PhysicsController
	return physObjects[numObjects++];
}

void DemoApplication::renderme()
{
	updateCamera();

	float m[16];

	if (m_physicsEnvironmentPtr)
	{

		if (getDebugMode() & IDebugDraw::DBG_DisableBulletLCP)
		{
			//don't use Bullet, use quickstep
			m_physicsEnvironmentPtr->setSolverType(0);
		} else
		{
			//Bullet LCP solver
			m_physicsEnvironmentPtr->setSolverType(1);
		}

		if (getDebugMode() & IDebugDraw::DBG_EnableCCD)
		{
			m_physicsEnvironmentPtr->setCcdMode(3);
		} else
		{
			m_physicsEnvironmentPtr->setCcdMode(0);
		}


		bool isSatEnabled = (getDebugMode() & IDebugDraw::DBG_EnableSatComparison);

		m_physicsEnvironmentPtr->EnableSatCollisionDetection(isSatEnabled);



		int numPhysicsObjects = m_physicsEnvironmentPtr->GetNumControllers();
		
		int i;

		for (i=0;i<numPhysicsObjects;i++)
		{

			CcdPhysicsController* ctrl = m_physicsEnvironmentPtr->GetPhysicsController(i);
			RigidBody* body = ctrl->GetRigidBody();
			
			body->m_worldTransform.getOpenGLMatrix( m );

			SimdVector3 wireColor(1.f,1.0f,0.5f); //wants deactivation
			if (i & 1)
			{
				wireColor = SimdVector3(0.f,0.0f,1.f);
			}
			///color differently for active, sleeping, wantsdeactivation states
			if (ctrl->GetRigidBody()->GetActivationState() == 1) //active
			{
				if (i & 1)
				{
					wireColor += SimdVector3 (1.f,0.f,0.f);
				} else
				{			
					wireColor += SimdVector3 (.5f,0.f,0.f);
				}
			}
			if (ctrl->GetRigidBody()->GetActivationState() == 2) //ISLAND_SLEEPING
			{
				if (i & 1)
				{
					wireColor += SimdVector3 (0.f,1.f, 0.f);
				} else
				{
					wireColor += SimdVector3 (0.f,0.5f,0.f);
				}
			}

			char	extraDebug[125];
			sprintf(extraDebug,"islId, Body=%i , %i",ctrl->GetRigidBody()->m_islandTag1,ctrl->GetRigidBody()->m_debugBodyId);
			ctrl->GetRigidBody()->GetCollisionShape()->SetExtraDebugInfo(extraDebug);

			float vec[16];
			SimdTransform ident;
			ident.setIdentity();
			ident.getOpenGLMatrix(vec);
			

			GL_ShapeDrawer::DrawOpenGL(m,ctrl->GetRigidBody()->GetCollisionShape(),wireColor,getDebugMode());

			


		}

		if (!(getDebugMode() & IDebugDraw::DBG_NoHelpText))
		{

			float xOffset = 10.f;
			float yStart = 20.f;

			float yIncr = -2.f;

			char buf[124];

			glColor3f(0, 0, 0);

	#ifdef USE_QUICKPROF


			if ( getDebugMode() & IDebugDraw::DBG_ProfileTimings)
			{
				static int counter = 0;
				counter++;
				std::map<std::string, hidden::ProfileBlock*>::iterator iter;
				for (iter = Profiler::mProfileBlocks.begin(); iter != Profiler::mProfileBlocks.end(); ++iter)
				{
					char blockTime[128];
					sprintf(blockTime, "%s: %lf",&((*iter).first[0]),Profiler::getBlockTime((*iter).first, Profiler::BLOCK_CYCLE_SECONDS));//BLOCK_TOTAL_PERCENT));
					glRasterPos3f(xOffset,yStart,0);
					BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),blockTime);
					yStart += yIncr;

				}
			}
	#endif //USE_QUICKPROF
			//profiling << Profiler::createStatsString(Profiler::BLOCK_TOTAL_PERCENT); 
			//<< std::endl;



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
			sprintf(buf,"d to toggle deactivation");
			BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
			yStart += yIncr;

			glRasterPos3f(xOffset,yStart,0);
			sprintf(buf,"a to draw temporal AABBs");
			BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
			yStart += yIncr;


			glRasterPos3f(xOffset,yStart,0);
			sprintf(buf,"h to toggle help text");
			BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
			yStart += yIncr;

			bool useBulletLCP = !(getDebugMode() & IDebugDraw::DBG_DisableBulletLCP);

			bool useCCD = (getDebugMode() & IDebugDraw::DBG_EnableCCD);

			glRasterPos3f(xOffset,yStart,0);
			sprintf(buf,"m Bullet GJK = %i",!isSatEnabled);
			BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
			yStart += yIncr;

			glRasterPos3f(xOffset,yStart,0);
			sprintf(buf,"n Bullet LCP = %i",useBulletLCP);
			BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
			yStart += yIncr;

			glRasterPos3f(xOffset,yStart,0);
			sprintf(buf,"1 CCD mode (adhoc) = %i",useCCD);
			BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
			yStart += yIncr;

			glRasterPos3f(xOffset,yStart,0);
			sprintf(buf,"+- shooting speed = %10.2f",m_ShootBoxInitialSpeed);
			BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
			yStart += yIncr;


		}

	}

}
