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

#include "CcdPhysicsEnvironment.h"
#include "CcdPhysicsController.h"
#include "MyMotionState.h"
//#include "GL_LineSegmentShape.h"
#include "CollisionShapes/BoxShape.h"
#include "CollisionShapes/Simplex1to4Shape.h"
#include "CollisionShapes/EmptyShape.h"

#include "Dynamics/RigidBody.h"
#include "ConstraintSolver/SimpleConstraintSolver.h"
#include "ConstraintSolver/OdeConstraintSolver.h"
#include "CollisionDispatch/CollisionDispatcher.h"
#include "BroadphaseCollision/SimpleBroadphase.h"
#include "IDebugDraw.h"

#include "GLDebugDrawer.h"

#include "PHY_Pro.h"
#include "BMF_Api.h"
#include <stdio.h> //printf debugging

#ifdef WIN32 //needed for glut.h
#include <windows.h>
#endif
#include <GL/glut.h>
#include "GL_ShapeDrawer.h"

#include "GlutStuff.h"

const int numObjects = 3;

const int maxNumObjects = 400;
MyMotionState ms[maxNumObjects];
CcdPhysicsController* physObjects[maxNumObjects] = {0,0,0,0};
int	shapeIndex[maxNumObjects];
CcdPhysicsEnvironment* physicsEnvironmentPtr = 0;


#define CUBE_HALF_EXTENTS 1

//GL_LineSegmentShape shapeE(SimdPoint3(-50,0,0),
//						   SimdPoint3(50,0,0));
CollisionShape* shapePtr[4] = 
{
	new BoxShape (SimdVector3(10,10,10)),
	new BoxShape (SimdVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS)),
	//new EmptyShape(),
	new BU_Simplex1to4(SimdPoint3(-1,-1,-1),SimdPoint3(1,-1,-1),SimdPoint3(-1,1,-1),SimdPoint3(0,0,1)),
	//new BoxShape (SimdVector3(0.4,1,0.8))

};


	GLDebugDrawer debugDrawer;

int main(int argc,char** argv)
{


	ConstraintSolver* solver = new SimpleConstraintSolver;
	//ConstraintSolver* solver = new OdeConstraintSolver;

	CollisionDispatcher* dispatcher = new	CollisionDispatcher();
		
	BroadphaseInterface* broadphase = new SimpleBroadphase();


	physicsEnvironmentPtr = new CcdPhysicsEnvironment(dispatcher,broadphase);
	physicsEnvironmentPtr->setDeactivationTime(0.f);


	physicsEnvironmentPtr->setGravity(0,-10,0);
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

	int i;
	for (i=0;i<numObjects;i++)
	{
		if (i>0)
			shapeIndex[i] = 1;//2 to test start with EmptyShape
		else
			shapeIndex[i] = 0;
	}
	for (i=0;i<numObjects;i++)
	{
		shapeProps.m_shape = shapePtr[shapeIndex[i]];
		shapeProps.m_shape->SetMargin(0.05f);


		bool isDyna = i>0;
		
		if (0)//i==1)
		{
			SimdQuaternion orn(0,0,0.1*SIMD_HALF_PI);
			ms[i].setWorldOrientation(orn.x(),orn.y(),orn.z(),orn[3]);
		}
		

		if (i>0)
		{
			ms[i].setWorldPosition(0,i*CUBE_HALF_EXTENTS*2 - CUBE_HALF_EXTENTS,0);
		} else
		{
			ms[i].setWorldPosition(0,-10,0);

		}
		
		ccdObjectCi.m_MotionState = &ms[i];
		ccdObjectCi.m_gravity = SimdVector3(0,0,0);
		ccdObjectCi.m_localInertiaTensor =SimdVector3(0,0,0);
		if (!isDyna)
		{
			shapeProps.m_mass = 0.f;
			ccdObjectCi.m_mass = shapeProps.m_mass;
		}
		else
		{
			shapeProps.m_mass = 1.f;
			ccdObjectCi.m_mass = shapeProps.m_mass;
		}

		
		SimdVector3 localInertia;
		if (shapePtr[shapeIndex[i]]->GetShapeType() == EMPTY_SHAPE_PROXYTYPE)
		{
			//take inertia from first shape
			shapePtr[1]->CalculateLocalInertia(shapeProps.m_mass,localInertia);
		} else
		{
			shapePtr[shapeIndex[i]]->CalculateLocalInertia(shapeProps.m_mass,localInertia);
		}
		ccdObjectCi.m_localInertiaTensor = localInertia;

		ccdObjectCi.m_collisionShape = shapePtr[shapeIndex[i]];


		physObjects[i]= new CcdPhysicsController( ccdObjectCi);
		physicsEnvironmentPtr->addCcdPhysicsController( physObjects[i]);

		

		physicsEnvironmentPtr->setDebugDrawer(&debugDrawer);
		
	}
	
	clientResetScene();
	{
		//physObjects[i]->SetAngularVelocity(0,0,-2,true);
		int constraintId;

			float pivotX=CUBE_HALF_EXTENTS,
				pivotY=CUBE_HALF_EXTENTS,
				pivotZ=CUBE_HALF_EXTENTS;
			float axisX=0,axisY=1,axisZ=0;


		constraintId =physicsEnvironmentPtr->createConstraint(
		physObjects[1],
		//0,
		physObjects[2],
			PHY_POINT2POINT_CONSTRAINT,
			//PHY_LINEHINGE_CONSTRAINT,
			pivotX,pivotY,pivotZ,
			axisX,axisY,axisZ
			);

	}

	setCameraDistance(46.f);

	return glutmain(argc, argv,640,480,"Constraint Demo. http://www.continuousphysics.com/Bullet/phpBB2/");
}

//to be implemented by the demo


void renderme()
{
	debugDrawer.SetDebugMode(getDebugMode());


	float m[16];
	int i;

	for (i=0;i<numObjects;i++)
	{
		SimdTransform transA;
		transA.setIdentity();
		
		float pos[3];
		float rot[4];

		ms[i].getWorldPosition(pos[0],pos[1],pos[2]);
		ms[i].getWorldOrientation(rot[0],rot[1],rot[2],rot[3]);

		SimdQuaternion q(rot[0],rot[1],rot[2],rot[3]);
		transA.setRotation(q);

		SimdPoint3 dpos;
		dpos.setValue(pos[0],pos[1],pos[2]);

		transA.setOrigin( dpos );
		transA.getOpenGLMatrix( m );
		
		SimdVector3 wireColor(0.f,0.f,1.f); //wants deactivation

		///color differently for active, sleeping, wantsdeactivation states
		if (physObjects[i]->GetRigidBody()->GetActivationState() == 1) //active
		{
			wireColor = SimdVector3 (1.f,0.f,0.f);
		}
		if (physObjects[i]->GetRigidBody()->GetActivationState() == 2) //ISLAND_SLEEPING
		{
			wireColor = SimdVector3 (0.f,1.f,0.f);
		}

		char	extraDebug[125];
		sprintf(extraDebug,"islId, Body=%i , %i",physObjects[i]->GetRigidBody()->m_islandTag1,physObjects[i]->GetRigidBody()->m_debugBodyId);
		physObjects[i]->GetRigidBody()->GetCollisionShape()->SetExtraDebugInfo(extraDebug);
		GL_ShapeDrawer::DrawOpenGL(m,physObjects[i]->GetRigidBody()->GetCollisionShape(),wireColor,getDebugMode());

		if (getDebugMode()!=0 && (i>0))
		{
			if (physObjects[i]->GetRigidBody()->GetCollisionShape()->GetShapeType() == EMPTY_SHAPE_PROXYTYPE)
			{
				physObjects[i]->GetRigidBody()->SetCollisionShape(shapePtr[1]);
			
				//remove the persistent collision pairs that were created based on the previous shape

				BroadphaseProxy* bpproxy = physObjects[i]->GetRigidBody()->m_broadphaseHandle;

				physicsEnvironmentPtr->GetBroadphase()->CleanProxyFromPairs(bpproxy);

				SimdVector3 newinertia;
				SimdScalar newmass = 10.f;
				physObjects[i]->GetRigidBody()->GetCollisionShape()->CalculateLocalInertia(newmass,newinertia);
				physObjects[i]->GetRigidBody()->setMassProps(newmass,newinertia);
				physObjects[i]->GetRigidBody()->updateInertiaTensor();

			}

		}


	}

	glRasterPos3f(20,20,0);
	char buf[124];
	sprintf(buf,"space to reset");
	BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
	glRasterPos3f(20,15,0);
	sprintf(buf,"c to show contact points");
	BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);

	glRasterPos3f(20,10,0);
	sprintf(buf,"cursor keys and z,x to navigate");
	BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);

	glRasterPos3f(20,5,0);
	sprintf(buf,"i to toggle simulation, s single step");
	BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);


	glRasterPos3f(20,0,0);
	sprintf(buf,"q to quit");
	BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);

	glRasterPos3f(20,-5,0);
	sprintf(buf,"d to toggle deactivation");
	BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);

}
void clientMoveAndDisplay()
{
	
	 glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	float deltaTime = 1.f/60.f;

	physicsEnvironmentPtr->proceedDeltaTime(0.f,deltaTime);
	
	renderme();

    glFlush();
    glutSwapBuffers();

}




void clientDisplay(void) {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

	

    glFlush();
    glutSwapBuffers();
}



///make this positive to show stack falling from a distance
///this shows the penalty tresholds in action, springy/spungy look
#define EXTRA_HEIGHT 20.f


void clientResetScene()
{
	int i;
	for (i=0;i<numObjects;i++)
	{
		if (i>0)
		{
			//stack them
			int colsize = 10;
			int row = (i*CUBE_HALF_EXTENTS*2)/(colsize*2*CUBE_HALF_EXTENTS);
			int col = (i)%(colsize)-colsize/2;

			physObjects[i]->setPosition(col*2*CUBE_HALF_EXTENTS + (row%2)*CUBE_HALF_EXTENTS,
				row*2*CUBE_HALF_EXTENTS+CUBE_HALF_EXTENTS+EXTRA_HEIGHT,
				0);
			physObjects[i]->setOrientation(0,0,0,1);
			physObjects[i]->SetLinearVelocity(0,0,0,false);
			physObjects[i]->SetAngularVelocity(0,0,0,false);
		} else
		{
			ms[i].setWorldPosition(0,-10,0);

		}
	}
}

void clientKeyboard(unsigned char key, int x, int y)
{
	defaultKeyboard(key, x, y);
}


void clientMouseFunc(int button, int state, int x, int y)
{

}
void	clientMotionFunc(int x,int y)
{
}