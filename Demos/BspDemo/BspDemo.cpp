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

//#include "GL_LineSegmentShape.h"
#include "CollisionShapes/BoxShape.h"
#include "CollisionShapes/SphereShape.h"
#include "CollisionShapes/CylinderShape.h"
#include "CollisionShapes/ConeShape.h"
#include "CollisionShapes/StaticPlaneShape.h"
#include "CollisionShapes/ConvexHullShape.h"
#include "CollisionShapes/TriangleMesh.h"
#include "CollisionShapes/ConvexTriangleMeshShape.h"
#include "CollisionShapes/TriangleMeshShape.h"
#include "CollisionShapes/TriangleIndexVertexArray.h"
#include "CollisionShapes/CompoundShape.h"


extern SimdVector3 gCameraUp;
extern int	gForwardAxis;

#include "CollisionShapes/Simplex1to4Shape.h"
#include "CollisionShapes/EmptyShape.h"

#include "Dynamics/RigidBody.h"
#include "CollisionDispatch/CollisionDispatcher.h"
#include "BroadphaseCollision/SimpleBroadphase.h"
#include "BroadphaseCollision/AxisSweep3.h"
#include "ConstraintSolver/Point2PointConstraint.h"
#include "ConstraintSolver/HingeConstraint.h"

#include "quickprof.h"
#include "IDebugDraw.h"

#include "GLDebugDrawer.h"


#define QUAKE_BSP_IMPORTING 1

#ifdef QUAKE_BSP_IMPORTING
#include "BspLoader.h"
#include "BspConverter.h"
#endif //QUAKE_BSP_IMPORTING


#include "PHY_Pro.h"
#include "BMF_Api.h"
#include <stdio.h> //printf debugging

float deltaTime = 1.f/60.f;
float bulletSpeed = 40.f;

#ifdef WIN32
#if _MSC_VER >= 1310
//only use SIMD Hull code under Win32
#define USE_HULL 1
#include "NarrowPhaseCollision/Hull.h"
#endif //_MSC_VER 
#endif //WIN32


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
#include "GL_ShapeDrawer.h"

#include "GlutStuff.h"


extern float eye[3];
extern int glutScreenWidth;
extern int glutScreenHeight;



int numObjects = 0;

const int maxNumObjects = 450;

SimdTransform startTransforms[maxNumObjects];


DefaultMotionState ms[maxNumObjects];
CcdPhysicsController* physObjects[maxNumObjects] = {0,0,0,0};

CcdPhysicsEnvironment* physicsEnvironmentPtr = 0;


#define CUBE_HALF_EXTENTS 1
#define EXTRA_HEIGHT -20.f


CollisionShape* gShapePtr[maxNumObjects];//1 rigidbody has 1 shape (no re-use of shapes)

////////////////////////////////////

///Very basic import
CcdPhysicsController*  CreatePhysicsObject(bool isDynamic, float mass, const SimdTransform& startTransform,CollisionShape* shape)
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

		physicsEnvironmentPtr->addCcdPhysicsController( physObjects[i]);

	}

	//return newly created PhysicsController
	return physObjects[numObjects++];
}



///BspToBulletConverter  extends the BspConverter to convert to Bullet datastructures
class BspToBulletConverter : public BspConverter
{
public:

		virtual void	AddConvexVerticesCollider(std::vector<SimdVector3>& vertices, bool isEntity, const SimdVector3& entityTargetLocation)
		{
			///perhaps we can do something special with entities (isEntity)
			///like adding a collision Triggering (as example)
			
			if (vertices.size() > 0)
			{
				bool isDynamic = false;
				float mass = 0.f;
				SimdTransform startTransform;
				//can use a shift
				startTransform.setIdentity();
				startTransform.setOrigin(SimdVector3(0,0,-10.f));
				//this create an internal copy of the vertices
				CollisionShape* shape = new ConvexHullShape(&vertices[0],vertices.size());

				CreatePhysicsObject(isDynamic, mass, startTransform,shape);
			}
		}
};





////////////////////////////////////



GLDebugDrawer debugDrawer;

char* makeExeToBspFilename(const char* lpCmdLine);
char* getLastFileName();


int main(int argc,char** argv)
{

	char* bspfilename = "BspDemo.bsp";

	printf("argc=%i\n",argc);
	{
		for (int i=0;i<argc;i++)
		{
			printf("argv[%i]=%s\n",i,argv[i]);
		}
		
		bspfilename = makeExeToBspFilename(argv[0]);
		printf("new name=%s\n",bspfilename);
	}
	if (argc>1)
	{
		bspfilename = argv[1];
	}

	gCameraUp = SimdVector3(0,0,1);
	gForwardAxis = 1;

	///Setup a Physics Simulation Environment
	CollisionDispatcher* dispatcher = new	CollisionDispatcher();
	SimdVector3 worldAabbMin(-10000,-10000,-10000);
	SimdVector3 worldAabbMax(10000,10000,10000);
	OverlappingPairCache* broadphase = new AxisSweep3(worldAabbMin,worldAabbMax);
	//BroadphaseInterface* broadphase = new SimpleBroadphase();
	physicsEnvironmentPtr = new CcdPhysicsEnvironment(dispatcher,broadphase);
	physicsEnvironmentPtr->setDeactivationTime(2.f);
	physicsEnvironmentPtr->setGravity(0,0,-10);
	physicsEnvironmentPtr->setDebugDrawer(&debugDrawer);



#ifdef QUAKE_BSP_IMPORTING

	void* memoryBuffer = 0;
	
	FILE* file = fopen(bspfilename,"r");
	if (!file)
	{
		//try again other path, 
		//sight... visual studio leaves the current working directory in the projectfiles folder
		//instead of executable folder. who wants this default behaviour?!?
		bspfilename = "../../bsptest.bsp";
		file = fopen(bspfilename,"r");
	}
	if (file)
	{
		BspLoader bspLoader;
		int size=0;
		if (fseek(file, 0, SEEK_END) || (size = ftell(file)) == EOF || fseek(file, 0, SEEK_SET)) {        /* File operations denied? ok, just close and return failure */
			printf("Error: cannot get filesize from %s\n", bspfilename);
		} else
		{
			//how to detect file size?
			memoryBuffer = malloc(size+1);
			fread(memoryBuffer,1,size,file);
			bspLoader.LoadBSPFile( memoryBuffer);

			BspToBulletConverter bsp2bullet;
			float bspScaling = 0.1f;
			bsp2bullet.convertBsp(bspLoader,bspScaling);

		}
		fclose(file);
	}

#endif




	clientResetScene();

	setCameraDistance(22.f);

	return glutmain(argc, argv,640,480,"Bullet Quake BSP Physics Viewer http://bullet.sourceforge.net");
}

//to be implemented by the demo
void renderme()
{
	debugDrawer.SetDebugMode(getDebugMode());



	float m[16];
	int i;


	if (getDebugMode() & IDebugDraw::DBG_DisableBulletLCP)
	{
		//don't use Bullet, use quickstep
		physicsEnvironmentPtr->setSolverType(0);
	} else
	{
		//Bullet LCP solver
		physicsEnvironmentPtr->setSolverType(1);
	}

	if (getDebugMode() & IDebugDraw::DBG_EnableCCD)
	{
		physicsEnvironmentPtr->setCcdMode(3);
	} else
	{
		physicsEnvironmentPtr->setCcdMode(0);
	}


	bool isSatEnabled = (getDebugMode() & IDebugDraw::DBG_EnableSatComparison);

	physicsEnvironmentPtr->EnableSatCollisionDetection(isSatEnabled);



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


		SimdVector3 wireColor(1.f,1.0f,0.5f); //wants deactivation
		if (i & 1)
		{
			wireColor = SimdVector3(0.f,0.0f,1.f);
		}
		///color differently for active, sleeping, wantsdeactivation states
		if (physObjects[i]->GetRigidBody()->GetActivationState() == 1) //active
		{
			if (i & 1)
			{
				wireColor += SimdVector3 (1.f,0.f,0.f);
			} else
			{			
				wireColor += SimdVector3 (.5f,0.f,0.f);
			}
		}
		if (physObjects[i]->GetRigidBody()->GetActivationState() == 2) //ISLAND_SLEEPING
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

		sprintf(extraDebug,"islandId=%i, Body=%i, ShapeType=%s",physObjects[i]->GetRigidBody()->m_islandTag1,physObjects[i]->GetRigidBody()->m_debugBodyId,physObjects[i]->GetRigidBody()->GetCollisionShape()->GetName());
		physObjects[i]->GetRigidBody()->GetCollisionShape()->SetExtraDebugInfo(extraDebug);
		GL_ShapeDrawer::DrawOpenGL(m,physObjects[i]->GetRigidBody()->GetCollisionShape(),wireColor,getDebugMode());

		///this block is just experimental code to show some internal issues with replacing shapes on the fly.
		if (getDebugMode()!=0 && (i>0))
		{
			if (physObjects[i]->GetRigidBody()->GetCollisionShape()->GetShapeType() == EMPTY_SHAPE_PROXYTYPE)
			{
				physObjects[i]->GetRigidBody()->SetCollisionShape(gShapePtr[1]);

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

	if (!(getDebugMode() & IDebugDraw::DBG_NoHelpText))
	{

		float xOffset = 10.f;
		float yStart = 20.f;

		float yIncr = -2.f;

		SimdVector3 offset(xOffset,0,0);
		SimdVector3 up = gCameraUp;
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


		SimdVector3 textPos = offset + up*yStart;
		glRasterPos3f(textPos.getX(),textPos.getY(),textPos.getZ());
		sprintf(buf,"leftmouse to pick");
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);

		yStart += yIncr;
		textPos = offset + up*yStart;
		glRasterPos3f(textPos.getX(),textPos.getY(),textPos.getZ());
		sprintf(buf,"rightmouse or . to shoot box");
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;

		textPos = offset + up*yStart;
		glRasterPos3f(textPos.getX(),textPos.getY(),textPos.getZ());
		sprintf(buf,"space to reset");
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;

		textPos = offset + up*yStart;
		glRasterPos3f(textPos.getX(),textPos.getY(),textPos.getZ());

		sprintf(buf,"cursor keys and z,x to navigate");
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;

		textPos = offset + up*yStart ;
		glRasterPos3f(textPos.getX(),textPos.getY(),textPos.getZ());

		sprintf(buf,"i to toggle simulation, s single step");
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;

		textPos = offset + up*yStart ;
		glRasterPos3f(textPos.getX(),textPos.getY(),textPos.getZ());

		sprintf(buf,"q to quit");
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;

		textPos = offset + up*yStart ;
		glRasterPos3f(textPos.getX(),textPos.getY(),textPos.getZ());

		sprintf(buf,"d to toggle deactivation");
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;

		textPos = offset + up*yStart ;
		glRasterPos3f(textPos.getX(),textPos.getY(),textPos.getZ());

		sprintf(buf,"a to draw temporal AABBs");
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;

	
		textPos = offset + up*yStart ;
		glRasterPos3f(textPos.getX(),textPos.getY(),textPos.getZ());
		sprintf(buf,"c to show contact points (wireframe more)");
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;


		textPos = offset + up*yStart ;
		glRasterPos3f(textPos.getX(),textPos.getY(),textPos.getZ());


		sprintf(buf,"h to toggle help text");
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;

		bool useBulletLCP = !(getDebugMode() & IDebugDraw::DBG_DisableBulletLCP);

		bool useCCD = (getDebugMode() & IDebugDraw::DBG_EnableCCD);

		textPos = offset + up*yStart ;
		glRasterPos3f(textPos.getX(),textPos.getY(),textPos.getZ());


		sprintf(buf,"m Bullet GJK = %i",!isSatEnabled);
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;

		textPos = offset + up*yStart ;
		glRasterPos3f(textPos.getX(),textPos.getY(),textPos.getZ());

		sprintf(buf,"n Bullet LCP = %i",useBulletLCP);
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;

		textPos = offset + up*yStart ;
		glRasterPos3f(textPos.getX(),textPos.getY(),textPos.getZ());

		sprintf(buf,"1 CCD mode (adhoc) = %i",useCCD);
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;

		textPos = offset + up*yStart ;
		glRasterPos3f(textPos.getX(),textPos.getY(),textPos.getZ());

		sprintf(buf,"+- shooting speed = %10.2f",bulletSpeed);
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
		yStart += yIncr;

	}

}

void clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	physicsEnvironmentPtr->proceedDeltaTime(0.f,deltaTime);

	renderme();

	glFlush();
	glutSwapBuffers();

}



void clientDisplay(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 


	physicsEnvironmentPtr->UpdateAabbs(deltaTime);

	renderme();


	glFlush();
	glutSwapBuffers();
}



///make this positive to show stack falling from a distance
///this shows the penalty tresholds in action, springy/spungy look

void clientResetScene()
{
	for (int i=0;i<numObjects;i++)
	{
		ms[i].m_worldTransform = startTransforms[i];
		physObjects[i]->setPosition(startTransforms[i].getOrigin().getX(),startTransforms[i].getOrigin().getY(),startTransforms[i].getOrigin().getZ());
		physObjects[i]->SetLinearVelocity(0,0,0,0);
		physObjects[i]->SetAngularVelocity(0,0,0,0);
		SimdQuaternion orn;
		startTransforms[i].getBasis().getRotation(orn);
		physObjects[i]->setOrientation(orn.x(),orn.y(),orn.z(),orn[3]);

	}

	//delete and reload, or keep transforms ready?
}



void	shootBox(const SimdVector3& destination)
{

	bool isDynamic = true;
	float mass = 1.f;
	SimdTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(SimdVector3(eye[0],eye[1],eye[2]));
	CollisionShape* boxShape = new BoxShape(SimdVector3(1.f,1.f,1.f));

	CreatePhysicsObject(isDynamic, mass, startTransform,boxShape);

	int i  = numObjects-1;



	SimdVector3 linVel(destination[0]-eye[0],destination[1]-eye[1],destination[2]-eye[2]);
	linVel.normalize();
	linVel*=bulletSpeed;

	physObjects[i]->setPosition(eye[0],eye[1],eye[2]);
	physObjects[i]->setOrientation(0,0,0,1);
	physObjects[i]->SetLinearVelocity(linVel[0],linVel[1],linVel[2],false);
	physObjects[i]->SetAngularVelocity(0,0,0,false);
}

void clientKeyboard(unsigned char key, int x, int y)
{

	if (key == '.')
	{
		shootBox(SimdVector3(0,0,0));
	}

	if (key == '+')
	{
		bulletSpeed += 10.f;
	}
	if (key == '-')
	{
		bulletSpeed -= 10.f;
	}

	defaultKeyboard(key, x, y);
}

int gPickingConstraintId = 0;
SimdVector3 gOldPickingPos;
float gOldPickingDist  = 0.f;
RigidBody* pickedBody = 0;//for deactivation state



SimdVector3	GetRayTo(int x,int y)
{
	float top = 1.f;
	float bottom = -1.f;
	float nearPlane = 1.f;
	float tanFov = (top-bottom)*0.5f / nearPlane;
	float fov = 2.0 * atanf (tanFov);

	SimdVector3	rayFrom(eye[0],eye[1],eye[2]);
	SimdVector3 rayForward = -rayFrom;
	rayForward.normalize();
	float farPlane = 600.f;
	rayForward*= farPlane;

	SimdVector3 rightOffset;
	SimdVector3 vertical = gCameraUp;

	SimdVector3 hor;
	hor = rayForward.cross(vertical);
	hor.normalize();
	vertical = hor.cross(rayForward);
	vertical.normalize();

	float tanfov = tanf(0.5f*fov);
	hor *= 2.f * farPlane * tanfov;
	vertical *= 2.f * farPlane * tanfov;
	SimdVector3 rayToCenter = rayFrom + rayForward;
	SimdVector3 dHor = hor * 1.f/float(glutScreenWidth);
	SimdVector3 dVert = vertical * 1.f/float(glutScreenHeight);
	SimdVector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
	rayTo += x * dHor;
	rayTo -= y * dVert;
	return rayTo;
}
void clientMouseFunc(int button, int state, int x, int y)
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
				if (physicsEnvironmentPtr)
				{
					float hit[3];
					float normal[3];
					PHY_IPhysicsController* hitObj = physicsEnvironmentPtr->rayTest(0,eye[0],eye[1],eye[2],rayTo.getX(),rayTo.getY(),rayTo.getZ(),hit[0],hit[1],hit[2],normal[0],normal[1],normal[2]);
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
				if (physicsEnvironmentPtr)
				{
					float hit[3];
					float normal[3];
					PHY_IPhysicsController* hitObj = physicsEnvironmentPtr->rayTest(0,eye[0],eye[1],eye[2],rayTo.getX(),rayTo.getY(),rayTo.getZ(),hit[0],hit[1],hit[2],normal[0],normal[1],normal[2]);
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

							gPickingConstraintId = physicsEnvironmentPtr->createConstraint(physCtrl,0,PHY_POINT2POINT_CONSTRAINT,
								localPivot.getX(),
								localPivot.getY(),
								localPivot.getZ(),
								0,0,0);
							//printf("created constraint %i",gPickingConstraintId);

							//save mouse position for dragging
							gOldPickingPos = rayTo;


							SimdVector3 eyePos(eye[0],eye[1],eye[2]);

							gOldPickingDist  = (pickPos-eyePos).length();

							Point2PointConstraint* p2p = static_cast<Point2PointConstraint*>(physicsEnvironmentPtr->getConstraintById(gPickingConstraintId));
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
				if (gPickingConstraintId && physicsEnvironmentPtr)
				{
					physicsEnvironmentPtr->removeConstraint(gPickingConstraintId);
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

void	clientMotionFunc(int x,int y)
{

	if (gPickingConstraintId && physicsEnvironmentPtr)
	{

		//move the constraint pivot

		Point2PointConstraint* p2p = static_cast<Point2PointConstraint*>(physicsEnvironmentPtr->getConstraintById(gPickingConstraintId));
		if (p2p)
		{
			//keep it at the same picking distance

			SimdVector3 newRayTo = GetRayTo(x,y);
			SimdVector3 eyePos(eye[0],eye[1],eye[2]);
			SimdVector3 dir = newRayTo-eyePos;
			dir.normalize();
			dir *= gOldPickingDist;

			SimdVector3 newPos = eyePos + dir;
			p2p->SetPivotB(newPos);
		}

	}
}

//some code that de-mangles the windows filename passed in as argument
char cleaned_filename[512];
char* getLastFileName()
{
	return cleaned_filename;
}
char* makeExeToBspFilename(const char* lpCmdLine)
{


	// We might get a windows-style path on the command line, this can mess up the DOM which expects
	// all paths to be URI's.  This block of code does some conversion to try and make the input
	// compliant without breaking the ability to accept a properly formatted URI.  Right now this only
	// displays the first filename
	const char *in = lpCmdLine;
	char* out = cleaned_filename;
	*out = NULL;
	// If the first character is a ", skip it (filenames with spaces in them are quoted)
	if(*in == '\"')
	{
		in++;
	}
	if(*(in+1) == ':')
	{
		// Second character is a :, assume we have a path with a drive letter and add a slash at the beginning
		*(out++) = '/';
	}
	int i;
	for(i =0; i<512; i++)
	{
		//if we get '.' we stop as well, unless it's the first character. Then we add .bsp as extension
		// If we hit a null or a quote, stop copying.  This will get just the first filename.
		if(i && (*in == '.'))
			break;
			
		// If we hit a null or a quote, stop copying.  This will get just the first filename.
		if(*in == NULL || *in == '\"')
			break;
		// Copy while swapping backslashes for forward ones
		if(*in == '\\')
		{
			*out = '/';
		}
		else
		{
			*out = *in;
		}
		in++;
		out++;
	}
	*(out++) = '.';
	*(out++) = 'b';
	*(out++) = 's';
	*(out++) = 'p';
	*(out++) = 0;
	
	return cleaned_filename;
}
