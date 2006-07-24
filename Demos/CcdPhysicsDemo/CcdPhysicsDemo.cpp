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

//#define USE_PARALLEL_DISPATCHER 1


#include "CcdPhysicsEnvironment.h"
#include "ParallelPhysicsEnvironment.h"

#include "CcdPhysicsController.h"
#include "MyMotionState.h"
//#include "GL_LineSegmentShape.h"
#include "CollisionShapes/BoxShape.h"
#include "CollisionShapes/SphereShape.h"
#include "CollisionShapes/ConeShape.h"
#include "CollisionShapes/StaticPlaneShape.h"
#include "CollisionShapes/CompoundShape.h"
#include "CollisionShapes/Simplex1to4Shape.h"
#include "CollisionShapes/EmptyShape.h"

#include "Dynamics/RigidBody.h"
#include "CollisionDispatch/CollisionDispatcher.h"

#include "ParallelIslandDispatcher.h"

#include "BroadphaseCollision/SimpleBroadphase.h"
#include "BroadphaseCollision/AxisSweep3.h"
#include "ConstraintSolver/Point2PointConstraint.h"
#include "ConstraintSolver/HingeConstraint.h"

#include "quickprof.h"
#include "IDebugDraw.h"

#include "GLDebugDrawer.h"




#include "PHY_Pro.h"
#include "BMF_Api.h"
#include <stdio.h> //printf debugging

float deltaTime = 1.f/60.f;
float bulletSpeed = 40.f;
bool createConstraint = true;
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

const int maxProxies = 32766;
const int maxOverlap = 65535;


#ifdef _DEBUG
const int numObjects = 20;
#else
const int numObjects = 120;
#endif

const int maxNumObjects = 32760;

MyMotionState ms[maxNumObjects];
CcdPhysicsController* physObjects[maxNumObjects] = {0,0,0,0};
int	shapeIndex[maxNumObjects];

#ifdef USE_PARALLEL_DISPATCHER
ParallelPhysicsEnvironment* physicsEnvironmentPtr = 0;
#else
CcdPhysicsEnvironment* physicsEnvironmentPtr = 0;
#endif

#define CUBE_HALF_EXTENTS 1

#define EXTRA_HEIGHT -20.f
//GL_LineSegmentShape shapeE(SimdPoint3(-50,0,0),
//						   SimdPoint3(50,0,0));
static const int numShapes = 4;

CollisionShape* shapePtr[numShapes] = 
{
	///Please don't make the box sizes larger then 1000: the collision detection will be inaccurate.
	///See http://www.continuousphysics.com/Bullet/phpBB2/viewtopic.php?t=346

//#define USE_GROUND_PLANE 1
#ifdef USE_GROUND_PLANE
	new StaticPlaneShape(SimdVector3(0,1,0),10),
#else
	new BoxShape (SimdVector3(450,10,450)),
#endif
		
		new BoxShape (SimdVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS)),
		new SphereShape (CUBE_HALF_EXTENTS- 0.05f),

		//new ConeShape(CUBE_HALF_EXTENTS,2.f*CUBE_HALF_EXTENTS),
		//new BU_Simplex1to4(SimdPoint3(-1,-1,-1),SimdPoint3(1,-1,-1),SimdPoint3(-1,1,-1),SimdPoint3(0,0,1)),

		//new EmptyShape(),

		new BoxShape (SimdVector3(0.4,1,0.8))

};



////////////////////////////////////






GLDebugDrawer debugDrawer;

int main(int argc,char** argv)
{


	CollisionDispatcher* dispatcher = new	CollisionDispatcher();
	ParallelIslandDispatcher* dispatcher2 = new	ParallelIslandDispatcher();
	
	SimdVector3 worldAabbMin(-30000,-30000,-30000);
	SimdVector3 worldAabbMax(30000,30000,30000);

	OverlappingPairCache* broadphase = new AxisSweep3(worldAabbMin,worldAabbMax,maxProxies,maxOverlap);
	//OverlappingPairCache* broadphase = new SimpleBroadphase(maxProxies,maxOverlap);

#ifdef USE_PARALLEL_DISPATCHER
	physicsEnvironmentPtr = new ParallelPhysicsEnvironment(dispatcher2,broadphase);
#else
	physicsEnvironmentPtr = new CcdPhysicsEnvironment(dispatcher,broadphase);
#endif
	physicsEnvironmentPtr->setDeactivationTime(2.f);

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

	

	SimdTransform tr;
	tr.setIdentity();

	int i;
	for (i=0;i<numObjects;i++)
	{
		if (i>0)
		{
			shapeIndex[i] = 1;//sphere
		}
		else
			shapeIndex[i] = 0;
	}

	CompoundShape* compoundShape = new CompoundShape();
	//shapePtr[1] = compoundShape;

	SimdTransform ident;
	ident.setIdentity();

	compoundShape->AddChildShape(ident,new BoxShape (SimdVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS)));

	for (i=0;i<numObjects;i++)
	{

		CcdConstructionInfo ccdObjectCi;
		ccdObjectCi.m_friction = 0.5f;

		ccdObjectCi.m_linearDamping = shapeProps.m_lin_drag;
		ccdObjectCi.m_angularDamping = shapeProps.m_ang_drag;

		shapeProps.m_shape = shapePtr[shapeIndex[i]];
		shapeProps.m_shape->SetMargin(0.05f);



		bool isDyna = i>0;
		//if (i==1)
		//	isDyna=false;

		if (0)//i==1)
		{
			SimdQuaternion orn(0,0,0.1*SIMD_HALF_PI);
			ms[i].setWorldOrientation(orn.x(),orn.y(),orn.z(),orn[3]);
		}


		if (i>0)
		{

			switch (i)
			{
			case 1:
				{
					ms[i].setWorldPosition(0,10,0);
					//for testing, rotate the ground cube so the stack has to recover a bit

					break;
				}
			case 2:
				{
					ms[i].setWorldPosition(0,8,2);
					break;
				}
			default:
				ms[i].setWorldPosition(0,i*CUBE_HALF_EXTENTS*2 - CUBE_HALF_EXTENTS,0);
			}

			float quatIma0,quatIma1,quatIma2,quatReal;
			SimdQuaternion quat;
			SimdVector3 axis(0,0,1);
			SimdScalar angle=0.5f;

			quat.setRotation(axis,angle);

			ms[i].setWorldOrientation(quat.getX(),quat.getY(),quat.getZ(),quat[3]);



		} else
		{
			ms[i].setWorldPosition(0,-10+EXTRA_HEIGHT,0);

		}

		ccdObjectCi.m_MotionState = &ms[i];
		ccdObjectCi.m_gravity = SimdVector3(0,0,0);
		ccdObjectCi.m_localInertiaTensor =SimdVector3(0,0,0);
		if (!isDyna)
		{
			shapeProps.m_mass = 0.f;
			ccdObjectCi.m_mass = shapeProps.m_mass;
			ccdObjectCi.m_collisionFlags = CollisionObject::isStatic;
			
			ccdObjectCi.m_collisionFilterGroup = CcdConstructionInfo::StaticFilter;
			ccdObjectCi.m_collisionFilterMask = CcdConstructionInfo::AllFilter ^ CcdConstructionInfo::StaticFilter;
		}
		else
		{
			shapeProps.m_mass = 1.f;
			ccdObjectCi.m_mass = shapeProps.m_mass;
			ccdObjectCi.m_collisionFlags = 0;

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

		// Only do CCD if  motion in one timestep (1.f/60.f) exceeds CUBE_HALF_EXTENTS
		physObjects[i]->GetRigidBody()->m_ccdSquareMotionTreshold = CUBE_HALF_EXTENTS;
		
		//Experimental: better estimation of CCD Time of Impact:
		//physObjects[i]->GetRigidBody()->m_ccdSweptShereRadius = 0.5*CUBE_HALF_EXTENTS;

		physicsEnvironmentPtr->addCcdPhysicsController( physObjects[i]);

		if (i==1)
		{
			//physObjects[i]->SetAngularVelocity(0,0,-2,true);
		}

		physicsEnvironmentPtr->setDebugDrawer(&debugDrawer);

	}


	clientResetScene();
	physicsEnvironmentPtr->SyncMotionStates(0.f);

	if (createConstraint)
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
			////PHY_POINT2POINT_CONSTRAINT,
			PHY_GENERIC_6DOF_CONSTRAINT,//can leave any of the 6 degree of freedom 'free' or 'locked'
			//PHY_LINEHINGE_CONSTRAINT,
			pivotX,pivotY,pivotZ,
			axisX,axisY,axisZ
			);

	}

	



	setCameraDistance(26.f);

	return glutmain(argc, argv,640,480,"Bullet Physics Demo. http://www.continuousphysics.com/Bullet/phpBB2/");
}

//to be implemented by the demo
void renderme()
{
	debugDrawer.SetDebugMode(getDebugMode());

	//render the hinge axis
	if (createConstraint)
	{
		SimdVector3 color(1,0,0);
		SimdVector3 dirLocal(0,1,0);
		SimdVector3 pivotInA(CUBE_HALF_EXTENTS,-CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS);
		SimdVector3 pivotInB(-CUBE_HALF_EXTENTS,-CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS);
		SimdVector3 from = physObjects[1]->GetRigidBody()->getCenterOfMassTransform()(pivotInA);
		SimdVector3 fromB = physObjects[2]->GetRigidBody()->getCenterOfMassTransform()(pivotInB);
		SimdVector3 dirWorldA = physObjects[1]->GetRigidBody()->getCenterOfMassTransform().getBasis() * dirLocal ;
		SimdVector3 dirWorldB = physObjects[2]->GetRigidBody()->getCenterOfMassTransform().getBasis() * dirLocal ;
		debugDrawer.DrawLine(from,from+dirWorldA,color);
		debugDrawer.DrawLine(fromB,fromB+dirWorldB,color);
	}

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


#ifdef USE_HULL
	//some testing code for SAT
	if (isSatEnabled)
	{
		for (int s=0;s<numShapes;s++)
		{
			CollisionShape* shape = shapePtr[s];

			if (shape->IsPolyhedral())
			{
				PolyhedralConvexShape* polyhedron = static_cast<PolyhedralConvexShape*>(shape);
				if (!polyhedron->m_optionalHull)
				{
					//first convert vertices in 'Point3' format
					int numPoints = polyhedron->GetNumVertices();
					Point3* points = new Point3[numPoints+1];
					//first 4 points should not be co-planar, so add central point to satisfy MakeHull
					points[0] = Point3(0.f,0.f,0.f);

					SimdVector3 vertex;
					for (int p=0;p<numPoints;p++)
					{
						polyhedron->GetVertex(p,vertex);
						points[p+1] = Point3(vertex.getX(),vertex.getY(),vertex.getZ());
					}

					Hull* hull = Hull::MakeHull(numPoints+1,points);
					polyhedron->m_optionalHull = hull;
				}

			}
		}

	}
#endif //USE_HULL


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
		sprintf(extraDebug,"islId, Body=%i , %i",physObjects[i]->GetRigidBody()->m_islandTag1,physObjects[i]->GetRigidBody()->m_debugBodyId);
		physObjects[i]->GetRigidBody()->GetCollisionShape()->SetExtraDebugInfo(extraDebug);

		float vec[16];
		SimdTransform ident;
		ident.setIdentity();
		ident.getOpenGLMatrix(vec);
		glPushMatrix(); 
	  glLoadMatrixf(vec);

		GL_ShapeDrawer::DrawOpenGL(m,physObjects[i]->GetRigidBody()->GetCollisionShape(),wireColor,getDebugMode());

		glPopMatrix(); 
		///this block is just experimental code to show some internal issues with replacing shapes on the fly.
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

	int i;
	for (i=0;i<numObjects;i++)
	{
		if (i>0)
		{

			if ((getDebugMode() & IDebugDraw::DBG_NoHelpText))
			{
				if (physObjects[i]->GetRigidBody()->GetCollisionShape()->GetShapeType() == BOX_SHAPE_PROXYTYPE)
				{
					physObjects[i]->GetRigidBody()->SetCollisionShape(shapePtr[2]);
				} else
				{
					physObjects[i]->GetRigidBody()->SetCollisionShape(shapePtr[1]);
				}

				BroadphaseProxy* bpproxy = physObjects[i]->GetRigidBody()->m_broadphaseHandle;
				physicsEnvironmentPtr->GetBroadphase()->CleanProxyFromPairs(bpproxy);
			}

			//stack them
			int colsize = 10;
			int row = (i*CUBE_HALF_EXTENTS*2)/(colsize*2*CUBE_HALF_EXTENTS);
			int row2 = row;
			int col = (i)%(colsize)-colsize/2;


			if (col>3)
			{
				col=11;
				row2 |=1;
			}
			physObjects[i]->setPosition(col*2*CUBE_HALF_EXTENTS + (row2%2)*CUBE_HALF_EXTENTS,
				row*2*CUBE_HALF_EXTENTS+CUBE_HALF_EXTENTS+EXTRA_HEIGHT,0);
			physObjects[i]->setOrientation(0,0,0,1);
			physObjects[i]->SetLinearVelocity(0,0,0,false);
			physObjects[i]->SetAngularVelocity(0,0,0,false);
		} 
	}
}



void	shootBox(const SimdVector3& destination)
{
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
	SimdVector3 vertical(0.f,1.f,0.f);
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
