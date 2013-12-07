
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_Z 5


#include "OpenGLWindow/SimpleOpenGL3App.h"
#include "Bullet3Common/b3Vector3.h"
#include "assert.h"
#include <stdio.h>

#include "btBulletDynamicsCommon.h"

class Bullet2RigidBodyDemo
{
protected:
	btDiscreteDynamicsWorld* m_dynamicsWorld;
	btCollisionDispatcher*	m_dispatcher;
	btBroadphaseInterface*	m_bp;
	btCollisionConfiguration* m_config;
	btConstraintSolver* m_solver;

public:
	Bullet2RigidBodyDemo()
	{
		m_config = 0;
		m_dispatcher = 0;
		m_bp = 0;
		m_solver = 0;
		m_dynamicsWorld = 0;
	}
	virtual void initPhysics()
	{
		m_config = new btDefaultCollisionConfiguration;
		m_dispatcher = new btCollisionDispatcher(m_config);
		m_bp = new btDbvtBroadphase();
		m_solver = new btSequentialImpulseConstraintSolver();
		m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_bp,m_solver,m_config);
	}
	virtual void exitPhysics()
	{
		delete m_dynamicsWorld;
		m_dynamicsWorld=0;
		delete m_solver;
		m_solver=0;
		delete m_bp;
		m_bp=0;
		delete m_dispatcher;
		m_dispatcher=0;
		delete m_config;
		m_config=0;
	}

	virtual ~Bullet2RigidBodyDemo()
	{
		btAssert(m_config == 0);
		btAssert(m_dispatcher == 0);
		btAssert(m_bp == 0);
		btAssert(m_solver == 0);
		btAssert(m_dynamicsWorld == 0);
	}

};

class BasicDemo : public Bullet2RigidBodyDemo
{
	SimpleOpenGL3App* m_glApp;

	btRigidBody*	m_pickedBody;
	btTypedConstraint* m_pickedConstraint;
	btVector3 m_oldPickingPos;
	btVector3 m_hitPos;
	btScalar m_oldPickingDist;



public:
	BasicDemo(SimpleOpenGL3App* app)
	:m_glApp(app),
	m_pickedBody(0),
	m_pickedConstraint(0)
	{
	}
	virtual ~BasicDemo()
	{
	}

	void	initPhysics()
	{
		Bullet2RigidBodyDemo::initPhysics();

		//create ground
		int cubeShapeId = m_glApp->registerCubeShape();
		float pos[]={0,0,0};
		float orn[]={0,0,0,1};
		

		{
			float color[]={0.3,0.3,1,1};
			float halfExtents[]={50,50,50,1};
			btTransform groundTransform;
			groundTransform.setIdentity();
			groundTransform.setOrigin(btVector3(0,-50,0));
			m_glApp->m_instancingRenderer->registerGraphicsInstance(cubeShapeId,groundTransform.getOrigin(),groundTransform.getRotation(),color,halfExtents);
			btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(halfExtents[0]),btScalar(halfExtents[1]),btScalar(halfExtents[2])));
			//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
			{
				btScalar mass(0.);
				//rigidbody is dynamic if and only if mass is non zero, otherwise static
				bool isDynamic = (mass != 0.f);
				btVector3 localInertia(0,0,0);
				if (isDynamic)
					groundShape->calculateLocalInertia(mass,localInertia);
				//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
				btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
				btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
				btRigidBody* body = new btRigidBody(rbInfo);
				//add the body to the dynamics world
				m_dynamicsWorld->addRigidBody(body);
			}
		}

		{
			float halfExtents[]={1,1,1,1};
			b3Vector4 colors[4] =
			{
				b3MakeVector4(1,0,0,1),
				b3MakeVector4(0,1,0,1),
				b3MakeVector4(0,1,1,1),
				b3MakeVector4(1,1,0,1),
			};
		


			btTransform startTransform;
			startTransform.setIdentity();
			btScalar mass = 1.f;
			btVector3 localInertia;
			btBoxShape* colShape = new btBoxShape(btVector3(halfExtents[0],halfExtents[1],halfExtents[2]));
			colShape ->calculateLocalInertia(mass,localInertia);

			for (int k=0;k<ARRAY_SIZE_Y;k++)
			{
				for (int i=0;i<ARRAY_SIZE_X;i++)
				{
					for(int j = 0;j<ARRAY_SIZE_Z;j++)
					{
						static int curColor=0;
						b3Vector4 color = colors[curColor];
						curColor++;
						curColor&=3;
						startTransform.setOrigin(btVector3(
											btScalar(2.0*i),
											btScalar(20+2.0*k),
											btScalar(2.0*j)));

						m_glApp->m_instancingRenderer->registerGraphicsInstance(cubeShapeId,startTransform.getOrigin(),startTransform.getRotation(),color,halfExtents);
			
						//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
						btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
						btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
						btRigidBody* body = new btRigidBody(rbInfo);
					

						m_dynamicsWorld->addRigidBody(body);
					}
				}
			}
		}

		m_glApp->m_instancingRenderer->writeTransforms();
	}
	void	exitPhysics()
	{
		Bullet2RigidBodyDemo::exitPhysics();
	}
	void	drawObjects()
	{
		//sync graphics -> physics world transforms
		{
			for (int i=0;i<m_dynamicsWorld->getNumCollisionObjects();i++)
			{
				btVector3 pos = m_dynamicsWorld->getCollisionObjectArray()[i]->getWorldTransform().getOrigin();
				btQuaternion orn = m_dynamicsWorld->getCollisionObjectArray()[i]->getWorldTransform().getRotation();
				m_glApp->m_instancingRenderer->writeSingleInstanceTransformToCPU(pos,orn,i);
			}
			m_glApp->m_instancingRenderer->writeTransforms();
		}

		m_glApp->m_instancingRenderer->renderScene();
	}

	btVector3	getRayTo(int x,int y)
	{
		if (!m_glApp->m_instancingRenderer)
		{
			btAssert(0);
			return btVector3(0,0,0);
		}

		float top = 1.f;
		float bottom = -1.f;
		float nearPlane = 1.f;
		float tanFov = (top-bottom)*0.5f / nearPlane;
		float fov = b3Scalar(2.0) * b3Atan(tanFov);

		btVector3 camPos,camTarget;
		m_glApp->m_instancingRenderer->getCameraPosition(camPos);
		m_glApp->m_instancingRenderer->getCameraTargetPosition(camTarget);

		btVector3	rayFrom = camPos;
		btVector3 rayForward = (camTarget-camPos);
		rayForward.normalize();
		float farPlane = 10000.f;
		rayForward*= farPlane;

		btVector3 rightOffset;
		btVector3 m_cameraUp=btVector3(0,1,0);
		btVector3 vertical = m_cameraUp;

		btVector3 hor;
		hor = rayForward.cross(vertical);
		hor.normalize();
		vertical = hor.cross(rayForward);
		vertical.normalize();

		float tanfov = tanf(0.5f*fov);


		hor *= 2.f * farPlane * tanfov;
		vertical *= 2.f * farPlane * tanfov;

		b3Scalar aspect;
		float width = m_glApp->m_instancingRenderer->getScreenWidth();
		float height = m_glApp->m_instancingRenderer->getScreenHeight();

		aspect =  width / height;
	
		hor*=aspect;


		btVector3 rayToCenter = rayFrom + rayForward;
		btVector3 dHor = hor * 1.f/width;
		btVector3 dVert = vertical * 1.f/height;


		btVector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
		rayTo += btScalar(x) * dHor;
		rayTo -= btScalar(y) * dVert;
		return rayTo;
	}

	
	bool	mouseMoveCallback(float x,float y)
	{
//		if (m_data->m_altPressed!=0 || m_data->m_controlPressed!=0)
	//		return false;
		
		if (m_pickedBody  && m_pickedConstraint)
		{
			btPoint2PointConstraint* pickCon = static_cast<btPoint2PointConstraint*>(m_pickedConstraint);
			if (pickCon)
			{
				//keep it at the same picking distance
				btVector3 newRayTo = getRayTo(x,y);
				btVector3 rayFrom;
				btVector3 oldPivotInB = pickCon->getPivotInB();
				btVector3 newPivotB;
				m_glApp->m_instancingRenderer->getCameraPosition(rayFrom);
				btVector3 dir = newRayTo-rayFrom;
				dir.normalize();
				dir *= m_oldPickingDist;

				newPivotB = rayFrom + dir;
				pickCon->setPivotB(newPivotB);
			}
		}
		
		return false;
	}
	bool	mouseButtonCallback(int button, int state, float x, float y)
	{

		if (state==1)
		{
			if(button==0)// && (m_data->m_altPressed==0 && m_data->m_controlPressed==0))
			{
				btVector3 camPos;
				m_glApp->m_instancingRenderer->getCameraPosition(camPos);

				btVector3 rayFrom = camPos;
				btVector3 rayTo = getRayTo(x,y);

				btCollisionWorld::ClosestRayResultCallback rayCallback(rayFrom,rayTo);
				m_dynamicsWorld->rayTest(rayFrom,rayTo,rayCallback);
				if (rayCallback.hasHit())
				{

					btVector3 pickPos = rayCallback.m_hitPointWorld;
					btRigidBody* body = (btRigidBody*)btRigidBody::upcast(rayCallback.m_collisionObject);
					if (body)
					{
						//other exclusions?
						if (!(body->isStaticObject() || body->isKinematicObject()))
						{
							m_pickedBody = body;
							m_pickedBody->setActivationState(DISABLE_DEACTIVATION);
							//printf("pickPos=%f,%f,%f\n",pickPos.getX(),pickPos.getY(),pickPos.getZ());
							btVector3 localPivot = body->getCenterOfMassTransform().inverse() * pickPos;
							btPoint2PointConstraint* p2p = new btPoint2PointConstraint(*body,localPivot);
							m_dynamicsWorld->addConstraint(p2p,true);
							m_pickedConstraint = p2p;
							btScalar mousePickClamping = 30.f;
							p2p->m_setting.m_impulseClamp = mousePickClamping;
							//very weak constraint for picking
							p2p->m_setting.m_tau = 0.001f;
						}
					}


//					pickObject(pickPos, rayCallback.m_collisionObject);
					m_oldPickingPos = rayTo;
					m_hitPos = pickPos;
					m_oldPickingDist  = (pickPos-rayFrom).length();
//					printf("hit !\n");
				//add p2p
				}
				
			}
		} else
		{
			if (button==0)
			{
				if (m_pickedConstraint)
				{
					m_dynamicsWorld->removeConstraint(m_pickedConstraint);
					delete m_pickedConstraint;
					m_pickedConstraint=0;
					m_pickedBody = 0;
				}
				//remove p2p
			}
		}

		//printf("button=%d, state=%d\n",button,state);
		return false;
	}

	void	stepSimulation()
	{
		m_dynamicsWorld->stepSimulation(1./60,0);
	}
};



BasicDemo* sDemo = 0;

static void MyMouseMoveCallback( float x, float y)
{
	bool handled = false;
	if (sDemo)
		handled = sDemo->mouseMoveCallback(x,y);
	if (!handled)
		b3DefaultMouseMoveCallback(x,y);
}
static void MyMouseButtonCallback(int button, int state, float x, float y)
{
	bool handled = false;
	//try picking first
	if (sDemo)
		handled = sDemo->mouseButtonCallback(button,state,x,y);

	if (!handled)
		b3DefaultMouseButtonCallback(button,state,x,y);
}


int main(int argc, char* argv[])
{
	
	float dt = 1./120.f;
#ifdef BT_DEBUG
	char* name = "Bullet 2 CPU BasicDemo (Debug build=SLOW)";
#else
	char* name = "Bullet 2 CPU BasicDemo";
#endif

	
	SimpleOpenGL3App* app = new SimpleOpenGL3App(name,1024,768);
	app->m_instancingRenderer->setCameraDistance(40);
	app->m_instancingRenderer->setCameraPitch(0);
	app->m_instancingRenderer->setCameraTargetPosition(b3MakeVector3(0,0,0));

	app->m_window->setMouseMoveCallback(MyMouseMoveCallback);
	app->m_window->setMouseButtonCallback(MyMouseButtonCallback);

	BasicDemo* demo = new BasicDemo(app);
	demo->initPhysics();
	sDemo = demo;

	GLint err = glGetError();
    assert(err==GL_NO_ERROR);
	
	do
	{
		GLint err = glGetError();
		assert(err==GL_NO_ERROR);
		app->m_instancingRenderer->init();
		app->m_instancingRenderer->updateCamera();
		
		demo->stepSimulation();
		demo->drawObjects();
		app->drawGrid(10,0.01);
		char bla[1024];
		static int frameCount = 0;
		frameCount++;
		sprintf(bla,"Simulation frame %d", frameCount);
		
		app->drawText(bla,10,10);
		app->swapBuffer();
	} while (!app->m_window->requestedExit());


	demo->exitPhysics();
	delete demo;

	delete app;
	return 0;
}
