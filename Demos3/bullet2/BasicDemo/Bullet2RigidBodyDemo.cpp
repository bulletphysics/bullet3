#include "Bullet2RigidBodyDemo.h"
#include "btBulletDynamicsCommon.h"
#include "OpenGLWindow/SimpleOpenGL3App.h"

Bullet2RigidBodyDemo::Bullet2RigidBodyDemo(SimpleOpenGL3App* app)
	:m_glApp(app),
	m_pickedBody(0),
	m_pickedConstraint(0)
{
	m_config = 0;
	m_dispatcher = 0;
	m_bp = 0;
	m_solver = 0;
	m_dynamicsWorld = 0;
}
void Bullet2RigidBodyDemo::initPhysics()
{
	m_config = new btDefaultCollisionConfiguration;
	m_dispatcher = new btCollisionDispatcher(m_config);
	m_bp = new btDbvtBroadphase();
	m_solver = new btSequentialImpulseConstraintSolver();
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_bp,m_solver,m_config);
}

void Bullet2RigidBodyDemo::exitPhysics()
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

void	Bullet2RigidBodyDemo::stepSimulation(float deltaTime)
{
	m_dynamicsWorld->stepSimulation(deltaTime);
}

Bullet2RigidBodyDemo::~Bullet2RigidBodyDemo()
{
	btAssert(m_config == 0);
	btAssert(m_dispatcher == 0);
	btAssert(m_bp == 0);
	btAssert(m_solver == 0);
	btAssert(m_dynamicsWorld == 0);
}

btVector3	Bullet2RigidBodyDemo::getRayTo(int x,int y)
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

	
bool	Bullet2RigidBodyDemo::mouseMoveCallback(float x,float y)
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
bool	Bullet2RigidBodyDemo::mouseButtonCallback(int button, int state, float x, float y)
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
