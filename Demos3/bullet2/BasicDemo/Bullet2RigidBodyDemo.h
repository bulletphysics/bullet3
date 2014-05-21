#ifndef BULLET2_RIGIDBODY_DEMO_H
#define BULLET2_RIGIDBODY_DEMO_H

#include "LinearMath/btVector3.h"

#include "../../AllBullet2Demos/BulletDemoInterface.h"

#include "OpenGLWindow/b3gWindowInterface.h"

class Bullet2RigidBodyDemo : public BulletDemoInterface
{
public:
	class btDiscreteDynamicsWorld* m_dynamicsWorld;
	class btCollisionDispatcher*	m_dispatcher;
	class btBroadphaseInterface*	m_bp;
	class btCollisionConfiguration* m_config;
	class btConstraintSolver* m_solver;

	class btRigidBody*	m_pickedBody;
	class btTypedConstraint* m_pickedConstraint;
	btVector3 m_oldPickingPos;
	btVector3 m_hitPos;
	btScalar m_oldPickingDist;
    bool    m_controlPressed;
    bool    m_altPressed;

public:

	class SimpleOpenGL3App* m_glApp;

	Bullet2RigidBodyDemo(SimpleOpenGL3App* app);
	virtual void initPhysics();
	virtual void exitPhysics();

	virtual ~Bullet2RigidBodyDemo();
	btVector3	getRayTo(int x,int y);
	virtual bool	mouseMoveCallback(float x,float y);
	virtual bool	mouseButtonCallback(int button, int state, float x, float y);
	virtual bool	keyboardCallback(int key, int state)
	{
        if (key==B3G_CONTROL)
        {
            m_controlPressed = (state==1);
        }
        if (key==B3G_ALT)
        {
            m_altPressed = (state==1);
        }
		return false;
	}

	virtual void	stepSimulation(float deltaTime);

};

#endif //BULLET2_RIGIDBODY_DEMO_H
