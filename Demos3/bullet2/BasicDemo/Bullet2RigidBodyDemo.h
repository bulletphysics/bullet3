#ifndef BULLET2_RIGIDBODY_DEMO_H
#define BULLET2_RIGIDBODY_DEMO_H

#include "LinearMath/btVector3.h"

#include "../../AllBullet2Demos/BulletDemoInterface.h"

#include "OpenGLWindow/b3gWindowInterface.h"
#include "../../../Demos/CommonPhysicsSetup.h"


class Bullet2RigidBodyDemo : public BulletDemoInterface
{
	CommonPhysicsSetup* m_physicsSetup;

public:

    bool    m_controlPressed;
    bool    m_altPressed;

public:

	struct SimpleOpenGL3App* m_glApp;

	Bullet2RigidBodyDemo(SimpleOpenGL3App* app, CommonPhysicsSetup* physicsSetup);
	virtual void initPhysics();
	virtual void exitPhysics();
	virtual void	renderScene();
    virtual void	physicsDebugDraw();
	virtual void	stepSimulation(float dt);
	virtual CommonPhysicsSetup* getPhysicsSetup()
	{
		return m_physicsSetup;
	}

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


};

#endif //BULLET2_RIGIDBODY_DEMO_H
