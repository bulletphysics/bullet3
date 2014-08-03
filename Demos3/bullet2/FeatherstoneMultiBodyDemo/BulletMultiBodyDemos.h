
#ifndef BULLET_MULTI_BODY_DEMOS_H
#define BULLET_MULTI_BODY_DEMOS_H

#include "LinearMath/btVector3.h"

#include "../../AllBullet2Demos/BulletDemoInterface.h"

#include "LinearMath/btAlignedObjectArray.h"

struct btMultiBodySettings
{
	btMultiBodySettings()
	{
		m_numLinks = 0;
		m_basePosition.setZero();
		m_isFixedBase = true;
		m_usePrismatic = false;
		m_canSleep = true;
		m_createConstraints = false;
		m_disableParentCollision = false;
	}
	int			m_numLinks;
	btVector3	m_basePosition;
	bool		m_isFixedBase;
	bool		m_usePrismatic;
	bool		m_canSleep;
	bool		m_createConstraints;
	bool		m_disableParentCollision;
};

class Bullet2MultiBodyDemo : public BulletDemoInterface
{
protected:

	SimpleOpenGL3App* m_glApp;

	class btRigidBody*	m_pickedBody;
	class btTypedConstraint* m_pickedConstraint;
	btVector3 m_oldPickingPos;
	btVector3 m_hitPos;
	btScalar m_oldPickingDist;

	class btMultiBodyPoint2Point*		m_pickingMultiBodyPoint2Point;

	class btMultiBodyDynamicsWorld* m_dynamicsWorld;
	class btCollisionDispatcher*	m_dispatcher;
	class btBroadphaseInterface*	m_broadphase;
	class btCollisionConfiguration* m_collisionConfiguration;
	class btMultiBodyConstraintSolver* m_solver;

	btAlignedObjectArray<class btCollisionShape*> m_collisionShapes;
	//btAlignedObjectArray<btMultiBodyLinkCollider*> m_linkColliders;

public:
	Bullet2MultiBodyDemo(SimpleOpenGL3App* app);
	virtual void initPhysics();
	virtual void exitPhysics();
	virtual ~Bullet2MultiBodyDemo();
	btVector3	getRayTo(int x,int y);
	virtual bool	mouseMoveCallback(float x,float y);
	virtual bool	mouseButtonCallback(int button, int state, float x, float y);
	virtual bool	keyboardCallback(int key, int state)
	{
		return false;
	}
};

class FeatherstoneDemo1 : public Bullet2MultiBodyDemo
{
	
public:

	FeatherstoneDemo1(SimpleOpenGL3App* app);
	virtual ~FeatherstoneDemo1();


	static BulletDemoInterface* MyCreateFunc(SimpleOpenGL3App* app)
	{
		return new FeatherstoneDemo1(app);
	}

	class btMultiBody* createFeatherstoneMultiBody(class btMultiBodyDynamicsWorld* world, const btMultiBodySettings& settings);

	void addColliders_testMultiDof(btMultiBody *pMultiBody, btMultiBodyDynamicsWorld *pWorld, const btVector3 &baseHalfExtents, const btVector3 &linkHalfExtents);
	void addBoxes_testMultiDof();

	void createGround();
	virtual void	initPhysics();
	virtual void	exitPhysics();
	virtual void	renderScene();
        virtual void    physicsDebugDraw();

	virtual void	stepSimulation(float deltaTime);
};


class FeatherstoneDemo2 : public FeatherstoneDemo1
{
	
public:

	FeatherstoneDemo2(SimpleOpenGL3App* app);
	virtual ~FeatherstoneDemo2();


	static BulletDemoInterface* MyCreateFunc(SimpleOpenGL3App* app)
	{
		return new FeatherstoneDemo2(app);
	}
	
	virtual void	initPhysics();
};


#endif //BULLET_MULTI_BODY_DEMOS_H

