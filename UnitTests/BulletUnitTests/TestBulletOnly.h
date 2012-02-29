#ifndef TESTBULLETONLY_HAS_BEEN_INCLUDED
#define TESTBULLETONLY_HAS_BEEN_INCLUDED

#include "cppunit/TestFixture.h"
#include "cppunit/extensions/HelperMacros.h"

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"

// ---------------------------------------------------------------------------

class TestBulletOnly : public CppUnit::TestFixture
{
	btCollisionConfiguration *          mCollisionConfig;
	btCollisionDispatcher *             mCollisionDispatch;
	btBroadphaseInterface *             mBroadphase;
	btConstraintSolver *                mConstraintSolver;
	btDiscreteDynamicsWorld *           mWorld;
	btCollisionShape*					mBodyShape;
	btRigidBody*						mRigidBody;
	btDefaultMotionState*				mMotionState;

public:

	void setUp() 
	{
		// Setup the world --
		mCollisionConfig = new btDefaultCollisionConfiguration;
		mCollisionDispatch = new btCollisionDispatcher( mCollisionConfig );
		mBroadphase = new btDbvtBroadphase();
		mConstraintSolver = new btSequentialImpulseConstraintSolver();
		mWorld = new btDiscreteDynamicsWorld( mCollisionDispatch, mBroadphase, mConstraintSolver, mCollisionConfig );
		mWorld->setGravity( btVector3( 0, -9.81, 0 ));

		// Set up the rigid body --
		mBodyShape = new btBoxShape( btVector3( 1, 1, 1 ) );
		btScalar mass = 1;
		btVector3 localInertia(0,0,0);
		mBodyShape->calculateLocalInertia(mass,localInertia);
		btTransform             bodyInitial;

		mMotionState =			new btDefaultMotionState();

		bodyInitial.setIdentity();
		bodyInitial.setOrigin( btVector3( 0, 0, 0 ) );
		mRigidBody = new btRigidBody( buildConstructionInfo(mass, 0.5, 0.5, 0, 0, *mMotionState, mBodyShape ) );

		mWorld->addRigidBody( mRigidBody );
	}

	void tearDown() 
	{
		mWorld->removeRigidBody(mRigidBody);
		delete mRigidBody;
		delete mMotionState;
		delete mBodyShape;
		delete mWorld;
		delete mConstraintSolver;
		delete mBroadphase;
		delete mCollisionDispatch;
		delete mCollisionConfig;
		
	}

	btRigidBody::btRigidBodyConstructionInfo buildConstructionInfo( btScalar mass, btScalar friction, btScalar restitution, btScalar linearDamping,
		btScalar angularDamping, btDefaultMotionState & state, btCollisionShape * shape )
	{
		btVector3 inertia(0,0,0);
		if (mass>0)
			shape->calculateLocalInertia( mass, inertia );

		btRigidBody::btRigidBodyConstructionInfo info( mass, &state, shape, inertia );

		info.m_friction = friction;
		info.m_restitution = restitution;
		info.m_linearDamping = linearDamping;
		info.m_angularDamping = angularDamping;

		return info;
	}

	void testKinematicVelocity0()
	{
		mRigidBody->setMassProps( 1, btVector3( 1, 1, 1 ) );
		mRigidBody->updateInertiaTensor();
		mRigidBody->setCollisionFlags( btCollisionObject::CF_KINEMATIC_OBJECT );
		// -- .

		// Interpolate the velocity -- 

		//btVector3 velocity( 1., 2., 3. ), spin( 0.1, 0.2, 0.3 );
		btVector3 velocity( 1., 2., 3. ), spin( 0.1, 0.2, .3 );
		btTransform interpolated;

		// TODO: This is inaccurate for small spins.
		btTransformUtil::integrateTransform( mRigidBody->getCenterOfMassTransform(), velocity, spin, 1.0f/60.f, interpolated );

		mRigidBody->setInterpolationWorldTransform( interpolated );

		mWorld->stepSimulation( 1.f/60.f, 60, 1.0f/60.f );
	
		CPPUNIT_ASSERT_DOUBLES_EQUAL( mRigidBody->getLinearVelocity().length2(), velocity.length2(), 1e-8 );
#ifdef BT_USE_DOUBLE_PRECISION
		CPPUNIT_ASSERT_DOUBLES_EQUAL( mRigidBody->getAngularVelocity().length2(), spin.length2(), 1e-4 );
#else
		CPPUNIT_ASSERT_DOUBLES_EQUAL( mRigidBody->getAngularVelocity().length2(), spin.length2(), 5e-3 );
#endif //CPPUNIT_ASSERT_DOUBLES_EQUAL
		
	}

	CPPUNIT_TEST_SUITE(TestBulletOnly);
	CPPUNIT_TEST(testKinematicVelocity0);
	CPPUNIT_TEST_SUITE_END();

private:

};

#endif
