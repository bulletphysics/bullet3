/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2026 Mykola

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h>
#include <gtest/gtest.h>

///Exercise activation through contacts and constraints in each supported island configuration.
class btSimulationIslandTest : public ::testing::TestWithParam<int>
{
protected:
	enum WorldOptions
	{
		MULTITHREADED = 1,
		DETERMINISTIC = 2,
		UNSPLIT = 4
	};

	btDefaultCollisionConfiguration m_configuration;
	btCollisionDispatcher m_dispatcher;
	btDbvtBroadphase m_broadphase;
	btSequentialImpulseConstraintSolver m_solver;
	btConstraintSolverPoolMt m_solverPool;
	btDiscreteDynamicsWorld* m_world;
	btSphereShape m_shape;
	btBoxShape m_boxShape;
	btSphereShape m_smallSphereShape;
	btRigidBody m_sleepingBody;
	btRigidBody m_activeBody;

	btSimulationIslandTest()
		: m_dispatcher(&m_configuration),
		  m_solverPool(2),
		  m_world(0),
		  m_shape(btScalar(0.5)),
		  m_boxShape(btVector3(btScalar(0.25), btScalar(0.25), btScalar(0.25))),
		  m_smallSphereShape(btScalar(0.125)),
		  m_sleepingBody(1, 0, &m_shape, btVector3(1, 1, 1)),
		  m_activeBody(1, 0, &m_shape, btVector3(1, 1, 1))
	{
	}

	virtual void SetUp()
	{
		if (GetParam() & MULTITHREADED)
		{
			m_world = new btDiscreteDynamicsWorldMt(&m_dispatcher, &m_broadphase, &m_solverPool, 0, &m_configuration);
		}
		else
		{
			m_world = new btDiscreteDynamicsWorld(&m_dispatcher, &m_broadphase, &m_solver, &m_configuration);
		}
		m_world->setGravity(btVector3(0, 0, 0));
		m_world->getDispatchInfo().m_deterministicOverlappingPairs = (GetParam() & DETERMINISTIC) != 0;
		m_world->getSimulationIslandManager()->setSplitIslands((GetParam() & UNSPLIT) == 0);
	}

	virtual void TearDown()
	{
		m_world->removeRigidBody(&m_activeBody);
		m_world->removeRigidBody(&m_sleepingBody);
		delete m_world;
	}

	void addBodies(btScalar separation)
	{
		btTransform transform = btTransform::getIdentity();
		transform.setOrigin(btVector3(separation, 0, 0));
		m_activeBody.setWorldTransform(transform);
		m_world->addRigidBody(&m_sleepingBody);
		m_world->addRigidBody(&m_activeBody);
		///Start asleep so the regression does not depend on a settling-time threshold.
		m_sleepingBody.setActivationState(ISLAND_SLEEPING);
	}

	int getNumPairContacts() const
	{
		int count = 0;
		for (int i = 0; i < m_dispatcher.getNumManifolds(); i++)
		{
			const btPersistentManifold* manifold = m_dispatcher.getManifoldByIndexInternal(i);
			if ((manifold->getBody0() == &m_sleepingBody && manifold->getBody1() == &m_activeBody) ||
				(manifold->getBody1() == &m_sleepingBody && manifold->getBody0() == &m_activeBody))
			{
				count += manifold->getNumContacts();
			}
		}
		return count;
	}
};

TEST_P(btSimulationIslandTest, BroadphaseOverlapDoesNotWakeSleepingBody)
{
	///The padded AABBs overlap, but the spheres do not touch.
	addBodies(btScalar(1.03));
	m_world->stepSimulation(btScalar(1.) / 60, 0);

	ASSERT_GT(m_broadphase.getOverlappingPairCache()->getNumOverlappingPairs(), 0);
	ASSERT_EQ(0, getNumPairContacts());
	EXPECT_EQ(ISLAND_SLEEPING, m_sleepingBody.getActivationState());
	EXPECT_NE(m_sleepingBody.getIslandTag(), m_activeBody.getIslandTag());
}

TEST_P(btSimulationIslandTest, ContactWakesSleepingBody)
{
	addBodies(btScalar(0.99));
	m_world->stepSimulation(btScalar(1.) / 60, 0);

	ASSERT_GT(getNumPairContacts(), 0);
	EXPECT_TRUE(m_sleepingBody.isActive());
	EXPECT_EQ(m_sleepingBody.getIslandTag(), m_activeBody.getIslandTag());
}

TEST_P(btSimulationIslandTest, ConstraintWakesSleepingBodyWithoutContact)
{
	addBodies(2);
	btPoint2PointConstraint constraint(m_sleepingBody, m_activeBody, btVector3(1, 0, 0), btVector3(-1, 0, 0));
	m_world->addConstraint(&constraint, true);
	m_world->stepSimulation(btScalar(1.) / 60, 0);

	EXPECT_EQ(0, getNumPairContacts());
	EXPECT_TRUE(m_sleepingBody.isActive());
	EXPECT_EQ(m_sleepingBody.getIslandTag(), m_activeBody.getIslandTag());
	m_world->removeConstraint(&constraint);
}

TEST_P(btSimulationIslandTest, KinematicBroadphaseOverlapDoesNotWakeSleepingBody)
{
	m_activeBody.setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
	m_activeBody.setActivationState(DISABLE_DEACTIVATION);
	addBodies(btScalar(1.03));
	m_world->stepSimulation(btScalar(1.) / 60, 0);

	ASSERT_GT(m_broadphase.getOverlappingPairCache()->getNumOverlappingPairs(), 0);
	ASSERT_EQ(0, getNumPairContacts());
	EXPECT_EQ(ISLAND_SLEEPING, m_sleepingBody.getActivationState());
}

TEST_P(btSimulationIslandTest, KinematicContactWakesSleepingBody)
{
	m_activeBody.setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT);
	m_activeBody.setActivationState(DISABLE_DEACTIVATION);
	addBodies(btScalar(0.99));
	m_world->stepSimulation(btScalar(1.) / 60, 0);

	ASSERT_GT(getNumPairContacts(), 0);
	EXPECT_TRUE(m_sleepingBody.isActive());
}

TEST_P(btSimulationIslandTest, ContactWithoutResponseDoesNotWakeSleepingBody)
{
	m_activeBody.setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);
	addBodies(btScalar(0.99));
	m_world->stepSimulation(btScalar(1.) / 60, 0);

	ASSERT_GT(getNumPairContacts(), 0);
	EXPECT_EQ(ISLAND_SLEEPING, m_sleepingBody.getActivationState());
	EXPECT_NE(m_sleepingBody.getIslandTag(), m_activeBody.getIslandTag());
}

TEST_P(btSimulationIslandTest, PredictiveContactWakesSleepingBody)
{
	///CCD must still connect the bodies before they overlap at a discrete time step.
	addBodies(2);
	m_activeBody.setLinearVelocity(btVector3(-120, 0, 0));
	m_activeBody.setCcdMotionThreshold(btScalar(0.1));
	m_activeBody.setCcdSweptSphereRadius(btScalar(0.4));
	m_world->stepSimulation(btScalar(1.) / 60, 0);

	ASSERT_GT(getNumPairContacts(), 0);
	EXPECT_TRUE(m_sleepingBody.isActive());
	EXPECT_EQ(m_sleepingBody.getIslandTag(), m_activeBody.getIslandTag());
}

TEST_P(btSimulationIslandTest, PassingSphereDoesNotMoveSleepingBox)
{
	btStaticPlaneShape planeShape(btVector3(0, 0, 1), 0);
	btRigidBody plane(0, 0, &planeShape);
	m_world->addRigidBody(&plane);
	m_world->setGravity(btVector3(0, 0, -10));
	m_sleepingBody.setCollisionShape(&m_boxShape);
	m_activeBody.setCollisionShape(&m_smallSphereShape);
	m_activeBody.setFriction(0);
	btTransform transform = btTransform::getIdentity();
	transform.setOrigin(btVector3(0, 0, btScalar(0.25)));
	m_sleepingBody.setWorldTransform(transform);
	addBodies(btScalar(0.405));
	transform.setOrigin(btVector3(btScalar(0.405), -1, btScalar(0.125)));
	m_activeBody.setWorldTransform(transform);
	m_activeBody.setLinearVelocity(btVector3(0, 1, 0));
	const btTransform initialTransform = m_sleepingBody.getWorldTransform();
	int overlappingSteps = 0;
	int contactSteps = 0;
	int awakeSteps = 0;

	///The sphere passes the box with a 3 cm gap while their padded AABBs overlap.
	for (int i = 0; i < 480; i++)
	{
		m_world->stepSimulation(btScalar(1.) / 240, 0);
		if (m_broadphase.getOverlappingPairCache()->findPair(m_sleepingBody.getBroadphaseHandle(), m_activeBody.getBroadphaseHandle()))
		{
			overlappingSteps++;
		}
		if (getNumPairContacts())
		{
			contactSteps++;
		}
		if (m_sleepingBody.isActive())
		{
			awakeSteps++;
		}
	}

	EXPECT_GT(overlappingSteps, 0);
	EXPECT_EQ(0, contactSteps);
	EXPECT_EQ(0, awakeSteps);
	EXPECT_GT(m_activeBody.getWorldTransform().getOrigin().y(), btScalar(0.5));
	EXPECT_EQ(initialTransform.getOrigin(), m_sleepingBody.getWorldTransform().getOrigin());
	EXPECT_EQ(initialTransform.getRotation(), m_sleepingBody.getWorldTransform().getRotation());
	m_world->removeRigidBody(&plane);
}

///Cover both island managers, deterministic ordering, and the unsplit solver path.
#if BT_THREADSAFE
INSTANTIATE_TEST_CASE_P(Worlds, btSimulationIslandTest, ::testing::Range(0, 8));
#else
INSTANTIATE_TEST_CASE_P(Worlds, btSimulationIslandTest, ::testing::Values(0, 2, 4, 6));
#endif

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
#if BT_THREADSAFE
	btITaskScheduler* scheduler = btGetOpenMPTaskScheduler();
	///The MT world requires a scheduler even when OpenMP is not available.
	if (!scheduler)
	{
		scheduler = btGetSequentialTaskScheduler();
	}
	scheduler->setNumThreads(btMin(2, scheduler->getMaxNumThreads()));
	btSetTaskScheduler(scheduler);
#endif
	return RUN_ALL_TESTS();
}
