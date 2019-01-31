#include <btBulletDynamicsCommon.h>
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "../Extras/Serialize/BulletWorldImporter/btMultiBodyWorldImporter.h"

#include <gtest/gtest.h>

static btMultiBodyDynamicsWorld *create_btMultiBodyDynamicsWorld()
{
	btDefaultCollisionConfiguration * collisionConfiguration = new btDefaultCollisionConfiguration();

	btCollisionDispatcher * dispatcher = new btCollisionDispatcher(collisionConfiguration);

	btDbvtBroadphase * broadphase = new btDbvtBroadphase();

	btMultiBodyConstraintSolver* solver = new btMultiBodyConstraintSolver();

	btMultiBodyDynamicsWorld * world = new btMultiBodyDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

	world->setGravity(btVector3(0, -10, 0));

	return world;
}

GTEST_TEST(BulletDynamics, DeterministicSaveRestore)
{
	btMultiBodyDynamicsWorld *initial_world = create_btMultiBodyDynamicsWorld();
	// create ground plane and box shapes
	// don't worry about gui things

	// step simulation until collision
	const double delta_t = 0.01;
	for (int i = 0; i < 50; ++i)
	{
		initial_world->stepSimulation(delta_t);
	}

	// serialize

	// step simulation until boxes are no longer moving
	for (int i = 0; i < 500; ++i)
	{
		initial_world->stepSimulation(delta_t);
	}

	// save position of all boxes

	// deserialize
	btMultiBodyDynamicsWorld *deserialized_world = create_btMultiBodyDynamicsWorld();
	btMultiBodyWorldImporter importer(deserialized_world);
	// importer.convertAllObjects(...);

	// step simulation until boxes are no longer moving
	for (int i = 0; i < 500; ++i)
	{
		deserialized_world->stepSimulation(delta_t);
	}

	// assert that the position of all boxes in deserialized_world are the same as in initial_world
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
