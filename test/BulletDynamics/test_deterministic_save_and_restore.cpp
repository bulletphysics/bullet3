#include "../../Extras/Serialize/BulletFileLoader/bFile.h"
#include <btBulletDynamicsCommon.h>
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "../../Extras/Serialize/BulletWorldImporter/btMultiBodyWorldImporter.h"
#include "../../Extras/Serialize/BulletWorldImporter/btBulletWorldImporter.h"
#include "../../Extras/Serialize/BulletWorldImporter/btWorldImporter.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"
#include "LinearMath/btVector3.h"

#ifdef VISUAL_DEBUG
#include "../../examples/OpenGLWindow/SimpleOpenGL3App.h"
#include "../../examples/ExampleBrowser/OpenGLGuiHelper.h"
#endif

#include <gtest/gtest.h>

// When adding a 4th box to this sim, determinism gets broken
// It can be resolved by making sure the contact manifolds are sorted
// after deserialization into the same order in which they were serialized,
// but this breaks determinism in examples/pybullet/unittests/saveRestoreStateTest.py
// We'll leave the code in for anyone who wants to try to replicate it, but will
// leave it disabled until the root cause of the issue is found
#define ADD_FOURTH_BOX 0

// Notes: Run CMake with -DDETERMINISM_VISUAL_DEBUG=ON in order to compile this
// program such that it opens a GUI window when run instead of running as a test

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

btRigidBody* CreateBox(btVector3 position, btVector3 velocity, btVector3 inertia, btScalar mass = 1.f)
{
	btBoxShape* colShape = new btBoxShape(btVector3(.1, .1, .1));
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(position);
	colShape->calculateLocalInertia(mass, inertia);
	btRigidBody* box = new btRigidBody(mass, 0, colShape, inertia);
	box->setWorldTransform(startTransform);
	box->setLinearVelocity(velocity);
	return box;
}

#ifdef VISUAL_DEBUG
void test_determinism(SimpleOpenGL3App * app, OpenGLGuiHelper * gui_helper)
#else
GTEST_TEST(BulletDynamics, DeterministicSaveRestore)
#endif
{
	static char filename[] = "test_serialize.bullet";
	btRigidBody* box_before[4];
	btRigidBody* box_after[4];
	btMultiBodyDynamicsWorld *initial_world = create_btMultiBodyDynamicsWorld();

	// create ground plane and box shapes

	btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));
	{
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0, -50, 0));
		btScalar mass(0.);
		btVector3 localInertia(0, 0, 0);
		groundShape->calculateLocalInertia(mass, localInertia);
		btRigidBody* body = new btRigidBody(mass, 0, groundShape, localInertia);
		body->setWorldTransform(groundTransform);
		initial_world->addRigidBody(body);
	}
	{
		btVector3 localInertia(0, 0, 0);

		// set initial position of boxes and linear velocity

		box_before[0] = CreateBox(btVector3(2,25,0), btVector3(-1,0,1), localInertia);
		initial_world->addRigidBody(box_before[0]);
		box_before[1] = CreateBox(btVector3(2,25,2), btVector3(-1,0,-1), localInertia);
		initial_world->addRigidBody(box_before[1]);
		box_before[2] = CreateBox(btVector3(0,25,1), btVector3(1,0,0), localInertia);
		initial_world->addRigidBody(box_before[2]);
#if ADD_FOURTH_BOX
		box_before[3] = CreateBox(btVector3(1,1,1), btVector3(0,26,0), localInertia);
		initial_world->addRigidBody(box_before[3]);
#endif
	}

#ifdef VISUAL_DEBUG
	gui_helper->autogenerateGraphicsObjects(initial_world);
	const float dist = 4;
	const float pitch = -45;
	const float yaw = 180;
	const float targetPos[3]={1,21,3};
	gui_helper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
#endif

	// step simulation until collision
	// in more detail - step until there are some contact points within the collision manifolds
	// serializing and deserializing these manifolds and their contact points is a key factor
	// for determinism, so we want to be sure we're stressing those code paths.

	const double delta_t = 0.01;
	int frame = 0;
	int total_contacts = 0;
	// By frame 94 there should be some contact points, but in order to avoid a potential infinite
	// loop, we'll set a hard timeout at 150 frames.
	const int TIMEOUT = 150;
	while (total_contacts == 0  && frame < TIMEOUT)
	{
		frame++;
		initial_world->stepSimulation(delta_t);
		const int num_manifolds = initial_world->getDispatcher()->getNumManifolds();
		int contacts_this_frame = 0;
		for (int i = 0; i < num_manifolds; ++i)
		{
			const btPersistentManifold* manifold = initial_world->getDispatcher()->getManifoldByIndexInternal(i);
			contacts_this_frame += manifold->getNumContacts();
		}
		total_contacts += contacts_this_frame;
#ifdef VISUAL_DEBUG
		gui_helper->syncPhysicsToGraphics(initial_world);
		gui_helper->render(initial_world);
		app->swapBuffer();
		usleep(10000);
#endif
	}
	ASSERT_LT(frame, TIMEOUT);

	// serialize
	btSerializer* s = new btDefaultSerializer;
	s->setSerializationFlags(BT_SERIALIZE_CONTACT_MANIFOLDS);
	initial_world->serialize(s);
	FILE* file = fopen(filename, "wb");
	fwrite(s->getBufferPointer(), s->getCurrentBufferSize(), 1, file);
	fclose(file);

	// step simulation until boxes are no longer moving
	const double VELOCITY_EPSILON = 1e-6;
	while(box_before[0]->getLinearVelocity().length() > VELOCITY_EPSILON || box_before[0]->getAngularVelocity().length() > VELOCITY_EPSILON ||
		  box_before[1]->getLinearVelocity().length() > VELOCITY_EPSILON || box_before[1]->getAngularVelocity().length() > VELOCITY_EPSILON ||
		  box_before[2]->getLinearVelocity().length() > VELOCITY_EPSILON || box_before[2]->getAngularVelocity().length() > VELOCITY_EPSILON
#if ADD_FOURTH_BOX
|| box_before[3]->getLinearVelocity().length() > VELOCITY_EPSILON || box_before[3]->getAngularVelocity().length() > VELOCITY_EPSILON
#endif
		   )
	{
		initial_world->stepSimulation(delta_t);
#ifdef VISUAL_DEBUG
		gui_helper->syncPhysicsToGraphics(initial_world);
		gui_helper->render(initial_world);
		app->swapBuffer();
		usleep(10000);
#endif
	}

	// save position of all boxes

	const btVector3 steady_state1 = box_before[0]->getCenterOfMassPosition();
	const btVector3 steady_state2 = box_before[1]->getCenterOfMassPosition();
	const btVector3 steady_state3 = box_before[2]->getCenterOfMassPosition();
#if ADD_FOURTH_BOX
    const btVector3 steady_state4 = box_before[3]->getCenterOfMassPosition();
#endif

	// deserialize

	btMultiBodyDynamicsWorld *deserialized_world = create_btMultiBodyDynamicsWorld();
	btMultiBodyWorldImporter importer(deserialized_world);
	importer.loadFile(filename); // load once to bring the objects into the world
	importer.setImporterFlags(eRESTORE_EXISTING_OBJECTS);
	importer.loadFile(filename); // load a second time with the above flag set to restore the contact manifolds

#ifdef VISUAL_DEBUG
	gui_helper->autogenerateGraphicsObjects(deserialized_world);
#endif

	box_after[0] = static_cast<btRigidBody*>(importer.getRigidBodyByIndex(1));
	box_after[1] = static_cast<btRigidBody*>(importer.getRigidBodyByIndex(2));
	box_after[2] = static_cast<btRigidBody*>(importer.getRigidBodyByIndex(3));
#if ADD_FOURTH_BOX
    box_after[3] = static_cast<btRigidBody*>(importer.getRigidBodyByIndex(4));
#endif

	// step simulation until boxes are no longer moving
	 while(box_after[0]->getLinearVelocity().length() > VELOCITY_EPSILON || box_after[0]->getAngularVelocity().length() > VELOCITY_EPSILON ||
	 	  box_after[1]->getLinearVelocity().length() > VELOCITY_EPSILON || box_after[1]->getAngularVelocity().length() > VELOCITY_EPSILON ||
	 	  box_after[2]->getLinearVelocity().length() > VELOCITY_EPSILON || box_after[2]->getAngularVelocity().length() > VELOCITY_EPSILON
#if ADD_FOURTH_BOX
		  || box_after[3]->getLinearVelocity().length() > VELOCITY_EPSILON || box_after[3]->getAngularVelocity().length() > VELOCITY_EPSILON
#endif
         )
	{
		deserialized_world->stepSimulation(delta_t);
#ifdef VISUAL_DEBUG
		gui_helper->syncPhysicsToGraphics(deserialized_world);
		gui_helper->render(deserialized_world);
		app->swapBuffer();
		usleep(10000);
#endif
	}

	const btVector3 after_deserialize1 = box_after[0]->getCenterOfMassPosition();
	const btVector3 after_deserialize2 = box_after[1]->getCenterOfMassPosition();
	const btVector3 after_deserialize3 = box_after[2]->getCenterOfMassPosition();
#if ADD_FOURTH_BOX
    const btVector3 after_deserialize4 = box_after[3]->getCenterOfMassPosition();
#endif

	// assert that the position of all boxes in deserialized_world are the same as in initial_world
	EXPECT_FLOAT_EQ(steady_state1.getX(),after_deserialize1.getX());
	EXPECT_FLOAT_EQ(steady_state1.getY(),after_deserialize1.getY());
	EXPECT_FLOAT_EQ(steady_state1.getZ(),after_deserialize1.getZ());
	EXPECT_FLOAT_EQ(steady_state2.getX(),after_deserialize2.getX());
	EXPECT_FLOAT_EQ(steady_state2.getY(),after_deserialize2.getY());
	EXPECT_FLOAT_EQ(steady_state2.getZ(),after_deserialize2.getZ());
	EXPECT_FLOAT_EQ(steady_state3.getX(),after_deserialize3.getX());
	EXPECT_FLOAT_EQ(steady_state3.getY(),after_deserialize3.getY());
	EXPECT_FLOAT_EQ(steady_state3.getZ(),after_deserialize3.getZ());
#if ADD_FOURTH_BOX
    EXPECT_FLOAT_EQ(steady_state4.getX(),after_deserialize4.getX());
	EXPECT_FLOAT_EQ(steady_state4.getY(),after_deserialize4.getY());
	EXPECT_FLOAT_EQ(steady_state4.getZ(),after_deserialize4.getZ());
#endif

#ifdef VISUAL_DEBUG
	// In VISUAL_DEBUG mode we want to leave the window open for inspection,
	// so we enter into an infinite loop to do so
	std::cout << "Test completed" << std::endl;
	while (true) {
		gui_helper->syncPhysicsToGraphics(deserialized_world);
		gui_helper->render(deserialized_world);
		app->swapBuffer();
	}
#endif
}


int main(int argc, char** argv)
{
#ifdef VISUAL_DEBUG
	SimpleOpenGL3App* app = new SimpleOpenGL3App("Determinism test",1024,768,true);
	OpenGLGuiHelper gui(app,false);
	test_determinism(app, &gui);
#else
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
#endif
}
