

#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/Character/btKinematicCharacterController.h>
#include <gtest/gtest.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>

GTEST_TEST(BulletDynamics, KinematicCharacterController)
{
	btPairCachingGhostObject* ghostObject = new btPairCachingGhostObject();
	btBoxShape* convexShape = new btBoxShape(btVector3(1, 1, 1));

	//For now only a simple test that it initializes correctly.
	btKinematicCharacterController* tested = new btKinematicCharacterController(ghostObject, convexShape, 1);
	EXPECT_TRUE(tested);

	EXPECT_FLOAT_EQ(-9.8 * 3.0, tested->getGravity().x());
	EXPECT_FLOAT_EQ(0, tested->getGravity().y());
	EXPECT_FLOAT_EQ(0, tested->getGravity().z());
}

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}