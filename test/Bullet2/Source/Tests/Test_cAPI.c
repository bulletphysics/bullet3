//
//  Test_cAPI.c
//  BulletTest
//


#include "Test_cAPI.h"

#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"

#include "vector.h"
#include "Utils.h"

int Test_cAPI(void)
{
	btBool versionTest1 = btGetVersion() == BT_BULLET_VERSION;
	if (!versionTest1) {
		// output error here
		return 1;
	}
	
	btBool versionTest2 = (&btGetVersion)() == btGetVersion();
	if (!versionTest2) {
		// output error here
		return 1;
	}
	
	// Check pitagoras
	btVector3 pythagorean = btVector3(3, 4, 0);
	if (btVector_norm(&pythagorean, BT_VEC3_MODE) != 5) {
		// output error here
		return 1;
	}

    return 0;
}
