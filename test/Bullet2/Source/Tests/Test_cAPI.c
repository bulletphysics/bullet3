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
	if (btVector3_norm(&pythagorean) != 5) {
		// output error here
		return 1;
	}
	
	// Check cross
	btVector3 x = btVector3(1, 0, 0);
	btVector3 y = btVector3(0, 1, 0);
	btVector3 z = btVector3(0, 0, 1);
	btVector3 cross = btVector3_cross(&x, &y);
	if (!btVector3_cmp(&z, &cross)) {
		// output error here
		return 1;
	}

    return 0;
}
