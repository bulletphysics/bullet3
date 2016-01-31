//
//  Test_cAPI.c
//  BulletTest
//


#include "Test_cAPI.h"

#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"

#include "LinearMath/btQuaternion.h"

#include "LinearMath/btMatrix3x3.h"

#include "LinearMath/btTransform.h"

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
	const btVector3 pythagorean = btVector3(3, 4, 0);
	if (btVector3_norm(&pythagorean) != 5) {
		// output error here
		return 1;
	}
	
	// Check cross
	const btVector3 x = btVector3(1, 0, 0);
	const btVector3 y = btVector3(0, 1, 0);
	const btVector3 z = btVector3(0, 0, 1);
	const btVector3 cross = btVector3_cross(&x, &y);
	if (!btVector3_cmp(&z, &cross)) {
		// output error here
		return 1;
	}
	
	// Check Quaternion
	const btQuaternion q = btQuaternion(0, 0, 1, 0);// Rotation around +Z
	btVector3 qMx;
	btQuaternion_makeQuatVector3Multiplication(&qMx, &q, &x);
	if (!btVector3_cmp(&y, &qMx)) {
		// output error here
		return 1;
	}
	
	// Check btMatrix3x3
	/*const btMatrix3x3 identityMatrix = btMatrix3x3(
		1, 0, 0,
		0, 1, 0,
		0, 0, 1);*/
	
	const btMatrix3x3 m1 = btMatrix3x3(
		1, 2, 3,
		4, 5, 6,
		7, 8, 9);
	
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			if (m1.m_el[i].m_floats[j] != (i * 3 + j + 1)) {
				// output error here
				return 1;
			}
	
    return 0;
}
