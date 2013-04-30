/*
Copyright (c) 2012 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include <stdio.h>

#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Collision/BroadPhaseCollision/b3DynamicBvhBroadphase.h"
#include "Bullet3Common/b3CommandLineArgs.h"
#include "Bullet3Common/b3MinMax.h"
#include "Bullet3Collision/BroadPhaseCollision/b3OverlappingPairCache.h"

int g_nPassed = 0;
int g_nFailed = 0;
bool g_testFailed = 0;

#define TEST_INIT g_testFailed = 0;
#define TEST_ASSERT(x) if( !(x) ){g_testFailed = 1;}
#define TEST_REPORT(testName) printf("[%s] %s\n",(g_testFailed)?"X":"O", testName); if(g_testFailed) g_nFailed++; else g_nPassed++;



inline void broadphaseTest()
{
	TEST_INIT;

	b3DynamicBvhBroadphase* bp = new b3DynamicBvhBroadphase(2);
	
	int group=1;
	int mask=1;
	b3Vector3 aabbMin(0,0,0);
	b3Vector3 aabbMax(1,1,1);
	int userId = 0;
	bp->createProxy(aabbMin,aabbMax,userId++,0,group,mask);

	aabbMin.setValue(1,1,1);
	aabbMax.setValue(2,2,2);

	
	bp->createProxy(aabbMin,aabbMax,userId++,0,group,mask);
	

	bp->calculateOverlappingPairs();
	
	int numOverlap = bp->getOverlappingPairCache()->getNumOverlappingPairs();
	
	
	TEST_ASSERT(numOverlap==1);

	delete bp;

	TEST_REPORT( "broadphaseTest" );
}

int main(int argc, char** argv)
{


	broadphaseTest();

	printf("%d tests passed\n",g_nPassed, g_nFailed);
	if (g_nFailed)
	{
		printf("%d tests failed\n",g_nFailed);
	}
	printf("End, press <enter>\n");

	getchar();

}

