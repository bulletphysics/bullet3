/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "SerializeDemo.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btHashMap.h"

class OurValue
	{
		int m_uid;

	public:
		OurValue(const btVector3& initialPos)
			:m_position(initialPos)
		{
			static int gUid=0;
			m_uid=gUid;
			gUid++;
		}

		btVector3	m_position;
		int	getUid() const
		{
			return m_uid;
		}
	};

	
int main(int argc,char** argv)
{
	GLDebugDrawer	gDebugDrawer;

	///testing the btHashMap	
	btHashMap<btHashKey<OurValue>,OurValue> map;
	
	OurValue value1(btVector3(2,3,4));
	btHashKey<OurValue> key1(value1.getUid());
	map.insert(key1,value1);


	OurValue value2(btVector3(5,6,7));
	btHashKey<OurValue> key2(value2.getUid());
	map.insert(key2,value2);
	

	{
		OurValue value3(btVector3(7,8,9));
		btHashKey<OurValue> key3(value3.getUid());
		map.insert(key3,value3);
	}


	map.remove(key2);

//	const OurValue* ourPtr = map.find(key1);
//	for (int i=0;i<map.size();i++)
//	{
//		OurValue* tmp = map.getAtIndex(i);
//		//printf("tmp value=%f,%f,%f\n",tmp->m_position.getX(),tmp->m_position.getY(),tmp->m_position.getZ());
//	}
	
	SerializeDemo ccdDemo;
	ccdDemo.initPhysics();
	ccdDemo.getDynamicsWorld()->setDebugDrawer(&gDebugDrawer);


#ifdef CHECK_MEMORY_LEAKS
	ccdDemo.exitPhysics();
#else
	return glutmain(argc, argv,640,480,"Bullet Physics Demo. http://bulletphysics.org",&ccdDemo);
#endif
	
	//default glut doesn't return from mainloop
	return 0;
}

