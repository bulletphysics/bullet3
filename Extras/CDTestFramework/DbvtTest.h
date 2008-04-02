/*
Bounding Volume Hierarchy Test
Copyright (c) 2008 Nathanael Presson, as part of Bullet Physics Library

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef DBVTTEST_H
#define DBVTTEST_H

#include "LinearMath/btAlignedObjectArray.h"

#include "CollisionTest.h"
#include "Profiling.h"

class DbvtTest : public CollisionTest
	{
		public:
								DbvtTest(int numBoxes);
		virtual					~DbvtTest();

		virtual	void			Init();
		virtual	void			Release();
		virtual	void			PerformTest();
		virtual	void			Select();
		virtual	void			Deselect();
		virtual	void			KeyboardCallback(unsigned char key, int x, int y);
		virtual	void			MouseCallback(int button, int state, int x, int y);
		virtual	void			MotionCallback(int x, int y);
				
		TwBar*											m_bar;
		btScalar										m_margin;
		btScalar										m_speed;
		btScalar										m_amp;
		Profiler										m_profiler;
		int												m_nbox;
		class btBroadphaseInterface*					m_broadphase;
		btAlignedObjectArray<btScalar>					m_times;
		bool											m_bfirsttime;
	};

#endif
