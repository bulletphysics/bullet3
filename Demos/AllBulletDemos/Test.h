/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef TEST_H
#define TEST_H


class Test;
struct Settings;
#include <stdarg.h>

typedef Test* TestCreateFcn();

struct Settings
{
	Settings() :
		hz(60.0f),
		iterationCount(10),
		drawStats(0),
		drawContacts(0),
		drawImpulses(0),
		drawAABBs(0),
		drawPairs(0),
		enableWarmStarting(1),
		enablePositionCorrection(1),
		pause(0)
		{}

	float hz;
	int iterationCount;
	int drawStats;
	int drawContacts;
	int drawImpulses;
	int drawAABBs;
	int drawPairs;
	int enableWarmStarting;
	int enablePositionCorrection;
	int pause;
};

struct TestEntry
{
	const char *name;
	TestCreateFcn *createFcn;
};

extern TestEntry g_testEntries[];


class Test
{
public:

	Test();
	virtual ~Test();

	void SetTextLine(int line) { m_textLine = line; }
	virtual void Step(const Settings* settings);
	virtual void Keyboard(unsigned char key) { 
}
//	void MouseDown(const b2Vec2& p);
	void MouseUp();
//	void MouseMove(const b2Vec2& p);
	void LaunchBomb();

protected:

	int m_textLine;
//	b2World* m_world;
//	b2Body* m_bomb;
//	b2MouseJoint* m_mouseJoint;
};

#endif
