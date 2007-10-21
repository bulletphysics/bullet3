

#include "Test.h"

#include "../CcdPhysicsDemo/CcdPhysicsDemo.h"

class CcdPhysicsTest : public Test
{
	CcdPhysicsDemo*	m_demo;
	CcdPhysicsTest()
	{
		m_demo = new CcdPhysicsDemo();
		m_demo->initPhysics();
	}
	virtual ~CcdPhysicsTest()
	{
		delete m_demo;
	}
public:
	void Step(const Settings* settings)
	{
		m_demo->clientMoveAndDisplay();
	}
	static Test* Create()
	{
		return new CcdPhysicsTest;
	}
};

#include "../BspDemo/BspDemo.h"

class BspDemoTest : public Test
{
	BspDemo*	m_demo;

	BspDemoTest()
	{
		m_demo = new BspDemo();
		m_demo->initPhysics("BspDemo.bsp");
	}
	virtual ~BspDemoTest()
	{
		delete m_demo;
	}
public:
	void Step(const Settings* settings)
	{
		m_demo->clientMoveAndDisplay();
	}
	static Test* Create()
	{
		return new BspDemoTest;
	}
};


TestEntry g_testEntries[] =
{
		{"CcdPhysics Test", CcdPhysicsTest::Create},
		{"BspDemo Test", BspDemoTest::Create},
		{0, 0}
};

