
#include "SharedMemoryInProcessPhysicsC_API.h"
#include "../Utils/b3Clock.h"

#include "PhysicsClientSharedMemory.h"
#include "../ExampleBrowser/InProcessExampleBrowser.h"
#include <stdio.h>
#include <string.h>
#include "PhysicsServerExampleBullet2.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../CommonInterfaces/CommonExampleInterface.h"
#include "InProcessMemory.h"
#include "RemoteGUIHelper.h"

#include "Bullet3Common/b3Logging.h"
class InProcessPhysicsClientSharedMemoryMainThread : public PhysicsClientSharedMemory
{
	btInProcessExampleBrowserMainThreadInternalData* m_data;
	b3Clock m_clock;

public:
	InProcessPhysicsClientSharedMemoryMainThread(int argc, char* argv[], bool useInProcessMemory)
	{
		int newargc = argc + 3;
		char** newargv = (char**)malloc(sizeof(void*) * newargc);
		char* t0 = (char*)"--unused";
		newargv[0] = t0;
		for (int i = 0; i < argc; i++)
			newargv[i + 1] = argv[i];
		newargv[argc + 1] = (char*)"--logtostderr";
		newargv[argc + 2] = (char*)"--start_demo_name=Physics Server";

		m_data = btCreateInProcessExampleBrowserMainThread(newargc, newargv, useInProcessMemory);
		SharedMemoryInterface* shMem = btGetSharedMemoryInterfaceMainThread(m_data);

		setSharedMemoryInterface(shMem);
	}

	virtual ~InProcessPhysicsClientSharedMemoryMainThread()
	{
		setSharedMemoryInterface(0);
		btShutDownExampleBrowserMainThread(m_data);
	}

	// return non-null if there is a status, nullptr otherwise
	virtual const struct SharedMemoryStatus* processServerStatus()
	{
		{
			if (btIsExampleBrowserMainThreadTerminated(m_data))
			{
				PhysicsClientSharedMemory::disconnectSharedMemory();
			}
		}
		{
			unsigned long int ms = m_clock.getTimeMilliseconds();
			if (ms > 2)
			{
				B3_PROFILE("m_clock.reset()");

				btUpdateInProcessExampleBrowserMainThread(m_data);
				m_clock.reset();
			}
		}
		{
			b3Clock::usleep(0);
		}
		const SharedMemoryStatus* stat = 0;

		{
			stat = PhysicsClientSharedMemory::processServerStatus();
		}

		return stat;
	}

	virtual bool submitClientCommand(const struct SharedMemoryCommand& command)
	{
		//        btUpdateInProcessExampleBrowserMainThread(m_data);
		return PhysicsClientSharedMemory::submitClientCommand(command);
	}
};

B3_SHARED_API b3PhysicsClientHandle b3CreateInProcessPhysicsServerAndConnectMainThread(int argc, char* argv[])
{
	InProcessPhysicsClientSharedMemoryMainThread* cl = new InProcessPhysicsClientSharedMemoryMainThread(argc, argv, 1);
	cl->setSharedMemoryKey(SHARED_MEMORY_KEY + 1);
	cl->connect();
	return (b3PhysicsClientHandle)cl;
}

B3_SHARED_API b3PhysicsClientHandle b3CreateInProcessPhysicsServerAndConnectMainThreadSharedMemory(int argc, char* argv[])
{
	InProcessPhysicsClientSharedMemoryMainThread* cl = new InProcessPhysicsClientSharedMemoryMainThread(argc, argv, 0);
	cl->setSharedMemoryKey(SHARED_MEMORY_KEY + 1);
	cl->connect();
	return (b3PhysicsClientHandle)cl;
}



class InProcessPhysicsClientSharedMemory : public PhysicsClientSharedMemory
{
	btInProcessExampleBrowserInternalData* m_data;
	char** m_newargv;

public:
	InProcessPhysicsClientSharedMemory(int argc, char* argv[], bool useInProcessMemory)
	{
		int newargc = argc + 2;
		m_newargv = (char**)malloc(sizeof(void*) * newargc);
		char* t0 = (char*)"--unused";
		m_newargv[0] = t0;

		for (int i = 0; i < argc; i++)
			m_newargv[i + 1] = argv[i];

		char* t1 = (char*)"--start_demo_name=Physics Server";
		m_newargv[argc + 1] = t1;
		m_data = btCreateInProcessExampleBrowser(newargc, m_newargv, useInProcessMemory);
		SharedMemoryInterface* shMem = btGetSharedMemoryInterface(m_data);
		setSharedMemoryInterface(shMem);
	}

	virtual ~InProcessPhysicsClientSharedMemory()
	{
		setSharedMemoryInterface(0);
		btShutDownExampleBrowser(m_data);
		free(m_newargv);
	}
};



B3_SHARED_API b3PhysicsClientHandle b3CreateInProcessPhysicsServerAndConnect(int argc, char* argv[])
{
	InProcessPhysicsClientSharedMemory* cl = new InProcessPhysicsClientSharedMemory(argc, argv, 1);
	cl->setSharedMemoryKey(SHARED_MEMORY_KEY + 1);
	cl->connect();
	return (b3PhysicsClientHandle)cl;
}
B3_SHARED_API b3PhysicsClientHandle b3CreateInProcessPhysicsServerAndConnectSharedMemory(int argc, char* argv[])
{
	InProcessPhysicsClientSharedMemory* cl = new InProcessPhysicsClientSharedMemory(argc, argv, 0);
	cl->setSharedMemoryKey(SHARED_MEMORY_KEY + 1);
	cl->connect();
	return (b3PhysicsClientHandle)cl;
}

class InProcessPhysicsClientExistingExampleBrowser : public PhysicsClientSharedMemory
{
	CommonExampleInterface* m_physicsServerExample;
	SharedMemoryInterface* m_sharedMem;
	b3Clock m_clock;
	unsigned long long int m_prevTime;
	struct GUIHelperInterface* m_guiHelper;

public:
	InProcessPhysicsClientExistingExampleBrowser(struct GUIHelperInterface* guiHelper, bool useInProcessMemory, bool skipGraphicsUpdate, bool ownsGuiHelper)
	{
		m_guiHelper = 0;
		if (ownsGuiHelper)
		{
			m_guiHelper = guiHelper;
		}
		
		m_sharedMem = 0;
		CommonExampleOptions options(guiHelper);

		if (useInProcessMemory)
		{
			m_sharedMem = new InProcessMemory;
			options.m_sharedMem = m_sharedMem;
		}

		options.m_skipGraphicsUpdate = skipGraphicsUpdate;
		m_physicsServerExample = PhysicsServerCreateFuncBullet2(options);
		m_physicsServerExample->initPhysics();
		//m_physicsServerExample->resetCamera();
		setSharedMemoryInterface(m_sharedMem);
		m_clock.reset();
		m_prevTime = m_clock.getTimeMicroseconds();
	}
	virtual ~InProcessPhysicsClientExistingExampleBrowser()
	{
		m_physicsServerExample->exitPhysics();
		//s_instancingRenderer->removeAllInstances();
		delete m_physicsServerExample;
		delete m_sharedMem;
		delete m_guiHelper;
	}

	// return non-null if there is a status, nullptr otherwise
	virtual const struct SharedMemoryStatus* processServerStatus()
	{
		m_physicsServerExample->updateGraphics();

		unsigned long long int curTime = m_clock.getTimeMicroseconds();
		unsigned long long int dtMicro = curTime - m_prevTime;
		m_prevTime = curTime;

		double dt = double(dtMicro) / 1000000.;

		m_physicsServerExample->stepSimulation(dt);
		{
			b3Clock::usleep(0);
		}
		const SharedMemoryStatus* stat = 0;

		{
			stat = PhysicsClientSharedMemory::processServerStatus();
		}

		return stat;
	}

	virtual void renderScene()
	{
		m_physicsServerExample->renderScene();
	}
	virtual void debugDraw(int debugDrawMode)
	{
		m_physicsServerExample->physicsDebugDraw(debugDrawMode);
	}
	virtual bool mouseMoveCallback(float x, float y)
	{
		return m_physicsServerExample->mouseMoveCallback(x, y);
	}
	virtual bool mouseButtonCallback(int button, int state, float x, float y)
	{
		return m_physicsServerExample->mouseButtonCallback(button, state, x, y);
	}
};

void b3InProcessDebugDrawInternal(b3PhysicsClientHandle clientHandle, int debugDrawMode)
{
	InProcessPhysicsClientExistingExampleBrowser* cl = (InProcessPhysicsClientExistingExampleBrowser*)clientHandle;
	cl->debugDraw(debugDrawMode);
}
void b3InProcessRenderSceneInternal(b3PhysicsClientHandle clientHandle)
{
	InProcessPhysicsClientExistingExampleBrowser* cl = (InProcessPhysicsClientExistingExampleBrowser*)clientHandle;
	cl->renderScene();
}

int b3InProcessMouseMoveCallback(b3PhysicsClientHandle clientHandle, float x, float y)
{
	InProcessPhysicsClientExistingExampleBrowser* cl = (InProcessPhysicsClientExistingExampleBrowser*)clientHandle;
	return cl->mouseMoveCallback(x, y);
}
int b3InProcessMouseButtonCallback(b3PhysicsClientHandle clientHandle, int button, int state, float x, float y)
{
	InProcessPhysicsClientExistingExampleBrowser* cl = (InProcessPhysicsClientExistingExampleBrowser*)clientHandle;
	return cl->mouseButtonCallback(button, state, x, y);
}

B3_SHARED_API b3PhysicsClientHandle b3CreateInProcessPhysicsServerFromExistingExampleBrowserAndConnect(void* guiHelperPtr)
{
	static DummyGUIHelper noGfx;

	GUIHelperInterface* guiHelper = (GUIHelperInterface*)guiHelperPtr;
	if (!guiHelper)
	{
		guiHelper = &noGfx;
	}
	bool useInprocessMemory = true;
	bool skipGraphicsUpdate = false;

	InProcessPhysicsClientExistingExampleBrowser* cl = new InProcessPhysicsClientExistingExampleBrowser(guiHelper, useInprocessMemory, skipGraphicsUpdate, false);

	cl->connect();
	return (b3PhysicsClientHandle)cl;
}

extern int gSharedMemoryKey;

B3_SHARED_API b3PhysicsClientHandle b3CreateInProcessPhysicsServerFromExistingExampleBrowserAndConnect3(void* guiHelperPtr, int sharedMemoryKey)
{
	static DummyGUIHelper noGfx;

	gSharedMemoryKey = sharedMemoryKey;
	GUIHelperInterface* guiHelper = (GUIHelperInterface*)guiHelperPtr;
	if (!guiHelper)
	{
		guiHelper = &noGfx;
	}
	bool useInprocessMemory = false;
	bool skipGraphicsUpdate = true;
	InProcessPhysicsClientExistingExampleBrowser* cl = new InProcessPhysicsClientExistingExampleBrowser(guiHelper, useInprocessMemory, skipGraphicsUpdate, false);

	cl->setSharedMemoryKey(sharedMemoryKey + 1);
	cl->connect();
	//backward compatiblity
	gSharedMemoryKey = SHARED_MEMORY_KEY;
	return (b3PhysicsClientHandle)cl;
}

B3_SHARED_API b3PhysicsClientHandle b3CreateInProcessPhysicsServerFromExistingExampleBrowserAndConnect4(void* guiHelperPtr, int sharedMemoryKey)
{
	gSharedMemoryKey = sharedMemoryKey;
	GUIHelperInterface* guiHelper = (GUIHelperInterface*)guiHelperPtr;
	bool ownsGuiHelper = false;
	if (!guiHelper)
	{
		guiHelper = new RemoteGUIHelper();
		ownsGuiHelper = true;
	}
	bool useInprocessMemory = false;
	bool skipGraphicsUpdate = false;
	InProcessPhysicsClientExistingExampleBrowser* cl = new InProcessPhysicsClientExistingExampleBrowser(guiHelper, useInprocessMemory, skipGraphicsUpdate, ownsGuiHelper);

	cl->setSharedMemoryKey(sharedMemoryKey + 1);
	cl->connect();
	//backward compatiblity
	gSharedMemoryKey = SHARED_MEMORY_KEY;
	return (b3PhysicsClientHandle)cl;
}

#ifdef BT_ENABLE_CLSOCKET
#include "RemoteGUIHelperTCP.h"

B3_SHARED_API b3PhysicsClientHandle b3CreateInProcessPhysicsServerFromExistingExampleBrowserAndConnectTCP(const char* hostName, int port)
{
	bool ownsGuiHelper = true;
	GUIHelperInterface* guiHelper = new RemoteGUIHelperTCP(hostName, port);
	
	bool useInprocessMemory = true;
	bool skipGraphicsUpdate = false;
	InProcessPhysicsClientExistingExampleBrowser* cl = new InProcessPhysicsClientExistingExampleBrowser(guiHelper, useInprocessMemory, skipGraphicsUpdate, ownsGuiHelper);

	//cl->setSharedMemoryKey(sharedMemoryKey + 1);
	cl->connect();
	//backward compatiblity
	gSharedMemoryKey = SHARED_MEMORY_KEY;
	return (b3PhysicsClientHandle)cl;
}



//backward compatiblity
B3_SHARED_API b3PhysicsClientHandle b3CreateInProcessPhysicsServerFromExistingExampleBrowserAndConnect2(void* guiHelperPtr)
{
	return b3CreateInProcessPhysicsServerFromExistingExampleBrowserAndConnect3(guiHelperPtr, SHARED_MEMORY_KEY);
}



#include "SharedMemoryCommands.h"
#include "PhysicsClientSharedMemory.h"
#include "GraphicsSharedMemoryBlock.h"
#include "PosixSharedMemory.h"
#include "Win32SharedMemory.h"
class InProcessGraphicsServerSharedMemory : public PhysicsClientSharedMemory
{
	btInProcessExampleBrowserInternalData* m_data2;
	char** m_newargv;
	SharedMemoryCommand m_command;
	
	GraphicsSharedMemoryBlock* m_testBlock1;
	SharedMemoryInterface* m_sharedMemory;

public:
	InProcessGraphicsServerSharedMemory(int port)
	{
		int newargc = 3;
		m_newargv = (char**)malloc(sizeof(void*) * newargc);
		char* t0 = (char*)"--unused";
		m_newargv[0] = t0;

		char* t1 = (char*)"--start_demo_name=Graphics Server";
		char portArg[1024];
		sprintf(portArg, "--port=%d", port);
		
		m_newargv[1] = t1;
		m_newargv[2] = portArg;
		bool useInProcessMemory = false;
		m_data2 = btCreateInProcessExampleBrowser(newargc, m_newargv, useInProcessMemory);
		SharedMemoryInterface* shMem = btGetSharedMemoryInterface(m_data2);
		
		setSharedMemoryInterface(shMem);
		///////////////////

#ifdef _WIN32
		m_sharedMemory = new Win32SharedMemoryServer();
#else
		m_sharedMemory = new PosixSharedMemory();
#endif

			/// server always has to create and initialize shared memory
		bool allowCreation = false;
		m_testBlock1 = (GraphicsSharedMemoryBlock*)m_sharedMemory->allocateSharedMemory(
			GRAPHICS_SHARED_MEMORY_KEY, GRAPHICS_SHARED_MEMORY_SIZE, allowCreation);

	}

	virtual ~InProcessGraphicsServerSharedMemory()
	{
		m_sharedMemory->releaseSharedMemory(GRAPHICS_SHARED_MEMORY_KEY, GRAPHICS_SHARED_MEMORY_SIZE);
		delete m_sharedMemory;
		
		setSharedMemoryInterface(0);
		btShutDownExampleBrowser(m_data2);
		free(m_newargv);
	}
	virtual bool canSubmitCommand() const
	{
		if (m_testBlock1)
		{
			if (m_testBlock1->m_magicId != GRAPHICS_SHARED_MEMORY_MAGIC_NUMBER)
			{
				return false;
			}
		}
		return true;
	}

	virtual struct SharedMemoryCommand* getAvailableSharedMemoryCommand()
	{
		return &m_command;
	}

	virtual bool submitClientCommand(const struct SharedMemoryCommand& command)
	{
		switch (command.m_type)
		{
		default:
		{
		}
		}
		return true;
	}


};


class InProcessGraphicsServerSharedMemoryMainThread : public PhysicsClientSharedMemory
{
	
	btInProcessExampleBrowserMainThreadInternalData* m_data2;
	char** m_newargv;
	SharedMemoryCommand m_command;

	GraphicsSharedMemoryBlock* m_testBlock1;
	SharedMemoryInterface* m_sharedMemory;
	b3Clock m_clock;

public:
	InProcessGraphicsServerSharedMemoryMainThread(int port)
	{
		int newargc = 3;
		m_newargv = (char**)malloc(sizeof(void*) * newargc);
		char* t0 = (char*)"--unused";
		m_newargv[0] = t0;

		
		char* t1 = (char*)"--start_demo_name=Graphics Server";
		m_newargv[1] = t1;
		char portArg[1024];
		sprintf(portArg, "--port=%d", port);
		m_newargv[2] = portArg;

		bool useInProcessMemory = false;
		m_data2 = btCreateInProcessExampleBrowserMainThread(newargc, m_newargv, useInProcessMemory);
		SharedMemoryInterface* shMem = btGetSharedMemoryInterfaceMainThread(m_data2);

		setSharedMemoryInterface(shMem);
		///////////////////

#ifdef _WIN32
		m_sharedMemory = new Win32SharedMemoryServer();
#else
		m_sharedMemory = new PosixSharedMemory();
#endif

		/// server always has to create and initialize shared memory
		bool allowCreation = false;
		m_testBlock1 = (GraphicsSharedMemoryBlock*)m_sharedMemory->allocateSharedMemory(
			GRAPHICS_SHARED_MEMORY_KEY, GRAPHICS_SHARED_MEMORY_SIZE, allowCreation);
		m_clock.reset();
	}

	virtual ~InProcessGraphicsServerSharedMemoryMainThread()
	{
		m_sharedMemory->releaseSharedMemory(GRAPHICS_SHARED_MEMORY_KEY, GRAPHICS_SHARED_MEMORY_SIZE);
		delete m_sharedMemory;

		setSharedMemoryInterface(0);
		btShutDownExampleBrowserMainThread(m_data2);
		free(m_newargv);
	}
	virtual bool canSubmitCommand() const
	{
		btUpdateInProcessExampleBrowserMainThread(m_data2);
		if (m_testBlock1)
		{
			if (m_testBlock1->m_magicId != GRAPHICS_SHARED_MEMORY_MAGIC_NUMBER)
			{
				return false;
			}
		}
		return true;
	}

	virtual struct SharedMemoryCommand* getAvailableSharedMemoryCommand()
	{
		return &m_command;
	}

	virtual bool submitClientCommand(const struct SharedMemoryCommand& command)
	{
		switch (command.m_type)
		{
		default:
		{
		}
		}
		return true;
	}

	// return non-null if there is a status, nullptr otherwise
	virtual const struct SharedMemoryStatus* processServerStatus()
	{
		{
			if (btIsExampleBrowserMainThreadTerminated(m_data2))
			{
				PhysicsClientSharedMemory::disconnectSharedMemory();
			}
		}
		{
			unsigned long int ms = m_clock.getTimeMilliseconds();
			if (ms > 2)
			{
				B3_PROFILE("m_clock.reset()");

				btUpdateInProcessExampleBrowserMainThread(m_data2);
				m_clock.reset();
			}
		}
		{
			b3Clock::usleep(0);
		}
		const SharedMemoryStatus* stat = 0;

		{
			stat = PhysicsClientSharedMemory::processServerStatus();
		}

		return stat;
	}

};



B3_SHARED_API b3PhysicsClientHandle b3CreateInProcessGraphicsServerAndConnectSharedMemory(int port)
{
	InProcessGraphicsServerSharedMemory* cl = new InProcessGraphicsServerSharedMemory(port);
	cl->setSharedMemoryKey(SHARED_MEMORY_KEY + 1);
	cl->connect();
	return (b3PhysicsClientHandle)cl;
}

B3_SHARED_API b3PhysicsClientHandle b3CreateInProcessGraphicsServerAndConnectMainThreadSharedMemory(int port)
{
	InProcessGraphicsServerSharedMemoryMainThread* cl = new InProcessGraphicsServerSharedMemoryMainThread(port);
	cl->setSharedMemoryKey(SHARED_MEMORY_KEY + 1);
	cl->connect();
	return (b3PhysicsClientHandle)cl;
}

#endif
