
#include "SharedMemoryInProcessPhysicsC_API.h"
#include "PhysicsClientSharedMemory.h"
#include"../ExampleBrowser/InProcessExampleBrowser.h"


class InProcessPhysicsClientSharedMemory : public PhysicsClientSharedMemory
{
	btInProcessExampleBrowserInternalData* m_data;
public:

	InProcessPhysicsClientSharedMemory(int argc, char* argv[])
	{
		int newargc = argc+2;
		char** newargv = (char**)malloc(sizeof(void*)*newargc);
		for (int i=0;i<argc;i++)
		newargv[i] = argv[i];

		char* t0 = (char*)"--logtostderr";
		char* t1 = (char*)"--start_demo_name=Physics Server";
		newargv[argc] = t0;
		newargv[argc+1] = t1;
		m_data = btCreateInProcessExampleBrowser(newargc,newargv);
		SharedMemoryInterface* shMem = btGetSharedMemoryInterface(m_data);

		setSharedMemoryInterface(shMem);
	}

	virtual ~InProcessPhysicsClientSharedMemory()
	{
		setSharedMemoryInterface(0);
		btShutDownExampleBrowser(m_data);
	}

};

b3PhysicsClientHandle b3CreateInProcessPhysicsServerAndConnect(int argc, char* argv[])
{
	
	InProcessPhysicsClientSharedMemory* cl = new InProcessPhysicsClientSharedMemory(argc, argv);
    cl->setSharedMemoryKey(SHARED_MEMORY_KEY);
    cl->connect();
	return (b3PhysicsClientHandle ) cl;
}

