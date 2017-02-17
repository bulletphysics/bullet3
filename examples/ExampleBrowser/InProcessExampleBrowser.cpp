#include "InProcessExampleBrowser.h"

//#define EXAMPLE_CONSOLE_ONLY
#ifdef EXAMPLE_CONSOLE_ONLY
	#include "EmptyBrowser.h"
	typedef EmptyBrowser DefaultBrowser;
#else
	#include "OpenGLExampleBrowser.h"
	typedef OpenGLExampleBrowser DefaultBrowser;
#endif //EXAMPLE_CONSOLE_ONLY

#include "Bullet3Common/b3CommandLineArgs.h"
#include "../Utils/b3Clock.h"

#include "ExampleEntries.h"
#include "Bullet3Common/b3Scalar.h"
#include "../SharedMemory/InProcessMemory.h"

void	ExampleBrowserThreadFunc(void* userPtr,void* lsMemory);
void*	ExampleBrowserMemoryFunc();

#include <stdio.h>
//#include "BulletMultiThreaded/PlatformDefinitions.h"

#include "Bullet3Common/b3Logging.h"
#include "ExampleEntries.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "EmptyExample.h"

#include "../SharedMemory/PhysicsServerExample.h"
#include "../SharedMemory/PhysicsClientExample.h"

#ifndef _WIN32
#include "../MultiThreading/b3PosixThreadSupport.h"



static b3ThreadSupportInterface* createExampleBrowserThreadSupport(int numThreads)
{
	b3PosixThreadSupport::ThreadConstructionInfo constructionInfo("testThreads",
                                                                ExampleBrowserThreadFunc,
                                                                ExampleBrowserMemoryFunc,
                                                                numThreads);
    b3ThreadSupportInterface* threadSupport = new b3PosixThreadSupport(constructionInfo);

	return threadSupport;

}



#elif defined( _WIN32)
#include "../MultiThreading/b3Win32ThreadSupport.h"

b3ThreadSupportInterface* createExampleBrowserThreadSupport(int numThreads)
{
	b3Win32ThreadSupport::Win32ThreadConstructionInfo threadConstructionInfo("testThreads",ExampleBrowserThreadFunc,ExampleBrowserMemoryFunc,numThreads);
	b3Win32ThreadSupport* threadSupport = new b3Win32ThreadSupport(threadConstructionInfo);
	return threadSupport;

}
#endif





class ExampleEntriesPhysicsServer : public ExampleEntries
{

	struct ExampleEntriesInternalData2* m_data;

public:

	ExampleEntriesPhysicsServer();
	virtual ~ExampleEntriesPhysicsServer();

	static void registerExampleEntry(int menuLevel, const char* name,const char* description, CommonExampleInterface::CreateFunc* createFunc, int option=0);

	virtual void initExampleEntries();

	virtual void initOpenCLExampleEntries();

	virtual int getNumRegisteredExamples();

	virtual CommonExampleInterface::CreateFunc* getExampleCreateFunc(int index);

	virtual const char* getExampleName(int index);

	virtual const char* getExampleDescription(int index);

	virtual int	getExampleOption(int index);

};


struct ExampleEntryPhysicsServer
{
	int									m_menuLevel;
	const char*							m_name;
	const char*							m_description;
	CommonExampleInterface::CreateFunc*		m_createFunc;
	int									m_option;

	ExampleEntryPhysicsServer(int menuLevel, const char* name)
		:m_menuLevel(menuLevel), m_name(name), m_description(0), m_createFunc(0), m_option(0)
	{
	}

	ExampleEntryPhysicsServer(int menuLevel, const char* name,const char* description, CommonExampleInterface::CreateFunc* createFunc, int option=0)
		:m_menuLevel(menuLevel), m_name(name), m_description(description), m_createFunc(createFunc), m_option(option)
	{
	}
};

struct ExampleEntriesInternalData2
{
        btAlignedObjectArray<ExampleEntryPhysicsServer> m_allExamples;
};

static ExampleEntryPhysicsServer gDefaultExamplesPhysicsServer[]=
{

	ExampleEntryPhysicsServer(0,"Robotics Control"),

	ExampleEntryPhysicsServer(1,"Physics Server", "Create a physics server that communicates with a physics client over shared memory",
			PhysicsServerCreateFunc),
    ExampleEntryPhysicsServer(1,"Physics Server (RTC)", "Create a physics server that communicates with a physics client over shared memory. At each update, the Physics Server will continue calling 'stepSimulation' based on the real-time clock (RTC).",
			PhysicsServerCreateFunc,PHYSICS_SERVER_USE_RTC_CLOCK),

	ExampleEntryPhysicsServer(1,"Physics Server (Logging)", "Create a physics server that communicates with a physics client over shared memory. It will log all commands to a file.",
			PhysicsServerCreateFunc,PHYSICS_SERVER_ENABLE_COMMAND_LOGGING),
	ExampleEntryPhysicsServer(1,"Physics Server (Replay Log)", "Create a physics server that replay a command log from disk.",
			PhysicsServerCreateFunc,PHYSICS_SERVER_REPLAY_FROM_COMMAND_LOG),


};


ExampleEntriesPhysicsServer::ExampleEntriesPhysicsServer()
{
	m_data = new ExampleEntriesInternalData2;
}

ExampleEntriesPhysicsServer::~ExampleEntriesPhysicsServer()
{
	delete m_data;
}

void ExampleEntriesPhysicsServer::initOpenCLExampleEntries()
{
}

void ExampleEntriesPhysicsServer::initExampleEntries()
{
	m_data->m_allExamples.clear();



	int numDefaultEntries = sizeof(gDefaultExamplesPhysicsServer)/sizeof(ExampleEntryPhysicsServer);
	for (int i=0;i<numDefaultEntries;i++)
	{
		m_data->m_allExamples.push_back(gDefaultExamplesPhysicsServer[i]);
	}

}

void ExampleEntriesPhysicsServer::registerExampleEntry(int menuLevel, const char* name,const char* description, CommonExampleInterface::CreateFunc* createFunc, int option)
{
}

int ExampleEntriesPhysicsServer::getNumRegisteredExamples()
{
	return m_data->m_allExamples.size();
}

CommonExampleInterface::CreateFunc* ExampleEntriesPhysicsServer::getExampleCreateFunc(int index)
{
	return m_data->m_allExamples[index].m_createFunc;
}

int ExampleEntriesPhysicsServer::getExampleOption(int index)
{
	return m_data->m_allExamples[index].m_option;
}

const char* ExampleEntriesPhysicsServer::getExampleName(int index)
{
	return m_data->m_allExamples[index].m_name;
}

const char* ExampleEntriesPhysicsServer::getExampleDescription(int index)
{
	return m_data->m_allExamples[index].m_description;
}




struct	ExampleBrowserArgs
{
	ExampleBrowserArgs()
		:m_fakeWork(1),m_argc(0)
	{
	}
	b3CriticalSection* m_cs;
	float m_fakeWork;
  int m_argc;
  char** m_argv;
};

struct ExampleBrowserThreadLocalStorage
{
	SharedMemoryInterface* m_sharedMem;
	int threadId;
};

enum TestExampleBrowserCommunicationEnums
{
	eRequestTerminateExampleBrowser = 13,
	eExampleBrowserIsUnInitialized,
	eExampleBrowserIsInitialized,
	eExampleBrowserInitializationFailed,
	eExampleBrowserHasTerminated
};

static double gMinUpdateTimeMicroSecs = 4000.;

void	ExampleBrowserThreadFunc(void* userPtr,void* lsMemory)
{
	printf("ExampleBrowserThreadFunc started\n");

	ExampleBrowserThreadLocalStorage* localStorage = (ExampleBrowserThreadLocalStorage*) lsMemory;

	ExampleBrowserArgs* args = (ExampleBrowserArgs*) userPtr;
	//int workLeft = true;
  b3CommandLineArgs args2(args->m_argc,args->m_argv);
	b3Clock clock;


	ExampleEntriesPhysicsServer examples;
	examples.initExampleEntries();

	DefaultBrowser* exampleBrowser = new DefaultBrowser(&examples);
	exampleBrowser->setSharedMemoryInterface(localStorage->m_sharedMem);

	bool init = exampleBrowser->init(args->m_argc,args->m_argv);
	clock.reset();
	if (init)
	{

		args->m_cs->lock();
		args->m_cs->setSharedParam(0,eExampleBrowserIsInitialized);
		args->m_cs->unlock();

		do
		{
			B3_PROFILE("ExampleBrowserThreadFunc");
			float deltaTimeInSeconds = clock.getTimeMicroseconds()/1000000.f;
			{
				if (deltaTimeInSeconds > 0.1)
				{
					deltaTimeInSeconds = 0.1;
				}
				if (deltaTimeInSeconds < (gMinUpdateTimeMicroSecs/1e6))
				{
					B3_PROFILE("clock.usleep");
					clock.usleep(gMinUpdateTimeMicroSecs/10.);
					exampleBrowser->updateGraphics();
				} else
				{
					B3_PROFILE("exampleBrowser->update");
					clock.reset();
					exampleBrowser->update(deltaTimeInSeconds);
				}
			}

		} while (!exampleBrowser->requestedExit() && (args->m_cs->getSharedParam(0)!=eRequestTerminateExampleBrowser));
	} else
	{
		args->m_cs->lock();
		args->m_cs->setSharedParam(0,eExampleBrowserInitializationFailed);
		args->m_cs->unlock();
	}

	delete exampleBrowser;
	args->m_cs->lock();
	args->m_cs->setSharedParam(0,eExampleBrowserHasTerminated);
	args->m_cs->unlock();
	printf("finished\n");
	//do nothing
}


void*	ExampleBrowserMemoryFunc()
{
	//don't create local store memory, just return 0
	return new ExampleBrowserThreadLocalStorage;
}





struct btInProcessExampleBrowserInternalData
{
	ExampleBrowserArgs m_args;
	b3ThreadSupportInterface* m_threadSupport;
	SharedMemoryInterface* m_sharedMem;
};



btInProcessExampleBrowserInternalData* btCreateInProcessExampleBrowser(int argc,char** argv2)
{

	btInProcessExampleBrowserInternalData* data = new btInProcessExampleBrowserInternalData;
	data->m_sharedMem = new InProcessMemory;

	int numThreads = 1;
	int i;

	data->m_threadSupport = createExampleBrowserThreadSupport(numThreads);

	printf("argc=%d\n", argc);
	for (i=0;i<argc;i++)
	{
		printf("argv[%d] = %s\n",i,argv2[i]);
	}

	for (i=0;i<data->m_threadSupport->getNumTasks();i++)
	{
		ExampleBrowserThreadLocalStorage* storage = (ExampleBrowserThreadLocalStorage*) data->m_threadSupport->getThreadLocalMemory(i);
		b3Assert(storage);
		storage->threadId = i;
		storage->m_sharedMem = data->m_sharedMem;
	}


	data->m_args.m_cs = data->m_threadSupport->createCriticalSection();
	data->m_args.m_cs->setSharedParam(0,eExampleBrowserIsUnInitialized);
 	data->m_args.m_argc = argc;
 	data->m_args.m_argv = argv2;


	for (i=0;i<numThreads;i++)
	{
		data->m_threadSupport->runTask(B3_THREAD_SCHEDULE_TASK, (void*) &data->m_args, i);
	}

	while (data->m_args.m_cs->getSharedParam(0)==eExampleBrowserIsUnInitialized)
	{
		b3Clock::usleep(1000);
	}

	return data;
}

bool btIsExampleBrowserTerminated(btInProcessExampleBrowserInternalData* data)
{
	return (data->m_args.m_cs->getSharedParam(0)==eExampleBrowserHasTerminated);
}

SharedMemoryInterface* btGetSharedMemoryInterface(btInProcessExampleBrowserInternalData* data)
{
	return data->m_sharedMem;
}

void btShutDownExampleBrowser(btInProcessExampleBrowserInternalData* data)
{
	int numActiveThreads = 1;

	data->m_args.m_cs->lock();
	data->m_args.m_cs->setSharedParam(0,eRequestTerminateExampleBrowser);
	data->m_args.m_cs->unlock();

	while (numActiveThreads)
                {
			int arg0,arg1;
                        if (data->m_threadSupport->isTaskCompleted(&arg0,&arg1,0))
                        {
                                numActiveThreads--;
                                printf("numActiveThreads = %d\n",numActiveThreads);

                        } else
                        {
//                              printf("polling..");
							b3Clock::usleep(1000);
                        }
                };

	printf("btShutDownExampleBrowser stopping threads\n");
	data->m_threadSupport->deleteCriticalSection(data->m_args.m_cs);

	delete data->m_threadSupport;
	delete data->m_sharedMem;
	delete data;
}

struct btInProcessExampleBrowserMainThreadInternalData
{
    ExampleEntriesPhysicsServer m_examples;
    DefaultBrowser*    m_exampleBrowser;
    SharedMemoryInterface* m_sharedMem;
    b3Clock m_clock;
};

btInProcessExampleBrowserMainThreadInternalData* btCreateInProcessExampleBrowserMainThread(int argc,char** argv)
{
    btInProcessExampleBrowserMainThreadInternalData* data = new btInProcessExampleBrowserMainThreadInternalData;
    data->m_examples.initExampleEntries();
    data->m_exampleBrowser = new DefaultBrowser(&data->m_examples);
    data->m_sharedMem = new InProcessMemory;
    data->m_exampleBrowser->setSharedMemoryInterface(data->m_sharedMem );
	bool init;
	init = data->m_exampleBrowser->init(argc,argv);
    data->m_clock.reset();
    return data;
}

bool btIsExampleBrowserMainThreadTerminated(btInProcessExampleBrowserMainThreadInternalData* data)
{
    return data->m_exampleBrowser->requestedExit();
}

void btUpdateInProcessExampleBrowserMainThread(btInProcessExampleBrowserMainThreadInternalData* data)
{
    float deltaTimeInSeconds = data->m_clock.getTimeMicroseconds()/1000000.f;
    data->m_clock.reset();
    data->m_exampleBrowser->update(deltaTimeInSeconds);
}
void btShutDownExampleBrowserMainThread(btInProcessExampleBrowserMainThreadInternalData* data)
{

    data->m_exampleBrowser->setSharedMemoryInterface(0);
    delete data->m_exampleBrowser;
    delete data;
}

class SharedMemoryInterface* btGetSharedMemoryInterfaceMainThread(btInProcessExampleBrowserMainThreadInternalData* data)
{
    return data->m_sharedMem;
}
