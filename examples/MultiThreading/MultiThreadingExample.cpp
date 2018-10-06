
#include "MultiThreadingExample.h"
#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "../CommonInterfaces/CommonRenderInterface.h"

#include "../CommonInterfaces/CommonExampleInterface.h"
#include "LinearMath/btTransform.h"

#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../RenderingExamples/TimeSeriesCanvas.h"
#include "stb_image/stb_image.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3Matrix3x3.h"
#include "../Utils/b3Clock.h"
#include "../CommonInterfaces/CommonParameterInterface.h"

#include "LinearMath/btAlignedObjectArray.h"
#define stdvector btAlignedObjectArray

#define MAGIC_RESET_NUMBER 64738

void SampleThreadFunc(void* userPtr, void* lsMemory);
void* SamplelsMemoryFunc();
void SamplelsMemoryReleaseFunc(void* ptr);

#include <stdio.h>
//#include "BulletMultiThreaded/PlatformDefinitions.h"

#ifndef _WIN32
#include "b3PosixThreadSupport.h"

b3ThreadSupportInterface* createThreadSupport(int numThreads)
{
	b3PosixThreadSupport::ThreadConstructionInfo constructionInfo("testThreads",
																  SampleThreadFunc,
																  SamplelsMemoryFunc,
																  SamplelsMemoryReleaseFunc,
																  numThreads);
	b3ThreadSupportInterface* threadSupport = new b3PosixThreadSupport(constructionInfo);

	return threadSupport;
}

#elif defined(_WIN32)
#include "b3Win32ThreadSupport.h"

b3ThreadSupportInterface* createThreadSupport(int numThreads)
{
	b3Win32ThreadSupport::Win32ThreadConstructionInfo threadConstructionInfo("testThreads", SampleThreadFunc, SamplelsMemoryFunc, SamplelsMemoryReleaseFunc, numThreads);
	b3Win32ThreadSupport* threadSupport = new b3Win32ThreadSupport(threadConstructionInfo);
	return threadSupport;
}
#endif

struct SampleJobInterface
{
	virtual void executeJob(int threadIndex) = 0;
};

struct SampleJob1 : public SampleJobInterface
{
	float m_fakeWork;
	int m_jobId;

	SampleJob1(int jobId)
		: m_fakeWork(0),
		  m_jobId(jobId)
	{
	}
	virtual ~SampleJob1() {}

	virtual void executeJob(int threadIndex)
	{
		printf("start SampleJob1 %d using threadIndex %d\n", m_jobId, threadIndex);
		//do some fake work
		for (int i = 0; i < 1000000; i++)
			m_fakeWork = b3Scalar(1.21) * m_fakeWork;
		printf("finished SampleJob1 %d using threadIndex %d\n", m_jobId, threadIndex);
	}
};

struct SampleArgs
{
	SampleArgs()
	{
	}
	b3CriticalSection* m_cs;

	btAlignedObjectArray<SampleJobInterface*> m_jobQueue;

	void submitJob(SampleJobInterface* job)
	{
		m_cs->lock();
		m_jobQueue.push_back(job);
		m_cs->unlock();
	}
	SampleJobInterface* consumeJob()
	{
		SampleJobInterface* job = 0;
		m_cs->lock();
		int sz = m_jobQueue.size();
		if (sz)
		{
			job = m_jobQueue[sz - 1];
			m_jobQueue.pop_back();
		}
		m_cs->unlock();
		return job;
	}
};
SampleArgs args;
struct SampleThreadLocalStorage
{
	int threadId;
};

void SampleThreadFunc(void* userPtr, void* lsMemory)
{
	printf("SampleThreadFunc thread started\n");

	SampleThreadLocalStorage* localStorage = (SampleThreadLocalStorage*)lsMemory;

	SampleArgs* args = (SampleArgs*)userPtr;

	bool requestExit = false;
	while (!requestExit)
	{
		SampleJobInterface* job = args->consumeJob();
		if (job)
		{
			job->executeJob(localStorage->threadId);
		}

		b3Clock::usleep(250);

		args->m_cs->lock();
		int exitMagicNumber = args->m_cs->getSharedParam(1);
		requestExit = (exitMagicNumber == MAGIC_RESET_NUMBER);
		args->m_cs->unlock();
	}

	printf("finished\n");
	//do nothing
}

void* SamplelsMemoryFunc()
{
	//don't create local store memory, just return 0
	return new SampleThreadLocalStorage;
}

void SamplelsMemoryReleaseFunc(void* ptr)
{
	SampleThreadLocalStorage* p = (SampleThreadLocalStorage*)ptr;
	delete p;
}

class MultiThreadingExample : public CommonExampleInterface
{
	CommonGraphicsApp* m_app;
	b3ThreadSupportInterface* m_threadSupport;
	btAlignedObjectArray<SampleJob1*> m_jobs;
	int m_numThreads;

public:
	MultiThreadingExample(GUIHelperInterface* guiHelper, int tutorialIndex)
		: m_app(guiHelper->getAppInterface()),
		  m_threadSupport(0),
		  m_numThreads(8)
	{
		//int numBodies = 1;

		m_app->setUpAxis(1);
	}
	virtual ~MultiThreadingExample()
	{
	}

	virtual void renderScene()
	{
	}
	virtual void initPhysics()
	{
		b3Printf("initPhysics");

		m_threadSupport = createThreadSupport(m_numThreads);

		for (int i = 0; i < m_threadSupport->getNumTasks(); i++)
		{
			SampleThreadLocalStorage* storage = (SampleThreadLocalStorage*)m_threadSupport->getThreadLocalMemory(i);
			b3Assert(storage);
			storage->threadId = i;
		}

		args.m_cs = m_threadSupport->createCriticalSection();
		args.m_cs->setSharedParam(0, 100);

		for (int i = 0; i < 100; i++)
		{
			SampleJob1* job = new SampleJob1(i);
			args.submitJob(job);
		}

		int i;
		for (i = 0; i < m_numThreads; i++)
		{
			m_threadSupport->runTask(B3_THREAD_SCHEDULE_TASK, (void*)&args, i);
		}
		b3Printf("Threads started");
	}
	virtual void exitPhysics()
	{
		b3Printf("exitPhysics, stopping threads");
		bool blockingWait = false;
		int arg0, arg1;

		args.m_cs->lock();
		//terminate all threads
		args.m_cs->setSharedParam(1, MAGIC_RESET_NUMBER);
		args.m_cs->unlock();

		if (blockingWait)
		{
			for (int i = 0; i < m_numThreads; i++)
			{
				m_threadSupport->waitForResponse(&arg0, &arg1);
				printf("finished waiting for response: %d %d\n", arg0, arg1);
			}
		}
		else
		{
			int numActiveThreads = m_numThreads;
			while (numActiveThreads)
			{
				if (m_threadSupport->isTaskCompleted(&arg0, &arg1, 0))
				{
					numActiveThreads--;
					printf("numActiveThreads = %d\n", numActiveThreads);
				}
				else
				{
					//				printf("polling..");
				}
			};
		}

		delete m_threadSupport;

		b3Printf("Threads stopped");
		for (int i = 0; i < m_jobs.size(); i++)
		{
			delete m_jobs[i];
		}
		m_jobs.clear();
	}

	virtual void stepSimulation(float deltaTime)
	{
	}

	virtual void physicsDebugDraw(int debugDrawFlags)
	{
	}
	virtual bool mouseMoveCallback(float x, float y)
	{
		return false;
	}
	virtual bool mouseButtonCallback(int button, int state, float x, float y)
	{
		return false;
	}
	virtual bool keyboardCallback(int key, int state)
	{
		return false;
	}

	virtual void resetCamera()
	{
		float dist = 10.5;
		float pitch = -32;
		float yaw = 136;
		float targetPos[3] = {0, 0, 0};
		if (m_app->m_renderer && m_app->m_renderer->getActiveCamera())
		{
			m_app->m_renderer->getActiveCamera()->setCameraDistance(dist);
			m_app->m_renderer->getActiveCamera()->setCameraPitch(pitch);
			m_app->m_renderer->getActiveCamera()->setCameraYaw(yaw);
			m_app->m_renderer->getActiveCamera()->setCameraTargetPosition(targetPos[0], targetPos[1], targetPos[2]);
		}
	}
};

class CommonExampleInterface* MultiThreadingExampleCreateFunc(struct CommonExampleOptions& options)
{
	return new MultiThreadingExample(options.m_guiHelper, options.m_option);
}
