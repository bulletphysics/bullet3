/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B3_THREAD_SUPPORT_INTERFACE_H
#define B3_THREAD_SUPPORT_INTERFACE_H

enum
{
	B3_THREAD_SCHEDULE_TASK = 1,
};

#include "Bullet3Common/b3Scalar.h"  //for B3_ATTRIBUTE_ALIGNED16
//#include "PlatformDefinitions.h"
//#include "PpuAddressSpace.h"

class b3Barrier
{
public:
	b3Barrier() {}
	virtual ~b3Barrier() {}

	virtual void sync() = 0;
	virtual void setMaxCount(int n) = 0;
	virtual int getMaxCount() = 0;
};

class b3CriticalSection
{
public:
	b3CriticalSection() {}
	virtual ~b3CriticalSection() {}

	B3_ATTRIBUTE_ALIGNED16(unsigned int mCommonBuff[32]);

	virtual unsigned int getSharedParam(int i) = 0;
	virtual void setSharedParam(int i, unsigned int p) = 0;

	virtual void lock() = 0;
	virtual void unlock() = 0;
};

class b3ThreadSupportInterface
{
public:
	virtual ~b3ThreadSupportInterface();

	virtual void runTask(int uiCommand, void* uiArgument0, int uiArgument1) = 0;

	virtual void waitForResponse(int* puiArgument0, int* puiArgument1) = 0;

	///non-blocking test if a task is completed. First implement all versions, and then enable this API
	virtual bool isTaskCompleted(int* puiArgument0, int* puiArgument1, int timeOutInMilliseconds) = 0;

	virtual void stopThreads() = 0;

	///tell the task scheduler to use no more than numTasks tasks
	virtual void setNumTasks(int numTasks) = 0;

	virtual int getNumTasks() const = 0;

	virtual b3Barrier* createBarrier() = 0;

	virtual b3CriticalSection* createCriticalSection() = 0;

	virtual void deleteBarrier(b3Barrier* barrier) = 0;

	virtual void deleteCriticalSection(b3CriticalSection* criticalSection) = 0;

	virtual void* getThreadLocalMemory(int taskId) { return 0; }
};

#endif  //B3_THREAD_SUPPORT_INTERFACE_H
