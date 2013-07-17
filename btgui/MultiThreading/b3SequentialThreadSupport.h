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

#include "Bullet3Common/b3Scalar.h"



#ifndef B3_SEQUENTIAL_THREAD_SUPPORT_H
#define B3_SEQUENTIAL_THREAD_SUPPORT_H

#include "Bullet3Common/b3AlignedObjectArray.h"

#include "b3ThreadSupportInterface.h"

typedef void (*SequentialThreadFunc)(void* userPtr,void* lsMemory);
typedef void* (*SequentiallsMemorySetupFunc)();



///The b3SequentialThreadSupport is a portable non-parallel implementation of the btThreadSupportInterface
///This is useful for debugging and porting SPU Tasks to other platforms.
class b3SequentialThreadSupport : public b3ThreadSupportInterface 
{
public:
	struct	btSpuStatus
	{
		int	m_taskId;
		int	m_commandId;
		int	m_status;

		SequentialThreadFunc	m_userThreadFunc;

		void*	m_userPtr; //for taskDesc etc
		void*	m_lsMemory; //initialized using SequentiallsMemorySetupFunc
	};
private:
	b3AlignedObjectArray<btSpuStatus>	m_activeSpuStatus;
	b3AlignedObjectArray<void*>			m_completeHandles;	
public:
	struct	SequentialThreadConstructionInfo
	{
		SequentialThreadConstructionInfo (const char* uniqueName,
									SequentialThreadFunc userThreadFunc,
									SequentiallsMemorySetupFunc	lsMemoryFunc
									)
									:m_uniqueName(uniqueName),
									m_userThreadFunc(userThreadFunc),
									m_lsMemoryFunc(lsMemoryFunc)
		{

		}

		const char*						m_uniqueName;
		SequentialThreadFunc		m_userThreadFunc;
		SequentiallsMemorySetupFunc	m_lsMemoryFunc;
	};

	b3SequentialThreadSupport(SequentialThreadConstructionInfo& threadConstructionInfo);
	virtual	~b3SequentialThreadSupport();
	void	startThreads(SequentialThreadConstructionInfo&	threadInfo);
///send messages to SPUs
	virtual	void sendRequest(int uiCommand, void* uiArgument0, int uiArgument1);
///check for messages from SPUs
	virtual	void waitForResponse(unsigned int *puiArgument0, unsigned int *puiArgument1);
///start the spus (can be called at the beginning of each frame, to make sure that the right SPU program is loaded)
	virtual	void startThreads();
///tell the task scheduler we are done with the SPU tasks
	virtual	void stopThreads();

	virtual void setNumTasks(int numTasks);

	virtual int getNumTasks() const
	{
		return 1;
	}
	virtual b3Barrier*	createBarrier();

	virtual b3CriticalSection* createCriticalSection();
	
    virtual void deleteBarrier(b3Barrier* barrier);
    
    virtual void deleteCriticalSection(b3CriticalSection* criticalSection);


};

#endif //B3_SEQUENTIAL_THREAD_SUPPORT_H

