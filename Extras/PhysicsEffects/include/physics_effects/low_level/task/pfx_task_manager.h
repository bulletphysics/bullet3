/*
Physics Effects Copyright(C) 2010 Sony Computer Entertainment Inc.
All rights reserved.

Physics Effects is open software; you can redistribute it and/or
modify it under the terms of the BSD License.

Physics Effects is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the BSD License for more details.

A copy of the BSD License is distributed with
Physics Effects under the filename: physics_effects_license.txt
*/

#ifndef _SCE_PFX_TASK_MANAGER_H
#define _SCE_PFX_TASK_MANAGER_H

#include "../../base_level/base/pfx_common.h"
#include "../../base_level/base/pfx_heap_manager.h"
#include "pfx_sync_components.h"

#define SCE_PFX_IO_BUFF_BYTES 1048576
namespace sce {
namespace PhysicsEffects {

//J 並列処理するためのタスクマネージャクラス
//E Task manager class for parallel computation

struct PfxTaskArg
{
	int taskId;
	int maxTasks;
	PfxBarrier *barrier;
	PfxCriticalSection *criticalSection;
	void *io;
	PfxUInt32 data[4];
};

typedef void (*PfxTaskEntry)(PfxTaskArg *arg);

class PfxTaskManager
{
protected:
	PfxUInt32 m_numTasks;
	PfxUInt32 m_maxTasks;
	SCE_PFX_PADDING(1,4)
	PfxUInt8 SCE_PFX_ALIGNED(16) m_ioBuff[SCE_PFX_IO_BUFF_BYTES];
	PfxHeapManager m_pool;
	PfxHeapManager m_ioPool;
	PfxTaskEntry m_taskEntry;
	PfxTaskArg *m_taskArg;
	SCE_PFX_PADDING(2,8)

	PfxTaskManager() : m_pool(NULL,0),m_ioPool(NULL,0) {}

public:
	void *allocate(size_t bytes) {return m_ioPool.allocate(bytes);}
	void deallocate(void *p) {m_ioPool.deallocate(p);}
	void clearPool() {m_ioPool.clear();}

	virtual PfxUInt32 getSharedParam(int i) = 0;
	virtual void setSharedParam(int i,PfxUInt32 p) = 0;

	virtual void startTask(int taskId,void *io,PfxUInt32 data1,PfxUInt32 data2,PfxUInt32 data3,PfxUInt32 data4) = 0;
	virtual void waitTask(int &taskId,PfxUInt32 &data1,PfxUInt32 &data2,PfxUInt32 &data3,PfxUInt32 &data4) = 0;

	virtual void setTaskEntry(void *entry) {m_taskEntry = (PfxTaskEntry)entry;}

public:
	PfxTaskManager(PfxUInt32 numTasks,PfxUInt32 maxTasks,void *workBuff,PfxUInt32 workBytes)
		: m_pool((unsigned char*)workBuff,workBytes),m_ioPool(m_ioBuff,SCE_PFX_IO_BUFF_BYTES)
	{
		SCE_PFX_ASSERT(numTasks>0);
		SCE_PFX_ASSERT(numTasks<=maxTasks);
		m_numTasks = numTasks;
		m_maxTasks = maxTasks;
		m_taskArg = (PfxTaskArg*)m_pool.allocate(sizeof(PfxTaskArg)*m_maxTasks);
	}

	virtual ~PfxTaskManager()
	{
		m_pool.clear();
	}
	
	virtual PfxUInt32 getNumTasks() const {return m_numTasks;}
	virtual void setNumTasks(PfxUInt32 tasks) {m_numTasks = SCE_PFX_MIN(tasks,m_maxTasks);}
	
	virtual void initialize() = 0;
	virtual void finalize() = 0;
};

} //namespace PhysicsEffects
} //namespace sce
#endif // _SCE_PFX_TASK_MANAGER_H