//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.

#include "task/PxTask.h"
#include "task/PxTaskDefine.h"
#include "foundation/PxErrors.h"

#include "PsThread.h"
#include "PsAtomic.h"
#include "PsMutex.h"
#include "PsHashMap.h"
#include "PsArray.h"
#include "PsAllocator.h"

#define DOT_LOG 0

#define LOCK()  shdfnd::Mutex::ScopedLock __lock__(mMutex)

namespace physx
{
    const int EOL = -1;
	typedef shdfnd::HashMap<const char *, PxTaskID> PxTaskNameToIDMap;

	struct PxTaskDepTableRow
	{
		PxTaskID    mTaskID;
		int       mNextDep;
	};
	typedef shdfnd::Array<PxTaskDepTableRow> PxTaskDepTable;

	class PxTaskTableRow
	{
	public:
		PxTaskTableRow() : mRefCount( 1 ), mStartDep(EOL), mLastDep(EOL) {}
		void addDependency( PxTaskDepTable& depTable, PxTaskID taskID )
		{
			int newDep = int(depTable.size());
			PxTaskDepTableRow row;
			row.mTaskID = taskID;
			row.mNextDep = EOL;
			depTable.pushBack( row );

			if( mLastDep == EOL )
			{
				mStartDep = mLastDep = newDep;
			}
			else
			{
				depTable[ uint32_t(mLastDep) ].mNextDep = newDep;
				mLastDep = newDep;
			}
		}

		PxTask *    mTask;
		volatile int mRefCount;
		PxTaskType::Enum mType;
		int       mStartDep;
		int       mLastDep;
	};
	typedef shdfnd::Array<PxTaskTableRow> PxTaskTable;


/* Implementation of PxTaskManager abstract API */
class PxTaskMgr : public PxTaskManager, public shdfnd::UserAllocated
{
	PX_NOCOPY(PxTaskMgr)
public:
	PxTaskMgr(PxErrorCallback& , PxCpuDispatcher*, PxGpuDispatcher*);
	~PxTaskMgr();

	void     setCpuDispatcher( PxCpuDispatcher& ref )
	{
		mCpuDispatcher = &ref;
	}

	void     setGpuDispatcher( PxGpuDispatcher& ref )
	{
		mGpuDispatcher = &ref;
	}

	PxCpuDispatcher* getCpuDispatcher() const
	{
		return mCpuDispatcher;
	}

	PxGpuDispatcher* getGpuDispatcher() const
	{
		return mGpuDispatcher;
	}

	void	resetDependencies();
	void	startSimulation();
	void	stopSimulation();
	void	taskCompleted( PxTask& task );

	PxTaskID  getNamedTask( const char *name );
	PxTaskID  submitNamedTask( PxTask *task, const char *name, PxTaskType::Enum type = PxTaskType::TT_CPU );
	PxTaskID  submitUnnamedTask( PxTask& task, PxTaskType::Enum type = PxTaskType::TT_CPU );
	PxTask*   getTaskFromID( PxTaskID );

	bool    dispatchTask( PxTaskID taskID, bool gpuGroupStart );
	bool    resolveRow( PxTaskID taskID, bool gpuGroupStart );

	void    release();

	void	finishBefore( PxTask& task, PxTaskID taskID );
	void	startAfter( PxTask& task, PxTaskID taskID );

	void	addReference( PxTaskID taskID );
	void	decrReference( PxTaskID taskID );
	int32_t	getReference( PxTaskID taskID ) const;

	void	decrReference( PxLightCpuTask& lighttask );
	void	addReference( PxLightCpuTask& lighttask );		

	PxErrorCallback&			mErrorCallback;
	PxCpuDispatcher           *mCpuDispatcher;
	PxGpuDispatcher           *mGpuDispatcher;		
	PxTaskNameToIDMap          mName2IDmap;
	volatile int			 mPendingTasks;
    shdfnd::Mutex            mMutex;

	PxTaskDepTable				 mDepTable;
	PxTaskTable				 mTaskTable;

	shdfnd::Array<PxTaskID>	 mStartDispatch;
	};

PxTaskManager* PxTaskManager::createTaskManager(PxErrorCallback& errorCallback, PxCpuDispatcher* cpuDispatcher, PxGpuDispatcher* gpuDispatcher)
{
	return PX_NEW(PxTaskMgr)(errorCallback, cpuDispatcher, gpuDispatcher);
}

PxTaskMgr::PxTaskMgr(PxErrorCallback& errorCallback, PxCpuDispatcher* cpuDispatcher, PxGpuDispatcher* gpuDispatcher)
	: mErrorCallback (errorCallback)
	, mCpuDispatcher( cpuDispatcher )
    , mGpuDispatcher( gpuDispatcher )	
	, mPendingTasks( 0 )
	, mDepTable(PX_DEBUG_EXP("PxTaskDepTable"))
	, mTaskTable(PX_DEBUG_EXP("PxTaskTable"))	
	, mStartDispatch(PX_DEBUG_EXP("StartDispatch"))
{
}

PxTaskMgr::~PxTaskMgr()
{
}

void PxTaskMgr::release()
{
	PX_DELETE(this);
}

void PxTaskMgr::decrReference(PxLightCpuTask& lighttask)
{
	/* This does not need a lock! */
	if (!shdfnd::atomicDecrement(&lighttask.mRefCount))
	{
		PX_ASSERT(mCpuDispatcher);
		if (mCpuDispatcher)
		{
			mCpuDispatcher->submitTask(lighttask);
		}
		else
		{
			lighttask.release();
		}
	}
}

void PxTaskMgr::addReference(PxLightCpuTask& lighttask)
{
	/* This does not need a lock! */
	shdfnd::atomicIncrement(&lighttask.mRefCount);
}

/*
 * Called by the owner (Scene) at the start of every frame, before
 * asking for tasks to be submitted.
 */
void PxTaskMgr::resetDependencies()
{
	PX_ASSERT( !mPendingTasks ); // only valid if you don't resubmit named tasks, this is true for the SDK
    PX_ASSERT( mCpuDispatcher );
    mTaskTable.clear();
    mDepTable.clear();
    mName2IDmap.clear();
    mPendingTasks = 0;
}

/* 
 * Called by the owner (Scene) to start simulating the task graph.
 * Dispatch all tasks with refCount == 1
 */
void PxTaskMgr::startSimulation()
{
    PX_ASSERT( mCpuDispatcher );

	if( mGpuDispatcher )
	{
		mGpuDispatcher->startSimulation();
	}

	/* Handle empty task graph */
	if( mPendingTasks == 0 )
    {

		return;
    }

    bool gpuDispatch = false;
    for( PxTaskID i = 0 ; i < mTaskTable.size() ; i++ )
    {
		if(	mTaskTable[ i ].mType == PxTaskType::TT_COMPLETED )
		{
			continue;
		}
		if( !shdfnd::atomicDecrement( &mTaskTable[ i ].mRefCount ) )
		{
			mStartDispatch.pushBack(i);
		}
	}
	for( uint32_t i=0; i<mStartDispatch.size(); ++i)
	{
		gpuDispatch |= dispatchTask( mStartDispatch[i], gpuDispatch );
	}
	//mStartDispatch.resize(0);
	mStartDispatch.forceSize_Unsafe(0);

    if( mGpuDispatcher && gpuDispatch )
	{
        mGpuDispatcher->finishGroup();
	}
}

void PxTaskMgr::stopSimulation()
{
	if( mGpuDispatcher )
	{
		mGpuDispatcher->stopSimulation();
	}
}

PxTaskID PxTaskMgr::getNamedTask( const char *name )
{
	const PxTaskNameToIDMap::Entry *ret;
    {
        LOCK();
		ret = mName2IDmap.find( name );
    }
    if( ret )
	{
        return ret->second;
	}
    else
	{
        // create named entry in task table, without a task
        return submitNamedTask( NULL, name, PxTaskType::TT_NOT_PRESENT );
}
}

PxTask* PxTaskMgr::getTaskFromID( PxTaskID id )
{
	LOCK(); // todo: reader lock necessary?
	return mTaskTable[ id ].mTask;
}


/* If called at runtime, must be thread-safe */
PxTaskID PxTaskMgr::submitNamedTask( PxTask *task, const char *name, PxTaskType::Enum type )
{
    if( task )
    {
        task->mTm = this;
        task->submitted();
    }

    LOCK();

	const PxTaskNameToIDMap::Entry *ret = mName2IDmap.find( name );
    if( ret )
    {
		PxTaskID prereg = ret->second;
		if( task )
		{
			/* name was registered for us by a dependent task */
			PX_ASSERT( !mTaskTable[ prereg ].mTask );
			PX_ASSERT( mTaskTable[ prereg ].mType == PxTaskType::TT_NOT_PRESENT );
			mTaskTable[ prereg ].mTask = task;
			mTaskTable[ prereg ].mType = type;
			task->mTaskID = prereg;
		}
		return prereg;
    }
    else
    {
        shdfnd::atomicIncrement(&mPendingTasks);
        PxTaskID id = static_cast<PxTaskID>(mTaskTable.size());
        mName2IDmap[ name ] = id;
        if( task )
		{
            task->mTaskID = id;
		}
        PxTaskTableRow r;
        r.mTask = task;
        r.mType = type;
		mTaskTable.pushBack(r);
        return id;
    }
}

/*
 * Add an unnamed task to the task table
 */
PxTaskID PxTaskMgr::submitUnnamedTask( PxTask& task, PxTaskType::Enum type )
{
    shdfnd::atomicIncrement(&mPendingTasks);

	task.mTm = this;
    task.submitted();
    
	LOCK();
    task.mTaskID = static_cast<PxTaskID>(mTaskTable.size());
    PxTaskTableRow r;
    r.mTask = &task;
    r.mType = type;
    mTaskTable.pushBack(r);
    return task.mTaskID;
}


/* Called by worker threads (or cooperating application threads) when a
 * PxTask has completed.  Propogate depdenencies, decrementing all
 * referenced tasks' refCounts.  If any of those reach zero, activate
 * those tasks.
 */
void PxTaskMgr::taskCompleted( PxTask& task )
{
    LOCK();
    if( resolveRow( task.mTaskID, false ) )
	{
        mGpuDispatcher->finishGroup();
	}
}

/* ================== Private Functions ======================= */

/*
 * Add a dependency to force 'task' to complete before the
 * referenced 'taskID' is allowed to be dispatched.
 */
void PxTaskMgr::finishBefore( PxTask& task, PxTaskID taskID )
{
    LOCK();
	PX_ASSERT( mTaskTable[ taskID ].mType != PxTaskType::TT_COMPLETED );

    mTaskTable[ task.mTaskID ].addDependency( mDepTable, taskID );
	shdfnd::atomicIncrement( &mTaskTable[ taskID ].mRefCount );
}


/*
 * Add a dependency to force 'task' to wait for the referenced 'taskID'
 * to complete before it is allowed to be dispatched.
 */
void PxTaskMgr::startAfter( PxTask& task, PxTaskID taskID )
{
    LOCK();
	PX_ASSERT( mTaskTable[ taskID ].mType != PxTaskType::TT_COMPLETED );

    mTaskTable[ taskID ].addDependency( mDepTable, task.mTaskID );
	shdfnd::atomicIncrement( &mTaskTable[ task.mTaskID ].mRefCount );
}


void PxTaskMgr::addReference( PxTaskID taskID )
{
    LOCK();
    shdfnd::atomicIncrement( &mTaskTable[ taskID ].mRefCount );
}

/*
 * Remove one reference count from a task.  Intended for use by the
 * GPU dispatcher, to remove reference counts when CUDA events are
 * resolved.  Must be done here to make it thread safe.
 */
void PxTaskMgr::decrReference( PxTaskID taskID )
{
    LOCK();

    if( !shdfnd::atomicDecrement( &mTaskTable[ taskID ].mRefCount ) )
    {
        if( dispatchTask( taskID, false ) )
        {
            mGpuDispatcher->finishGroup();
        }
    }
}

int32_t PxTaskMgr::getReference(PxTaskID taskID) const
{
	return mTaskTable[ taskID ].mRefCount;
}

/*
 * A task has completed, decrement all dependencies and submit tasks
 * that are ready to run.  Signal simulation end if ther are no more
 * pending tasks.
 */
bool PxTaskMgr::resolveRow( PxTaskID taskID, bool gpuGroupStart )
{
    int depRow = mTaskTable[ taskID ].mStartDep;

	uint32_t streamIndex = 0;
	bool syncRequired = false;
	if( mTaskTable[ taskID ].mTask )
	{
		streamIndex = mTaskTable[ taskID ].mTask->mStreamIndex;
	}

    while( depRow != EOL )
    {
        PxTaskDepTableRow& row = mDepTable[ uint32_t(depRow) ];
        PxTaskTableRow& dtt = mTaskTable[ row.mTaskID ];

		// pass stream index to (up to one) dependent GPU task
		if( dtt.mTask && dtt.mType == PxTaskType::TT_GPU && streamIndex )
		{
			if( dtt.mTask->mStreamIndex )
			{
				PX_ASSERT( dtt.mTask->mStreamIndex != streamIndex );
				dtt.mTask->mPreSyncRequired = true;
			}
			else if( syncRequired )
			{
				dtt.mTask->mPreSyncRequired = true;
			}
			else
			{
				dtt.mTask->mStreamIndex = streamIndex;
				/* only one forward task gets to use this stream */
				syncRequired = true;
			}
		}

        if( !shdfnd::atomicDecrement( &dtt.mRefCount ) )
		{
			gpuGroupStart |= dispatchTask( row.mTaskID, gpuGroupStart );
		}

        depRow = row.mNextDep;
    }

    shdfnd::atomicDecrement( &mPendingTasks );
    return gpuGroupStart;
}

/*
 * Submit a ready task to its appropriate dispatcher.
 */
bool PxTaskMgr::dispatchTask( PxTaskID taskID, bool gpuGroupStart )
{
	LOCK(); // todo: reader lock necessary?
    PxTaskTableRow& tt = mTaskTable[ taskID ];

    // prevent re-submission
    if( tt.mType == PxTaskType::TT_COMPLETED )
    {		
		mErrorCallback.reportError(PxErrorCode::eDEBUG_WARNING, "PxTask dispatched twice", __FILE__, __LINE__);
        return false;
    }

    switch ( tt.mType )
    {
    case PxTaskType::TT_CPU:
        mCpuDispatcher->submitTask( *tt.mTask );
        break;

	case PxTaskType::TT_GPU:
#if PX_WINDOWS_FAMILY
        if( mGpuDispatcher )
        {
			if( !gpuGroupStart )
			{
				mGpuDispatcher->startGroup();
			}
			mGpuDispatcher->submitTask( *tt.mTask );
			gpuGroupStart = true;
		}
		else
#endif
		{
			mErrorCallback.reportError(PxErrorCode::eDEBUG_WARNING, "No GPU dispatcher", __FILE__, __LINE__);
		}
		break;

    case PxTaskType::TT_NOT_PRESENT:
		/* No task registered with this taskID, resolve its dependencies */
		PX_ASSERT(!tt.mTask);
		//shdfnd::getFoundation().error(PX_INFO, "unregistered task resolved");
        gpuGroupStart |= resolveRow( taskID, gpuGroupStart );
		break;
	case PxTaskType::TT_COMPLETED:
    default:
        mErrorCallback.reportError(PxErrorCode::eDEBUG_WARNING, "Unknown task type", __FILE__, __LINE__);
        gpuGroupStart |= resolveRow( taskID, gpuGroupStart );
        break;
    }

    tt.mType = PxTaskType::TT_COMPLETED;
    return gpuGroupStart;
}

}// end physx namespace
