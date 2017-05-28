/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include <stdio.h> //printf debugging
#include <algorithm>


// choose threading providers:
#if BT_USE_TBB
#define USE_TBB 1     // use Intel Threading Building Blocks for thread management
#endif

#if BT_USE_PPL
#define USE_PPL 1     // use Microsoft Parallel Patterns Library (installed with Visual Studio 2010 and later)
#endif // BT_USE_PPL

#if BT_USE_OPENMP
#define USE_OPENMP 1  // use OpenMP (also need to change compiler options for OpenMP support)
#endif


#if USE_OPENMP

#include <omp.h>

#endif // #if USE_OPENMP


#if USE_PPL

#include <ppl.h>  // if you get a compile error here, check whether your version of Visual Studio includes PPL
// Visual Studio 2010 and later should come with it
#include <concrtrm.h>  // for GetProcessorCount()
#endif // #if USE_PPL


#if USE_TBB

#define __TBB_NO_IMPLICIT_LINKAGE 1
#include <tbb/tbb.h>
#include <tbb/task_scheduler_init.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

#endif // #if USE_TBB



class TaskManager
{
public:
    enum Api
    {
        apiNone,
        apiOpenMP,
        apiTbb,
        apiPpl,
        apiCount
    };
    static const char* getApiName( Api api )
    {
        switch ( api )
        {
        case apiNone: return "None";
        case apiOpenMP: return "OpenMP";
        case apiTbb: return "Intel TBB";
        case apiPpl: return "MS PPL";
        default: return "unknown";
        }
    }

    TaskManager()
    {
        m_api = apiNone;
        m_numThreads = 0;
#if USE_TBB
        m_tbbSchedulerInit = NULL;
#endif // #if USE_TBB
    }

    Api getApi() const
    {
        return m_api;
    }

    bool isSupported( Api api ) const
    {
#if USE_OPENMP
        if ( api == apiOpenMP )
        {
            return true;
        }
#endif
#if USE_TBB
        if ( api == apiTbb )
        {
            return true;
        }
#endif
#if USE_PPL
        if ( api == apiPpl )
        {
            return true;
        }
#endif
        // apiNone is always "supported"
        return api == apiNone;
    }

    void setApi( Api api )
    {
        if (isSupported(api))
        {
            m_api = api;
        }
        else
        {
            // no compile time support for selected API, fallback to "none"
            m_api = apiNone;
        }
    }

    static int getMaxNumThreads()
    {
#if USE_OPENMP
        return omp_get_max_threads();
#elif USE_PPL
        return concurrency::GetProcessorCount();
#elif USE_TBB
        return tbb::task_scheduler_init::default_num_threads();
#endif
        return 1;
    }

    int getNumThreads() const
    {
        return m_numThreads;
    }

    int setNumThreads( int numThreads )
    {
        m_numThreads = ( std::max )( 1, numThreads );

#if USE_OPENMP
        omp_set_num_threads( m_numThreads );
#endif

#if USE_PPL
        {
            using namespace concurrency;
            if ( CurrentScheduler::Id() != -1 )
            {
                CurrentScheduler::Detach();
            }
            SchedulerPolicy policy;
            policy.SetConcurrencyLimits( m_numThreads, m_numThreads );
            CurrentScheduler::Create( policy );
        }
#endif

#if USE_TBB
        if ( m_tbbSchedulerInit )
        {
            delete m_tbbSchedulerInit;
            m_tbbSchedulerInit = NULL;
        }
        m_tbbSchedulerInit = new tbb::task_scheduler_init( m_numThreads );
#endif
        return m_numThreads;
    }

    void init()
    {
        if (m_numThreads == 0)
        {
#if USE_PPL
            setApi( apiPpl );
#endif
#if USE_TBB
            setApi( apiTbb );
#endif
#if USE_OPENMP
            setApi( apiOpenMP );
#endif
            setNumThreads(getMaxNumThreads());
        }
        else
        {
            setNumThreads(m_numThreads);
        }
    }

    void shutdown()
    {
#if USE_TBB
        if ( m_tbbSchedulerInit )
        {
            delete m_tbbSchedulerInit;
            m_tbbSchedulerInit = NULL;
        }
#endif
    }

private:
    Api m_api;
    int m_numThreads;
#if USE_TBB
    tbb::task_scheduler_init* m_tbbSchedulerInit;
#endif // #if USE_TBB
};

extern TaskManager gTaskMgr;


inline static void initTaskScheduler()
{
    gTaskMgr.init();
}

inline static void cleanupTaskScheduler()
{
    gTaskMgr.shutdown();
}


#if USE_TBB
///
/// TbbBodyAdapter -- Converts a body object that implements the
///                   "forLoop(int iBegin, int iEnd) const" function
///  into a TBB compatible object that takes a tbb::blocked_range<int> type.
///
template <class TBody>
struct TbbBodyAdapter
{
    const TBody* mBody;

    void operator()( const tbb::blocked_range<int>& range ) const
    {
        mBody->forLoop( range.begin(), range.end() );
    }
};
#endif // #if USE_TBB

#if USE_PPL
///
/// PplBodyAdapter -- Converts a body object that implements the
///                   "forLoop(int iBegin, int iEnd) const" function
///  into a PPL compatible object that implements "void operator()( int ) const"
///
template <class TBody>
struct PplBodyAdapter
{
    const TBody* mBody;
    int mGrainSize;
    int mIndexEnd;

    void operator()( int i ) const
    {
        mBody->forLoop( i, (std::min)(i + mGrainSize, mIndexEnd) );
    }
};
#endif // #if USE_PPL


///
/// parallelFor -- interface for submitting work expressed as a for loop to the worker threads
///
template <class TBody>
void parallelFor( int iBegin, int iEnd, int grainSize, const TBody& body )
{
#if USE_OPENMP
    if ( gTaskMgr.getApi() == TaskManager::apiOpenMP )
    {
#pragma omp parallel for schedule(static, 1)
        for ( int i = iBegin; i < iEnd; i += grainSize )
        {
            body.forLoop( i, (std::min)( i + grainSize, iEnd ) );
        }
        return;
    }
#endif // #if USE_OPENMP

#if USE_PPL
    if ( gTaskMgr.getApi() == TaskManager::apiPpl )
    {
        // PPL dispatch
        PplBodyAdapter<TBody> pplBody;
        pplBody.mBody = &body;
        pplBody.mGrainSize = grainSize;
        pplBody.mIndexEnd = iEnd;
        // note: MSVC 2010 doesn't support partitioner args, so avoid them
        concurrency::parallel_for( iBegin,
                                   iEnd,
                                   grainSize,
                                   pplBody
                                   );
        return;
    }
#endif //#if USE_PPL

#if USE_TBB
    if ( gTaskMgr.getApi() == TaskManager::apiTbb )
    {
        // TBB dispatch
        TbbBodyAdapter<TBody> tbbBody;
        tbbBody.mBody = &body;
        tbb::parallel_for( tbb::blocked_range<int>( iBegin, iEnd, grainSize ),
                           tbbBody,
                           tbb::simple_partitioner()
                           );
        return;
    }
#endif // #if USE_TBB

    {
        // run on main thread
        body.forLoop( iBegin, iEnd );
    }

}




