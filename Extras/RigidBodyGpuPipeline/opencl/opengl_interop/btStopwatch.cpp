/*
Stopwatch for timing and profiling for the Bullet Physics Library, http://bulletphysics.org
Copyright (c) 2003-2011 Erwin Coumans

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btStopwatch.h"


#ifdef __CELLOS_LV2__
#include <sys/sys_time.h>
#include <sys/time_util.h>
#include <stdio.h>
#endif

#if defined (SUNOS) || defined (__SUNOS__) 
#include <stdio.h> 
#endif

#if defined(WIN32) || defined(_WIN32)

#define BT_USE_WINDOWS_TIMERS
#define WIN32_LEAN_AND_MEAN
#define NOWINRES
#define NOMCX
#define NOIME 

#ifdef _XBOX
	#include <Xtl.h>
#else //_XBOX
	#include <windows.h>
#endif //_XBOX

#include <time.h>


#else //_WIN32
#include <sys/time.h>
#endif //_WIN32

#define mymin(a,b) (a > b ? a : b)

struct btStopwatchData
{

#ifdef BT_USE_WINDOWS_TIMERS
	LARGE_INTEGER mClockFrequency;
	DWORD mStartTick;
	LONGLONG mPrevElapsedTime;
	LARGE_INTEGER mStartTime;
#else
#ifdef __CELLOS_LV2__
	uint64_t	mStartTime;
#else
	struct timeval mStartTime;
#endif
#endif //__CELLOS_LV2__

};


btStopwatch::btStopwatch()
{
	m_data = new btStopwatchData;
#ifdef BT_USE_WINDOWS_TIMERS
	QueryPerformanceFrequency(&m_data->mClockFrequency);
#endif
	reset();
}

btStopwatch::~btStopwatch()
{
	delete m_data;
}

btStopwatch::btStopwatch(const btStopwatch& other)
{
	m_data = new btStopwatchData;
	*m_data = *other.m_data;
}

btStopwatch& btStopwatch::operator=(const btStopwatch& other)
{
	*m_data = *other.m_data;
	return *this;
}


	/// Resets the initial reference time.
void btStopwatch::reset()
{
#ifdef BT_USE_WINDOWS_TIMERS
	QueryPerformanceCounter(&m_data->mStartTime);
	m_data->mStartTick = GetTickCount();
	m_data->mPrevElapsedTime = 0;
#else
#ifdef __CELLOS_LV2__

	typedef uint64_t  ClockSize;
	ClockSize newTime;
	//__asm __volatile__( "mftb %0" : "=r" (newTime) : : "memory");
	SYS_TIMEBASE_GET( newTime );
	m_data->mStartTime = newTime;
#else
	gettimeofday(&m_data->mStartTime, 0);
#endif
#endif
}

/// Returns the time in ms since the last call to reset or since 
/// the btStopwatch was created.
float btStopwatch::getTimeMilliseconds()
{
	return getTimeMicroseconds()/1000.f;
}

	/// Returns the time in us since the last call to reset or since 
	/// the stopwatch was created.
unsigned long int btStopwatch::getTimeMicroseconds()
{
#ifdef BT_USE_WINDOWS_TIMERS
		LARGE_INTEGER currentTime;
		QueryPerformanceCounter(&currentTime);
		LONGLONG elapsedTime = currentTime.QuadPart - m_data->mStartTime.QuadPart;

		// Compute the number of millisecond ticks elapsed.
		unsigned long msecTicks = (unsigned long)(1000 * elapsedTime / m_data->mClockFrequency.QuadPart);

		// Check for unexpected leaps in the Win32 performance counter.  
		// (This is caused by unexpected data across the PCI to ISA 
		// bridge, aka south bridge.  See Microsoft KB274323.)
		unsigned long elapsedTicks = GetTickCount() - m_data->mStartTick;
		signed long msecOff = (signed long)(msecTicks - elapsedTicks);
		if (msecOff < -100 || msecOff > 100)
		{
			// Adjust the starting time forwards.
			LONGLONG msecAdjustment = mymin(msecOff * 
				m_data->mClockFrequency.QuadPart / 1000, elapsedTime - 
				m_data->mPrevElapsedTime);
			m_data->mStartTime.QuadPart += msecAdjustment;
			elapsedTime -= msecAdjustment;
		}

		// Store the current elapsed time for adjustments next time.
		m_data->mPrevElapsedTime = elapsedTime;

		// Convert to microseconds.
		unsigned long usecTicks = (unsigned long)(1000000 * elapsedTime / 
			m_data->mClockFrequency.QuadPart);

		return usecTicks;
#else

#ifdef __CELLOS_LV2__
		uint64_t freq=sys_time_get_timebase_frequency();
		double dFreq=((double) freq)/ 1000000.0;
		typedef uint64_t  ClockSize;
		ClockSize newTime;
		//__asm __volatile__( "mftb %0" : "=r" (newTime) : : "memory");
		SYS_TIMEBASE_GET( newTime );

		return (unsigned long int)((double(newTime-m_data->mStartTime)) / dFreq);
#else

		struct timeval currentTime;
		gettimeofday(&currentTime, 0);
		return (currentTime.tv_sec - m_data->mStartTime.tv_sec) * 1000000 + (currentTime.tv_usec - m_data->mStartTime.tv_usec);
#endif//__CELLOS_LV2__
#endif 
}



