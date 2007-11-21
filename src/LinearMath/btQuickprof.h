/*
Copyright (c) 2006 Tyler Streeter

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

*/
	
// Please visit the project website (http://quickprof.sourceforge.net) 
// for usage instructions.

// Credits: The Clock class was inspired by the Timer classes in 
// Ogre (www.ogre3d.org).

#ifndef QUICK_PROF_H
#define QUICK_PROF_H

#include "btScalar.h"

//#define USE_QUICKPROF 1
//Don't use quickprof for now, because it contains STL. TODO: replace STL by Bullet container classes.


//if you don't need btClock, you can comment next line
#define USE_BT_CLOCK 1

#ifdef USE_BT_CLOCK
#ifdef __CELLOS_LV2__
#include <sys/sys_time.h>
#include <stdio.h>
typedef uint64_t __int64;
#endif

#if defined (SUNOS) || defined (__SUNOS__) 
        #include <stdio.h> 
#endif

#if defined(WIN32) || defined(_WIN32)

 #define USE_WINDOWS_TIMERS 
   #define WIN32_LEAN_AND_MEAN 
   #define NOWINRES 
   #define NOMCX 
   #define NOIME 
#ifdef _XBOX
	#include <Xtl.h>
#else
	#include <windows.h>
#endif
	#include <time.h>

#else
	#include <sys/time.h>
#endif

#define mymin(a,b) (a > b ? a : b)

/// basic clock
class btClock
	{
	public:
		btClock()
		{
#ifdef USE_WINDOWS_TIMERS
			QueryPerformanceFrequency(&mClockFrequency);
#endif
			reset();
		}

		~btClock()
		{
		}

		/// Resets the initial reference time.
		void reset()
		{
#ifdef USE_WINDOWS_TIMERS
			QueryPerformanceCounter(&mStartTime);
			mStartTick = GetTickCount();
			mPrevElapsedTime = 0;
#else
#ifdef __CELLOS_LV2__

	typedef uint64_t __int64;
	typedef __int64  ClockSize;
	ClockSize newTime;
	__asm __volatile__( "mftb %0" : "=r" (newTime) : : "memory");
	mStartTime = newTime;
#else
			gettimeofday(&mStartTime, 0);
#endif

#endif
		}

		/// Returns the time in ms since the last call to reset or since 
		/// the btClock was created.
		unsigned long int getTimeMilliseconds()
		{
#ifdef USE_WINDOWS_TIMERS
			LARGE_INTEGER currentTime;
			QueryPerformanceCounter(&currentTime);
			LONGLONG elapsedTime = currentTime.QuadPart - 
				mStartTime.QuadPart;

			// Compute the number of millisecond ticks elapsed.
			unsigned long msecTicks = (unsigned long)(1000 * elapsedTime / 
				mClockFrequency.QuadPart);

			// Check for unexpected leaps in the Win32 performance counter.  
			// (This is caused by unexpected data across the PCI to ISA 
			// bridge, aka south bridge.  See Microsoft KB274323.)
			unsigned long elapsedTicks = GetTickCount() - mStartTick;
			signed long msecOff = (signed long)(msecTicks - elapsedTicks);
			if (msecOff < -100 || msecOff > 100)
			{
				// Adjust the starting time forwards.
				LONGLONG msecAdjustment = mymin(msecOff * 
					mClockFrequency.QuadPart / 1000, elapsedTime - 
					mPrevElapsedTime);
				mStartTime.QuadPart += msecAdjustment;
				elapsedTime -= msecAdjustment;

				// Recompute the number of millisecond ticks elapsed.
				msecTicks = (unsigned long)(1000 * elapsedTime / 
					mClockFrequency.QuadPart);
			}

			// Store the current elapsed time for adjustments next time.
			mPrevElapsedTime = elapsedTime;

			return msecTicks;
#else
			
#ifdef __CELLOS_LV2__
	__int64 freq=sys_time_get_timebase_frequency();
	 double dFreq=((double) freq) / 1000.0;
	typedef uint64_t __int64;
	typedef __int64  ClockSize;
	ClockSize newTime;
	__asm __volatile__( "mftb %0" : "=r" (newTime) : : "memory");
	
	return (newTime-mStartTime) / dFreq;
#else

			struct timeval currentTime;
			gettimeofday(&currentTime, 0);
			return (currentTime.tv_sec - mStartTime.tv_sec) * 1000 + 
				(currentTime.tv_usec - mStartTime.tv_usec) / 1000;
#endif //__CELLOS_LV2__
#endif
		}

		/// Returns the time in us since the last call to reset or since 
		/// the Clock was created.
		unsigned long int getTimeMicroseconds()
		{
#ifdef USE_WINDOWS_TIMERS
			LARGE_INTEGER currentTime;
			QueryPerformanceCounter(&currentTime);
			LONGLONG elapsedTime = currentTime.QuadPart - 
				mStartTime.QuadPart;

			// Compute the number of millisecond ticks elapsed.
			unsigned long msecTicks = (unsigned long)(1000 * elapsedTime / 
				mClockFrequency.QuadPart);

			// Check for unexpected leaps in the Win32 performance counter.  
			// (This is caused by unexpected data across the PCI to ISA 
			// bridge, aka south bridge.  See Microsoft KB274323.)
			unsigned long elapsedTicks = GetTickCount() - mStartTick;
			signed long msecOff = (signed long)(msecTicks - elapsedTicks);
			if (msecOff < -100 || msecOff > 100)
			{
				// Adjust the starting time forwards.
				LONGLONG msecAdjustment = mymin(msecOff * 
					mClockFrequency.QuadPart / 1000, elapsedTime - 
					mPrevElapsedTime);
				mStartTime.QuadPart += msecAdjustment;
				elapsedTime -= msecAdjustment;
			}

			// Store the current elapsed time for adjustments next time.
			mPrevElapsedTime = elapsedTime;

			// Convert to microseconds.
			unsigned long usecTicks = (unsigned long)(1000000 * elapsedTime / 
				mClockFrequency.QuadPart);

			return usecTicks;
#else

#ifdef __CELLOS_LV2__
	__int64 freq=sys_time_get_timebase_frequency();
	 double dFreq=((double) freq)/ 1000000.0;
	typedef uint64_t __int64;
	typedef __int64  ClockSize;
	ClockSize newTime;
	__asm __volatile__( "mftb %0" : "=r" (newTime) : : "memory");
	
	return (newTime-mStartTime) / dFreq;
#else

			struct timeval currentTime;
			gettimeofday(&currentTime, 0);
			return (currentTime.tv_sec - mStartTime.tv_sec) * 1000000 + 
				(currentTime.tv_usec - mStartTime.tv_usec);
#endif//__CELLOS_LV2__
#endif 
		}

	private:
#ifdef USE_WINDOWS_TIMERS
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

#endif //USE_BT_CLOCK


#ifdef USE_QUICKPROF


#include <iostream>
#include <fstream>
#include <string>
#include <map>




namespace hidden
{
	/// A simple data structure representing a single timed block 
	/// of code.
	struct ProfileBlock
	{
		ProfileBlock()
		{
			currentBlockStartMicroseconds = 0;
			currentCycleTotalMicroseconds = 0;
			lastCycleTotalMicroseconds = 0;
			totalMicroseconds = 0;
		}

		/// The starting time (in us) of the current block update.
		unsigned long int currentBlockStartMicroseconds;

		/// The accumulated time (in us) spent in this block during the 
		/// current profiling cycle.
		unsigned long int currentCycleTotalMicroseconds;

		/// The accumulated time (in us) spent in this block during the 
		/// past profiling cycle.
		unsigned long int lastCycleTotalMicroseconds;

		/// The total accumulated time (in us) spent in this block.
		unsigned long int totalMicroseconds;
	};

};

/// A static class that manages timing for a set of profiling blocks.
class btProfiler
{
public:
	/// A set of ways to retrieve block timing data.
	enum BlockTimingMethod
	{
		/// The total time spent in the block (in seconds) since the 
		/// profiler was initialized.
		BLOCK_TOTAL_SECONDS,

		/// The total time spent in the block (in ms) since the 
		/// profiler was initialized.
		BLOCK_TOTAL_MILLISECONDS,

		/// The total time spent in the block (in us) since the 
		/// profiler was initialized.
		BLOCK_TOTAL_MICROSECONDS,

		/// The total time spent in the block, as a % of the total 
		/// elapsed time since the profiler was initialized.
		BLOCK_TOTAL_PERCENT,

		/// The time spent in the block (in seconds) in the most recent 
		/// profiling cycle.
		BLOCK_CYCLE_SECONDS,

		/// The time spent in the block (in ms) in the most recent 
		/// profiling cycle.
		BLOCK_CYCLE_MILLISECONDS,

		/// The time spent in the block (in us) in the most recent 
		/// profiling cycle.
		BLOCK_CYCLE_MICROSECONDS,

		/// The time spent in the block (in seconds) in the most recent 
		/// profiling cycle, as a % of the total cycle time.
		BLOCK_CYCLE_PERCENT
	};

	/// Initializes the profiler.  This must be called first.  If this is 
	/// never called, the profiler is effectively disabled; all other 
	/// functions will return immediately.  The first parameter 
	/// is the name of an output data file; if this string is not empty, 
	/// data will be saved on every profiling cycle; if this string is 
	/// empty, no data will be saved to a file.  The second parameter 
	/// determines which timing method is used when printing data to the 
	/// output file.
	inline static void init(const std::string outputFilename="", 
		BlockTimingMethod outputMethod=BLOCK_CYCLE_MILLISECONDS);

	/// Cleans up allocated memory.
	inline static void destroy();

	/// Begins timing the named block of code.
	inline static void beginBlock(const std::string& name);

	/// Updates the accumulated time spent in the named block by adding 
	/// the elapsed time since the last call to startBlock for this block 
	/// name.
	inline static void endBlock(const std::string& name);

	/// Returns the time spent in the named block according to the 
	/// given timing method.  See comments on BlockTimingMethod for details.
	inline static double getBlockTime(const std::string& name, 
		BlockTimingMethod method=BLOCK_CYCLE_MILLISECONDS);

	/// Defines the end of a profiling cycle.  Use this regularly if you 
	/// want to generate detailed timing information.  This must not be 
	/// called within a timing block.
	inline static void endProfilingCycle();

	/// A helper function that creates a string of statistics for 
	/// each timing block.  This is mainly for printing an overall 
	/// summary to the command line.
	inline static std::string createStatsString(
		BlockTimingMethod method=BLOCK_TOTAL_PERCENT);

//private:
	inline btProfiler();

	inline ~btProfiler();

	/// Prints an error message to standard output.
	inline static void printError(const std::string& msg)
	{
		//btAssert(0);
		std::cout << "[QuickProf error] " << msg << std::endl;
	}

	/// Determines whether the profiler is enabled.
	static bool mEnabled;

	/// The clock used to time profile blocks.
	static btClock mClock;

	/// The starting time (in us) of the current profiling cycle.
	static unsigned long int mCurrentCycleStartMicroseconds;

	/// The duration (in us) of the most recent profiling cycle.
	static unsigned long int mLastCycleDurationMicroseconds;

	/// Internal map of named profile blocks.
	static std::map<std::string, hidden::ProfileBlock*> mProfileBlocks;

	/// The data file used if this feature is enabled in 'init.'
	static std::ofstream mOutputFile;

	/// Tracks whether we have begun print data to the output file.
	static bool mFirstFileOutput;

	/// The method used when printing timing data to an output file.
	static BlockTimingMethod mFileOutputMethod;

	/// The number of the current profiling cycle.
	static unsigned long int mCycleNumber;
};


btProfiler::btProfiler()
{
	// This never gets called because a btProfiler instance is never 
	// created.
}

btProfiler::~btProfiler()
{
	// This never gets called because a btProfiler instance is never 
	// created.
}

void btProfiler::init(const std::string outputFilename, 
	BlockTimingMethod outputMethod)
{
	mEnabled = true;
	
	if (!outputFilename.empty())
	{
		mOutputFile.open(outputFilename.c_str());
	}

	mFileOutputMethod = outputMethod;

	mClock.reset();

	// Set the start time for the first cycle.
	mCurrentCycleStartMicroseconds = mClock.getTimeMicroseconds();
}

void btProfiler::destroy()
{
	if (!mEnabled)
	{
		return;
	}

	if (mOutputFile.is_open())
	{
		mOutputFile.close();
	}

	// Destroy all ProfileBlocks.
	while (!mProfileBlocks.empty())
	{
		delete (*mProfileBlocks.begin()).second;
		mProfileBlocks.erase(mProfileBlocks.begin());
	}
}

void btProfiler::beginBlock(const std::string& name)
{
	if (!mEnabled)
	{
		return;
	}

	if (name.empty())
	{
		printError("Cannot allow unnamed profile blocks.");
		return;
	}

	hidden::ProfileBlock* block = mProfileBlocks[name];

	if (!block)
	{
		// Create a new ProfileBlock.
		mProfileBlocks[name] = new hidden::ProfileBlock();
		block = mProfileBlocks[name];
	}

	// We do this at the end to get more accurate results.
	block->currentBlockStartMicroseconds = mClock.getTimeMicroseconds();
}

void btProfiler::endBlock(const std::string& name)
{
	if (!mEnabled)
	{
		return;
	}

	// We do this at the beginning to get more accurate results.
	unsigned long int endTick = mClock.getTimeMicroseconds();

	hidden::ProfileBlock* block = mProfileBlocks[name];

	if (!block)
	{
		// The named block does not exist.  Print an error.
		printError("The profile block named '" + name + 
			"' does not exist.");
		return;
	}

	unsigned long int blockDuration = endTick - 
		block->currentBlockStartMicroseconds;
	block->currentCycleTotalMicroseconds += blockDuration;
	block->totalMicroseconds += blockDuration;
}

double btProfiler::getBlockTime(const std::string& name, 
	BlockTimingMethod method)
{
	if (!mEnabled)
	{
		return 0;
	}

	hidden::ProfileBlock* block = mProfileBlocks[name];

	if (!block)
	{
		// The named block does not exist.  Print an error.
		printError("The profile block named '" + name + 
			"' does not exist.");
		return 0;
	}

	double result = 0;

	switch(method)
	{
		case BLOCK_TOTAL_SECONDS:
			result = (double)block->totalMicroseconds * (double)0.000001;
			break;
		case BLOCK_TOTAL_MILLISECONDS:
			result = (double)block->totalMicroseconds * (double)0.001;
			break;
		case BLOCK_TOTAL_MICROSECONDS:
			result = (double)block->totalMicroseconds;
			break;
		case BLOCK_TOTAL_PERCENT:
		{
			double timeSinceInit = (double)mClock.getTimeMicroseconds();
			if (timeSinceInit <= 0)
			{
				result = 0;
			}
			else
			{
				result = 100.0 * (double)block->totalMicroseconds / 
					timeSinceInit;
			}
			break;
		}
		case BLOCK_CYCLE_SECONDS:
			result = (double)block->lastCycleTotalMicroseconds * 
				(double)0.000001;
			break;
		case BLOCK_CYCLE_MILLISECONDS:
			result = (double)block->lastCycleTotalMicroseconds * 
				(double)0.001;
			break;
		case BLOCK_CYCLE_MICROSECONDS:
			result = (double)block->lastCycleTotalMicroseconds;
			break;
		case BLOCK_CYCLE_PERCENT:
		{
			if (0 == mLastCycleDurationMicroseconds)
			{
				// We have not yet finished a cycle, so just return zero 
				// percent to avoid a divide by zero error.
				result = 0;
			}
			else
			{
				result = 100.0 * (double)block->lastCycleTotalMicroseconds / 
					mLastCycleDurationMicroseconds;
			}
			break;
		}
		default:
			break;
	}

	return result;
}

void btProfiler::endProfilingCycle()
{
	if (!mEnabled)
	{
		return;
	}

	// Store the duration of the cycle that just finished.
	mLastCycleDurationMicroseconds = mClock.getTimeMicroseconds() - 
		mCurrentCycleStartMicroseconds;

	// For each block, update data for the cycle that just finished.
	std::map<std::string, hidden::ProfileBlock*>::iterator iter;
	for (iter = mProfileBlocks.begin(); iter != mProfileBlocks.end(); ++iter)
	{
		hidden::ProfileBlock* block = (*iter).second;
		block->lastCycleTotalMicroseconds = 
			block->currentCycleTotalMicroseconds;
		block->currentCycleTotalMicroseconds = 0;
	}

	if (mOutputFile.is_open())
	{
		// Print data to the output file.
		if (mFirstFileOutput)
		{
			// On the first iteration, print a header line that shows the 
			// names of each profiling block.
			mOutputFile << "#cycle, ";

			for (iter = mProfileBlocks.begin(); iter != mProfileBlocks.end(); 
				++iter)
			{
				mOutputFile << (*iter).first << ", ";
			}

			mOutputFile << std::endl;
			mFirstFileOutput = false;
		}

		mOutputFile << mCycleNumber << ", ";

		for (iter = mProfileBlocks.begin(); iter != mProfileBlocks.end(); 
			++iter)
		{
			mOutputFile << getBlockTime((*iter).first, mFileOutputMethod) 
				<< ", ";
		}

		mOutputFile << std::endl;
	}

	++mCycleNumber;
	mCurrentCycleStartMicroseconds = mClock.getTimeMicroseconds();
}

std::string btProfiler::createStatsString(BlockTimingMethod method)
{
	if (!mEnabled)
	{
		return "";
	}

	std::string s;
	std::string suffix;

	switch(method)
	{
		case BLOCK_TOTAL_SECONDS:
			suffix = "s";
			break;
		case BLOCK_TOTAL_MILLISECONDS:
			suffix = "ms";
			break;
		case BLOCK_TOTAL_MICROSECONDS:
			suffix = "us";
			break;
		case BLOCK_TOTAL_PERCENT:
		{
			suffix = "%";
			break;
		}
		case BLOCK_CYCLE_SECONDS:
			suffix = "s";
			break;
		case BLOCK_CYCLE_MILLISECONDS:
			suffix = "ms";
			break;
		case BLOCK_CYCLE_MICROSECONDS:
			suffix = "us";
			break;
		case BLOCK_CYCLE_PERCENT:
		{
			suffix = "%";
			break;
		}
		default:
			break;
	}

	std::map<std::string, hidden::ProfileBlock*>::iterator iter;
	for (iter = mProfileBlocks.begin(); iter != mProfileBlocks.end(); ++iter)
	{
		if (iter != mProfileBlocks.begin())
		{
			s += "\n";
		}

		char blockTime[64];
		sprintf(blockTime, "%lf", getBlockTime((*iter).first, method));

		s += (*iter).first;
		s += ": ";
		s += blockTime;
		s += " ";
		s += suffix;
	}

	return s;
}


#define BEGIN_PROFILE(a) btProfiler::beginBlock(a)
#define END_PROFILE(a) btProfiler::endBlock(a)

#else //USE_QUICKPROF
#define BEGIN_PROFILE(a)
#define END_PROFILE(a)

#endif //USE_QUICKPROF


/***************************************************************************************************
**
** profile.h
**
** Real-Time Hierarchical Profiling for Game Programming Gems 3
**
** by Greg Hjelstrom & Byon Garrabrant
**
***************************************************************************************************/

/*
** A node in the Profile Hierarchy Tree
*/
class	CProfileNode {

public:
	CProfileNode( const char * name, CProfileNode * parent );
	~CProfileNode( void );

	CProfileNode * Get_Sub_Node( const char * name );

	CProfileNode * Get_Parent( void )		{ return Parent; }
	CProfileNode * Get_Sibling( void )		{ return Sibling; }
	CProfileNode * Get_Child( void )			{ return Child; }

	void				Reset( void );
	void				Call( void );
	bool				Return( void );

	const char *	Get_Name( void )				{ return Name; }
	int				Get_Total_Calls( void )		{ return TotalCalls; }
	float				Get_Total_Time( void )		{ return TotalTime; }

protected:

	const char *	Name;
	int				TotalCalls;
	float				TotalTime;
	unsigned long int			StartTime;
	int				RecursionCounter;

	CProfileNode *	Parent;
	CProfileNode *	Child;
	CProfileNode *	Sibling;
};

/*
** An iterator to navigate through the tree
*/
class CProfileIterator
{
public:
	// Access all the children of the current parent
	void				First(void);
	void				Next(void);
	bool				Is_Done(void);
    bool                Is_Root(void) { return (CurrentParent->Get_Parent() == 0); }

	void				Enter_Child( int index );		// Make the given child the new parent
	void				Enter_Largest_Child( void );	// Make the largest child the new parent
	void				Enter_Parent( void );			// Make the current parent's parent the new parent

	// Access the current child
	const char *	Get_Current_Name( void )			{ return CurrentChild->Get_Name(); }
	int				Get_Current_Total_Calls( void )	{ return CurrentChild->Get_Total_Calls(); }
	float				Get_Current_Total_Time( void )	{ return CurrentChild->Get_Total_Time(); }

	// Access the current parent
	const char *	Get_Current_Parent_Name( void )			{ return CurrentParent->Get_Name(); }
	int				Get_Current_Parent_Total_Calls( void )	{ return CurrentParent->Get_Total_Calls(); }
	float				Get_Current_Parent_Total_Time( void )	{ return CurrentParent->Get_Total_Time(); }

protected:

	CProfileNode *	CurrentParent;
	CProfileNode *	CurrentChild;

	CProfileIterator( CProfileNode * start );
	friend	class		CProfileManager;
};


/*
** The Manager for the Profile system
*/
class	CProfileManager {
public:
	static	void						Start_Profile( const char * name );
	static	void						Stop_Profile( void );

	static	void						Reset( void );
	static	void						Increment_Frame_Counter( void );
	static	int						Get_Frame_Count_Since_Reset( void )		{ return FrameCounter; }
	static	float						Get_Time_Since_Reset( void );

	static	CProfileIterator *	Get_Iterator( void )	{ return new CProfileIterator( &Root ); }
	static	void						Release_Iterator( CProfileIterator * iterator ) { delete iterator; }

private:
	static	CProfileNode			Root;
	static	CProfileNode *			CurrentNode;
	static	int						FrameCounter;
	static	unsigned long int					ResetTime;
};


/*
** ProfileSampleClass is a simple way to profile a function's scope
** Use the PROFILE macro at the start of scope to time
*/
class	CProfileSample {
public:
	CProfileSample( const char * name )
	{ 
		CProfileManager::Start_Profile( name ); 
	}
	
	~CProfileSample( void )					
	{ 
		CProfileManager::Stop_Profile(); 
	}
};

#if !defined(NO_PROFILE)
#define	PROFILE( name )			CProfileSample __profile( name )
#else
#define	PROFILE( name )
#endif




#endif //QUICK_PROF_H


