/*
CDTestFramework http://codercorner.com
Copyright (c) 2007-2008 Pierre Terdiman,  pierre@codercorner.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#ifndef PROFILING_H
#define PROFILING_H

	__forceinline void	StartProfile(int& val)
	{
#ifdef WIN32
		__asm{
			cpuid
			rdtsc
			mov		ebx, val
			mov		[ebx], eax
		}
#endif
	}

	__forceinline void	EndProfile(int& val)
	{
#ifdef WIN32
		__asm{
			cpuid
			rdtsc
			mov		ebx, val
			sub		eax, [ebx]
			mov		[ebx], eax
		}
#endif
	}

	class Profiler
	{
		public:
								Profiler() : mCycles(0), mTime(0.0f), mNbQueries(0), mMsTime(0.0f)
								{
									QueryPerformanceFrequency((LARGE_INTEGER*)&mFreq);
								}

		inline_	void			Start()
								{
									QueryPerformanceCounter((LARGE_INTEGER*)&mCounter0);
									//StartProfile(mCycles);
								}

		inline_	void			End()
								{
									//EndProfile(mCycles);
									QueryPerformanceCounter((LARGE_INTEGER*)&mCounter1);
								}

					void	Reset()
								{
									mCycles = 0;
									mTime = 0.0f;
									mNbQueries = 0;
									mMsTime=0.0f;
								}

				void			Accum()
								{
									double t = ((double)mCounter1 - (double)mCounter0)/(double)mFreq;
									float mms = (float)t*1000000;

									mTime += mms;
									mNbQueries++;
									if(mNbQueries==100)
									{
										mNbQueries=1;
										mTime = mms;
									}

									mMsTime = mTime/float(mNbQueries);
								}

				void			AddToTweakBar(TwBar* tbar)
								{
									TwAddVarRO(tbar, "Microseconds", TW_TYPE_FLOAT, &mMsTime, " group='Profiling' ");
								}

				__int64			mFreq;
				__int64			mCounter0;
				__int64			mCounter1;
				int				mCycles;
				float			mTime;
				udword			mNbQueries;
				float			mMsTime;
	};

#endif	// PROFILING_H

