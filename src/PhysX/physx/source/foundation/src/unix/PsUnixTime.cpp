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
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#include "Ps.h"
#include "PsTime.h"

#include <time.h>
#include <sys/time.h>

#if PX_APPLE_FAMILY
#include <mach/mach_time.h>
#endif

// Use real-time high-precision timer.
#if !PX_APPLE_FAMILY
#define CLOCKID CLOCK_REALTIME
#endif

namespace physx
{
namespace shdfnd
{

static const CounterFrequencyToTensOfNanos gCounterFreq = Time::getCounterFrequency();

const CounterFrequencyToTensOfNanos& Time::getBootCounterFrequency()
{
	return gCounterFreq;
}

static Time::Second getTimeSeconds()
{
	static struct timeval _tv;
	gettimeofday(&_tv, NULL);
	return double(_tv.tv_sec) + double(_tv.tv_usec) * 0.000001;
}

Time::Time()
{
	mLastTime = getTimeSeconds();
}

Time::Second Time::getElapsedSeconds()
{
	Time::Second curTime = getTimeSeconds();
	Time::Second diff = curTime - mLastTime;
	mLastTime = curTime;
	return diff;
}

Time::Second Time::peekElapsedSeconds()
{
	Time::Second curTime = getTimeSeconds();
	Time::Second diff = curTime - mLastTime;
	return diff;
}

Time::Second Time::getLastTime() const
{
	return mLastTime;
}

#if PX_APPLE_FAMILY
CounterFrequencyToTensOfNanos Time::getCounterFrequency()
{
	mach_timebase_info_data_t info;
	mach_timebase_info(&info);
	// mach_absolute_time * (info.numer/info.denom) is in units of nano seconds
	return CounterFrequencyToTensOfNanos(info.numer, info.denom * 10);
}

uint64_t Time::getCurrentCounterValue()
{
	return mach_absolute_time();
}

#else

CounterFrequencyToTensOfNanos Time::getCounterFrequency()
{
	return CounterFrequencyToTensOfNanos(1, 10);
}

uint64_t Time::getCurrentCounterValue()
{
	struct timespec mCurrTimeInt;
	clock_gettime(CLOCKID, &mCurrTimeInt);
	// Convert to nanos as this doesn't cause a large divide here
	return (static_cast<uint64_t>(mCurrTimeInt.tv_sec) * 1000000000) + (static_cast<uint64_t>(mCurrTimeInt.tv_nsec));
}
#endif

} // namespace shdfnd
} // namespace physx
