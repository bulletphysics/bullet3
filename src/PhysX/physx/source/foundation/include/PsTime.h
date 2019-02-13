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

#ifndef PSFOUNDATION_PSTIME_H
#define PSFOUNDATION_PSTIME_H

#include "Ps.h"

#if PX_LINUX || PX_ANDROID
#include <time.h>
#endif

namespace physx
{
namespace shdfnd
{

struct CounterFrequencyToTensOfNanos
{
	uint64_t mNumerator;
	uint64_t mDenominator;
	CounterFrequencyToTensOfNanos(uint64_t inNum, uint64_t inDenom) : mNumerator(inNum), mDenominator(inDenom)
	{
	}

	// quite slow.
	uint64_t toTensOfNanos(uint64_t inCounter) const
	{
		return (inCounter * mNumerator) / mDenominator;
	}
};

class PX_FOUNDATION_API Time
{
  public:
	typedef double Second;
	static const uint64_t sNumTensOfNanoSecondsInASecond = 100000000;
	// This is supposedly guaranteed to not change after system boot
	// regardless of processors, speedstep, etc.
	static const CounterFrequencyToTensOfNanos& getBootCounterFrequency();

	static CounterFrequencyToTensOfNanos getCounterFrequency();

	static uint64_t getCurrentCounterValue();

	// SLOW!!
	// Thar be a 64 bit divide in thar!
	static uint64_t getCurrentTimeInTensOfNanoSeconds()
	{
		uint64_t ticks = getCurrentCounterValue();
		return getBootCounterFrequency().toTensOfNanos(ticks);
	}

	Time();
	Second getElapsedSeconds();
	Second peekElapsedSeconds();
	Second getLastTime() const;

  private:
#if PX_LINUX || PX_ANDROID || PX_APPLE_FAMILY || PX_PS4
	Second mLastTime;
#else
	int64_t mTickCount;
#endif
};
} // namespace shdfnd
} // namespace physx

#endif // #ifndef PSFOUNDATION_PSTIME_H
