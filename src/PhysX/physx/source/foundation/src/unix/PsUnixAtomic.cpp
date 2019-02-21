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
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
#include "Ps.h"
#include "PsAtomic.h"

#if ! PX_EMSCRIPTEN
#define PAUSE() asm("nop")
#else
#define PAUSE()
#endif

namespace physx
{
namespace shdfnd
{

void* atomicCompareExchangePointer(volatile void** dest, void* exch, void* comp)
{
	return __sync_val_compare_and_swap(const_cast<void**>(dest), comp, exch);
}

int32_t atomicCompareExchange(volatile int32_t* dest, int32_t exch, int32_t comp)
{
	return __sync_val_compare_and_swap(dest, comp, exch);
}

int32_t atomicIncrement(volatile int32_t* val)
{
	return __sync_add_and_fetch(val, 1);
}

int32_t atomicDecrement(volatile int32_t* val)
{
	return __sync_sub_and_fetch(val, 1);
}

int32_t atomicAdd(volatile int32_t* val, int32_t delta)
{
	return __sync_add_and_fetch(val, delta);
}

int32_t atomicMax(volatile int32_t* val, int32_t val2)
{
	int32_t oldVal, newVal;

	do
	{
		PAUSE();
		oldVal = *val;

		if(val2 > oldVal)
			newVal = val2;
		else
			newVal = oldVal;

	} while(atomicCompareExchange(val, newVal, oldVal) != oldVal);

	return *val;
}

int32_t atomicExchange(volatile int32_t* val, int32_t val2)
{
	int32_t newVal, oldVal;

	do
	{
		PAUSE();
		oldVal = *val;
		newVal = val2;
	} while(atomicCompareExchange(val, newVal, oldVal) != oldVal);

	return oldVal;
}

} // namespace shdfnd
} // namespace physx
