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

#include "windows/PsWindowsInclude.h"
#include "PsAtomic.h"

namespace physx
{
namespace shdfnd
{

int32_t atomicExchange(volatile int32_t* val, int32_t val2)
{
	return (int32_t)InterlockedExchange((volatile LONG*)val, (LONG)val2);
}

int32_t atomicCompareExchange(volatile int32_t* dest, int32_t exch, int32_t comp)
{
	return (int32_t)InterlockedCompareExchange((volatile LONG*)dest, exch, comp);
}

void* atomicCompareExchangePointer(volatile void** dest, void* exch, void* comp)
{
	return InterlockedCompareExchangePointer((volatile PVOID*)dest, exch, comp);
}

int32_t atomicIncrement(volatile int32_t* val)
{
	return (int32_t)InterlockedIncrement((volatile LONG*)val);
}

int32_t atomicDecrement(volatile int32_t* val)
{
	return (int32_t)InterlockedDecrement((volatile LONG*)val);
}

int32_t atomicAdd(volatile int32_t* val, int32_t delta)
{
	LONG newValue, oldValue;
	do
	{
		oldValue = *val;
		newValue = oldValue + delta;
	} while(InterlockedCompareExchange((volatile LONG*)val, newValue, oldValue) != oldValue);

	return newValue;
}

int32_t atomicMax(volatile int32_t* val, int32_t val2)
{
	// Could do this more efficiently in asm...

	LONG newValue, oldValue;

	do
	{
		oldValue = *val;

		if(val2 > oldValue)
			newValue = val2;
		else
			newValue = oldValue;

	} while(InterlockedCompareExchange((volatile LONG*)val, newValue, oldValue) != oldValue);

	return newValue;
}

} // namespace shdfnd
} // namespace physx
