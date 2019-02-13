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

#include "PsCpu.h"
#pragma warning(push)
//'symbol' is not defined as a preprocessor macro, replacing with '0' for 'directives'
#pragma warning(disable : 4668)
#if PX_VC == 10
#pragma warning(disable : 4987) // nonstandard extension used: 'throw (...)'
#endif
#include <intrin.h>
#pragma warning(pop)

namespace physx
{
namespace shdfnd
{

#if PX_ARM
#define cpuid(reg) reg[0] = reg[1] = reg[2] = reg[3] = 0;

uint8_t Cpu::getCpuId()
{
	uint32_t cpuInfo[4];
	cpuid(cpuInfo);
	return static_cast<uint8_t>(cpuInfo[1] >> 24); // APIC Physical ID
}
#else
uint8_t Cpu::getCpuId()
{
	int CPUInfo[4];
	int InfoType = 1;
	__cpuid(CPUInfo, InfoType);
	return static_cast<uint8_t>(CPUInfo[1] >> 24); // APIC Physical ID
}
#endif
}
}
