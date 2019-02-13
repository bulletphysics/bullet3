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

#include "foundation/PxSimpleTypes.h"
#include "PsCpu.h"

#if PX_X86 && !defined(__EMSCRIPTEN__)
#define cpuid(op, reg)                                                                                                 \
	__asm__ __volatile__("pushl %%ebx      \n\t" /* save %ebx */                                                       \
	                     "cpuid            \n\t"                                                                       \
	                     "movl %%ebx, %1   \n\t" /* save what cpuid just put in %ebx */                                \
	                     "popl %%ebx       \n\t" /* restore the old %ebx */                                            \
	                     : "=a"(reg[0]), "=r"(reg[1]), "=c"(reg[2]), "=d"(reg[3])                                      \
	                     : "a"(op)                                                                                     \
	                     : "cc")
#else
#define cpuid(op, reg) reg[0] = reg[1] = reg[2] = reg[3] = 0;
#endif

namespace physx
{
namespace shdfnd
{

uint8_t Cpu::getCpuId()
{
	uint32_t cpuInfo[4];
	cpuid(1, cpuInfo);
	return static_cast<uint8_t>(cpuInfo[1] >> 24); // APIC Physical ID
}
}
}
