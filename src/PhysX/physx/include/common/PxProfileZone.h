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

#ifndef PXFOUNDATION_PXPROFILEZONE_H
#define PXFOUNDATION_PXPROFILEZONE_H

#include "foundation/PxProfiler.h"
#include "PxFoundation.h"

#if PX_DEBUG || PX_CHECKED || PX_PROFILE
	#define PX_PROFILE_ZONE(x, y)										\
		physx::PxProfileScoped PX_CONCAT(_scoped, __LINE__)(PxGetProfilerCallback(), x, false, y)
	#define PX_PROFILE_START_CROSSTHREAD(x, y)							\
		if(PxGetProfilerCallback())										\
			PxGetProfilerCallback()->zoneStart(x, true, y)
	#define PX_PROFILE_STOP_CROSSTHREAD(x, y)							\
		if(PxGetProfilerCallback())										\
			PxGetProfilerCallback()->zoneEnd(NULL, x, true, y)
#else
	#define PX_PROFILE_ZONE(x, y)
	#define PX_PROFILE_START_CROSSTHREAD(x, y)
	#define PX_PROFILE_STOP_CROSSTHREAD(x, y)
#endif

#define PX_PROFILE_POINTER_TO_U64(pointer) static_cast<uint64_t>(reinterpret_cast<size_t>(pointer))

#endif // PXFOUNDATION_PXPROFILEZONE_H
