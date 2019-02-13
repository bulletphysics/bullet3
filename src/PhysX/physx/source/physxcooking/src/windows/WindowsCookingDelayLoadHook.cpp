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


#include "windows/PxWindowsDelayLoadHook.h"
#include "windows/PsWindowsInclude.h"
#include "windows/CmWindowsLoadLibrary.h"

// Prior to Visual Studio 2015 Update 3, these hooks were non-const.
#define DELAYIMP_INSECURE_WRITABLE_HOOKS
#include <delayimp.h>

static const physx::PxDelayLoadHook* gDelayLoadHook = NULL;

void physx::PxSetPhysXCookingDelayLoadHook(const physx::PxDelayLoadHook* hook)
{
	gDelayLoadHook = hook;
}

using namespace physx;

#pragma comment(lib, "delayimp")

FARPROC WINAPI delayHook(unsigned dliNotify, PDelayLoadInfo pdli)
{
	switch (dliNotify) {
	case dliStartProcessing :
		break;

	case dliNotePreLoadLibrary :
		{
			return Cm::physXCommonDliNotePreLoadLibrary(pdli->szDll,gDelayLoadHook);
		}
		break;

	case dliNotePreGetProcAddress :
		break;

	case dliFailLoadLib :
		break;

	case dliFailGetProc :
		break;

	case dliNoteEndProcessing :
		break;

	default :

		return NULL;
	}

	return NULL;
}

PfnDliHook __pfnDliNotifyHook2 = delayHook;
