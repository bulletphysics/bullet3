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


#ifndef CM_WINDOWS_LOADLIBRARY_H
#define CM_WINDOWS_LOADLIBRARY_H

#include "foundation/PxPreprocessor.h"
#include "windows/PxWindowsDelayLoadHook.h"
#include "windows/PsWindowsInclude.h"

#ifdef PX_SECURE_LOAD_LIBRARY
#include "nvSecureLoadLibrary.h"
#endif


namespace physx
{
namespace Cm
{
	EXTERN_C IMAGE_DOS_HEADER __ImageBase;

	PX_INLINE HMODULE WINAPI loadLibrary(const char* name)
	{
#ifdef PX_SECURE_LOAD_LIBRARY
		HMODULE retVal = nvLoadSignedLibrary(name,true);
		if(!retVal)
		{
			exit(1);
		}
		return retVal;
#else
		return ::LoadLibraryA( name );
#endif		
	};

	PX_INLINE FARPROC WINAPI physXCommonDliNotePreLoadLibrary(const char* libraryName, const physx::PxDelayLoadHook* delayLoadHook)
	{	
		if(!delayLoadHook)
		{
			return (FARPROC)loadLibrary(libraryName);
		}
		else
		{
			if(strstr(libraryName, "PhysXFoundation"))
			{
				return (FARPROC)Cm::loadLibrary(delayLoadHook->getPhysXFoundationDllName());
			}

			if(strstr(libraryName, "PhysXCommon"))
			{
				return (FARPROC)Cm::loadLibrary(delayLoadHook->getPhysXCommonDllName());
			}
		}
		return NULL;
    }
} // namespace Cm
} // namespace physx


#endif	// CM_WINDOWS_LOADLIBRARY_H
