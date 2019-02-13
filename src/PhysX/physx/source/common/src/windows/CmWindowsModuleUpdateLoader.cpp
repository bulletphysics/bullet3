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

#include "PsFoundation.h"

#ifdef SUPPORT_UPDATE_LOADER_LOGGING
#if PX_X86
#define NX_USE_SDK_DLLS
#include "PhysXUpdateLoader.h"
#endif
#endif /* SUPPORT_UPDATE_LOADER_LOGGING */

#include "windows/CmWindowsModuleUpdateLoader.h"
#include "windows/CmWindowsLoadLibrary.h"


namespace physx { namespace Cm {

#if PX_VC
#pragma warning(disable: 4191)	//'operator/operation' : unsafe conversion from 'type of expression' to 'type required'
#endif


typedef HMODULE (*GetUpdatedModule_FUNC)(const char*, const char*);

#ifdef SUPPORT_UPDATE_LOADER_LOGGING
#if PX_X86
typedef void (*setLogging_FUNC)(PXUL_ErrorCode, pt2LogFunc);

static void LogMessage(PXUL_ErrorCode messageType, char* message)
{
	switch(messageType)
	{
	case PXUL_ERROR_MESSAGES:
		getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, 
			"PhysX Update Loader Error: %s.", message);
		break;
	case PXUL_WARNING_MESSAGES:
		getFoundation().error(PX_WARN, "PhysX Update Loader Warning: %s.", message);
		break;
	case PXUL_INFO_MESSAGES:
		getFoundation().error(PX_INFO, "PhysX Update Loader Information: %s.", message);
		break;
	default:
		getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__,
			"Unknown message type from update loader.");
		break;
	}
}
#endif
#endif /* SUPPORT_UPDATE_LOADER_LOGGING */

CmModuleUpdateLoader::CmModuleUpdateLoader(const char* updateLoaderDllName)
	: mGetUpdatedModuleFunc(NULL)
{
	mUpdateLoaderDllHandle = loadLibrary(updateLoaderDllName);

	if (mUpdateLoaderDllHandle != NULL)
	{
		mGetUpdatedModuleFunc = GetProcAddress(mUpdateLoaderDllHandle, "GetUpdatedModule");

#ifdef SUPPORT_UPDATE_LOADER_LOGGING
#if PX_X86
		setLogging_FUNC setLoggingFunc;
		setLoggingFunc = (setLogging_FUNC)GetProcAddress(mUpdateLoaderDllHandle, "setLoggingFunction");
		if(setLoggingFunc != NULL)		
		{
           setLoggingFunc(PXUL_ERROR_MESSAGES, LogMessage);
        }
#endif
#endif /* SUPPORT_UPDATE_LOADER_LOGGING */
	}
}

CmModuleUpdateLoader::~CmModuleUpdateLoader()
{
	if (mUpdateLoaderDllHandle != NULL)
	{
		FreeLibrary(mUpdateLoaderDllHandle);
		mUpdateLoaderDllHandle = NULL;
	}
}

HMODULE CmModuleUpdateLoader::LoadModule(const char* moduleName, const char* appGUID)
{
	HMODULE result = NULL;

	if (mGetUpdatedModuleFunc != NULL)
	{
		// Try to get the module through PhysXUpdateLoader
		GetUpdatedModule_FUNC getUpdatedModuleFunc = (GetUpdatedModule_FUNC)mGetUpdatedModuleFunc;
		result = getUpdatedModuleFunc(moduleName, appGUID);
	}
	else
	{
		// If no PhysXUpdateLoader, just load the DLL directly
		result = loadLibrary(moduleName);
	}

	return result;
}

}; // end of namespace
}; // end of namespace
