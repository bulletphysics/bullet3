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

#include "foundation/PxAssert.h"

#include <stdio.h>
#include "PsString.h"

#if PX_WINDOWS_FAMILY
#include <crtdbg.h>
#elif PX_SWITCH
#include "switch/PsSwitchAbort.h"
#endif

namespace
{
class DefaultAssertHandler : public physx::PxAssertHandler
{
	virtual void operator()(const char* expr, const char* file, int line, bool& ignore)
	{
		PX_UNUSED(ignore); // is used only in debug windows config
		char buffer[1024];
#if PX_WINDOWS_FAMILY
		sprintf_s(buffer, "%s(%d) : Assertion failed: %s\n", file, line, expr);
#else
		sprintf(buffer, "%s(%d) : Assertion failed: %s\n", file, line, expr);
#endif
		physx::shdfnd::printString(buffer);
#if PX_WINDOWS_FAMILY&& PX_DEBUG && PX_DEBUG_CRT
		// _CrtDbgReport returns -1 on error, 1 on 'retry', 0 otherwise including 'ignore'.
		// Hitting 'abort' will terminate the process immediately.
		int result = _CrtDbgReport(_CRT_ASSERT, file, line, NULL, "%s", buffer);
		int mode = _CrtSetReportMode(_CRT_ASSERT, _CRTDBG_REPORT_MODE);
		ignore = _CRTDBG_MODE_WNDW == mode && result == 0;
		if(ignore)
			return;
		__debugbreak();
#elif PX_WINDOWS_FAMILY&& PX_CHECKED
		__debugbreak();
#elif PX_SWITCH
		abort(buffer);
#else
		abort();
#endif
	}
};

DefaultAssertHandler sAssertHandler;
physx::PxAssertHandler* sAssertHandlerPtr = &sAssertHandler;
}

namespace physx
{

PxAssertHandler& PxGetAssertHandler()
{
	return *sAssertHandlerPtr;
}

void PxSetAssertHandler(PxAssertHandler& handler)
{
	sAssertHandlerPtr = &handler;
}
} // end of physx namespace
