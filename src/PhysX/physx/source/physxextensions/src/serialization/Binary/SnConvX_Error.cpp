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

#include "foundation/PxErrorCallback.h"
#include "SnConvX.h"
#include <stdarg.h>
#include "PsString.h"
#include "PsFoundation.h"

#define MAX_DISPLAYED_ISSUES   10

using namespace physx;

void Sn::ConvX::resetNbErrors()
{
	mNbErrors = 0;
	mNbWarnings = 0;
}

int Sn::ConvX::getNbErrors() const
{
	return mNbErrors;
}

void Sn::ConvX::displayMessage(PxErrorCode::Enum code, const char* format, ...)
{
	if(silentMode())
		return;
		
	int sum = mNbWarnings + mNbErrors;
	if(sum >= MAX_DISPLAYED_ISSUES)
		return;

	bool display = false;

	if(code==PxErrorCode::eINTERNAL_ERROR || code==PxErrorCode::eINVALID_OPERATION || code==PxErrorCode::eINVALID_PARAMETER)
	{
		mNbErrors++;
		display = true;
	}
	else if(code == PxErrorCode::eDEBUG_WARNING)
	{
		mNbWarnings++;	
		display = true;
	}

	if(display || ((sum == 0) && verboseMode()) )
	{
		va_list va;
		va_start(va, format);
		Ps::getFoundation().errorImpl(code, __FILE__, __LINE__, format, va);
		va_end(va); 
	}

	if(display)
	{
		if( sum == 0)
		{
			Ps::getFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "Hit warnings or errors: skipping further verbose output.\n");		    
		}
		else if(sum == MAX_DISPLAYED_ISSUES-1)
		{
			Ps::getFoundation().error(PxErrorCode::eDEBUG_INFO, __FILE__, __LINE__, "Exceeding 10 warnings or errors: skipping further output.\n");		    
		}
	}

	return;
}
