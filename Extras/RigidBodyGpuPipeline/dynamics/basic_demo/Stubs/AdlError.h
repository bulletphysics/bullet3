/*
Copyright (c) 2012 Advanced Micro Devices, Inc.  

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
//Originally written by Takahiro Harada


#ifndef CL_ERROR_H
#define CL_ERROR_H

#ifdef DX11RENDER
#include <windows.h>
#endif

#ifdef _DEBUG
	#include <assert.h>
	#define CLASSERT(x) if(!(x)){__debugbreak(); }
	#define ADLASSERT(x) if(!(x)){__debugbreak(); }
#else
	#define CLASSERT(x) if(x){}
	#define ADLASSERT(x) if(x){}

#endif




#ifdef _DEBUG
	#define COMPILE_TIME_ASSERT(x) {int compileTimeAssertFailed[x]; compileTimeAssertFailed[0];}
#else
	#define COMPILE_TIME_ASSERT(x)
#endif

#ifdef _DEBUG
	#include <stdarg.h>
	#include <stdio.h>
	__inline
	void debugPrintf(const char *fmt, ...)
	{
		va_list arg;
		va_start(arg, fmt);
#ifdef DX11RENDER
		char buf[256];
		vsprintf_s( buf, 256, fmt, arg );
#ifdef UNICODE
		WCHAR wbuf[256];
		int sizeWide = MultiByteToWideChar(0,0,buf,-1,wbuf,0);
		MultiByteToWideChar(0,0,buf,-1,wbuf,sizeWide);

//		swprintf_s( wbuf, 256, L"%s", buf );
		OutputDebugString( wbuf );
#else
		OutputDebugString( buf );
#endif
#else
		vprintf(fmt, arg);
#endif
		va_end(arg);
	}
#else
	__inline
	void debugPrintf(const char *fmt, ...)
	{
	}
#endif


#define WARN(msg) debugPrintf("WARNING: %s\n", msg);

#endif

