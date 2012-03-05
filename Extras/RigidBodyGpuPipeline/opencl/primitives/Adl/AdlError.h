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


#ifndef ADL_ERROR_H
#define ADL_ERROR_H

#if defined(ADL_DUMP_DX11_ERROR)
	#include <windows.h>
#endif
#ifdef _DEBUG
	#include <assert.h>
	#include <stdarg.h>
	#include <stdio.h>
#endif


namespace adl
{

#ifdef _DEBUG
	#define ADLASSERT(x) if(!(x)){__debugbreak(); }
#else
	#define ADLASSERT(x) if(x){}
#endif

#ifdef _DEBUG
	#define COMPILE_TIME_ASSERT(x) {int compileTimeAssertFailed[x]; compileTimeAssertFailed[0];}
#else
	#define COMPILE_TIME_ASSERT(x)
#endif

#ifdef _DEBUG
	__inline
	void debugPrintf(const char *fmt, ...)
	{
		va_list arg;
		va_start(arg, fmt);
#if defined(ADL_DUMP_DX11_ERROR)
		const int size = 1024*10;
		char buf[size];
		vsprintf_s( buf, size, fmt, arg );
#ifdef UNICODE
		WCHAR wbuf[size];
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

};

#endif

