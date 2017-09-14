
// C RunTime Header Files
#ifndef GWEN_MACROS_H
#define GWEN_MACROS_H
#include <stdlib.h>
#include <stdarg.h>
#if !defined(__APPLE__) && !defined(__OpenBSD__) && !defined(__FreeBSD__)
	#include <malloc.h>
#else
	#include <stdlib.h>
#endif //__APPLE__
#include <memory.h>
#include <algorithm>

// Not tied to platform api.
#define GwenUtil_Min( a, b ) ( ( (a) < (b) ) ? (a) : (b) )
#define GwenUtil_Max( a, b ) ( ( (a) > (b) ) ? (a) : (b) )
#define GwenUtil_VSWPrintFSafeSized( _DstBuf_ARRAY_, _Format, _ArgList ) GwenUtil_VSWPrintFSafe( _DstBuf_ARRAY_, sizeof( _DstBuf_ARRAY_ ) / sizeof( wchar_t ), _Format, _ArgList )

#ifdef _WIN32

	#ifndef NOMINMAX
        #define NOMINMAX
        #endif
	#include <windows.h>

	#define GwenUtil_VSNPrintFSafe( _DstBuf, _DstSize, _MaxCount, _Format, _ArgList ) vsnprintf_s( _DstBuf, _DstSize, _MaxCount, _Format, _ArgList )
	#define GwenUtil_VSWPrintFSafe( _DstBuf, _SizeInWords, _Format, _ArgList ) vswprintf_s( _DstBuf, _SizeInWords, _Format, _ArgList )
	#define GwenUtil_OutputDebugCharString( lpOutputString ) OutputDebugStringA( lpOutputString )
	#define GwenUtil_OutputDebugWideString( lpOutputString ) OutputDebugStringW( lpOutputString )
	#define GwenUtil_WideStringToFloat( _Str ) _wtof( _Str )

#elif defined(__APPLE__)

	#include <CoreFoundation/CoreFoundation.h>

	#define GwenUtil_VSNPrintFSafe( _DstBuf, _DstSize, _MaxCount, _Format, _ArgList ) vsnprintf( _DstBuf, _DstSize, _Format, _ArgList )
	#define GwenUtil_VSWPrintFSafe( _DstBuf, _SizeInWords, _Format, _ArgList ) vswprintf( _DstBuf, _SizeInWords, _Format, _ArgList )
	#define GwenUtil_OutputDebugCharString( lpOutputString ) //printf( lpOutputString )
	#define GwenUtil_OutputDebugWideString( lpOutputString ) //wprintf( lpOutputString  )
	#define GwenUtil_WideStringToFloat( _Str ) wcstof(_Str, NULL)

#elif defined(__linux__) || defined(__OpenBSD__)

	#define GwenUtil_VSNPrintFSafe( _DstBuf, _DstSize, _MaxCount, _Format, _ArgList ) vsnprintf( _DstBuf, _DstSize, _Format, _ArgList )
	#define GwenUtil_VSWPrintFSafe( _DstBuf, _SizeInWords, _Format, _ArgList ) vswprintf( _DstBuf, _SizeInWords, _Format, _ArgList )
	#define GwenUtil_OutputDebugCharString( lpOutputString ) //printf( lpOutputString )
	#define GwenUtil_OutputDebugWideString( lpOutputString ) //wprintf( lpOutputString  )
	#define GwenUtil_WideStringToFloat( _Str ) wcstof(_Str, NULL)

#else

	#error MUST_IMPLEMENT_PLATFORM

#endif


namespace Gwen
{
	template <typename T1, typename T2, typename T3 >
	T1 Clamp( T1 current, T2 vmin, T3 vmax )
	{
		if ( current > vmax ) return (T1)vmax;
		if ( current < vmin ) return (T1)vmin;
		return current;
	}

	template <typename T, typename T2>
	inline T Approach( T fCurrent, T fTarget, T2 fDelta )
	{
		if ( fCurrent < fTarget )
		{
			fCurrent += fDelta;
			if ( fCurrent > fTarget ) return fTarget;
		}
		else if ( fCurrent > fTarget )
		{
			fCurrent -= fDelta;
			if ( fCurrent < fTarget ) return fTarget;
		}

		return fCurrent;
	}
}

#endif
