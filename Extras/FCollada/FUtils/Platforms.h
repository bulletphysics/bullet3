/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/
/*
	Based on the FS Import classes:
	Copyright (C) 2005-2006 Feeling Software Inc
	Copyright (C) 2005-2006 Autodesk Media Entertainment
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#ifndef _PLATFORMS_H_
#define _PLATFORMS_H_

#ifdef FCOLLADA_DLL
// Disable the "private member not available for export" warning,
// because I don't feel like writing interfaces
#pragma warning(disable:4251) 
#ifdef FCOLLADA_INTERNAL
#define FCOLLADA_EXPORT __declspec(dllexport)
#else
#define FCOLLADA_EXPORT __declspec(dllimport)
#endif
#else
#define FCOLLADA_EXPORT
#endif

// Ensure that both UNICODE and _UNICODE are set.
#ifdef UNICODE
#ifndef _UNICODE
#define _UNICODE
#endif
#else
#ifdef _UNICODE
#define UNICODE
#endif
#endif

#include <math.h>

#ifdef WIN32
#pragma warning(disable:4702)
#include <windows.h>
#else
#ifdef MAC_TIGER
#include <ctype.h>
#include <wctype.h>
#else // MAC_TIGER
#if defined(LINUX)
#else // LINUX
#error "Unsupported platform."
#endif // LINUX 
#endif // MAC_TIGER

#endif // WIN32

// Cross-platform type definitions
#ifdef WIN32

#define int8 char
typedef short int16;
typedef long int32;
typedef __int64 int64;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned long uint32;
typedef unsigned __int64 uint64;

#else // For LINUX and MAC_TIGER

#define int8 char
typedef short int16;
typedef long int32;
typedef long long int64;
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned long uint32;
typedef unsigned long long uint64;
#endif

// Important functions that some OSes have missing!
#ifdef MAC_TIGER
inline char* strlower(char* str) { char* it = str; while (*it != 0) { *it = tolower(*it); ++it; } return str; }
inline wchar_t* wcslwr(wchar_t* str) { wchar_t* it = str; while (*it != 0) { *it = towlower(*it); ++it; } return str; }
#ifndef isinf
#define isinf __isinff
#endif
#define stricmp strcasecmp

#endif // MAC_TIGER

// Cross-platform needed functions
#ifdef WIN32

#define vsnprintf _vsnprintf
#define snprintf _snprintf
#define vsnwprintf _vsnwprintf
#define snwprintf _snwprintf
#define strlower _strlwr

#else // WIN32

#define vsnwprintf vswprintf
#define snwprintf swprintf

#endif // WIN32

// For Doxygen purposes, we stopped using the "using namespace std;" statement and use shortcuts instead.

/** A STL string. */
typedef std::string string;

/** A STL map. */
template <typename _Kty, typename _Ty>
class map : public std::map<_Kty, _Ty> {};

// fstring and character definition
#ifdef UNICODE
#ifdef WIN32

#include <tchar.h>
	typedef TCHAR fchar;
	typedef std::basic_string<fchar> fstring;
	#define FC(a) __T(a)

	#define fstrlen _tcslen
	#define fstrcmp _tcscmp
	#define fstricmp _tcsicmp
	#define fstrncpy _tcsncpy
	#define fstrrchr _tcsrchr
	#define fstrlower _tcslwr
	#define fsnprintf _sntprintf
	#define fvsnprintf _vsntprintf

	#define fchdir _tchdir

#else // For MacOSX and Linux platforms

	#define fchar wchar_t
	typedef std::wstring fstring;
	#define FC(a) L ## a

	#define fstrlen wcslen
	#define fstrcmp wcscmp
	#define fstricmp wcsicmp
	#define fstrncpy wcsncpy
	#define fstrrchr wcsrchr
	#define fstrlower wcslwr
	#define fsnprintf swprintf
	#define fvsnprintf vswprintf

	#define fchdir(a) chdir(FUStringConversion::ToString(a).c_str())

#endif // WIN32

#else // UNICODE

typedef char fchar;
typedef std::basic_string<fchar> fstring;
#define FC(a) a

#define fstrlen strlen
#define fstrcmp strcmp
#define fstricmp stricmp
#define fstrncpy strncpy
#define fstrrchr strrchr
#define fstrlower strlower
#define fsnprintf snprintf
#define fvsnprintf vsnprintf

#define fatol atol
#define fatof atof
#define fchdir chdir

#endif // UNICODE

#ifndef WIN32
#define MAX_PATH 1024
#endif // !WIN32

#endif // _PLATFORMS_H_
