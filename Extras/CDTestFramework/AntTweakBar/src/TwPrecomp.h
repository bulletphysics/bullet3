//	---------------------------------------------------------------------------
//
//	@file		TwPrecomp.h
//	@brief		Precompiled header
//	@author		Philippe Decaudin - http://www.antisphere.com
//  @license    This file is part of the AntTweakBar library.
//				Copyright © 2005, 2006 Philippe Decaudin.
//              For conditions of distribution and use, see License.txt
//
//	notes:		Private header
//				TAB=4
//
//	---------------------------------------------------------------------------


#if !defined ANT_TW_PRECOMP_INCLUDED
#define ANT_TW_PRECOMP_INCLUDED


#if defined _MSC_VER
#	pragma warning(disable: 4514) 	// unreferenced inline function has been removed
#	pragma warning(disable: 4710) 	// function not inlined
#	pragma warning(disable: 4786) 	// template name truncated
#	pragma warning(disable: 4530) 	// exceptions not handled
#	define _CRT_SECURE_NO_DEPRECATE	// visual 8 secure crt warning
#endif

#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <float.h>

#if defined(_MSC_VER) && _MSC_VER<=1200
#	pragma warning(push, 3)
#endif
#include <string>
#include <vector>
#include <map>
#include <list>
#include <set>
#if defined(_MSC_VER) && _MSC_VER<=1200
#	pragma warning(pop)
#endif
#if defined(_UNIX)
#	define ANT_UNIX
#	include <X11/cursorfont.h>
#	include <GL/glx.h>
#	undef _WIN32
#	undef WIN32
#	undef _WIN64
#	undef WIN64
#	undef _WINDOWS
#	undef ANT_WINDOWS
#elif defined(_WINDOWS) || defined(WIN32) || defined(WIN64) || defined(_WIN32) || defined(_WIN64)
#	define ANT_WINDOWS
#	define WIN32_LEAN_AND_MEAN		// Exclude rarely-used stuff from Windows headers
#	include <windows.h>
#	include <shellapi.h>
#endif	// defined _WINDOWS

#include <GL/gl.h>	// must be included after windows.h
#define  ANT_OGL_HEADER_INCLUDED


#endif	// !defined ANT_TW_PRECOMP_INCLUDED
