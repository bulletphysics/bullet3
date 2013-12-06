/*
* Copyright 1993-2006 NVIDIA Corporation.  All rights reserved.
*
* NOTICE TO USER:   
*
* This source code is subject to NVIDIA ownership rights under U.S. and 
* international Copyright laws.  
*
* NVIDIA MAKES NO REPRESENTATION ABOUT THE SUITABILITY OF THIS SOURCE 
* CODE FOR ANY PURPOSE.  IT IS PROVIDED "AS IS" WITHOUT EXPRESS OR 
* IMPLIED WARRANTY OF ANY KIND.  NVIDIA DISCLAIMS ALL WARRANTIES WITH 
* REGARD TO THIS SOURCE CODE, INCLUDING ALL IMPLIED WARRANTIES OF 
* MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE.   
* IN NO EVENT SHALL NVIDIA BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL, 
* OR CONSEQUENTIAL DAMAGES, OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS 
* OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE 
* OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE 
* OR PERFORMANCE OF THIS SOURCE CODE.  
*
* U.S. Government End Users.  This source code is a "commercial item" as 
* that term is defined at 48 C.F.R. 2.101 (OCT 1995), consisting  of 
* "commercial computer software" and "commercial computer software 
* documentation" as such terms are used in 48 C.F.R. 12.212 (SEPT 1995) 
* and is provided to the U.S. Government only as a commercial end item.  
* Consistent with 48 C.F.R.12.212 and 48 C.F.R. 227.7202-1 through 
* 227.7202-4 (JUNE 1995), all U.S. Government End Users acquire the 
* source code with only those rights set forth herein.
*/

#ifndef CUTIL_GL_ERROR
#define CUTIL_GL_ERROR

/* CUda UTility Library */

// includes, system
#ifdef _WIN32
#  define WINDOWS_LEAN_AND_MEAN
#  include <windows.h>
#  include <stdlib.h>
#  undef min
#  undef max
#endif

// includes, graphics
#if defined (__APPLE__) || defined(MACOSX)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

////////////////////////////////////////////////////////////////////////////
//! Check for OpenGL error
//! @return CUTTrue if no GL error has been encountered, otherwise 0
//! @param file  __FILE__ macro
//! @param line  __LINE__ macro
//! @note The GL error is listed on stderr
//! @note This function should be used via the CHECK_ERROR_GL() macro
////////////////////////////////////////////////////////////////////////////
CUTBoolean CUTIL_API
cutCheckErrorGL( const char* file, const int line) 
{
	CUTBoolean ret_val = CUTTrue;

	// check for error
	GLenum gl_error = glGetError();
	if (gl_error != GL_NO_ERROR) 
	{
		fprintf(stderr, "GL Error in file '%s' in line %d :\n", file, line);
		fprintf(stderr, "%s\n", gluErrorString(gl_error));
		ret_val = CUTFalse;
	}
	return ret_val;
}

#ifdef _DEBUG

#define CUT_CHECK_ERROR_GL()                                               \
	if( CUTFalse == cutCheckErrorGL( __FILE__, __LINE__)) {                  \
	exit(EXIT_FAILURE);                                                  \
	}

#endif // _DEBUG

#endif // CUTIL_GL_ERROR
