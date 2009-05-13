/*
  FLUIDS v.1 - SPH Fluid Simulator for CPU and GPU
  Copyright (C) 2009. Rama Hoetzlein, http://www.rchoetzlein.com

  ZLib license
  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

#ifndef COMMON_DEF
	#define COMMON_DEF

	// Global defs

	//#define		USE_SHADOWS			// Disable if you don't have FBOs, or to greatly speed up demo.

//	#define		USE_JPEG

	#define		BUILD_CUDA			// CUDA - Visual Studio 2005 only (as of April 2009)

	#define TEX_SIZE		2048	
	#define LIGHT_NEAR		0.5
	#define LIGHT_FAR		300.0
	#define DEGtoRAD		(3.141592/180.0)

	#ifdef _MSC_VER
		#include <windows.h>
	#else
		typedef unsigned int		DWORD;
	#endif
	typedef unsigned int		uint;

	#define COLOR(r,g,b)	( (DWORD(r*255.0f)<<24) | (DWORD(g*255.0f)<<16) | (DWORD(b*255.0f)<<8) )
	#define COLORA(r,g,b,a)	( (DWORD(r*255.0f)<<24) | (DWORD(g*255.0f)<<16) | (DWORD(b*255.0f)<<8) | DWORD(a*255.0f) )
	#define RED(c)			(float((c>>24) & 0xFF)/255.0)
	#define GRN(c)			(float((c>>16) & 0xFF)/255.0)
	#define BLUE(c)			(float((c>>8) & 0xFF)/255.0)
	#define ALPH(c)			(float(c & 0xFF)/255.0)
#endif