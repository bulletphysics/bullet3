/*
Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
Copyright (C) 2006 - 2010 Sony Computer Entertainment Inc. 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_OCL_UTILS_H
#define BT_OCL_UTILS_H

#ifdef USE_MINICL
	#include <MiniCL/cl.h>
#else
	#ifdef __APPLE__
		#include <OpenCL/cl.h>
	#else
		#include <CL/cl.h>
	#endif __APPLE__
#endif

#include <stdio.h>


//#define oclCHECKERROR(a, b) btAssert((a) == (b))
#define oclCHECKERROR(a, b) if((a)!=(b)) { printf("OCL Error : %d\n", (a)); btAssert((a) == (b)); }


void btOclPrintDevInfo(cl_device_id device);
cl_device_id btOclGetDev(cl_context cxMainContext, unsigned int nr);
cl_device_id btOclGetMaxFlopsDev(cl_context cxMainContext);
char* btOclLoadProgSource(const char* cFilename, const char* cPreamble, size_t* szFinalLength);
cl_device_id btOclGetFirstDev(cl_context cxMainContext);
#endif //BT_OCL_UTILS_H
