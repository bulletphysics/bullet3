/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include <MiniCL/cl_MiniCL_Defs.h>

#define MSTRINGIFY(A) A
#define LOCAL_SIZE_MAX 1024U

#include "../ParticlesOCL.cl"

MINICL_REGISTER(kComputeCellId)
MINICL_REGISTER(kClearCellStart)
MINICL_REGISTER(kFindCellStart)
MINICL_REGISTER(kIntegrateMotion)
MINICL_REGISTER(kCollideParticles)
MINICL_REGISTER(kBitonicSortCellIdLocal)
MINICL_REGISTER(kBitonicSortCellIdLocal1)
MINICL_REGISTER(kBitonicSortCellIdMergeGlobal)
MINICL_REGISTER(kBitonicSortCellIdMergeLocal)

