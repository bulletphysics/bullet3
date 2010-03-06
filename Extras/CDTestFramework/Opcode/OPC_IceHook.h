/*
 *	OPCODE - Optimized Collision Detection
 * http://www.codercorner.com/Opcode.htm
 * 
 * Copyright (c) 2001-2008 Pierre Terdiman,  pierre@codercorner.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

// Should be included by Opcode.h if needed

	#define ICE_DONT_CHECK_COMPILER_OPTIONS

	// From Windows...
	typedef int                 BOOL;
	#ifndef FALSE
	#define FALSE               0
	#endif

	#ifndef TRUE
	#define TRUE                1
	#endif

	#include <stdio.h>
	#include <stdlib.h>
	#include <assert.h>
	#include <string.h>
	#include <float.h>
	#include <Math.h>

	#ifndef ASSERT
		#define	ASSERT(exp)	{}
	#endif
	#define ICE_COMPILE_TIME_ASSERT(exp)	extern char ICE_Dummy[ (exp) ? 1 : -1 ]

	#define	Log				{}
	#define	SetIceError		false
	#define	EC_OUTOFMEMORY	"Out of memory"

	#include ".\Ice\IcePreprocessor.h"

	#undef ICECORE_API
	#define ICECORE_API	OPCODE_API

	#include ".\Ice\IceTypes.h"
	#include ".\Ice\IceFPU.h"
	#include ".\Ice\IceMemoryMacros.h"

	namespace Opcode
	{
	namespace IceCore
	{
		#include ".\Ice\IceAllocator.h"
		#include ".\Ice\IceUtils.h"
		#include ".\Ice\IceBitArray.h"
		#include ".\Ice\IceContainer.h"
		#include ".\Ice\IcePairs.h"
		#include ".\Ice\IceRevisitedRadix.h"
		#include ".\Ice\IceRandom.h"
		#include ".\Ice\IceHashing.h"
	}
	using namespace IceCore;

	#define ICEMATHS_API	OPCODE_API
	namespace IceMaths
	{
		#include ".\Ice\IceAxes.h"
		#include ".\Ice\IcePoint.h"
		#include ".\Ice\IceHPoint.h"
		#include ".\Ice\IceMatrix3x3.h"
		#include ".\Ice\IceMatrix4x4.h"
		#include ".\Ice\IcePlane.h"
		#include ".\Ice\IceRay.h"
		#include ".\Ice\IceIndexedTriangle.h"
		#include ".\Ice\IceTriangle.h"
		#include ".\Ice\IceTriList.h"
		#include ".\Ice\IceAABB.h"
		#include ".\Ice\IceOBB.h"
		#include ".\Ice\IceBoundingSphere.h"
		#include ".\Ice\IceSegment.h"
		#include ".\Ice\IceLSS.h"
	}
	using namespace IceMaths;
}
