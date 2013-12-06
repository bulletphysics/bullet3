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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Main file for Opcode.dll.
 *	\file		Opcode.h
 *	\author		Pierre Terdiman
 *	\date		March, 20, 2001
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef __OPCODE_H__
#define __OPCODE_H__

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Compilation messages
#if defined(OPCODE_EXPORTS)
	#pragma message("Compiling OPCODE")
#elif !defined(OPCODE_EXPORTS)
	#pragma message("Using OPCODE")
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Automatic linking
	#ifndef BAN_OPCODE_AUTOLINK
		#ifdef _DEBUG
			#pragma comment(lib, "Opcode_D.lib")
		#else
			#pragma comment(lib, "Opcode.lib")
		#endif
	#endif
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Preprocessor
#ifndef ICE_NO_DLL
	#ifdef OPCODE_EXPORTS
		#define OPCODE_API __declspec(dllexport)
	#else
		#define OPCODE_API __declspec(dllimport)
	#endif
#else
		#define OPCODE_API
#endif

	#include "OPC_IceHook.h"

	namespace Opcode
	{
		// Bulk-of-the-work
		#include "OPC_Settings.h"
		#include "OPC_Common.h"
		#include "OPC_MeshInterface.h"
		// Builders
		#include "OPC_TreeBuilders.h"
		// Trees
		#include "OPC_AABBTree.h"
		#include "OPC_OptimizedTree.h"
		// Models
		#include "OPC_BaseModel.h"
		#include "OPC_Model.h"
		#include "OPC_HybridModel.h"
		// Colliders
		#include "OPC_Collider.h"
		#include "OPC_VolumeCollider.h"
		#include "OPC_TreeCollider.h"
		#include "OPC_RayCollider.h"
		#include "OPC_SphereCollider.h"
		#include "OPC_OBBCollider.h"
		#include "OPC_AABBCollider.h"
		#include "OPC_LSSCollider.h"
		#include "OPC_PlanesCollider.h"
		// Usages
		#include "OPC_Picking.h"
		// Sweep-and-prune
		#include "OPC_BoxPruning.h"
		#include "OPC_SweepAndPrune.h"
		#include "OPC_ArraySAP.h"

		FUNCTION OPCODE_API bool InitOpcode();
		FUNCTION OPCODE_API bool CloseOpcode();
	}

#endif // __OPCODE_H__
