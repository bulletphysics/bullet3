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
 *	Contains compilation flags.
 *	\file		OPC_Settings.h
 *	\author		Pierre Terdiman
 *	\date		May, 12, 2001
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef __OPC_SETTINGS_H__
#define __OPC_SETTINGS_H__

	//! Use CPU comparisons (comment that line to use standard FPU compares)
	#define OPC_CPU_COMPARE

	//! Use FCOMI / FCMOV on Pentium-Pro based processors (comment that line to use plain C++)
	#define OPC_USE_FCOMI

	//! Use epsilon value in tri-tri overlap test
	#define OPC_TRITRI_EPSILON_TEST

	//! Use tree-coherence or not [not implemented yet]
//	#define OPC_USE_TREE_COHERENCE

	//! Use callbacks or direct pointers. Using callbacks might be a bit slower (but probably not much)
//	#define OPC_USE_CALLBACKS

	//! Support triangle and vertex strides or not. Using strides might be a bit slower (but probably not much)
//	#define OPC_USE_STRIDE

	//! Discard negative pointer in vanilla trees
	#define OPC_NO_NEG_VANILLA_TREE

	//! Use a callback in the ray collider
	#define OPC_RAYHIT_CALLBACK

	// NB: no compilation flag to enable/disable stats since they're actually needed in the box/box overlap test

#endif //__OPC_SETTINGS_H__