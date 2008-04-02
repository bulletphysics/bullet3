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
 *	Contains code for box pruning.
 *	\file		IceBoxPruning.h
 *	\author		Pierre Terdiman
 *	\date		January, 29, 2000
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef __OPC_BOXPRUNING_H__
#define __OPC_BOXPRUNING_H__

	// Optimized versions
	FUNCTION OPCODE_API bool CompleteBoxPruning(udword nb, const AABB** array, Pairs& pairs, const Axes& axes);
	FUNCTION OPCODE_API bool BipartiteBoxPruning(udword nb0, const AABB** array0, udword nb1, const AABB** array1, Pairs& pairs, const Axes& axes);

	// Brute-force versions
	FUNCTION OPCODE_API bool BruteForceCompleteBoxTest(udword nb, const AABB** array, Pairs& pairs);
	FUNCTION OPCODE_API bool BruteForceBipartiteBoxTest(udword nb0, const AABB** array0, udword nb1, const AABB** array1, Pairs& pairs);

#endif //__OPC_BOXPRUNING_H__