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
 *	\file		Opcode.cpp
 *	\author		Pierre Terdiman
 *	\date		March, 20, 2001
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
	Finding a good name is difficult!
	Here's the draft for this lib.... Spooky, uh?

	VOID?			Very Optimized Interference Detection
	ZOID?			Zappy's Optimized Interference Detection
	CID?			Custom/Clever Interference Detection
	AID / ACID!		Accurate Interference Detection
	QUID?			Quick Interference Detection
	RIDE?			Realtime Interference DEtection
	WIDE?			Wicked Interference DEtection (....)
	GUID!
	KID !			k-dop interference detection :)
	OPCODE!			OPtimized COllision DEtection
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Precompiled Header
#include "Stdafx.h"

bool Opcode::InitOpcode()
{
	Log("// Initializing OPCODE\n\n");
//	LogAPIInfo();
	return true;
}

void ReleasePruningSorters();
bool Opcode::CloseOpcode()
{
	Log("// Closing OPCODE\n\n");

	ReleasePruningSorters();

	return true;
}

#ifdef ICE_MAIN

void ModuleAttach(HINSTANCE hinstance)
{
}

void ModuleDetach()
{
}

#endif