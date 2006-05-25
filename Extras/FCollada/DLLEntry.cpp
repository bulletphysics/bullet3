/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"

HINSTANCE hInstance = NULL;

BOOL WINAPI DllMain(HINSTANCE _hInstance, ULONG UNUSED(fdwReason), LPVOID UNUSED(lpvReserved))
{
	hInstance = _hInstance;
	return TRUE;
}

namespace FCollada
{
	FCOLLADA_EXPORT unsigned long GetVersion() { return FCOLLADA_VERSION; }
};

#include "FMath/FMColor.h"
#include "FUtils/FUDebug.h"
#include "FUtils/FULogFile.h"

// Trick the linker so that it adds the functionalities of the classes that are not used internally.
FCOLLADA_EXPORT void TrickLinker()
{
	// FMColor
	FMColor* color = NULL;
	float* f = NULL;
	color->ToFloats(f, 4);

	// FULogFile
	FULogFile* logFile = NULL;
	logFile->WriteLine("Test");

	// FUDebug
	DebugOut("Tricking Linker...");
}