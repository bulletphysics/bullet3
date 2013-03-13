/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#include "Gwen/Gwen.h"
#include "Gwen/Platform.h"
#include "Gwen/Controls/Dialogs/FileSave.h"

using namespace Gwen;
using namespace Gwen::Dialogs;

void Gwen::Dialogs::FileSaveEx( bool bUseSystem, const String& Name, const String& StartPath, const String& Extension, Gwen::Event::Handler* pHandler, Gwen::Event::Handler::FunctionStr fnCallback )
{
	if ( bUseSystem && Gwen::Platform::FileSave( Name, StartPath, Extension, pHandler, fnCallback ) )
	{
		return;
	}

	//
	// TODO: SHOW GWEN FILE SELECTION DIALOG
	//
}