/*
	GWEN
	Copyright (c) 2010 Facepunch Studios
	See license in Gwen.h
*/

#pragma once
#ifndef GWEN_PLATFORM_H
#define GWEN_PLATFORM_H

#include "Gwen/Structures.h"
#include "Gwen/Events.h"

namespace Gwen
{
	namespace Platform
	{
		//
		// Set the system cursor to iCursor
		// Cursors are defined in Structures.h
		//
		void GWEN_EXPORT SetCursor( unsigned char iCursor );

		//
		// Used by copy/paste
		//
		UnicodeString GWEN_EXPORT GetClipboardText();
		bool GWEN_EXPORT SetClipboardText( const UnicodeString& str );

		//
		// Needed for things like double click
		//
		float GWEN_EXPORT GetTimeInSeconds();

		//
		// System Dialogs ( Can return false if unhandled )
		//
		bool GWEN_EXPORT FileOpen( const String& Name, const String& StartPath, const String& Extension, Gwen::Event::Handler* pHandler, Event::Handler::FunctionStr fnCallback );
		bool GWEN_EXPORT FileSave( const String& Name, const String& StartPath, const String& Extension, Gwen::Event::Handler* pHandler, Event::Handler::FunctionStr fnCallback );
	}

}
#endif
