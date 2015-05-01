/*
	GWEN
	Copyright (c) 2011 Facepunch Studios
	See license in Gwen.h
*/

#include "Gwen/Macros.h"
#include "Gwen/Platform.h"

#ifndef _WIN32

#include <time.h>

static Gwen::UnicodeString gs_ClipboardEmulator;

void Gwen::Platform::SetCursor( unsigned char iCursor )
{
	// No platform independent way to do this
}

Gwen::UnicodeString Gwen::Platform::GetClipboardText()
{
	return gs_ClipboardEmulator;
}

bool Gwen::Platform::SetClipboardText( const Gwen::UnicodeString& str )
{
	gs_ClipboardEmulator = str;
	return true;
}

float Gwen::Platform::GetTimeInSeconds()
{
	float fSeconds = (float) clock() / (float)CLOCKS_PER_SEC;
	return fSeconds;
}

bool Gwen::Platform::FileOpen( const String& Name, const String& StartPath, const String& Extension, Gwen::Event::Handler* pHandler, Event::Handler::FunctionStr fnCallback )
{
	// No platform independent way to do this.
	// Ideally you would open a system dialog here

	return false;
}

bool Gwen::Platform::FileSave( const String& Name, const String& StartPath, const String& Extension, Gwen::Event::Handler* pHandler, Gwen::Event::Handler::FunctionStr fnCallback )
{
	// No platform independent way to do this.
	// Ideally you would open a system dialog here

	return false;
}

#endif // ndef WIN32