/*
	GWEN
	Copyright (c) 2011 Facepunch Studios
	See license in Gwen.h
*/

#include "Gwen/Macros.h"
#include "Gwen/Platform.h"

#ifdef _WIN32

#include <windows.h>

#include <mmsystem.h>
#pragma comment(lib, "winmm.lib")

using namespace Gwen;
using namespace Gwen::Platform;

#ifdef UNICODE
static LPWSTR iCursorConvertion[] =
#else
static LPSTR iCursorConvertion[] =
#endif
	{
		IDC_ARROW,
		IDC_IBEAM,
		IDC_SIZENS,
		IDC_SIZEWE,
		IDC_SIZENWSE,
		IDC_SIZENESW,
		IDC_SIZEALL,
		IDC_NO,
		IDC_WAIT,
		IDC_HAND};

void Gwen::Platform::SetCursor(unsigned char iCursor)
{
	// Todo.. Properly.
	::SetCursor(LoadCursor(NULL, iCursorConvertion[iCursor]));
}

Gwen::UnicodeString Gwen::Platform::GetClipboardText()
{
	if (!OpenClipboard(NULL)) return L"";

	HANDLE hData = GetClipboardData(CF_UNICODETEXT);

	if (hData == NULL)
	{
		CloseClipboard();
		return L"";
	}

	wchar_t* buffer = (wchar_t*)GlobalLock(hData);
	UnicodeString str = buffer;
	GlobalUnlock(hData);
	CloseClipboard();
	return str;
}

bool Gwen::Platform::SetClipboardText(const Gwen::UnicodeString& str)
{
	if (!OpenClipboard(NULL)) return false;

	EmptyClipboard();

	// Create a buffer to hold the string
	size_t iDataSize = (str.length() + 1) * sizeof(wchar_t);
	HGLOBAL clipbuffer = GlobalAlloc(GMEM_DDESHARE, iDataSize);

	// Copy the string into the buffer
	wchar_t* buffer = (wchar_t*)GlobalLock(clipbuffer);
	//wcscpy_s( buffer, iDataSize, str.c_str() );
	memcpy(buffer, str.c_str(), iDataSize);
	GlobalUnlock(clipbuffer);

	// Place it on the clipboard
	SetClipboardData(CF_UNICODETEXT, clipbuffer);

	CloseClipboard();
	return true;
}

double GetPerformanceFrequency()
{
	static double Frequency = 0.0f;

	if (Frequency == 0.0f)
	{
		__int64 perfFreq;
		QueryPerformanceFrequency((LARGE_INTEGER*)&perfFreq);
		Frequency = 1.0 / (double)perfFreq;
	}

	return Frequency;
}

float Gwen::Platform::GetTimeInSeconds()
{
#if 1

	static float fCurrentTime = 0.0f;
	static __int64 iLastTime = 0;

	__int64 thistime;
	QueryPerformanceCounter((LARGE_INTEGER*)&thistime);

	float fSecondsDifference = (double)(thistime - iLastTime) * GetPerformanceFrequency();
	if (fSecondsDifference > 0.1f) fSecondsDifference = 0.1f;

	fCurrentTime += fSecondsDifference;

	iLastTime = thistime;

	return fCurrentTime;

#else

	return timeGetTime() / 1000.0;

#endif
}

bool Gwen::Platform::FileOpen(const String& Name, const String& StartPath, const String& Extension, Gwen::Event::Handler* pHandler, Event::Handler::FunctionStr fnCallback)
{
	char Filestring[256];
	String returnstring;

	char FilterBuffer[512];
	{
		memset(FilterBuffer, 0, sizeof(FilterBuffer));
		memcpy(FilterBuffer, Extension.c_str(), GwenUtil_Min(Extension.size(), 512));
		for (int i = 0; i < 512; i++)
		{
			if (FilterBuffer[i] == '|')
				FilterBuffer[i] = 0;
		}
	}

	OPENFILENAMEA opf;
	opf.hwndOwner = 0;
	opf.lpstrFilter = FilterBuffer;
	opf.lpstrCustomFilter = 0;
	opf.nMaxCustFilter = 0L;
	opf.nFilterIndex = 1L;
	opf.lpstrFile = Filestring;
	opf.lpstrFile[0] = '\0';
	opf.nMaxFile = 256;
	opf.lpstrFileTitle = 0;
	opf.nMaxFileTitle = 50;
	opf.lpstrInitialDir = StartPath.c_str();
	opf.lpstrTitle = Name.c_str();
	opf.nFileOffset = 0;
	opf.nFileExtension = 0;
	opf.lpstrDefExt = "*.*";
	opf.lpfnHook = NULL;
	opf.lCustData = 0;
	opf.Flags = (OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR) & ~OFN_ALLOWMULTISELECT;
	opf.lStructSize = sizeof(OPENFILENAME);

	if (GetOpenFileNameA(&opf))
	{
		if (pHandler && fnCallback)
		{
			(pHandler->*fnCallback)(opf.lpstrFile);
		}
	}
	else
	{
		if (pHandler && fnCallback)
		{
			(pHandler->*fnCallback)("");
		}
	}

	return true;
}

bool Gwen::Platform::FileSave(const String& Name, const String& StartPath, const String& Extension, Gwen::Event::Handler* pHandler, Gwen::Event::Handler::FunctionStr fnCallback)
{
	char Filestring[256];
	String returnstring;

	char FilterBuffer[512];
	{
		memset(FilterBuffer, 0, sizeof(FilterBuffer));
		memcpy(FilterBuffer, Extension.c_str(), GwenUtil_Min(Extension.size(), 512));
		for (int i = 0; i < 512; i++)
		{
			if (FilterBuffer[i] == '|')
				FilterBuffer[i] = 0;
		}
	}

	OPENFILENAMEA opf;
	opf.hwndOwner = 0;
	opf.lpstrFilter = FilterBuffer;
	opf.lpstrCustomFilter = 0;
	opf.nMaxCustFilter = 0L;
	opf.nFilterIndex = 1L;
	opf.lpstrFile = Filestring;
	opf.lpstrFile[0] = '\0';
	opf.nMaxFile = 256;
	opf.lpstrFileTitle = 0;
	opf.nMaxFileTitle = 50;
	opf.lpstrInitialDir = StartPath.c_str();
	opf.lpstrTitle = Name.c_str();
	opf.nFileOffset = 0;
	opf.nFileExtension = 0;
	opf.lpstrDefExt = "*.*";
	opf.lpfnHook = NULL;
	opf.lCustData = 0;
	opf.Flags = (OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR) & ~OFN_ALLOWMULTISELECT;
	opf.lStructSize = sizeof(OPENFILENAME);

	if (GetSaveFileNameA(&opf))
	{
		if (pHandler && fnCallback)
		{
			(pHandler->*fnCallback)(opf.lpstrFile);
		}
	}
	else
	{
		if (pHandler && fnCallback)
		{
			(pHandler->*fnCallback)("");
		}
	}

	return true;
}

#endif  // WIN32