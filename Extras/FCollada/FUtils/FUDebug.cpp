/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FUtils/FUDebug.h"
#include "FUtils/FUStringConversion.h"

#ifdef _DEBUG

#if defined(LINUX) || defined(MAC_TIGER)
#define STRING_OUT(sz) cerr << (sz) << endl
#elif defined(WIN32)
#define STRING_OUT(sz) OutputDebugString(sz); OutputDebugString(FC("\n"))
#endif

static void DebugString(const fchar* message)
{
	STRING_OUT(message);
}

#ifdef UNICODE
static void DebugString(const char* message)
{
	fstring str = TO_FSTRING(message);
	DebugString(str.c_str());
}
#endif // UNICODE

void DebugOut(const char* filename, uint32 line, const char* message, ...)
{
	va_list vars;
	va_start(vars, message);
	DebugOutV(filename, line, message, vars);
	va_end(vars);
}

void DebugOut(const char* message, ...)
{
	va_list vars;
	va_start(vars, message);
	DebugOutV(message, vars);
	va_end(vars);
}

void DebugOutV(const char* filename, uint32 line, const char* message, va_list& vars)
{
	char buffer[256];
	snprintf(buffer, 256, "[%s@%lu] ", filename, line);
	buffer[255] = 0;
	DebugString(buffer);

	DebugOutV(message, vars);
}

void DebugOutV(const char* message, va_list& vars)
{
	uint32 length = (uint32) strlen(message);
	char* buffer = new char[length + 256];
	vsnprintf(buffer, length + 256, message, vars);
	buffer[length + 255] = 0;

	DebugString(buffer);
	SAFE_DELETE_ARRAY(buffer);
}

#ifdef UNICODE
void DebugOut(const char* filename, uint32 line, const fchar* message, ...)
{
	va_list vars;
	va_start(vars, message);
	DebugOutV(filename, line, message, vars);
	va_end(vars);
}

void DebugOut(const fchar* message, ...)
{
	va_list vars;
	va_start(vars, message);
	DebugOutV(message, vars);
	va_end(vars);
}

void DebugOutV(const char* filename, uint32 line, const fchar* message, va_list& vars)
{
	char buffer[256];
	snprintf(buffer, 256, "[%s@%lu] ", filename, line);
	buffer[255] = 0;
	DebugString(buffer);

	DebugOutV(message, vars);
}

void DebugOutV(const fchar* message, va_list& vars)
{
	uint32 length = (uint32) fstrlen(message);
	fchar* buffer = new fchar[length + 256];
	fvsnprintf(buffer, length + 256, message, vars);
	buffer[length + 255] = 0;

	DebugString(buffer);
	SAFE_DELETE_ARRAY(buffer);
}
#endif // UNICODE
#endif // _DEBUG

