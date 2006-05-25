/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FUtils/FULogFile.h"
#include "FUtils/FUFile.h"

FULogFile::FULogFile(const char* filename)
{
	file = new FUFile(filename, FUFile::WRITE);
}

FULogFile::~FULogFile()
{
	SAFE_DELETE(file);
}

void FULogFile::WriteLine(const char* filename, uint32 linenum, const char* message, ...)
{
	WriteLine("[%s:%d]", filename, (unsigned int) linenum);

	va_list vars;
	va_start(vars, message);
	WriteLineV(message, vars);
	va_end(vars);
}

void FULogFile::WriteLine(const char* message, ...)
{
	va_list vars;
	va_start(vars, message);
	WriteLineV(message, vars);
	va_end(vars);
}

void FULogFile::WriteLineV(const char* message, va_list& vars)
{
	size_t len = strlen(message);
	char* buffer = new char[len + 1024];
	vsnprintf(buffer, len + 1024, message, vars);
	buffer[len + 1023] = 0;

	if (file->IsOpen())
	{
		file->Write(buffer, strlen(buffer) - 1);
		file->Write("\n\r", 2);
	}

	SAFE_DELETE_ARRAY(buffer);
}

void FULogFile::Flush()
{
	file->Flush();
}
