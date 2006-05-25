/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FUtils/FUFile.h"

FUFile::FUFile(const char* filename, Mode mode)
{
	const char* openMode;
	switch (mode)
	{
	case READ: openMode = "rb"; break;
	case WRITE: openMode = "wb"; break;
	default: openMode = "rb"; break;
	}

	filePtr = fopen(filename, openMode);
}

FUFile::FUFile()
{
	filePtr = NULL;
}

FUFile::~FUFile()
{
	if (filePtr != NULL)
	{
		Close();
	}
}

// Retrieve the file length
uint32 FUFile::GetLength()
{
	FUAssert(IsOpen(), return 0);

	uint32 currentPosition = ftell(filePtr);
	if (fseek(filePtr, 0, SEEK_END) != 0) return 0;

	uint32 length = ftell(filePtr);
	if (fseek(filePtr, currentPosition, SEEK_SET) != 0) return 0;

	return length;
}

// Reads in a piece of the file into the given buffer
bool FUFile::Read(void* buffer, size_t length)
{
	FUAssert(IsOpen(), return false);
	return fread(buffer, length, 1, filePtr) == 1;
}

// Write out some data to a file
bool FUFile::Write(const void* buffer, size_t length)
{
	FUAssert(IsOpen(), return false);
	return fwrite(buffer, length, 1, filePtr) == 1;
}

// Flush/close the file stream
void FUFile::Flush()
{
	FUAssert(IsOpen(), );
	fflush(filePtr);
}

void FUFile::Close()
{
	FUAssert(IsOpen(), );
	fclose(filePtr);
	filePtr = NULL;
}
