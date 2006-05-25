/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#ifndef _FU_LOG_FILE_H_
#define _FU_LOG_FILE_H_

class FUFile;

class FCOLLADA_EXPORT FULogFile
{
private:
	FUFile* file;

public:
	FULogFile(const char* filename);
	~FULogFile();

	void WriteLine(const char* filename, uint32 linenum, const char* message, ...);
	void WriteLine(const char* message, ...);
	void WriteLineV(const char* message, va_list& vars);

	void Flush();
};

#endif // _FU_LOG_FILE_H_
