/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/
/*
	Based on the FS Import classes:
	Copyright (C) 2005-2006 Feeling Software Inc
	Copyright (C) 2005-2006 Autodesk Media Entertainment
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#ifndef _FU_FILE_MANAGER_H_
#define _FU_FILE_MANAGER_H_

class FUFile;

class FCOLLADA_EXPORT FUFileManager
{
private:
	FStringList pathStack;

public:
	FUFileManager();
	~FUFileManager();

	// Root path stack
	const fstring& GetCurrentPath() { return pathStack.back(); }
	void PushRootPath(const fstring& path);
	void PopRootPath();
	void PushRootFile(const fstring& filename);
	void PopRootFile();

	// Some file access
	FUFile* OpenFile(const fstring& filename, bool write=true);

	// Extract information from filenames
	static fstring StripFileFromPath(const fstring& filename);
	static fstring GetFileExtension(const fstring& filename);

	// Make a file path relative/absolute
	fstring MakeFilePathAbsolute(const fstring& filePath);
	fstring MakeFilePathRelative(const fstring& filePath);

	// Transform a file URL into a file path
	fstring GetFilePath(const fstring& fileURL);

	// Transform a file path into a file URL
	fstring GetFileURL(const fstring& filepath, bool relative);

	// For a relative path, extract the list of the individual paths that must be traversed to get to the file.
	static void ExtractPathStack(const fstring& filename, FStringList& list, bool includeFilename);
};

#endif // _FU_FILE_MANAGER_H_

