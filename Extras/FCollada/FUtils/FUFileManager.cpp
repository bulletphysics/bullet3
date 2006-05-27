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

#include "StdAfx.h"
#include "FUtils/FUFile.h"
#include "FUtils/FUFileManager.h"
#include "FUtils/FUStringConversion.h"

#if defined(WIN32)
	#include <direct.h>
#endif

FUFileManager::FUFileManager()
{
	// Push on the stack the original root path
	char fullPath[MAX_PATH];
	getcwd(fullPath, MAX_PATH);
	pathStack.push_back(TO_FSTRING(fullPath));
}

FUFileManager::~FUFileManager()
{
}

// Set a new root path
void FUFileManager::PushRootPath(const fstring& path)
{
	// Ensure that the new root path is an absolute root path.
	fstring absolutePath = MakeFilePathAbsolute(path);
	pathStack.push_back(absolutePath);
	fchdir(path.c_str());
}

// Go back to the previous root path
void FUFileManager::PopRootPath()
{
	if (pathStack.size() > 1)
	{
		pathStack.pop_back();
		fchdir(pathStack.back().c_str());
	}
}

// Set the current path root, using a known filename
void FUFileManager::PushRootFile(const fstring& filename)
{
	// Strip the filename of the actual file's name
	fstring path = StripFileFromPath(filename);
	PushRootPath(path);
}

void FUFileManager::PopRootFile()
{
	PopRootPath();
}

// Open a file to read
FUFile* FUFileManager::OpenFile(const fstring& filename, bool write)
{
	fstring absoluteFilename = MakeFilePathAbsolute(filename);
	string dumpFilename = FUStringConversion::ToString(absoluteFilename);
	return new FUFile(dumpFilename.c_str(), write ? FUFile::WRITE : FUFile::READ);
}

// Massage a given filename to be absolute
fstring FUFileManager::GetFilePath(const fstring& fileURL)
{
	// Strip any prefix
	fstring filename = fileURL;
	std::replace(filename.begin(), filename.end(), '\\', '/');
	if (filename.size() > 7 && filename.substr(0, 7) == FC("file://"))
	{
        filename = filename.c_str() + 7;

		if (filename.size() > 3 && filename[0] == '/' && (filename[2] == ':' || filename[2] == '|'))
		{
			filename = filename.c_str() + 1;
		}

		if (filename[1] == '|') filename[1] = ':';
	}

	// Replace any '%' character string into the wanted characters: %20 is common.
	for (size_t pos = filename.find('%'); pos != fstring::npos; pos = filename.find('%'))
	{
		const fchar* pc = filename.c_str() + pos + 1; // +1 to skip/erase the '%' character
		uint32 value = FUStringConversion::HexToUInt32(&pc, 2);
		size_t count = (pc - filename.c_str()) - pos;
		filename.erase(pos, count);
		filename.insert(pos, 1, (fchar) value);
	}

	return MakeFilePathAbsolute(filename);
}

// Transform a file path into a file URL
fstring FUFileManager::GetFileURL(const fstring& filepath, bool relative)
{
	fstring url;
	if (relative)
	{
		url = MakeFilePathRelative(filepath);
		if (!url.empty() && url[0] != '.')
		{
			// Unable to make the path relative, so return an absolute path
			relative = false;
		}
	}

	if (!relative)
	{
		// Transform into an absolute file path
		fstring url = MakeFilePathAbsolute(filepath);
		std::replace(url.begin(), url.end(), ':', '|');
		std::replace(url.begin(), url.end(), '\\', '/');
        url = fstring(FC("file://")) + url;
	}

	// Remove any invalid character(s) using the %X guideline
	/*for (size_t p = 0; p < url.size(); ++p)
	{
		fchar c = url[p];
		if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
			c == '.' || c == '|' || c == '/' || c == '-' || c == '_')
		{
			// Valid characters
		}
		else
		{
			url.erase(p, 1);
			globalBuilder.set('%');  <---- NEED TO BE IN HEX
			globalBuilder.append(c);
			url.append(globalBuilder.ToCharPtr());
		}
	}*/

	return url;
}

// Strip a full filename of its filename, returning the path
fstring FUFileManager::StripFileFromPath(const fstring& filename)
{
	fchar fullPath[MAX_PATH + 1];
	fstrncpy(fullPath, filename.c_str(), MAX_PATH);
	fullPath[MAX_PATH] = 0;
	fchar* lastSlash = fstrrchr(fullPath, FC('/'));
	fchar* lastBackslash = fstrrchr(fullPath, FC('\\'));
	lastSlash = max(lastSlash, lastBackslash);
	if (lastSlash != NULL) *lastSlash = 0;
	return fstring(fullPath);
}

// Extract the file extension out of a filename
fstring FUFileManager::GetFileExtension(const fstring& _filename)
{
	fchar filename[MAX_PATH];
	fstrncpy(filename, _filename.c_str(), MAX_PATH);
	filename[MAX_PATH - 1] = 0;

	fchar* lastPeriod = fstrrchr(filename, '.');
	if (lastPeriod == NULL) return fstring();

	fchar* lastSlash = fstrrchr(filename, '/');
	fchar* lastBackslash = fstrrchr(filename, '\\');
	lastSlash = max(lastSlash, lastBackslash);
	if (lastSlash > lastPeriod) return fstring();

	fstrlower(lastPeriod + 1);	
	return fstring(lastPeriod + 1);
}

// For a relative path, extract the list of the individual paths that must be traversed to get to the file.
void FUFileManager::ExtractPathStack(const fstring& name, FStringList& list, bool includeFilename)
{
	list.clear();
	list.reserve(6);

	fstring split = name;
	while (split.length() > 0)
	{
		// Extract out the next path
		size_t slashIndex = split.find_first_of('/');
		size_t bslashIndex = split.find_first_of('\\');
		size_t pathSeparator;
		if (slashIndex != fstring::npos && bslashIndex != fstring::npos) pathSeparator = min(slashIndex, bslashIndex);
		else if (bslashIndex != fstring::npos) pathSeparator = bslashIndex;
		else if (slashIndex != fstring::npos) pathSeparator = slashIndex;
		else
		{
			if (includeFilename) list.push_back(split);
			break;
		}

		list.push_back(split.substr(0, pathSeparator));
		split = split.substr(pathSeparator + 1, split.length() - pathSeparator - 1);
	}
}

// Make a file path relative/absolute
fstring FUFileManager::MakeFilePathAbsolute(const fstring& _filePath)
{
	fstring filePath = _filePath;
	if ((filePath.size() > 1 && (filePath[0] == '/' || filePath[0] == '\\')) ||
		(filePath.size() > 2 && (filePath[1] == ':' || filePath[1] == '|')))
	{
		// Already an absolute filepath
	}
	else
	{
		// Relative file path.
		FStringList documentPaths, localPaths;
		ExtractPathStack(pathStack.back(), documentPaths, true);
		ExtractPathStack(filePath, localPaths, true);
		for (FStringList::iterator it = localPaths.begin(); it != localPaths.end(); ++it)
		{
			// Look for special relative path tokens: '.' and '..'
			if ((*it) == FC(".")) {} // do nothing
			else if ((*it) == FC("..")) { documentPaths.pop_back(); } // pop one path out
			else { documentPaths.push_back(*it); } // traverse this path
		}

		// Recreate the absolute filename
		filePath.clear();
		for (FStringList::iterator it = documentPaths.begin(); it != documentPaths.end(); ++it)
		{
			if (!filePath.empty()) filePath.push_back('/');
			filePath += (*it);
		}

		if (filePath.size() < 2 || (filePath[1] != ':' && filePath[1] != '|'))
		{
			filePath.insert(0, "/");//'/');
		}
	}

#ifdef WIN32
	std::replace(filePath.begin(), filePath.end(), '/', '\\');
#endif // WIN32

	return filePath;
}

fstring FUFileManager::MakeFilePathRelative(const fstring& _filePath)
{
	fstring filePath = _filePath;
	if (!filePath.empty() && filePath[0] != '.')
	{
		// First, ensure we have an absolute file path
		filePath = MakeFilePathAbsolute(_filePath);

		// Relative file path.
		FStringList documentPaths, localPaths;
		ExtractPathStack(pathStack.back(), documentPaths, true);
		ExtractPathStack(filePath, localPaths, true);

		// Extract the filename from the path stack
		fstring filename = localPaths.back();
		localPaths.pop_back();

		// Look for commonality in the path stacks
		size_t documentPathCount = documentPaths.size();
		size_t filePathCount = localPaths.size();
		size_t matchIndex = 0;
		for (; matchIndex < filePathCount && matchIndex < documentPathCount; ++matchIndex)
		{
			if (fstricmp(documentPaths[matchIndex].c_str(), localPaths[matchIndex].c_str()) != 0) break;
		}

		if (matchIndex != 0)
		{
			// There is some similar part, so generate the relative filename
			fstring relativePath;

			if (documentPathCount > matchIndex)
			{
				// Backtrack the document's path
				for (size_t i = matchIndex; i < documentPathCount; ++i)
				{
					relativePath += fstring(FC("../"));
				}
			}
			else
			{
				// Start at the document's root folder
				relativePath = FC("./");
			}

			// Add the file's relative path
			for (size_t i = matchIndex; i < filePathCount; ++i)
			{
				relativePath += filePath[i] + fstring(FC("/"));
			}
			filePath = relativePath + filename;
		}
	}
	return filePath;
}
