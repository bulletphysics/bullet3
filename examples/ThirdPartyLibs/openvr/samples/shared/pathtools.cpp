//========= Copyright Valve Corporation ============//
#include "compat.h"
#include "strtools.h"
#include "pathtools.h"

#if defined(_WIN32)
#include <windows.h>
#include <direct.h>
#include <shobjidl.h>
#include <knownfolders.h>
#include <shlobj.h>
#include <share.h>

#undef GetEnvironmentVariable
#else
#include <dlfcn.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#endif
#if defined OSX
#include <Foundation/Foundation.h>
#include <AppKit/AppKit.h>
#include <mach-o/dyld.h>
#define _S_IFDIR S_IFDIR  // really from tier0/platform.h which we dont have yet
#endif

#include <sys/stat.h>

#include <algorithm>

/** Returns the path (including filename) to the current executable */
std::string Path_GetExecutablePath()
{
#if defined(_WIN32)
	wchar_t *pwchPath = new wchar_t[MAX_UNICODE_PATH];
	char *pchPath = new char[MAX_UNICODE_PATH_IN_UTF8];
	::GetModuleFileNameW(NULL, pwchPath, MAX_UNICODE_PATH);
	WideCharToMultiByte(CP_UTF8, 0, pwchPath, -1, pchPath, MAX_UNICODE_PATH_IN_UTF8, NULL, NULL);
	delete[] pwchPath;

	std::string sPath = pchPath;
	delete[] pchPath;
	return sPath;
#elif defined(OSX)
	char rchPath[1024];
	uint32_t nBuff = sizeof(rchPath);
	bool bSuccess = _NSGetExecutablePath(rchPath, &nBuff) == 0;
	rchPath[nBuff - 1] = '\0';
	if (bSuccess)
		return rchPath;
	else
		return "";
#elif defined LINUX
	char rchPath[1024];
	size_t nBuff = sizeof(rchPath);
	ssize_t nRead = readlink("/proc/self/exe", rchPath, nBuff - 1);
	if (nRead != -1)
	{
		rchPath[nRead] = 0;
		return rchPath;
	}
	else
	{
		return "";
	}
#else
	AssertMsg(false, "Implement Plat_GetExecutablePath");
	return "";
#endif
}

/** Returns the path of the current working directory */
std::string Path_GetWorkingDirectory()
{
	std::string sPath;
#if defined(_WIN32)
	wchar_t buf[MAX_UNICODE_PATH];
	sPath = UTF16to8(_wgetcwd(buf, MAX_UNICODE_PATH));
#else
	char buf[1024];
	sPath = getcwd(buf, sizeof(buf));
#endif
	return sPath;
}

/** Sets the path of the current working directory. Returns true if this was successful. */
bool Path_SetWorkingDirectory(const std::string &sPath)
{
	bool bSuccess;
#if defined(_WIN32)
	std::wstring wsPath = UTF8to16(sPath.c_str());
	bSuccess = 0 == _wchdir(wsPath.c_str());
#else
	bSuccess = 0 == chdir(sPath.c_str());
#endif
	return bSuccess;
}

/** Returns the specified path without its filename */
std::string Path_StripFilename(const std::string &sPath, char slash)
{
	if (slash == 0)
		slash = Path_GetSlash();

	std::string::size_type n = sPath.find_last_of(slash);
	if (n == std::string::npos)
		return sPath;
	else
		return std::string(sPath.begin(), sPath.begin() + n);
}

/** returns just the filename from the provided full or relative path. */
std::string Path_StripDirectory(const std::string &sPath, char slash)
{
	if (slash == 0)
		slash = Path_GetSlash();

	std::string::size_type n = sPath.find_last_of(slash);
	if (n == std::string::npos)
		return sPath;
	else
		return std::string(sPath.begin() + n + 1, sPath.end());
}

/** returns just the filename with no extension of the provided filename. 
* If there is a path the path is left intact. */
std::string Path_StripExtension(const std::string &sPath)
{
	for (std::string::const_reverse_iterator i = sPath.rbegin(); i != sPath.rend(); i++)
	{
		if (*i == '.')
		{
			return std::string(sPath.begin(), i.base() - 1);
		}

		// if we find a slash there is no extension
		if (*i == '\\' || *i == '/')
			break;
	}

	// we didn't find an extension
	return sPath;
}

/** returns just extension of the provided filename (if any). */
std::string Path_GetExtension(const std::string &sPath)
{
	for (std::string::const_reverse_iterator i = sPath.rbegin(); i != sPath.rend(); i++)
	{
		if (*i == '.')
		{
			return std::string(i.base(), sPath.end());
		}

		// if we find a slash there is no extension
		if (*i == '\\' || *i == '/')
			break;
	}

	// we didn't find an extension
	return "";
}

bool Path_IsAbsolute(const std::string &sPath)
{
	if (sPath.empty())
		return false;

#if defined(WIN32)
	if (sPath.size() < 3)  // must be c:\x or \\x at least
		return false;

	if (sPath[1] == ':')  // drive letter plus slash, but must test both slash cases
	{
		if (sPath[2] == '\\' || sPath[2] == '/')
			return true;
	}
	else if (sPath[0] == '\\' && sPath[1] == '\\')  // UNC path
		return true;
#else
	if (sPath[0] == '\\' || sPath[0] == '/')  // any leading slash
		return true;
#endif

	return false;
}

/** Makes an absolute path from a relative path and a base path */
std::string Path_MakeAbsolute(const std::string &sRelativePath, const std::string &sBasePath, char slash)
{
	if (slash == 0)
		slash = Path_GetSlash();

	if (Path_IsAbsolute(sRelativePath))
		return sRelativePath;
	else
	{
		if (!Path_IsAbsolute(sBasePath))
			return "";

		std::string sCompacted = Path_Compact(Path_Join(sBasePath, sRelativePath, slash), slash);
		if (Path_IsAbsolute(sCompacted))
			return sCompacted;
		else
			return "";
	}
}

/** Fixes the directory separators for the current platform */
std::string Path_FixSlashes(const std::string &sPath, char slash)
{
	if (slash == 0)
		slash = Path_GetSlash();

	std::string sFixed = sPath;
	for (std::string::iterator i = sFixed.begin(); i != sFixed.end(); i++)
	{
		if (*i == '/' || *i == '\\')
			*i = slash;
	}

	return sFixed;
}

char Path_GetSlash()
{
#if defined(_WIN32)
	return '\\';
#else
	return '/';
#endif
}

/** Jams two paths together with the right kind of slash */
std::string Path_Join(const std::string &first, const std::string &second, char slash)
{
	if (slash == 0)
		slash = Path_GetSlash();

	// only insert a slash if we don't already have one
	std::string::size_type nLen = first.length();
	if (!nLen)
		return second;
#if defined(_WIN32)
	if (first.back() == '\\' || first.back() == '/')
		nLen--;
#else
	char last_char = first[first.length() - 1];
	if (last_char == '\\' || last_char == '/')
		nLen--;
#endif

	return first.substr(0, nLen) + std::string(1, slash) + second;
}

std::string Path_Join(const std::string &first, const std::string &second, const std::string &third, char slash)
{
	return Path_Join(Path_Join(first, second, slash), third, slash);
}

std::string Path_Join(const std::string &first, const std::string &second, const std::string &third, const std::string &fourth, char slash)
{
	return Path_Join(Path_Join(Path_Join(first, second, slash), third, slash), fourth, slash);
}

std::string Path_Join(
	const std::string &first,
	const std::string &second,
	const std::string &third,
	const std::string &fourth,
	const std::string &fifth,
	char slash)
{
	return Path_Join(Path_Join(Path_Join(Path_Join(first, second, slash), third, slash), fourth, slash), fifth, slash);
}

std::string Path_RemoveTrailingSlash(const std::string &sRawPath, char slash)
{
	if (slash == 0)
		slash = Path_GetSlash();

	std::string sPath = sRawPath;
	std::string::size_type nCurrent = sRawPath.length();
	if (nCurrent == 0)
		return sPath;

	int nLastFound = -1;
	nCurrent--;
	while (nCurrent != 0)
	{
		if (sRawPath[nCurrent] == slash)
		{
			nLastFound = (int)nCurrent;
			nCurrent--;
		}
		else
		{
			break;
		}
	}

	if (nLastFound >= 0)
	{
		sPath.erase(nLastFound, std::string::npos);
	}

	return sPath;
}

/** Removes redundant <dir>/.. elements in the path. Returns an empty path if the 
* specified path has a broken number of directories for its number of ..s */
std::string Path_Compact(const std::string &sRawPath, char slash)
{
	if (slash == 0)
		slash = Path_GetSlash();

	std::string sPath = Path_FixSlashes(sRawPath, slash);
	std::string sSlashString(1, slash);

	// strip out all /./
	for (std::string::size_type i = 0; (i + 3) < sPath.length();)
	{
		if (sPath[i] == slash && sPath[i + 1] == '.' && sPath[i + 2] == slash)
		{
			sPath.replace(i, 3, sSlashString);
		}
		else
		{
			++i;
		}
	}

	// get rid of trailing /. but leave the path separator
	if (sPath.length() > 2)
	{
		std::string::size_type len = sPath.length();
		if (sPath[len - 1] == '.' && sPath[len - 2] == slash)
		{
			// sPath.pop_back();
			sPath[len - 1] = 0;  // for now, at least
		}
	}

	// get rid of leading ./
	if (sPath.length() > 2)
	{
		if (sPath[0] == '.' && sPath[1] == slash)
		{
			sPath.replace(0, 2, "");
		}
	}

	// each time we encounter .. back up until we've found the previous directory name
	// then get rid of both
	std::string::size_type i = 0;
	while (i < sPath.length())
	{
		if (i > 0 && sPath.length() - i >= 2 && sPath[i] == '.' && sPath[i + 1] == '.' && (i + 2 == sPath.length() || sPath[i + 2] == slash) && sPath[i - 1] == slash)
		{
			// check if we've hit the start of the string and have a bogus path
			if (i == 1)
				return "";

			// find the separator before i-1
			std::string::size_type iDirStart = i - 2;
			while (iDirStart > 0 && sPath[iDirStart - 1] != slash)
				--iDirStart;

			// remove everything from iDirStart to i+2
			sPath.replace(iDirStart, (i - iDirStart) + 3, "");

			// start over
			i = 0;
		}
		else
		{
			++i;
		}
	}

	return sPath;
}

/** Returns the path to the current DLL or exe */
std::string Path_GetThisModulePath()
{
	// gets the path of vrclient.dll itself
#ifdef WIN32
	HMODULE hmodule = NULL;

	::GetModuleHandleEx(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, reinterpret_cast<LPCTSTR>(Path_GetThisModulePath), &hmodule);

	wchar_t *pwchPath = new wchar_t[MAX_UNICODE_PATH];
	char *pchPath = new char[MAX_UNICODE_PATH_IN_UTF8];
	::GetModuleFileNameW(hmodule, pwchPath, MAX_UNICODE_PATH);
	WideCharToMultiByte(CP_UTF8, 0, pwchPath, -1, pchPath, MAX_UNICODE_PATH_IN_UTF8, NULL, NULL);
	delete[] pwchPath;

	std::string sPath = pchPath;
	delete[] pchPath;
	return sPath;

#elif defined(OSX) || defined(LINUX)
	// get the addr of a function in vrclient.so and then ask the dlopen system about it
	Dl_info info;
	dladdr((void *)Path_GetThisModulePath, &info);
	return info.dli_fname;
#endif
}

/** returns true if the specified path exists and is a directory */
bool Path_IsDirectory(const std::string &sPath)
{
	std::string sFixedPath = Path_FixSlashes(sPath);
	if (sFixedPath.empty())
		return false;
	char cLast = sFixedPath[sFixedPath.length() - 1];
	if (cLast == '/' || cLast == '\\')
		sFixedPath.erase(sFixedPath.end() - 1, sFixedPath.end());

		// see if the specified path actually exists.

#if defined(POSIX)
	struct stat buf;
	if (stat(sFixedPath.c_str(), &buf) == -1)
	{
		return false;
	}

#if defined(LINUX) || defined(OSX)
	return S_ISDIR(buf.st_mode);
#else
	return (buf.st_mode & _S_IFDIR) != 0;
#endif

#else
	struct _stat buf;
	std::wstring wsFixedPath = UTF8to16(sFixedPath.c_str());
	if (_wstat(wsFixedPath.c_str(), &buf) == -1)
	{
		return false;
	}

	return (buf.st_mode & _S_IFDIR) != 0;
#endif
}

/** returns true if the specified path represents an app bundle */
bool Path_IsAppBundle(const std::string &sPath)
{
#if defined(OSX)
	NSBundle *bundle = [NSBundle bundleWithPath:[NSString stringWithUTF8String:sPath.c_str()]];
	bool bisAppBundle = (nullptr != bundle);
	[bundle release];
	return bisAppBundle;
#else
	return false;
#endif
}

//-----------------------------------------------------------------------------
// Purpose: returns true if the the path exists
//-----------------------------------------------------------------------------
bool Path_Exists(const std::string &sPath)
{
	std::string sFixedPath = Path_FixSlashes(sPath);
	if (sFixedPath.empty())
		return false;

#if defined(WIN32)
	struct _stat buf;
	std::wstring wsFixedPath = UTF8to16(sFixedPath.c_str());
	if (_wstat(wsFixedPath.c_str(), &buf) == -1)
	{
		return false;
	}
#else
	struct stat buf;
	if (stat(sFixedPath.c_str(), &buf) == -1)
	{
		return false;
	}
#endif

	return true;
}

//-----------------------------------------------------------------------------
// Purpose: helper to find a directory upstream from a given path
//-----------------------------------------------------------------------------
std::string Path_FindParentDirectoryRecursively(const std::string &strStartDirectory, const std::string &strDirectoryName)
{
	std::string strFoundPath = "";
	std::string strCurrentPath = Path_FixSlashes(strStartDirectory);
	if (strCurrentPath.length() == 0)
		return "";

	bool bExists = Path_Exists(strCurrentPath);
	std::string strCurrentDirectoryName = Path_StripDirectory(strCurrentPath);
	if (bExists && stricmp(strCurrentDirectoryName.c_str(), strDirectoryName.c_str()) == 0)
		return strCurrentPath;

	while (bExists && strCurrentPath.length() != 0)
	{
		strCurrentPath = Path_StripFilename(strCurrentPath);
		strCurrentDirectoryName = Path_StripDirectory(strCurrentPath);
		bExists = Path_Exists(strCurrentPath);
		if (bExists && stricmp(strCurrentDirectoryName.c_str(), strDirectoryName.c_str()) == 0)
			return strCurrentPath;
	}

	return "";
}

//-----------------------------------------------------------------------------
// Purpose: helper to find a subdirectory upstream from a given path
//-----------------------------------------------------------------------------
std::string Path_FindParentSubDirectoryRecursively(const std::string &strStartDirectory, const std::string &strDirectoryName)
{
	std::string strFoundPath = "";
	std::string strCurrentPath = Path_FixSlashes(strStartDirectory);
	if (strCurrentPath.length() == 0)
		return "";

	bool bExists = Path_Exists(strCurrentPath);
	while (bExists && strCurrentPath.length() != 0)
	{
		strCurrentPath = Path_StripFilename(strCurrentPath);
		bExists = Path_Exists(strCurrentPath);

		if (Path_Exists(Path_Join(strCurrentPath, strDirectoryName)))
		{
			strFoundPath = Path_Join(strCurrentPath, strDirectoryName);
			break;
		}
	}
	return strFoundPath;
}

//-----------------------------------------------------------------------------
// Purpose: reading and writing files in the vortex directory
//-----------------------------------------------------------------------------
unsigned char *Path_ReadBinaryFile(const std::string &strFilename, int *pSize)
{
	FILE *f;
#if defined(POSIX)
	f = fopen(strFilename.c_str(), "rb");
#else
	std::wstring wstrFilename = UTF8to16(strFilename.c_str());
	// the open operation needs to be sharable, therefore use of _wfsopen instead of _wfopen_s
	f = _wfsopen(wstrFilename.c_str(), L"rb", _SH_DENYNO);
#endif

	unsigned char *buf = NULL;

	if (f != NULL)
	{
		fseek(f, 0, SEEK_END);
		int size = ftell(f);
		fseek(f, 0, SEEK_SET);

		buf = new unsigned char[size];
		if (buf && fread(buf, size, 1, f) == 1)
		{
			if (pSize)
				*pSize = size;
		}
		else
		{
			delete[] buf;
			buf = 0;
		}

		fclose(f);
	}

	return buf;
}

uint32_t Path_ReadBinaryFile(const std::string &strFilename, unsigned char *pBuffer, uint32_t unSize)
{
	FILE *f;
#if defined(POSIX)
	f = fopen(strFilename.c_str(), "rb");
#else
	std::wstring wstrFilename = UTF8to16(strFilename.c_str());
	errno_t err = _wfopen_s(&f, wstrFilename.c_str(), L"rb");
	if (err != 0)
	{
		f = NULL;
	}
#endif

	uint32_t unSizeToReturn = 0;

	if (f != NULL)
	{
		fseek(f, 0, SEEK_END);
		uint32_t size = (uint32_t)ftell(f);
		fseek(f, 0, SEEK_SET);

		if (size > unSize || !pBuffer)
		{
			unSizeToReturn = (uint32_t)size;
		}
		else
		{
			if (fread(pBuffer, size, 1, f) == 1)
			{
				unSizeToReturn = (uint32_t)size;
			}
		}

		fclose(f);
	}

	return unSizeToReturn;
}

bool Path_WriteBinaryFile(const std::string &strFilename, unsigned char *pData, unsigned nSize)
{
	FILE *f;
#if defined(POSIX)
	f = fopen(strFilename.c_str(), "wb");
#else
	std::wstring wstrFilename = UTF8to16(strFilename.c_str());
	errno_t err = _wfopen_s(&f, wstrFilename.c_str(), L"wb");
	if (err != 0)
	{
		f = NULL;
	}
#endif

	size_t written = 0;
	if (f != NULL)
	{
		written = fwrite(pData, sizeof(unsigned char), nSize, f);
		fclose(f);
	}

	return written = nSize ? true : false;
}

std::string Path_ReadTextFile(const std::string &strFilename)
{
	// doing it this way seems backwards, but I don't
	// see an easy way to do this with C/C++ style IO
	// that isn't worse...
	int size;
	unsigned char *buf = Path_ReadBinaryFile(strFilename, &size);
	if (!buf)
		return "";

	// convert CRLF -> LF
	size_t outsize = 1;
	for (int i = 1; i < size; i++)
	{
		if (buf[i] == '\n' && buf[i - 1] == '\r')  // CRLF
			buf[outsize - 1] = '\n';               // ->LF
		else
			buf[outsize++] = buf[i];  // just copy
	}

	std::string ret((char *)buf, outsize);
	delete[] buf;
	return ret;
}

bool Path_WriteStringToTextFile(const std::string &strFilename, const char *pchData)
{
	FILE *f;
#if defined(POSIX)
	f = fopen(strFilename.c_str(), "w");
#else
	std::wstring wstrFilename = UTF8to16(strFilename.c_str());
	errno_t err = _wfopen_s(&f, wstrFilename.c_str(), L"w");
	if (err != 0)
	{
		f = NULL;
	}
#endif

	bool ok = false;

	if (f != NULL)
	{
		ok = fputs(pchData, f) >= 0;
		fclose(f);
	}

	return ok;
}

bool Path_WriteStringToTextFileAtomic(const std::string &strFilename, const char *pchData)
{
	std::string strTmpFilename = strFilename + ".tmp";

	if (!Path_WriteStringToTextFile(strTmpFilename, pchData))
		return false;

		// Platform specific atomic file replacement
#if defined(_WIN32)
	std::wstring wsFilename = UTF8to16(strFilename.c_str());
	std::wstring wsTmpFilename = UTF8to16(strTmpFilename.c_str());
	if (!::ReplaceFileW(wsFilename.c_str(), wsTmpFilename.c_str(), nullptr, 0, 0, 0))
	{
		// if we couldn't ReplaceFile, try a non-atomic write as a fallback
		if (!Path_WriteStringToTextFile(strFilename, pchData))
			return false;
	}
#elif defined(POSIX)
	if (rename(strTmpFilename.c_str(), strFilename.c_str()) == -1)
		return false;
#else
#error Do not know how to write atomic file
#endif

	return true;
}

#if defined(WIN32)
#define FILE_URL_PREFIX "file:///"
#else
#define FILE_URL_PREFIX "file://"
#endif

// ----------------------------------------------------------------------------------------------------------------------------
// Purpose: Turns a path to a file on disk into a URL (or just returns the value if it's already a URL)
// ----------------------------------------------------------------------------------------------------------------------------
std::string Path_FilePathToUrl(const std::string &sRelativePath, const std::string &sBasePath)
{
	if (!strnicmp(sRelativePath.c_str(), "http://", 7) || !strnicmp(sRelativePath.c_str(), "https://", 8) || !strnicmp(sRelativePath.c_str(), "file://", 7))
	{
		return sRelativePath;
	}
	else
	{
		std::string sAbsolute = Path_MakeAbsolute(sRelativePath, sBasePath);
		if (sAbsolute.empty())
			return sAbsolute;
		return std::string(FILE_URL_PREFIX) + sAbsolute;
	}
}

// -----------------------------------------------------------------------------------------------------
// Purpose: Strips off file:// off a URL and returns the path. For other kinds of URLs an empty string is returned
// -----------------------------------------------------------------------------------------------------
std::string Path_UrlToFilePath(const std::string &sFileUrl)
{
	if (!strnicmp(sFileUrl.c_str(), FILE_URL_PREFIX, strlen(FILE_URL_PREFIX)))
	{
		return sFileUrl.c_str() + strlen(FILE_URL_PREFIX);
	}
	else
	{
		return "";
	}
}

// -----------------------------------------------------------------------------------------------------
// Purpose: Returns the root of the directory the system wants us to store user documents in
// -----------------------------------------------------------------------------------------------------
std::string GetUserDocumentsPath()
{
#if defined(WIN32)
	WCHAR rwchPath[MAX_PATH];

	if (!SUCCEEDED(SHGetFolderPathW(NULL, CSIDL_MYDOCUMENTS | CSIDL_FLAG_CREATE, NULL, 0, rwchPath)))
	{
		return "";
	}

	// Convert the path to UTF-8 and store in the output
	std::string sUserPath = UTF16to8(rwchPath);

	return sUserPath;
#elif defined(OSX)
	@autoreleasepool
	{
		NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
		if ([paths count] == 0)
		{
			return "";
		}

		return [[paths objectAtIndex:0] UTF8String];
	}
#elif defined(LINUX)
	// @todo: not solved/changed as part of OSX - still not real - just removed old class based steam cut and paste
	const char *pchHome = getenv("HOME");
	if (pchHome == NULL)
	{
		return "";
	}
	return pchHome;
#endif
}
