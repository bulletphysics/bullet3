//========= Copyright Valve Corporation ============//
#pragma once

#include <string>

/** Returns the path (including filename) to the current executable */
std::string Path_GetExecutablePath();

/** Returns the path of the current working directory */
std::string Path_GetWorkingDirectory();

/** Sets the path of the current working directory. Returns true if this was successful. */
bool Path_SetWorkingDirectory( const std::string & sPath );

/** returns the path (including filename) of the current shared lib or DLL */
std::string Path_GetModulePath();

/** Returns the specified path without its filename.
* If slash is unspecified the native path separator of the current platform
* will be used. */
std::string Path_StripFilename( const std::string & sPath, char slash = 0 );

/** returns just the filename from the provided full or relative path. */
std::string Path_StripDirectory( const std::string & sPath, char slash = 0 );

/** returns just the filename with no extension of the provided filename. 
* If there is a path the path is left intact. */
std::string Path_StripExtension( const std::string & sPath );

/** Returns true if the path is absolute */
bool Path_IsAbsolute( const std::string & sPath );

/** Makes an absolute path from a relative path and a base path */
std::string Path_MakeAbsolute( const std::string & sRelativePath, const std::string & sBasePath, char slash = 0 );

/** Fixes the directory separators for the current platform.
* If slash is unspecified the native path separator of the current platform
* will be used. */
std::string Path_FixSlashes( const std::string & sPath, char slash = 0 );

/** Returns the path separator for the current platform */
char Path_GetSlash();

/** Jams two paths together with the right kind of slash */
std::string Path_Join( const std::string & first, const std::string & second, char slash = 0 );
std::string Path_Join( const std::string & first, const std::string & second, const std::string & third, char slash = 0 );
std::string Path_Join( const std::string & first, const std::string & second, const std::string & third, const std::string &fourth, char slash = 0 );
std::string Path_Join( 
	const std::string & first, 
	const std::string & second, 
	const std::string & third, 
	const std::string & fourth, 
	const std::string & fifth, 
	char slash = 0 );


/** Removes redundant <dir>/.. elements in the path. Returns an empty path if the 
* specified path has a broken number of directories for its number of ..s.
* If slash is unspecified the native path separator of the current platform
* will be used. */
std::string Path_Compact( const std::string & sRawPath, char slash = 0 );

/** returns true if the specified path exists and is a directory */
bool Path_IsDirectory( const std::string & sPath );

/** Returns the path to the current DLL or exe */
std::string GetThisModulePath();

/** returns true if the the path exists */
bool Path_Exists( const std::string & sPath );

/** Helper functions to find parent directories or subdirectories of parent directories */
std::string Path_FindParentDirectoryRecursively( const std::string &strStartDirectory, const std::string &strDirectoryName );
std::string Path_FindParentSubDirectoryRecursively( const std::string &strStartDirectory, const std::string &strDirectoryName );

/** Path operations to read or write text/binary files */
unsigned char * Path_ReadBinaryFile( const std::string &strFilename, int *pSize );
std::string Path_ReadTextFile( const std::string &strFilename );
bool Path_WriteStringToTextFile( const std::string &strFilename, const char *pchData );

//-----------------------------------------------------------------------------
#if defined(_WIN32)
#define DYNAMIC_LIB_EXT	".dll"
#ifdef _WIN64
#define PLATSUBDIR	"win64"
#else
#define PLATSUBDIR	"win32"
#endif
#elif defined(OSX)
#define DYNAMIC_LIB_EXT	".dylib"
#define PLATSUBDIR	"osx32"
#elif defined(LINUX)
#define DYNAMIC_LIB_EXT	".so"
#define PLATSUBDIR	"linux32"
#else
#warning "Unknown platform for PLATSUBDIR"
#define PLATSUBDIR	"unknown_platform"
#endif
