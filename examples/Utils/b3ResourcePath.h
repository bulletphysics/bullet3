#ifndef _B3_RESOURCE_PATH_H
#define _B3_RESOURCE_PATH_H

#include <string>

typedef bool (* PFN_FIND_FILE)(void* userPointer, const char* orgFileName, char* relativeFileName, int maxRelativeFileNameMaxLen);

class b3ResourcePath
{
public:
	static int getExePath(char* path, int maxPathLenInBytes);
	static int findResourcePath(const char* resourceName, char* resourcePathOut, int resourcePathMaxNumBytes, PFN_FIND_FILE findFile, void* userPointer=0);
	static void setAdditionalSearchPath(const char* path);
};
#endif
