#ifndef _B3_RESOURCE_PATH_H
#define _B3_RESOURCE_PATH_H 

#include <string>

class b3ResourcePath
{
public:
	static int getExePath(char* path, int maxPathLenInBytes);
	static int findResourcePath(const char* sourceName, char* resourcePath, int maxResourcePathLenInBytes);
};
#endif

