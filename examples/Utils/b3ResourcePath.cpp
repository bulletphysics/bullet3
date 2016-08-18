#include "b3ResourcePath.h"
#include "Bullet3Common/b3Logging.h"
#ifdef __APPLE__
#include <mach-o/dyld.h>	/* _NSGetExecutablePath */
#else
#ifdef _WIN32
#include <windows.h>
#else
//not Mac, not Windows, let's cross the fingers it is Linux :-)
#include <unistd.h>
#endif
#endif


#include "Bullet3Common/b3FileUtils.h"
#define B3_MAX_EXE_PATH_LEN 4096

int b3ResourcePath::getExePath(char* path, int maxPathLenInBytes)
{
	int numBytes = 0;

#if __APPLE__
 uint32_t  bufsize = uint32_t(maxPathLenInBytes);

	if (_NSGetExecutablePath(path, &bufsize)!=0)
	{
		b3Warning("Cannot find executable path\n");
		return false;
	} else
	{
		numBytes = strlen(path);
	}
#else
#ifdef _WIN32
	//https://msdn.microsoft.com/en-us/library/windows/desktop/ms683197(v=vs.85).aspx

	HMODULE hModule = GetModuleHandle(NULL);
	numBytes = GetModuleFileNameA(hModule, path, maxPathLenInBytes);

#else
	///http://stackoverflow.com/questions/933850/how-to-find-the-location-of-the-executable-in-c
	numBytes = (int)readlink("/proc/self/exe", path, maxPathLenInBytes-1);
	if (numBytes > 0) 
	{
		path[numBytes] = 0;
	} else
	{
		b3Warning("Cannot find executable path\n");
	}
#endif //_WIN32
#endif //__APPLE__

	return numBytes;
}

int b3ResourcePath::findResourcePath(const char* resourceName, char* resourcePath, int resourcePathMaxNumBytes)
{
	//first find in a resource/<exeName> location, then in various folders within 'data' using b3FileUtils
	char exePath[B3_MAX_EXE_PATH_LEN];

	int l = b3ResourcePath::getExePath(exePath, B3_MAX_EXE_PATH_LEN);
	if (l)
	{
 		char pathToExe[B3_MAX_EXE_PATH_LEN];

        	int exeNamePos = b3FileUtils::extractPath(exePath,pathToExe,B3_MAX_EXE_PATH_LEN);
        	if (exeNamePos)
        	{
				sprintf(resourcePath,"%s../data/%s",pathToExe,resourceName);
				//printf("try resource at %s\n", resourcePath);	
				if (b3FileUtils::findFile(resourcePath, resourcePath, resourcePathMaxNumBytes))
				{
					return strlen(resourcePath);
				}

				sprintf(resourcePath,"%s../resources/%s/%s",pathToExe,&exePath[exeNamePos],resourceName);
				//printf("try resource at %s\n", resourcePath);	
				if (b3FileUtils::findFile(resourcePath, resourcePath, resourcePathMaxNumBytes))
				{
					return strlen(resourcePath);
				}
         sprintf(resourcePath,"%s.runfiles/google3/third_party/bullet/data/%s",exePath,resourceName);
				//printf("try resource at %s\n", resourcePath);	
				if (b3FileUtils::findFile(resourcePath, resourcePath, resourcePathMaxNumBytes))
				{
					return strlen(resourcePath);
				}  
        	}
	}

	bool res = b3FileUtils::findFile(resourceName, resourcePath, resourcePathMaxNumBytes);
	if (res)
        {
                return strlen(resourcePath);
        }

	return 0;
}

