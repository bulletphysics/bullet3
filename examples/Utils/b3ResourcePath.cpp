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

struct TempResourcePath
{
	char* m_path;
	TempResourcePath(int len)
	{
		m_path = (char*)malloc(len);
		memset(m_path,0,len);
	}
	virtual ~TempResourcePath()
	{
		free(m_path);
	}
};

static char sAdditionalSearchPath[B3_MAX_EXE_PATH_LEN] = {0};

void b3ResourcePath::setAdditionalSearchPath(const char* path)
{
	if (path)
	{
		int len = strlen(path);
		if (len<(B3_MAX_EXE_PATH_LEN-1))
		{
			strcpy(sAdditionalSearchPath,path);
			sAdditionalSearchPath[len] = 0;
		}
	} else
	{
		sAdditionalSearchPath[0] = 0;
	}
}

int b3ResourcePath::findResourcePath(const char* resourceName, char* resourcePathOut, int resourcePathMaxNumBytes)
{
	//first find in a resource/<exeName> location, then in various folders within 'data' using b3FileUtils
	char exePath[B3_MAX_EXE_PATH_LEN];

	bool res = b3FileUtils::findFile(resourceName, resourcePathOut, resourcePathMaxNumBytes);
	if (res)
    {
            return strlen(resourcePathOut);
    }

	if (sAdditionalSearchPath[0])
	{
		TempResourcePath tmpPath(resourcePathMaxNumBytes+1024);
		char* resourcePathIn = tmpPath.m_path;
		sprintf(resourcePathIn,"%s/%s",sAdditionalSearchPath,resourceName);
		//printf("try resource at %s\n", resourcePath);	
		if (b3FileUtils::findFile(resourcePathIn, resourcePathOut, resourcePathMaxNumBytes))
		{
			return strlen(resourcePathOut);
		}
	}

	int l = b3ResourcePath::getExePath(exePath, B3_MAX_EXE_PATH_LEN);
	if (l)
	{
 		char pathToExe[B3_MAX_EXE_PATH_LEN];

        	int exeNamePos = b3FileUtils::extractPath(exePath,pathToExe,B3_MAX_EXE_PATH_LEN);
        	if (exeNamePos)
        	{
				TempResourcePath tmpPath(resourcePathMaxNumBytes+1024);
				char* resourcePathIn = tmpPath.m_path;
				sprintf(resourcePathIn,"%s../data/%s",pathToExe,resourceName);
				//printf("try resource at %s\n", resourcePath);	
				if (b3FileUtils::findFile(resourcePathIn, resourcePathOut, resourcePathMaxNumBytes))
				{
					return strlen(resourcePathOut);
				}

				sprintf(resourcePathIn,"%s../resources/%s/%s",pathToExe,&exePath[exeNamePos],resourceName);
				//printf("try resource at %s\n", resourcePath);	
				if (b3FileUtils::findFile(resourcePathIn, resourcePathOut, resourcePathMaxNumBytes))
				{
					return strlen(resourcePathOut);
				}
				sprintf(resourcePathIn,"%s.runfiles/google3/third_party/bullet/data/%s",exePath,resourceName);
				//printf("try resource at %s\n", resourcePath);	
				if (b3FileUtils::findFile(resourcePathIn, resourcePathOut, resourcePathMaxNumBytes))
				{
					return strlen(resourcePathOut);
				}  
        	}
	}



	return 0;
}

