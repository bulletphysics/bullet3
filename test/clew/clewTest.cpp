// clewTest.cpp : Defines the entry point for the console application.
//

#include "clew.h"
#include <stdio.h>

int main(int argc, char* argv[])
{
	int result = -1;

#ifdef _WIN32
	const char* cl = "OpenCL.dll";
#elif defined __APPLE__
	const char* cl = "/System/Library/Frameworks/OpenCL.framework/Versions/Current/OpenCL";
#else  //presumable Linux?
	//linux (tested on Ubuntu 12.10 with Catalyst 13.4 beta drivers, not that there is no symbolic link from libOpenCL.so
	const char* cl = "libOpenCL.so.1";
	result = clewInit(cl);
	if (result != CLEW_SUCCESS)
	{
		cl = "libOpenCL.so";
	}
	else
	{
		clewExit();
	}
#endif
	result = clewInit(cl);
	if (result != CLEW_SUCCESS)
		printf("clewInit failed with error code %d\n", result);
	else
	{
		printf("clewInit succesfull using %s\n", cl);

		//some test and then
		clewExit();
	}

	return 0;
}
