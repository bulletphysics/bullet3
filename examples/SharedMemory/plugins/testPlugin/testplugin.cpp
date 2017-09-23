
#include "testplugin.h"
#include "../../SharedMemoryPublic.h"

#include <stdio.h>

B3_SHARED_API int initPlugin()
{
	printf("hi!\n");
	return SHARED_MEMORY_MAGIC_NUMBER;
}

B3_SHARED_API int executePluginCommand(const char* arguments)
{
	printf("arguments:%s\n",arguments);
	return 42;
}


B3_SHARED_API void exitPlugin()
{
	printf("bye!\n");
}
