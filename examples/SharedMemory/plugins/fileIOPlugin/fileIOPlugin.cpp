

#include "fileIOPlugin.h"
#include "../../SharedMemoryPublic.h"
#include "../b3PluginContext.h"
#include <stdio.h>
#include "../../../CommonInterfaces/CommonFileIOInterface.h"
#include "../../../Utils/b3ResourcePath.h"
#include "../../../Utils/b3BulletDefaultFileIO.h"


#include "zipFileIO.h"

//#define B3_USE_ZLIB
#ifdef B3_USE_ZLIB
typedef ZipFileIO MyFileIO;
#else
typedef b3BulletDefaultFileIO MyFileIO;
#endif

struct FileIOClass
{
	int m_testData;

	MyFileIO m_fileIO;

	FileIOClass()
		: m_testData(42),
		m_fileIO()//"e:/develop/bullet3/data/plane.zip")
	{
	}
	virtual ~FileIOClass()
	{
	}
};

B3_SHARED_API int initPlugin_fileIOPlugin(struct b3PluginContext* context)
{
	FileIOClass* obj = new FileIOClass();
	context->m_userPointer = obj;
	return SHARED_MEMORY_MAGIC_NUMBER;
}


B3_SHARED_API int executePluginCommand_fileIOPlugin(struct b3PluginContext* context, const struct b3PluginArguments* arguments)
{
	printf("text argument:%s\n", arguments->m_text);
	printf("int args: [");
	for (int i = 0; i < arguments->m_numInts; i++)
	{
		printf("%d", arguments->m_ints[i]);
		if ((i + 1) < arguments->m_numInts)
		{
			printf(",");
		}
	}
	printf("]\nfloat args: [");
	for (int i = 0; i < arguments->m_numFloats; i++)
	{
		printf("%f", arguments->m_floats[i]);
		if ((i + 1) < arguments->m_numFloats)
		{
			printf(",");
		}
	}
	printf("]\n");

	FileIOClass* obj = (FileIOClass*)context->m_userPointer;

	b3SharedMemoryStatusHandle statusHandle;
	int statusType = -1;
	int bodyUniqueId = -1;

	b3SharedMemoryCommandHandle command =
		b3LoadUrdfCommandInit(context->m_physClient, arguments->m_text);

	statusHandle = b3SubmitClientCommandAndWaitStatus(context->m_physClient, command);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_URDF_LOADING_COMPLETED)
	{
		bodyUniqueId = b3GetStatusBodyIndex(statusHandle);
	}
	return bodyUniqueId;
}

B3_SHARED_API struct CommonFileIOInterface* getFileIOFunc_fileIOPlugin(struct b3PluginContext* context)
{
	FileIOClass* obj = (FileIOClass*)context->m_userPointer;
	return &obj->m_fileIO;
}

B3_SHARED_API void exitPlugin_fileIOPlugin(struct b3PluginContext* context)
{
	FileIOClass* obj = (FileIOClass*)context->m_userPointer;
	delete obj;
	context->m_userPointer = 0;
}
