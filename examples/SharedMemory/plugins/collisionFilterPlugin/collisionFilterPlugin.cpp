
//tinyRendererPlugin implements the TinyRenderer as a plugin
//it is statically linked when using preprocessor #define STATIC_LINK_VR_PLUGIN 
//otherwise you can dynamically load it using pybullet.loadPlugin

#include "collisionFilterPlugin.h"
#include "../../SharedMemoryPublic.h"
#include "../b3PluginContext.h"
#include <stdio.h>

struct CollisionFilterMyClass
{
	int m_testData;

	CollisionFilterMyClass()
		:m_testData(42)
	{
	}
	virtual ~CollisionFilterMyClass()
	{
	}
};

B3_SHARED_API int initPlugin_collisionFilterPlugin(struct b3PluginContext* context)
{
	CollisionFilterMyClass* obj = new CollisionFilterMyClass();
	context->m_userPointer = obj;

	printf("hi!\n");
	return SHARED_MEMORY_MAGIC_NUMBER;
}


B3_SHARED_API int preTickPluginCallback_collisionFilterPlugin(struct b3PluginContext* context)
{
	//apply pd control here, apply forces using the PD gains
	return 0;
}


B3_SHARED_API int postTickPluginCallback_collisionFilterPlugin(struct b3PluginContext* context)
{
	CollisionFilterMyClass* obj = (CollisionFilterMyClass* )context->m_userPointer;
	obj->m_testData++;
	return 0;
}

B3_SHARED_API int executePluginCommand_collisionFilterPlugin(struct b3PluginContext* context, const struct b3PluginArguments* arguments)
{
	//set the PD gains
	printf("text argument:%s\n",arguments->m_text);
	printf("int args: [");
	for (int i=0;i<arguments->m_numInts;i++)
	{
		printf("%d", arguments->m_ints[i]);
		if ((i+1)<arguments->m_numInts)
		{
			printf(",");
		}
	}
	printf("]\nfloat args: [");
	for (int i=0;i<arguments->m_numFloats;i++)
	{
		printf("%f", arguments->m_floats[i]);
		if ((i+1)<arguments->m_numFloats)
		{
			printf(",");
		}
	}
	printf("]\n");

	CollisionFilterMyClass* obj = (CollisionFilterMyClass*) context->m_userPointer;
	
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


B3_SHARED_API void exitPlugin_collisionFilterPlugin(struct b3PluginContext* context)
{
	CollisionFilterMyClass* obj = (CollisionFilterMyClass*) context->m_userPointer;
	delete obj;
	context->m_userPointer = 0;

	printf("bye!\n");
}
