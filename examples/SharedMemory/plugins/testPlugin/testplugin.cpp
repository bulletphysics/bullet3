
//test plugin, can load a URDF file, example usage on a Windows machine:

/*
import pybullet as p
p.connect(p.GUI)
pluginUid = p.loadPlugin("E:/develop/bullet3/bin/pybullet_testplugin_vs2010_x64_debug.dll")
commandUid = 0
argument = "plane.urdf"
p.executePluginCommand(pluginUid,commandUid,argument)
p.unloadPlugin(pluginUid)
*/

#include "testplugin.h"
#include "../../SharedMemoryPublic.h"
#include "../b3PluginContext.h"
#include <stdio.h>

B3_SHARED_API int initPlugin()
{
	printf("hi!\n");
	return SHARED_MEMORY_MAGIC_NUMBER;
}

B3_SHARED_API int executePluginCommand(struct b3PluginContext* context)
{
	printf("arguments:%s\n",context->m_arguments);

	b3SharedMemoryStatusHandle statusHandle;
	int statusType = -1;
	int bodyUniqueId = -1;

	b3SharedMemoryCommandHandle command =
		b3LoadUrdfCommandInit(context->m_physClient, context->m_arguments);

	statusHandle = b3SubmitClientCommandAndWaitStatus(context->m_physClient, command);
	statusType = b3GetStatusType(statusHandle);
	if (statusType == CMD_URDF_LOADING_COMPLETED)
	{
		bodyUniqueId = b3GetStatusBodyIndex(statusHandle);	
	}
	return bodyUniqueId;
}


B3_SHARED_API void exitPlugin()
{
	printf("bye!\n");
}
