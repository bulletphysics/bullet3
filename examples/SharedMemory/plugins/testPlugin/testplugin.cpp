
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

struct MyClass
{
	int m_testData;

	MyClass()
		:m_testData(42)
	{
	}
	virtual ~MyClass()
	{
	}
};

B3_SHARED_API int initPlugin(struct b3PluginContext* context)
{
	MyClass* obj = new MyClass();
	context->m_userPointer = obj;

	printf("hi!\n");
	return SHARED_MEMORY_MAGIC_NUMBER;
}


B3_SHARED_API int preTickPluginCallback(struct b3PluginContext* context)
{
	MyClass* obj = (MyClass* )context->m_userPointer;
	
	{
		b3SharedMemoryCommandHandle commandHandle = b3RequestVREventsCommandInit(context->m_physClient);
		int deviceTypeFilter = VR_DEVICE_CONTROLLER;
		b3VREventsSetDeviceTypeFilter(commandHandle, deviceTypeFilter);

		b3SharedMemoryStatusHandle statusHandle = b3SubmitClientCommandAndWaitStatus(context->m_physClient, commandHandle);
		int statusType = b3GetStatusType(statusHandle);
		if (statusType == CMD_REQUEST_VR_EVENTS_DATA_COMPLETED)
		{
			struct b3VREventsData vrEvents;
		
			int i = 0;
			b3GetVREventsData(context->m_physClient, &vrEvents);
			if (vrEvents.m_numControllerEvents)
			{
				//this is only for a test, normally you wouldn't print to the console at each simulation substep!
				printf("got %d VR controller events!\n", vrEvents.m_numControllerEvents);
			}
		}
	}
	{
		b3KeyboardEventsData keyboardEventsData;
		b3SharedMemoryCommandHandle	commandHandle = b3RequestKeyboardEventsCommandInit(context->m_physClient);
		b3SubmitClientCommandAndWaitStatus(context->m_physClient, commandHandle);
		b3GetKeyboardEventsData(context->m_physClient, &keyboardEventsData);
		if (keyboardEventsData.m_numKeyboardEvents)
		{
			//this is only for a test, normally you wouldn't print to the console at each simulation substep!
			printf("got %d keyboard events\n", keyboardEventsData.m_numKeyboardEvents);
		}
	}

	{
		b3MouseEventsData mouseEventsData;
		b3SharedMemoryCommandHandle commandHandle = b3RequestMouseEventsCommandInit(context->m_physClient);
		b3SubmitClientCommandAndWaitStatus(context->m_physClient, commandHandle);
		b3GetMouseEventsData(context->m_physClient, &mouseEventsData);
		if (mouseEventsData.m_numMouseEvents)
		{
			//this is only for a test, normally you wouldn't print to the console at each simulation substep!
			printf("got %d mouse events\n", mouseEventsData.m_numMouseEvents);
		}
	}

	return 0;
}


B3_SHARED_API int postTickPluginCallback(struct b3PluginContext* context)
{
	MyClass* obj = (MyClass* )context->m_userPointer;
	obj->m_testData++;
	return 0;
}

B3_SHARED_API int executePluginCommand(struct b3PluginContext* context, const struct b3PluginArguments* arguments)
{
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

	MyClass* obj = (MyClass*) context->m_userPointer;
	
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


B3_SHARED_API void exitPlugin(struct b3PluginContext* context)
{
	MyClass* obj = (MyClass*) context->m_userPointer;
	delete obj;
	context->m_userPointer = 0;

	printf("bye!\n");
}
