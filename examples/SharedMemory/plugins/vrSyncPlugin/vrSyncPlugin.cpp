
//vrSyncPlugin plugin, will query the vr controller events
//and set change the user constraint to match the pose

//in Python you can load and configure the plugin like this:
//plugin = p.loadPlugin("e:/develop/bullet3/bin/pybullet_vrSyncPlugin_vs2010_x64_release.dll")
//could also be plugin = p.loadPlugin("vrSyncPlugin.so") on Mac/Linux
//controllerId = 3
//p.executePluginCommand(plugin ,"bla", [controllerId,pr2_cid],[50])

#include "vrSyncPlugin.h"
#include "../../SharedMemoryPublic.h"
#include "../b3PluginContext.h"
#include <stdio.h>

struct MyClass
{
	int m_testData;

	int m_controllerId;
	int m_constraintId;
	float m_maxForce;
	MyClass()
		:m_testData(42),
		m_controllerId(-1),
		m_constraintId(-1),
		m_maxForce(0)
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

	printf("hi vrSyncPlugin!\n");
	return SHARED_MEMORY_MAGIC_NUMBER;
}


B3_SHARED_API int preTickPluginCallback(struct b3PluginContext* context)
{
	MyClass* obj = (MyClass* )context->m_userPointer;
	if (obj->m_controllerId>=0)
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
				for (int n=0;n<vrEvents.m_numControllerEvents;n++)
				{
					b3VRControllerEvent& event = vrEvents.m_controllerEvents[n];
					if (event.m_controllerId ==obj->m_controllerId)
					{
						b3SharedMemoryCommandHandle commandHandle;
						int userConstraintUniqueId = obj->m_constraintId;
						commandHandle = b3InitChangeUserConstraintCommand(context->m_physClient, userConstraintUniqueId);
						double pos[4] = {event.m_pos[0],event.m_pos[1],event.m_pos[2],1};
						b3InitChangeUserConstraintSetPivotInB(commandHandle, pos);
						double orn[4] = {event.m_orn[0],event.m_orn[1],event.m_orn[2],event.m_orn[3]};
						b3InitChangeUserConstraintSetFrameInB(commandHandle, orn);
						b3InitChangeUserConstraintSetMaxForce(commandHandle, obj->m_maxForce);
						
						b3SharedMemoryStatusHandle statusHandle = b3SubmitClientCommandAndWaitStatus(context->m_physClient, commandHandle);
						//this is basically equivalent to doing this in Python/pybullet:
						//p.changeConstraint(pr2_cid, e[POSITION], e[ORIENTATION], maxForce=500)
					}
				}
			}
		}
	}

	return 0;
}



B3_SHARED_API int executePluginCommand(struct b3PluginContext* context, const struct b3PluginArguments* arguments)
{
	MyClass* obj = (MyClass*) context->m_userPointer;
	if (arguments->m_numInts>=2 && arguments->m_numFloats >= 0)
	{
		obj->m_constraintId = arguments->m_ints[1];
		printf("obj->m_constraintId=%d\n", obj->m_constraintId);
		obj->m_maxForce = arguments->m_floats[0];
		printf("obj->m_maxForce = %f\n", obj->m_maxForce);
		obj->m_controllerId = arguments->m_ints[0];
		printf("obj->m_controllerId=%d\n", obj->m_controllerId);

	}
	return 0;
}


B3_SHARED_API void exitPlugin(struct b3PluginContext* context)
{
	MyClass* obj = (MyClass*) context->m_userPointer;
	delete obj;
	context->m_userPointer = 0;

	printf("bye vrSyncPlugin!\n");
}
