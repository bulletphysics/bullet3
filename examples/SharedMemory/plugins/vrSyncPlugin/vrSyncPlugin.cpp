
//vrSyncPlugin plugin, will query the vr controller events
//and set change the user constraint to match the pose

//in Python you can load and configure the plugin like this:
//plugin = p.loadPlugin("e:/develop/bullet3/bin/pybullet_vrSyncPlugin_vs2010_x64_release.dll")
//could also be plugin = p.loadPlugin("vrSyncPlugin.so") on Mac/Linux
//controllerId = 3

#include "vrSyncPlugin.h"
#include "../../SharedMemoryPublic.h"
#include "../b3PluginContext.h"
#include <stdio.h>

struct MyClass
{
	int m_testData;

	int m_controllerId;
	int m_constraintId;
	int m_constraintId2;
	int m_gripperId;
	float m_maxForce;
	float m_maxForce2;
	MyClass()
		:m_testData(42),
		m_controllerId(-1),
		m_constraintId(-1),
		m_constraintId2(-1),
		m_gripperId(-1),
		m_maxForce(0),
		m_maxForce2(0)
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
						if (obj->m_constraintId>=0)
						{
							//this is basically equivalent to doing this in Python/pybullet:
							//p.changeConstraint(pr2_cid, e[POSITION], e[ORIENTATION], maxForce=...)
							b3SharedMemoryCommandHandle commandHandle;
							int userConstraintUniqueId = obj->m_constraintId;
							commandHandle = b3InitChangeUserConstraintCommand(context->m_physClient, userConstraintUniqueId);
							double pos[4] = {event.m_pos[0],event.m_pos[1],event.m_pos[2],1};
							b3InitChangeUserConstraintSetPivotInB(commandHandle, pos);
							double orn[4] = {event.m_orn[0],event.m_orn[1],event.m_orn[2],event.m_orn[3]};
							b3InitChangeUserConstraintSetFrameInB(commandHandle, orn);
							b3InitChangeUserConstraintSetMaxForce(commandHandle, obj->m_maxForce);
							b3SharedMemoryStatusHandle statusHandle = b3SubmitClientCommandAndWaitStatus(context->m_physClient, commandHandle);
						}
						// apply the analogue button to close the constraint, using a gear constraint with position target
						if (obj->m_constraintId2>=0)
						{
							//this block is similar to
							//p.changeConstraint(c,gearRatio=1, erp=..., relativePositionTarget=relPosTarget, maxForce=...)
							//printf("obj->m_constraintId2=%d\n", obj->m_constraintId2);
							b3SharedMemoryCommandHandle commandHandle;
							commandHandle = b3InitChangeUserConstraintCommand(context->m_physClient, obj->m_constraintId2);

							//0 -> open, 1 = closed
							double openPos = 1.;
							double relPosTarget = openPos - (event.m_analogAxis*openPos);
							b3InitChangeUserConstraintSetRelativePositionTarget(commandHandle, relPosTarget);
							b3InitChangeUserConstraintSetERP(commandHandle,1);
							b3SharedMemoryStatusHandle statusHandle = b3SubmitClientCommandAndWaitStatus(context->m_physClient, commandHandle);
						}
						//printf("event.m_analogAxis=%f\n", event.m_analogAxis);

						// use the pr2_gripper motors to keep the gripper centered/symmetric around the center axis
						if (obj->m_gripperId>=0)
						{
							//this block is similar to
							//b = p.getJointState(pr2_gripper,2)[0]
							//print("b = " + str(b))
							//p.setJointMotorControl2(pr2_gripper, 0, p.POSITION_CONTROL, targetPosition=b, force=3)

							//printf("obj->m_gripperId=%d\n", obj->m_gripperId);
							{
								b3SharedMemoryCommandHandle cmd_handle =
								b3RequestActualStateCommandInit(context->m_physClient, obj->m_gripperId);
								b3SharedMemoryStatusHandle status_handle =
								b3SubmitClientCommandAndWaitStatus(context->m_physClient, cmd_handle);

								int status_type = b3GetStatusType(status_handle);
								if (status_type == CMD_ACTUAL_STATE_UPDATE_COMPLETED)
								{
									//printf("status_type == CMD_ACTUAL_STATE_UPDATE_COMPLETED\n");

									b3JointSensorState sensorState;
									if (b3GetJointState(context->m_physClient, status_handle, 2, &sensorState))
									{
										
										
										b3SharedMemoryCommandHandle commandHandle;
										double targetPosition = sensorState.m_jointPosition;
										//printf("targetPosition =%f\n", targetPosition);
										if (1)
										{
											b3JointInfo info;
											b3GetJointInfo(context->m_physClient, obj->m_gripperId, 0, &info);
											commandHandle = b3JointControlCommandInit2(context->m_physClient, obj->m_gripperId, CONTROL_MODE_POSITION_VELOCITY_PD);
											double kp = .1;
											double targetVelocity = 0;
											double kd = .6;
											b3JointControlSetDesiredPosition(commandHandle, info.m_qIndex, targetPosition);
											b3JointControlSetKp(commandHandle, info.m_uIndex, kp);
											b3JointControlSetDesiredVelocity(commandHandle, info.m_uIndex,targetVelocity);
											b3JointControlSetKd(commandHandle, info.m_uIndex, kd);
											b3JointControlSetMaximumForce(commandHandle, info.m_uIndex, obj->m_maxForce2);
											b3SubmitClientCommandAndWaitStatus(context->m_physClient, cmd_handle);
										}
									} else
									{
										//printf("???\n");
									}
									
								} else
								{
									//printf("no\n");
								}

							}
							
						}
						
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
	if (arguments->m_numInts>=4 && arguments->m_numFloats >= 2)
	{
		obj->m_constraintId = arguments->m_ints[1];
		obj->m_constraintId2 = arguments->m_ints[2];
		obj->m_gripperId = arguments->m_ints[3];
		printf("obj->m_constraintId=%d\n", obj->m_constraintId);
		obj->m_maxForce = arguments->m_floats[0];
		obj->m_maxForce2 = arguments->m_floats[1];
		printf("obj->m_maxForce = %f\n", obj->m_maxForce);
		obj->m_controllerId = arguments->m_ints[0];
		printf("obj->m_controllerId=%d\n", obj->m_controllerId);

		b3SharedMemoryCommandHandle command = b3InitSyncBodyInfoCommand(context->m_physClient);
		b3SharedMemoryStatusHandle statusHandle = b3SubmitClientCommandAndWaitStatus(context->m_physClient, command);
		int statusType = b3GetStatusType(statusHandle);

		if (statusType != CMD_SYNC_BODY_INFO_COMPLETED) 
		{

		}
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
