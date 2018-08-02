
#include "pdControlPlugin.h"
#include "../../SharedMemoryPublic.h"
#include "../b3PluginContext.h"
#include <stdio.h>


#include "LinearMath/btScalar.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../../b3RobotSimulatorClientAPI_NoDirect.h"
#include "../../b3RobotSimulatorClientAPI_InternalData.h"

struct MyPDControl
{
	int m_objectUniqueId;
	int m_linkIndex;
	btScalar m_desiredPosition;
	btScalar m_desiredVelocity;
	btScalar m_kd;
	btScalar m_kp;
	btScalar m_maxForce;
};

struct MyPDControlContainer
{
	int m_testData;
	btAlignedObjectArray<MyPDControl> m_controllers;
	b3RobotSimulatorClientAPI_NoDirect m_api;
	MyPDControlContainer()
		:m_testData(42)
	{
	}
	virtual ~MyPDControlContainer()
	{
	}
};

B3_SHARED_API int initPlugin_pdControlPlugin(struct b3PluginContext* context)
{
	MyPDControlContainer* obj = new MyPDControlContainer();
	b3RobotSimulatorClientAPI_InternalData data;
	data.m_physicsClientHandle = context->m_physClient;
	data.m_guiHelper = 0;
	obj->m_api.setInternalData(&data);
	context->m_userPointer = obj;

	return SHARED_MEMORY_MAGIC_NUMBER;
}


B3_SHARED_API int preTickPluginCallback_pdControlPlugin(struct b3PluginContext* context)
{
	//apply pd control here, apply forces using the PD gains
	MyPDControlContainer* obj = (MyPDControlContainer*)context->m_userPointer;
	
	for (int i = 0; i < obj->m_controllers.size(); i++)
	{
		const MyPDControl& pdControl = obj->m_controllers[i];
		
		int dof1 = 0;
		b3JointSensorState actualState;
		if (obj->m_api.getJointState(pdControl.m_objectUniqueId, pdControl.m_linkIndex,&actualState))
		{
			if (pdControl.m_maxForce>0)
			{
				//compute torque
				btScalar qActual = actualState.m_jointPosition;
				btScalar qdActual = actualState.m_jointVelocity;

				btScalar positionError = (pdControl.m_desiredPosition -qActual);
				double desiredVelocity = 0;
				btScalar velocityError = (pdControl.m_desiredVelocity-qdActual);

				btScalar force = pdControl.m_kp * positionError + pdControl.m_kd*velocityError;

				btClamp(force,-pdControl.m_maxForce,pdControl.m_maxForce);
			
				//apply torque
				b3RobotSimulatorJointMotorArgs args(CONTROL_MODE_TORQUE);
				args.m_maxTorqueValue = force;
				obj->m_api.setJointMotorControl(pdControl.m_objectUniqueId, pdControl.m_linkIndex, args);
			}
		}

	}

	return 0;
}



B3_SHARED_API int executePluginCommand_pdControlPlugin(struct b3PluginContext* context, const struct b3PluginArguments* arguments)
{
	MyPDControlContainer* obj = (MyPDControlContainer*)context->m_userPointer;
	
	obj->m_api.syncBodies();
	
	int numObj = obj->m_api.getNumBodies();
	//printf("numObj = %d\n", numObj);

	//protocol:
	//first int is the type of command
	//second int is body unique id
	//third int is link index

	if (arguments->m_numInts != 3)
		return -1;

	switch (arguments->m_ints[0])
	{
	case	eSetPDControl:
	{
		if (arguments->m_numFloats < 5)
			return -1;
		MyPDControl controller;
		controller.m_desiredPosition = arguments->m_floats[0];
		controller.m_desiredVelocity = arguments->m_floats[1];
		controller.m_kd = arguments->m_floats[2];
		controller.m_kp = arguments->m_floats[3];
		controller.m_maxForce = arguments->m_floats[4];
		controller.m_objectUniqueId = arguments->m_ints[1];
		controller.m_linkIndex = arguments->m_ints[2];
		int foundIndex = -1;
		for (int i = 0; i < obj->m_controllers.size(); i++)
		{
			if (obj->m_controllers[i].m_objectUniqueId == controller.m_objectUniqueId && obj->m_controllers[i].m_linkIndex == controller.m_linkIndex)
			{
				obj->m_controllers[i] = controller;
				foundIndex=i;
			}
		}
		if (foundIndex<0)
		{
			obj->m_controllers.push_back(controller);
		}
		break;
	}
	case	eRemovePDControl:
	{
		MyPDControl controller;
		controller.m_objectUniqueId = arguments->m_ints[1];
		controller.m_linkIndex = arguments->m_ints[2];

		for (int i = 0; i < obj->m_controllers.size(); i++)
		{
			if (obj->m_controllers[i].m_objectUniqueId == controller.m_objectUniqueId && obj->m_controllers[i].m_linkIndex == controller.m_linkIndex)
			{
				obj->m_controllers.removeAtIndex(i);
				break;
			}
		}
		break;
	}
	default:
		{
			return -1;
		}
	}

	int result = 42;
	return result;
}


B3_SHARED_API void exitPlugin_pdControlPlugin(struct b3PluginContext* context)
{
	MyPDControlContainer* obj = (MyPDControlContainer*) context->m_userPointer;
	delete obj;
	context->m_userPointer = 0;

}
