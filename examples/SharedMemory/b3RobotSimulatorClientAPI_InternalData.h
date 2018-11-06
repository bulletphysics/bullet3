#ifndef B3_ROBOT_SIMULATOR_CLIENT_API_INTERNAL_DATA_H
#define B3_ROBOT_SIMULATOR_CLIENT_API_INTERNAL_DATA_H

#include "../SharedMemory/PhysicsClientC_API.h"

struct b3RobotSimulatorClientAPI_InternalData
{
	b3PhysicsClientHandle m_physicsClientHandle;
	struct GUIHelperInterface* m_guiHelper;

	b3RobotSimulatorClientAPI_InternalData()
		: m_physicsClientHandle(0),
		  m_guiHelper(0)
	{
	}
};

#endif  //B3_ROBOT_SIMULATOR_CLIENT_API_INTERNAL_DATA_H