#include "b3RobotSimulatorClientAPI_NoGUI.h"

#include "PhysicsClientC_API.h"
#include "b3RobotSimulatorClientAPI_InternalData.h"

#ifdef BT_ENABLE_ENET
#include "PhysicsClientUDP_C_API.h"
#endif  //PHYSICS_UDP

#ifdef BT_ENABLE_CLSOCKET
#include "PhysicsClientTCP_C_API.h"
#endif  //PHYSICS_TCP

#ifndef BT_DISABLE_PHYSICS_DIRECT
#include "PhysicsDirectC_API.h"
#endif //BT_DISABLE_PHYSICS_DIRECT

#include "SharedMemoryPublic.h"
#include "Bullet3Common/b3Logging.h"



b3RobotSimulatorClientAPI_NoGUI::b3RobotSimulatorClientAPI_NoGUI()
{
}

b3RobotSimulatorClientAPI_NoGUI::~b3RobotSimulatorClientAPI_NoGUI()
{
}


bool b3RobotSimulatorClientAPI_NoGUI::connect(int mode, const std::string& hostName, int portOrKey)
{
	if (m_data->m_physicsClientHandle)
	{
		b3Warning("Already connected, disconnect first.");
		return false;
	}
	b3PhysicsClientHandle sm = 0;
	int udpPort = 1234;
	int tcpPort = 6667;
	int key = SHARED_MEMORY_KEY;

	switch (mode)
	{
	
	case eCONNECT_DIRECT:
		{
#ifndef BT_DISABLE_PHYSICS_DIRECT
			sm = b3ConnectPhysicsDirect();
#endif //BT_DISABLE_PHYSICS_DIRECT

			break;
		}
	case eCONNECT_SHARED_MEMORY:
		{
			if (portOrKey >= 0)
			{
				key = portOrKey;
			}
			sm = b3ConnectSharedMemory(key);
			break;
		}
	case eCONNECT_UDP:
		{
			if (portOrKey >= 0)
			{
				udpPort = portOrKey;
			}
#ifdef BT_ENABLE_ENET

			sm = b3ConnectPhysicsUDP(hostName.c_str(), udpPort);
#else
			b3Warning("UDP is not enabled in this build");
#endif  //BT_ENABLE_ENET

			break;
		}
	case eCONNECT_TCP:
		{
			if (portOrKey >= 0)
			{
				tcpPort = portOrKey;
			}
#ifdef BT_ENABLE_CLSOCKET

			sm = b3ConnectPhysicsTCP(hostName.c_str(), tcpPort);
#else
			b3Warning("TCP is not enabled in this pybullet build");
#endif  //BT_ENABLE_CLSOCKET
			break;
		}

	default:
		{
			b3Warning("connectPhysicsServer unexpected argument");
		}
	};

	if (sm)
	{
		m_data->m_physicsClientHandle = sm;
		if (!b3CanSubmitCommand(m_data->m_physicsClientHandle))
		{
			disconnect();
			return false;
		}
		return true;
	}
	return false;
}
