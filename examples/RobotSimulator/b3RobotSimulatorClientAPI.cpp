#include "b3RobotSimulatorClientAPI.h"

#include "../SharedMemory/PhysicsClientC_API.h"
#include "../SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"
#ifdef BT_ENABLE_ENET
#include "../SharedMemory/PhysicsClientUDP_C_API.h"
#endif  //PHYSICS_UDP

#ifdef BT_ENABLE_CLSOCKET
#include "../SharedMemory/PhysicsClientTCP_C_API.h"
#endif  //PHYSICS_TCP

#include "../SharedMemory/PhysicsDirectC_API.h"

#include "../SharedMemory/SharedMemoryInProcessPhysicsC_API.h"

#include "../SharedMemory/SharedMemoryPublic.h"
#include "Bullet3Common/b3Logging.h"

#ifdef BT_ENABLE_GRPC
#include "../SharedMemory/PhysicsClientGRPC_C_API.h"
#endif

b3RobotSimulatorClientAPI::b3RobotSimulatorClientAPI()
{
}

b3RobotSimulatorClientAPI::~b3RobotSimulatorClientAPI()
{
}

void b3RobotSimulatorClientAPI::renderScene()
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}
	if (m_data->m_guiHelper)
	{
		b3InProcessRenderSceneInternal(m_data->m_physicsClientHandle);
	}
}

void b3RobotSimulatorClientAPI::debugDraw(int debugDrawMode)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return;
	}
	if (m_data->m_guiHelper)
	{
		b3InProcessDebugDrawInternal(m_data->m_physicsClientHandle, debugDrawMode);
	}
}

bool b3RobotSimulatorClientAPI::mouseMoveCallback(float x, float y)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}
	if (m_data->m_guiHelper)
	{
		return b3InProcessMouseMoveCallback(m_data->m_physicsClientHandle, x, y) != 0;
	}
	return false;
}
bool b3RobotSimulatorClientAPI::mouseButtonCallback(int button, int state, float x, float y)
{
	if (!isConnected())
	{
		b3Warning("Not connected");
		return false;
	}
	if (m_data->m_guiHelper)
	{
		return b3InProcessMouseButtonCallback(m_data->m_physicsClientHandle, button, state, x, y) != 0;
	}
	return false;
}

bool b3RobotSimulatorClientAPI::connect(int mode, const std::string& hostName, int portOrKey)
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
		case eCONNECT_EXISTING_EXAMPLE_BROWSER:
		{
			sm = b3CreateInProcessPhysicsServerFromExistingExampleBrowserAndConnect(m_data->m_guiHelper);
			break;
		}

		case eCONNECT_GUI:
		{
			int argc = 0;
			char* argv[1] = {0};
#ifdef __APPLE__
			sm = b3CreateInProcessPhysicsServerAndConnectMainThread(argc, argv);
#else
			sm = b3CreateInProcessPhysicsServerAndConnect(argc, argv);
#endif
			break;
		}
		case eCONNECT_GUI_SERVER:
		{
			int argc = 0;
			char* argv[1] = {0};
#ifdef __APPLE__
			sm = b3CreateInProcessPhysicsServerAndConnectMainThread(argc, argv);
#else
			sm = b3CreateInProcessPhysicsServerAndConnect(argc, argv);
#endif
			break;
		}
		case eCONNECT_DIRECT:
		{
			sm = b3ConnectPhysicsDirect();
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
		case eCONNECT_GRPC:
		{
#ifdef BT_ENABLE_GRPC
			sm = b3ConnectPhysicsGRPC(hostName.c_str(), tcpPort);
#else
			b3Warning("GRPC is not enabled in this pybullet build");
#endif
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
