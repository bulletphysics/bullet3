#ifdef BT_ENABLE_MUJOCO
#include "MuJoCoPhysicsC_API.h"
#include "MuJoCoPhysicsServerCommandProcessor.h"
#include "MuJoCoPhysicsClient.h"

B3_SHARED_API b3PhysicsClientHandle b3ConnectPhysicsMuJoCo()
{
	MuJoCoPhysicsServerCommandProcessor* sdk = new MuJoCoPhysicsServerCommandProcessor;

	MuJoCoPhysicsClient* direct = new MuJoCoPhysicsClient(sdk, true);
	bool connected;
	connected = direct->connect();
	return (b3PhysicsClientHandle)direct;
}
#endif  //BT_ENABLE_MUJOCO