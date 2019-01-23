#ifdef BT_ENABLE_PHYSX
#include "PhysXC_API.h"
#include "PhysXServerCommandProcessor.h"
#include "PhysXClient.h"

B3_SHARED_API b3PhysicsClientHandle b3ConnectPhysX()
{
	PhysXServerCommandProcessor* sdk = new PhysXServerCommandProcessor;

	PhysXClient* direct = new PhysXClient(sdk, true);
	bool connected;
	connected = direct->connect();
	return (b3PhysicsClientHandle)direct;
}
#endif  //BT_ENABLE_PHYSX