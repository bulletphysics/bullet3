#ifdef BT_ENABLE_PHYSX
#include "PhysXC_API.h"
#include "../PhysicsDirect.h"
#include "PhysXServerCommandProcessor.h"


B3_SHARED_API b3PhysicsClientHandle b3ConnectPhysX()
{
	PhysXServerCommandProcessor* sdk = new PhysXServerCommandProcessor;

	PhysicsDirect* direct = new PhysicsDirect(sdk, true);
	bool connected;
	connected = direct->connect();
	return (b3PhysicsClientHandle)direct;
}
#endif  //BT_ENABLE_PHYSX