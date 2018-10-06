#include "PhysicsDirectC_API.h"

#include "PhysicsDirect.h"

#include "PhysicsServerCommandProcessor.h"

//think more about naming. The b3ConnectPhysicsLoopback
B3_SHARED_API b3PhysicsClientHandle b3ConnectPhysicsDirect()
{
	PhysicsServerCommandProcessor* sdk = new PhysicsServerCommandProcessor;

	PhysicsDirect* direct = new PhysicsDirect(sdk, true);
	bool connected;
	connected = direct->connect();
	return (b3PhysicsClientHandle)direct;
}

//
