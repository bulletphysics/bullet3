#include "PhysicsDirectC_API.h"

#include "PhysicsDirect.h"

#include "PhysicsServerCommandProcessor.h"



//think more about naming. The b3ConnectPhysicsLoopback
b3PhysicsClientHandle b3ConnectPhysicsDirect()
{
	PhysicsServerCommandProcessor* sdk = new PhysicsServerCommandProcessor;

	PhysicsDirect* direct = new PhysicsDirect(sdk);
	bool connected = direct->connect();
	return (b3PhysicsClientHandle  )direct;
}



//

