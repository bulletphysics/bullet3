#include "PhysicsDirectC_API.h"

#include "PhysicsDirect.h"



//think more about naming. The b3ConnectPhysicsLoopback
b3PhysicsClientHandle b3ConnectPhysicsDirect()
{
	PhysicsDirect* direct = new PhysicsDirect();
	bool connected = direct->connect();
	return (b3PhysicsClientHandle  )direct;
}

