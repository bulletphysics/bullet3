
#include "PhysicsClientTCP_C_API.h"
#include "PhysicsClientTCP.h"
#include "PhysicsDirect.h"
#include <stdio.h>

B3_SHARED_API	b3PhysicsClientHandle b3ConnectPhysicsTCP(const char* hostName, int port)
{

	TcpNetworkedPhysicsProcessor* tcp = new TcpNetworkedPhysicsProcessor(hostName, port);

	PhysicsDirect* direct = new PhysicsDirect(tcp, true);

	bool connected;
	connected = direct->connect();
	if (connected)
	{
		printf("b3ConnectPhysicsTCP connected successfully.\n");
	}
	else
	{
		printf("b3ConnectPhysicsTCP connection failed.\n");

	}
	return (b3PhysicsClientHandle)direct;
}



