#ifdef BT_ENABLE_GRPC

#include "PhysicsClientGRPC_C_API.h"
#include "PhysicsClientGRPC.h"
#include "PhysicsDirect.h"
#include <stdio.h>

B3_SHARED_API b3PhysicsClientHandle b3ConnectPhysicsGRPC(const char* hostName, int port)
{
	GRPCNetworkedPhysicsProcessor* tcp = new GRPCNetworkedPhysicsProcessor(hostName, port);

	PhysicsDirect* direct = new PhysicsDirect(tcp, true);

	bool connected;
	connected = direct->connect();
	if (connected)
	{
		printf("b3ConnectPhysicsGRPC connected successfully.\n");
	}
	else
	{
		printf("b3ConnectPhysicsGRPC connection failed.\n");
	}
	return (b3PhysicsClientHandle)direct;
}

#endif  //BT_ENABLE_GRPC