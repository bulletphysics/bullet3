#ifndef PHYSICS_CLIENT_UDP_C_API_H
#define PHYSICS_CLIENT_UDP_C_API_H

#include "PhysicsClientC_API.h"

#ifdef __cplusplus
extern "C" {
#endif


	///send physics commands using UDP networking
	b3PhysicsClientHandle b3ConnectPhysicsUDP(const char* hostName, int port);


#ifdef __cplusplus
}
#endif

#endif //PHYSICS_CLIENT_UDP_C_API_H
