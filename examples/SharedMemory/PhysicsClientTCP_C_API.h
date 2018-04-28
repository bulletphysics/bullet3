#ifndef PHYSICS_CLIENT_TCP_C_API_H
#define PHYSICS_CLIENT_TCP_C_API_H

#include "PhysicsClientC_API.h"

#ifdef __cplusplus
extern "C" {
#endif


	///send physics commands using TCP networking
B3_SHARED_API	b3PhysicsClientHandle b3ConnectPhysicsTCP(const char* hostName, int port);


#ifdef __cplusplus
}
#endif

#endif //PHYSICS_CLIENT_TCP_C_API_H
