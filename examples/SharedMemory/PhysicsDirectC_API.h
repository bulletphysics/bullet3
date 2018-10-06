#ifndef PHYSICS_DIRECT_C_API_H
#define PHYSICS_DIRECT_C_API_H

#include "PhysicsClientC_API.h"

#ifdef __cplusplus
extern "C"
{
#endif

	///think more about naming. Directly execute commands without transport (no shared memory, UDP, socket, grpc etc)
	B3_SHARED_API b3PhysicsClientHandle b3ConnectPhysicsDirect();

#ifdef __cplusplus
}
#endif

#endif  //PHYSICS_DIRECT_C_API_H
