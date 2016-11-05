#ifndef PHYSICS_CLIENT_SHARED_MEMORY_H
#define PHYSICS_CLIENT_SHARED_MEMORY_H

#include "PhysicsClientC_API.h"

#ifdef __cplusplus
extern "C" {
#endif

	b3PhysicsClientHandle b3ConnectSharedMemory(int key);

#ifdef __cplusplus
}
#endif

#endif //PHYSICS_CLIENT_SHARED_MEMORY_H
