#ifndef DART_PHYSICS_C_API_H
#define DART_PHYSICS_C_API_H

#ifdef BT_ENABLE_DART

#include "../PhysicsClientC_API.h"

#ifdef __cplusplus
extern "C" { 
#endif

//think more about naming. The b3ConnectPhysicsLoopback
B3_SHARED_API	b3PhysicsClientHandle b3ConnectPhysicsDART();

#ifdef __cplusplus
}
#endif

#endif //BT_ENABLE_DART
#endif //DART_PHYSICS_C_API_H
