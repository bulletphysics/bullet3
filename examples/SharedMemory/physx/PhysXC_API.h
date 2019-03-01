#ifndef PHYSX_C_API_H
#define PHYSX_C_API_H

#ifdef BT_ENABLE_PHYSX

#include "../PhysicsClientC_API.h"

#ifdef __cplusplus
extern "C"
{
#endif

	B3_SHARED_API b3PhysicsClientHandle b3ConnectPhysX(int argc, char* argv[]);

#ifdef __cplusplus
}
#endif

#endif  //BT_ENABLE_PHYSX
#endif  //PHYSX_C_API_H
