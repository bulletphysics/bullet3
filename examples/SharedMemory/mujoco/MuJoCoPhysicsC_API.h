#ifndef MUJOCO_PHYSICS_C_API_H
#define MUJOCO_PHYSICS_C_API_H

#ifdef BT_ENABLE_MUJOCO

#include "../PhysicsClientC_API.h"

#ifdef __cplusplus
extern "C" { 
#endif

//think more about naming. The b3ConnectPhysicsLoopback
B3_SHARED_API	b3PhysicsClientHandle b3ConnectPhysicsMuJoCo();

#ifdef __cplusplus
}
#endif

#endif //BT_ENABLE_MUJOCO
#endif //MUJOCO_PHYSICS_C_API_H
