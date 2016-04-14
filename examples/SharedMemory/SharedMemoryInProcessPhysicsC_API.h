
#ifndef IN_PROCESS_PHYSICS_C_API_H
#define IN_PROCESS_PHYSICS_C_API_H

#include "PhysicsClientC_API.h"

#ifdef __cplusplus
extern "C" { 
#endif


///think more about naming. The b3ConnectPhysicsLoopback
b3PhysicsClientHandle b3CreateInProcessPhysicsServerAndConnect(int argc, char* argv[]);

b3PhysicsClientHandle b3CreateInProcessPhysicsServerAndConnectMainThread(int argc, char* argv[]);

    
    
    
#ifdef __cplusplus
}
#endif

#endif //IN_PROCESS_PHYSICS_C_API_H
