
#ifndef IN_PROCESS_PHYSICS_C_API_H
#define IN_PROCESS_PHYSICS_C_API_H

#include "PhysicsClientC_API.h"

#ifdef __cplusplus
extern "C" { 
#endif


///think more about naming. The b3ConnectPhysicsLoopback
b3PhysicsClientHandle b3CreateInProcessPhysicsServerAndConnect(int argc, char* argv[]);

b3PhysicsClientHandle b3CreateInProcessPhysicsServerAndConnectMainThread(int argc, char* argv[]);

b3PhysicsClientHandle b3CreateInProcessPhysicsServerFromExistingExampleBrowserAndConnect(void* guiHelperPtr);

///ignore the following APIs, they are for internal use for example browser
void b3InProcessRenderSceneInternal(b3PhysicsClientHandle clientHandle);
void b3InProcessDebugDrawInternal(b3PhysicsClientHandle clientHandle, int debugDrawMode);
int b3InProcessMouseMoveCallback(b3PhysicsClientHandle clientHandle,float x,float y);
int b3InProcessMouseButtonCallback(b3PhysicsClientHandle clientHandle, int button, int state, float x, float y);

#ifdef __cplusplus
}
#endif

#endif //IN_PROCESS_PHYSICS_C_API_H
