#ifndef COLLISION_FILTER_PLUGIN_H
#define COLLISION_FILTER_PLUGIN_H

#include "../b3PluginAPI.h"

#ifdef __cplusplus
extern "C" 
{
#endif

//the following 3 APIs are required
B3_SHARED_API int initPlugin_collisionFilterPlugin(struct b3PluginContext* context);
B3_SHARED_API void exitPlugin_collisionFilterPlugin(struct b3PluginContext* context);
B3_SHARED_API int executePluginCommand_collisionFilterPlugin(struct b3PluginContext* context, const struct b3PluginArguments* arguments);

//all the APIs below are optional
B3_SHARED_API int preTickPluginCallback_collisionFilterPlugin(struct b3PluginContext* context);
B3_SHARED_API int postTickPluginCallback_collisionFilterPlugin(struct b3PluginContext* context);


#ifdef __cplusplus
};
#endif

#endif//#define COLLISION_FILTER_PLUGIN_H
