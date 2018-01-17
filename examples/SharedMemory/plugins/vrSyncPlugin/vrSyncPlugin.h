#ifndef TEST_PLUGIN_H
#define TEST_PLUGIN_H

#include "../b3PluginAPI.h"

#ifdef __cplusplus
extern "C" 
{
#endif

//initPlugin, exitPlugin and executePluginCommand are required, otherwise plugin won't load
B3_SHARED_API int initPlugin_vrSyncPlugin(struct b3PluginContext* context);
B3_SHARED_API void exitPlugin_vrSyncPlugin(struct b3PluginContext* context);
B3_SHARED_API int executePluginCommand_vrSyncPlugin(struct b3PluginContext* context, const struct b3PluginArguments* arguments);

//optional APIs
B3_SHARED_API int preTickPluginCallback_vrSyncPlugin(struct b3PluginContext* context);


#ifdef __cplusplus
};
#endif

#endif//#define TEST_PLUGIN_H
