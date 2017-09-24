#ifndef TEST_PLUGIN_H
#define TEST_PLUGIN_H

#include "../b3PluginAPI.h"

#ifdef __cplusplus
extern "C" 
{
#endif

//initPlugin, exitPlugin and executePluginCommand are required, otherwise plugin won't load
B3_SHARED_API int initPlugin(struct b3PluginContext* context);
B3_SHARED_API void exitPlugin(struct b3PluginContext* context);
B3_SHARED_API int executePluginCommand(struct b3PluginContext* context, const struct b3PluginArguments* arguments);

//preTickPluginCallback and postTickPluginCallback are optional.
B3_SHARED_API int preTickPluginCallback(struct b3PluginContext* context);
B3_SHARED_API int postTickPluginCallback(struct b3PluginContext* context);



#ifdef __cplusplus
};
#endif

#endif//#define TEST_PLUGIN_H
