#ifndef GRPC_PLUGIN_H
#define GRPC_PLUGIN_H

#include "../b3PluginAPI.h"

#ifdef __cplusplus
extern "C"
{
#endif

	//the following 3 APIs are required
	B3_SHARED_API int initPlugin_grpcPlugin(struct b3PluginContext* context);
	B3_SHARED_API void exitPlugin_grpcPlugin(struct b3PluginContext* context);
	B3_SHARED_API int executePluginCommand_grpcPlugin(struct b3PluginContext* context, const struct b3PluginArguments* arguments);

	//all the APIs below are optional
	B3_SHARED_API int preTickPluginCallback_grpcPlugin(struct b3PluginContext* context);
	B3_SHARED_API int postTickPluginCallback_grpcPlugin(struct b3PluginContext* context);

	B3_SHARED_API int processClientCommands_grpcPlugin(struct b3PluginContext* context);

#ifdef __cplusplus
};
#endif

#endif  //#define GRPC_PLUGIN_H
