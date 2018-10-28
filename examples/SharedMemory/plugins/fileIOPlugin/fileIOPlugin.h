#ifndef FILE_IO_PLUGIN_H
#define FILE_IO_PLUGIN_H

#include "../b3PluginAPI.h"

#ifdef __cplusplus
extern "C"
{
#endif

	//initPlugin, exitPlugin and executePluginCommand are required, otherwise plugin won't load
	B3_SHARED_API int initPlugin_fileIOPlugin(struct b3PluginContext* context);
	B3_SHARED_API void exitPlugin_fileIOPlugin(struct b3PluginContext* context);
	B3_SHARED_API int executePluginCommand_fileIOPlugin(struct b3PluginContext* context, const struct b3PluginArguments* arguments);

	//all the APIs below are optional
	B3_SHARED_API struct CommonFileIOInterface* getFileIOFunc_fileIOPlugin(struct b3PluginContext* context);
	

#ifdef __cplusplus
};
#endif

#endif  //#define FILE_IO_PLUGIN_H
