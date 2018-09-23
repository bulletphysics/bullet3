#ifndef PID_CONTROL_PLUGIN_H
#define PID_CONTROL_PLUGIN_H

#include "../b3PluginAPI.h"

#ifdef __cplusplus
extern "C"
{
#endif

	//the following 3 APIs are required
	B3_SHARED_API int initPlugin_pdControlPlugin(struct b3PluginContext* context);
	B3_SHARED_API void exitPlugin_pdControlPlugin(struct b3PluginContext* context);
	B3_SHARED_API int executePluginCommand_pdControlPlugin(struct b3PluginContext* context, const struct b3PluginArguments* arguments);

	///
	enum PDControlCommandEnum
	{
		eSetPDControl = 1,
		eRemovePDControl = 2,
	};

	//all the APIs below are optional
	B3_SHARED_API int preTickPluginCallback_pdControlPlugin(struct b3PluginContext* context);

#ifdef __cplusplus
};
#endif

#endif  //#define PID_CONTROL_PLUGIN_H
