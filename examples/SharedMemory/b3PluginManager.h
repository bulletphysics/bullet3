#ifndef B3_PLUGIN_MANAGER_H
#define B3_PLUGIN_MANAGER_H

#include "plugins/b3PluginAPI.h"

enum b3PluginManagerTickMode
{
	B3_PRE_TICK_MODE = 1,
	B3_POST_TICK_MODE,
	B3_PROCESS_CLIENT_COMMANDS_TICK,
};

struct b3PluginFunctions
{
	//required
	PFN_INIT m_initFunc;
	PFN_EXIT m_exitFunc;
	PFN_EXECUTE m_executeCommandFunc;

	//optional
	PFN_TICK m_preTickFunc;
	PFN_TICK m_postTickFunc;
	PFN_GET_RENDER_INTERFACE m_getRendererFunc;
	PFN_TICK m_processClientCommandsFunc;
	PFN_TICK m_processNotificationsFunc;
	PFN_GET_COLLISION_INTERFACE m_getCollisionFunc;
	PFN_GET_FILEIO_INTERFACE m_fileIoFunc;
	
	b3PluginFunctions(PFN_INIT initFunc, PFN_EXIT exitFunc, PFN_EXECUTE executeCommandFunc)
		:m_initFunc(initFunc),
		m_exitFunc(exitFunc),
		m_executeCommandFunc(executeCommandFunc),
		m_preTickFunc(0),
		m_postTickFunc(0),
		m_getRendererFunc(0),
		m_processClientCommandsFunc(0),
		m_processNotificationsFunc(0),
		m_getCollisionFunc(0),
		m_fileIoFunc(0)
	{
	}
};

class b3PluginManager
{
	struct b3PluginManagerInternalData* m_data;

public:
	b3PluginManager(class PhysicsCommandProcessorInterface* physSdk);
	virtual ~b3PluginManager();

	int loadPlugin(const char* pluginPath, const char* postFixStr = "");
	void unloadPlugin(int pluginUniqueId);
	int executePluginCommand(int pluginUniqueId, const struct b3PluginArguments* arguments);
	void addEvents(const struct b3VRControllerEvent* vrControllerEvents, int numVRControllerEvents, const struct b3KeyboardEvent* keyEvents, int numKeyEvents, const struct b3MouseEvent* mouseEvents, int numMouseEvents);
	void clearEvents();

	void addNotification(const struct b3Notification& notification);
	void reportNotifications();

	void tickPlugins(double timeStep, b3PluginManagerTickMode tickMode);

	int registerStaticLinkedPlugin(const char* pluginPath, b3PluginFunctions& functions, bool initPlugin = true);

	void selectPluginRenderer(int pluginUniqueId);
	struct UrdfRenderingInterface* getRenderInterface();

	void selectFileIOPlugin(int pluginUniqueId);
	struct CommonFileIOInterface* getFileIOInterface();

	void selectCollisionPlugin(int pluginUniqueId);
	struct b3PluginCollisionInterface* getCollisionInterface();

	const struct b3UserDataValue* getReturnData(int pluginUniqueId);
};

#endif  //B3_PLUGIN_MANAGER_H
