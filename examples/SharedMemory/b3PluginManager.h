#ifndef B3_PLUGIN_MANAGER_H
#define B3_PLUGIN_MANAGER_H

#include "plugins/b3PluginAPI.h"

class b3PluginManager
{
	struct b3PluginManagerInternalData* m_data;

	public:
		
		b3PluginManager(class PhysicsCommandProcessorInterface* physSdk);
		virtual ~b3PluginManager();
		
		int loadPlugin(const char* pluginPath, const char* postFixStr ="");
		void unloadPlugin(int pluginUniqueId);
		int executePluginCommand(int pluginUniqueId, const struct b3PluginArguments* arguments);
		void addEvents(const struct b3VRControllerEvent* vrControllerEvents, int numVRControllerEvents, const struct b3KeyboardEvent* keyEvents, int numKeyEvents, const struct b3MouseEvent* mouseEvents, int numMouseEvents);
		void clearEvents();

		void addNotification(const struct b3Notification& notification);
		void reportNotifications();

		void tickPlugins(double timeStep, bool isPreTick);

		int registerStaticLinkedPlugin(const char* pluginPath, PFN_INIT initFunc,PFN_EXIT exitFunc, PFN_EXECUTE executeCommandFunc, PFN_TICK preTickFunc, PFN_TICK postTickFunc, PFN_GET_RENDER_INTERFACE getRendererFunc);
		
		void selectPluginRenderer(int pluginUniqueId);
		UrdfRenderingInterface* getRenderInterface();
};

#endif //B3_PLUGIN_MANAGER_H
