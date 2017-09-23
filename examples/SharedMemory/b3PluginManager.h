#ifndef B3_PLUGIN_MANAGER_H
#define B3_PLUGIN_MANAGER_H

class b3PluginManager
{
	struct b3PluginManagerInternalData* m_data;

	public:
		
		b3PluginManager(class PhysicsCommandProcessorInterface* physSdk);
		virtual ~b3PluginManager();
		
		int loadPlugin(const char* pluginPath);
		void unloadPlugin(int pluginUniqueId);
		int executePluginCommand(int pluginUniqueId, const char* arguments);
		
};

#endif //B3_PLUGIN_MANAGER_H
