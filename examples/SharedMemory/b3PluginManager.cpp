
#include "b3PluginManager.h"
#include "Bullet3Common/b3HashMap.h"
#include "Bullet3Common/b3ResizablePool.h"
#include "PhysicsClientC_API.h"
#include "PhysicsDirect.h"
#include "plugins/b3PluginContext.h"

#ifdef _WIN32
    #define WIN32_LEAN_AND_MEAN
    #define VC_EXTRALEAN
    #include <windows.h>

    typedef HMODULE             B3_DYNLIB_HANDLE;

    #define B3_DYNLIB_OPEN    LoadLibrary
    #define B3_DYNLIB_CLOSE   FreeLibrary
    #define B3_DYNLIB_IMPORT  GetProcAddress
#else
    #include <dlfcn.h>
    
    typedef void*                   B3_DYNLIB_HANDLE;

    #define B3_DYNLIB_OPEN(path)  dlopen(path, RTLD_NOW | RTLD_GLOBAL)
    #define B3_DYNLIB_CLOSE       dlclose
    #define B3_DYNLIB_IMPORT      dlsym
#endif

struct b3Plugin
{
	B3_DYNLIB_HANDLE m_pluginHandle;
	bool m_ownsPluginHandle;
	std::string m_pluginPath;
	int m_pluginUniqueId;
	PFN_INIT m_initFunc;
	PFN_EXIT m_exitFunc;
	PFN_EXECUTE m_executeCommandFunc;
	
	PFN_TICK m_preTickFunc;
	PFN_TICK m_postTickFunc;

	void* m_userPointer;

	b3Plugin()
		:m_pluginHandle(0),
		m_ownsPluginHandle(false),
		m_pluginUniqueId(-1),
		m_initFunc(0),
		m_exitFunc(0),
		m_executeCommandFunc(0),
		m_preTickFunc(0),
		m_postTickFunc(0),
		m_userPointer(0)
	{
	}
	void clear()
	{
		if (m_ownsPluginHandle)
		{
			B3_DYNLIB_CLOSE(m_pluginHandle);
		}
		m_pluginHandle = 0;
		m_initFunc = 0;
		m_exitFunc = 0;
		m_executeCommandFunc = 0;
		m_preTickFunc = 0;
		m_postTickFunc = 0;
		m_userPointer = 0;
	}
};

typedef b3PoolBodyHandle<b3Plugin> b3PluginHandle;

struct b3PluginManagerInternalData
{
	b3ResizablePool<b3PluginHandle> m_plugins;
	b3HashMap<b3HashString, b3PluginHandle> m_pluginMap;
	PhysicsDirect* m_physicsDirect;
};

b3PluginManager::b3PluginManager(class PhysicsCommandProcessorInterface* physSdk)
{
	m_data = new b3PluginManagerInternalData;
	m_data->m_physicsDirect = new PhysicsDirect(physSdk,false);

}

b3PluginManager::~b3PluginManager()
{
	while (m_data->m_pluginMap.size())
	{
		b3PluginHandle* plugin = m_data->m_pluginMap.getAtIndex(0);
		unloadPlugin(plugin->m_pluginUniqueId);
	}
	delete m_data->m_physicsDirect;
	m_data->m_pluginMap.clear();
	m_data->m_plugins.exitHandles();
	delete m_data;
}


int b3PluginManager::loadPlugin(const char* pluginPath)
{
	int pluginUniqueId = -1;

	b3Plugin* pluginOrg = m_data->m_pluginMap.find(pluginPath);
	if (pluginOrg)
	{
		//already loaded
		pluginUniqueId = pluginOrg->m_pluginUniqueId;
	}
	else
	{
		pluginUniqueId = m_data->m_plugins.allocHandle();
		b3PluginHandle* plugin = m_data->m_plugins.getHandle(pluginUniqueId);
		plugin->m_pluginUniqueId = pluginUniqueId;
		B3_DYNLIB_HANDLE pluginHandle = B3_DYNLIB_OPEN(pluginPath);
		bool ok = false;
		if (pluginHandle)
		{

			plugin->m_initFunc = (PFN_INIT)B3_DYNLIB_IMPORT(pluginHandle, "initPlugin");
			plugin->m_exitFunc = (PFN_EXIT)B3_DYNLIB_IMPORT(pluginHandle, "exitPlugin");
			plugin->m_executeCommandFunc = (PFN_EXECUTE)B3_DYNLIB_IMPORT(pluginHandle, "executePluginCommand");
			plugin->m_preTickFunc = (PFN_TICK)B3_DYNLIB_IMPORT(pluginHandle, "preTickPluginCallback");
			plugin->m_postTickFunc = (PFN_TICK)B3_DYNLIB_IMPORT(pluginHandle, "postTickPluginCallback");
			
			if (plugin->m_initFunc && plugin->m_exitFunc && plugin->m_executeCommandFunc)
			{

				b3PluginContext context;
				context.m_userPointer = plugin->m_userPointer;
				context.m_physClient = (b3PhysicsClientHandle) m_data->m_physicsDirect;
				int version = plugin->m_initFunc(&context);
				//keep the user pointer persistent
				plugin->m_userPointer = context.m_userPointer;
				if (version == SHARED_MEMORY_MAGIC_NUMBER)
				{
					ok = true;
					plugin->m_ownsPluginHandle = true;
					plugin->m_pluginHandle = pluginHandle;
					plugin->m_pluginPath = pluginPath;
					m_data->m_pluginMap.insert(pluginPath, *plugin);
				}
				else
				{
					int expect = SHARED_MEMORY_MAGIC_NUMBER;
					b3Warning("Warning: plugin is wrong version: expected %d, got %d\n", expect, version);
				}
			}
			else
			{
				b3Warning("Loaded plugin but couldn't bind functions");
			}

			if (!ok)
			{
				B3_DYNLIB_CLOSE(pluginHandle);
			}
		}
		else
		{
			b3Warning("Warning: couldn't load plugin %s\n", pluginPath);
		}
		if (!ok)
		{
			m_data->m_plugins.freeHandle(pluginUniqueId);
			pluginUniqueId = -1;
		}
	}
	return pluginUniqueId;
}

void b3PluginManager::unloadPlugin(int pluginUniqueId)
{
	b3PluginHandle* plugin = m_data->m_plugins.getHandle(pluginUniqueId);
	if (plugin)
	{
		b3PluginContext context;
		context.m_userPointer = plugin->m_userPointer;
		context.m_physClient = (b3PhysicsClientHandle) m_data->m_physicsDirect;

		plugin->m_exitFunc(&context);
		m_data->m_pluginMap.remove(plugin->m_pluginPath.c_str());
		m_data->m_plugins.freeHandle(pluginUniqueId);
	}
}
		
void b3PluginManager::tickPlugins(double timeStep, bool isPreTick)
{
	for (int i=0;i<m_data->m_pluginMap.size();i++)
	{
		b3PluginHandle* plugin = m_data->m_pluginMap.getAtIndex(i);

		PFN_TICK  tick = isPreTick? plugin->m_preTickFunc : plugin->m_postTickFunc;
		if (tick)
		{
			b3PluginContext context;
			context.m_userPointer = plugin->m_userPointer;
			context.m_physClient = (b3PhysicsClientHandle) m_data->m_physicsDirect;
			int result = tick(&context);
			plugin->m_userPointer = context.m_userPointer;
		}
	}
}

int b3PluginManager::executePluginCommand(int pluginUniqueId, const b3PluginArguments* arguments)
{
	int result = -1;

	b3PluginHandle* plugin = m_data->m_plugins.getHandle(pluginUniqueId);
	if (plugin)
	{
		b3PluginContext context;
		context.m_userPointer = plugin->m_userPointer;
		context.m_physClient = (b3PhysicsClientHandle) m_data->m_physicsDirect;
		
		result = plugin->m_executeCommandFunc(&context, arguments);
		plugin->m_userPointer = context.m_userPointer;
	}
	return result;
}


int b3PluginManager::registerStaticLinkedPlugin(const char* pluginPath, PFN_INIT initFunc,PFN_EXIT exitFunc, PFN_EXECUTE executeCommandFunc, PFN_TICK preTickFunc, PFN_TICK postTickFunc)
{

	b3Plugin orgPlugin;

	
	int pluginUniqueId = m_data->m_plugins.allocHandle();
	b3PluginHandle* pluginHandle = m_data->m_plugins.getHandle(pluginUniqueId);
	pluginHandle->m_pluginHandle = 0;
	pluginHandle->m_ownsPluginHandle =false;
	pluginHandle->m_pluginUniqueId = pluginUniqueId;
	pluginHandle->m_executeCommandFunc = executeCommandFunc;
	pluginHandle->m_exitFunc = exitFunc;
	pluginHandle->m_initFunc = initFunc;
	pluginHandle->m_preTickFunc = preTickFunc;
	pluginHandle->m_postTickFunc = postTickFunc;
	pluginHandle->m_pluginHandle = 0;
	pluginHandle->m_pluginPath = pluginPath;
	pluginHandle->m_userPointer = 0;
	
	m_data->m_pluginMap.insert(pluginPath, *pluginHandle);

	{
		b3PluginContext context;
		context.m_userPointer = 0;
		context.m_physClient = (b3PhysicsClientHandle) m_data->m_physicsDirect;
		int result = pluginHandle->m_initFunc(&context);
		pluginHandle->m_userPointer = context.m_userPointer;
	}
	return pluginUniqueId;
}
