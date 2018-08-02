
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

    #define B3_DYNLIB_OPEN    LoadLibraryA
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
	PFN_TICK m_processNotificationsFunc;

	PFN_GET_RENDER_INTERFACE m_getRendererFunc;
	
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
		m_processNotificationsFunc(0),
		m_getRendererFunc(0),
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
		m_getRendererFunc = 0;
	}
};

typedef b3PoolBodyHandle<b3Plugin> b3PluginHandle;

struct b3PluginManagerInternalData
{
	b3ResizablePool<b3PluginHandle> m_plugins;
	b3HashMap<b3HashString, int> m_pluginMap;
	PhysicsDirect* m_physicsDirect;
	b3AlignedObjectArray<b3KeyboardEvent> m_keyEvents;
	b3AlignedObjectArray<b3VRControllerEvent> m_vrEvents;
	b3AlignedObjectArray<b3MouseEvent> m_mouseEvents;
	b3AlignedObjectArray<b3Notification> m_notifications[2];
	int m_activeNotificationsBufferIndex;
	int m_activeRendererPluginUid;

	b3PluginManagerInternalData()
		:m_activeNotificationsBufferIndex(0), m_activeRendererPluginUid(-1)
	{
	}
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
		int* pluginUidPtr = m_data->m_pluginMap.getAtIndex(0);
		if (pluginUidPtr)
		{
			int pluginUid = *pluginUidPtr;
			unloadPlugin(*pluginUidPtr);
		}
	}
	delete m_data->m_physicsDirect;
	m_data->m_pluginMap.clear();
	m_data->m_plugins.exitHandles();
	delete m_data;
}

void b3PluginManager::addEvents(const struct b3VRControllerEvent* vrControllerEvents, int numVRControllerEvents, const struct b3KeyboardEvent* keyEvents, int numKeyEvents, const struct b3MouseEvent* mouseEvents, int numMouseEvents)
{

	for (int i = 0; i < numKeyEvents; i++)
	{
		m_data->m_keyEvents.push_back(keyEvents[i]);
	}

	for (int i = 0; i < numVRControllerEvents; i++)
	{
		m_data->m_vrEvents.push_back(vrControllerEvents[i]);
	}
	for (int i = 0; i < numMouseEvents; i++)
	{
		m_data->m_mouseEvents.push_back(mouseEvents[i]);
	}
}

void b3PluginManager::clearEvents()
{
	m_data->m_keyEvents.resize(0);
	m_data->m_vrEvents.resize(0);
	m_data->m_mouseEvents.resize(0);
}

void b3PluginManager::addNotification(const struct b3Notification& notification)
{
	m_data->m_notifications[m_data->m_activeNotificationsBufferIndex].push_back(notification);
}

int b3PluginManager::loadPlugin(const char* pluginPath, const char* postFixStr)
{
	int pluginUniqueId = -1;

	int* pluginUidPtr = m_data->m_pluginMap.find(pluginPath);
	if (pluginUidPtr)
	{
		//already loaded
		pluginUniqueId = *pluginUidPtr;
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
			std::string postFix = postFixStr;
			std::string initStr = std::string("initPlugin") + postFix;
			std::string exitStr = std::string("exitPlugin") + postFix;
			std::string executePluginCommandStr = std::string("executePluginCommand") + postFix;
			std::string preTickPluginCallbackStr = std::string("preTickPluginCallback") + postFix;
			std::string postTickPluginCallback = std::string("postTickPluginCallback") + postFix;
			std::string processNotificationsStr = std::string("processNotifications") + postFix;
			std::string getRendererStr = std::string("getRenderInterface") + postFix;

			plugin->m_initFunc = (PFN_INIT)B3_DYNLIB_IMPORT(pluginHandle, initStr.c_str());
			plugin->m_exitFunc = (PFN_EXIT)B3_DYNLIB_IMPORT(pluginHandle, exitStr.c_str());
			plugin->m_executeCommandFunc = (PFN_EXECUTE)B3_DYNLIB_IMPORT(pluginHandle, executePluginCommandStr.c_str());
			plugin->m_preTickFunc = (PFN_TICK)B3_DYNLIB_IMPORT(pluginHandle, preTickPluginCallbackStr.c_str());
			plugin->m_postTickFunc = (PFN_TICK)B3_DYNLIB_IMPORT(pluginHandle, postTickPluginCallback.c_str());
			plugin->m_processNotificationsFunc = (PFN_TICK)B3_DYNLIB_IMPORT(pluginHandle, processNotificationsStr.c_str());
			plugin->m_getRendererFunc =  (PFN_GET_RENDER_INTERFACE)B3_DYNLIB_IMPORT(pluginHandle, getRendererStr.c_str());
			
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
					m_data->m_pluginMap.insert(pluginPath, pluginUniqueId);
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

	//for now, automatically select the loaded plugin as active renderer. If wanted, we can add some 'select' mechanism.
	if (pluginUniqueId>=0)
	{
		b3PluginHandle* plugin = m_data->m_plugins.getHandle(pluginUniqueId);
		if (plugin && plugin->m_getRendererFunc)
		{
			selectPluginRenderer(pluginUniqueId);
		}
	}

	return pluginUniqueId;
}

void b3PluginManager::unloadPlugin(int pluginUniqueId)
{
	b3PluginHandle* plugin = m_data->m_plugins.getHandle(pluginUniqueId);
	if (plugin)
	{
		b3PluginContext context = {0};
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
		int* pluginUidPtr = m_data->m_pluginMap.getAtIndex(i);
		b3PluginHandle* plugin = 0;

		if (pluginUidPtr)
		{
			int pluginUid = *pluginUidPtr;
			plugin = m_data->m_plugins.getHandle(pluginUid);
		}
		else
		{
			continue;
		}
		
		PFN_TICK  tick = isPreTick? plugin->m_preTickFunc : plugin->m_postTickFunc;
		if (tick)
		{
			b3PluginContext context = {0};
			context.m_userPointer = plugin->m_userPointer;
			context.m_physClient = (b3PhysicsClientHandle) m_data->m_physicsDirect;
			context.m_numMouseEvents = m_data->m_mouseEvents.size();
			context.m_mouseEvents = m_data->m_mouseEvents.size() ? &m_data->m_mouseEvents[0] : 0;
			context.m_numKeyEvents = m_data->m_keyEvents.size();
			context.m_keyEvents = m_data->m_keyEvents.size() ? &m_data->m_keyEvents[0] : 0;
			context.m_numVRControllerEvents = m_data->m_vrEvents.size();
			context.m_vrControllerEvents = m_data->m_vrEvents.size()? &m_data->m_vrEvents[0]:0;
			int result = tick(&context);
			plugin->m_userPointer = context.m_userPointer;
		}
	}
}

void b3PluginManager::reportNotifications()
{
	b3AlignedObjectArray<b3Notification> &notifications = m_data->m_notifications[m_data->m_activeNotificationsBufferIndex];
	if (notifications.size() == 0)
	{
		return;
	}

	// Swap notification buffers.
	m_data->m_activeNotificationsBufferIndex = 1 - m_data->m_activeNotificationsBufferIndex;

	for (int i=0;i<m_data->m_pluginMap.size();i++)
	{
		int* pluginUidPtr = m_data->m_pluginMap.getAtIndex(i);
		b3PluginHandle* plugin = 0;

		if (pluginUidPtr)
		{
			int pluginUid = *pluginUidPtr;
			plugin = m_data->m_plugins.getHandle(pluginUid);
		}
		else
		{
			continue;
		}

		if (plugin->m_processNotificationsFunc) {
			b3PluginContext context = {0};
			context.m_userPointer = plugin->m_userPointer;
			context.m_physClient = (b3PhysicsClientHandle) m_data->m_physicsDirect;
			context.m_numNotifications = notifications.size();
			context.m_notifications = notifications.size() ? &notifications[0] : 0;
			plugin->m_processNotificationsFunc(&context);
		}
	}
	notifications.clear();
}

int b3PluginManager::executePluginCommand(int pluginUniqueId, const b3PluginArguments* arguments)
{
	int result = -1;

	b3PluginHandle* plugin = m_data->m_plugins.getHandle(pluginUniqueId);
	if (plugin)
	{
		b3PluginContext context = {0};
		context.m_userPointer = plugin->m_userPointer;
		context.m_physClient = (b3PhysicsClientHandle) m_data->m_physicsDirect;

		result = plugin->m_executeCommandFunc(&context, arguments);
		plugin->m_userPointer = context.m_userPointer;
	}
	return result;
}


int b3PluginManager::registerStaticLinkedPlugin(const char* pluginPath, PFN_INIT initFunc,PFN_EXIT exitFunc, PFN_EXECUTE executeCommandFunc, PFN_TICK preTickFunc, PFN_TICK postTickFunc, PFN_GET_RENDER_INTERFACE getRendererFunc)
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
	pluginHandle->m_getRendererFunc = getRendererFunc;
	pluginHandle->m_pluginHandle = 0;
	pluginHandle->m_pluginPath = pluginPath;
	pluginHandle->m_userPointer = 0;
	

	m_data->m_pluginMap.insert(pluginPath, pluginUniqueId);

	{
		b3PluginContext context = {0};
		context.m_userPointer = 0;
		context.m_physClient = (b3PhysicsClientHandle) m_data->m_physicsDirect;

		int result = pluginHandle->m_initFunc(&context);
		pluginHandle->m_userPointer = context.m_userPointer;
	}
	return pluginUniqueId;
}

void b3PluginManager::selectPluginRenderer(int pluginUniqueId)
{
	m_data->m_activeRendererPluginUid = pluginUniqueId;
}

UrdfRenderingInterface* b3PluginManager::getRenderInterface()
{
	UrdfRenderingInterface* renderer = 0;

	if (m_data->m_activeRendererPluginUid>=0)
	{
		b3PluginHandle* plugin = m_data->m_plugins.getHandle(m_data->m_activeRendererPluginUid);
		if (plugin)
		{
			b3PluginContext context = {0};
			context.m_userPointer = plugin->m_userPointer;
			context.m_physClient = (b3PhysicsClientHandle) m_data->m_physicsDirect;

			renderer = plugin->m_getRendererFunc(&context);
		}
	}
	return renderer;
}

