
#include "b3PluginManager.h"
#include "Bullet3Common/b3HashMap.h"
#include "Bullet3Common/b3ResizablePool.h"
#include "PhysicsClientC_API.h"
#include "PhysicsDirect.h"
#include "plugins/b3PluginContext.h"
#include "../Utils/b3BulletDefaultFileIO.h"
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define VC_EXTRALEAN
#include <windows.h>

typedef HMODULE B3_DYNLIB_HANDLE;

#define B3_DYNLIB_OPEN LoadLibraryA
#define B3_DYNLIB_CLOSE FreeLibrary
#define B3_DYNLIB_IMPORT GetProcAddress
#else
#include <dlfcn.h>

typedef void* B3_DYNLIB_HANDLE;

#ifdef B3_USE_DLMOPEN
#define B3_DYNLIB_OPEN(path) dlmopen(LM_ID_NEWLM, path, RTLD_LAZY)
#else
#define B3_DYNLIB_OPEN(path) dlopen(path, RTLD_NOW | RTLD_GLOBAL)
#endif
#define B3_DYNLIB_CLOSE dlclose
#define B3_DYNLIB_IMPORT dlsym
#endif

struct b3Plugin
{
	B3_DYNLIB_HANDLE m_pluginHandle;
	bool m_ownsPluginHandle;
	bool m_isInitialized;
	std::string m_pluginPath;
	int m_pluginUniqueId;
	PFN_INIT m_initFunc;
	PFN_EXIT m_exitFunc;
	PFN_EXECUTE m_executeCommandFunc;

	PFN_TICK m_preTickFunc;
	PFN_TICK m_postTickFunc;
	PFN_TICK m_processNotificationsFunc;
	PFN_TICK m_processClientCommandsFunc;

	PFN_GET_RENDER_INTERFACE m_getRendererFunc;
	PFN_GET_COLLISION_INTERFACE m_getCollisionFunc;
	PFN_GET_FILEIO_INTERFACE m_getFileIOFunc;

	void* m_userPointer;

	b3Plugin()
		: m_pluginHandle(0),
		  m_ownsPluginHandle(false),
		  m_isInitialized(false),
		  m_pluginUniqueId(-1),
		  m_initFunc(0),
		  m_exitFunc(0),
		  m_executeCommandFunc(0),
		  m_preTickFunc(0),
		  m_postTickFunc(0),
		  m_processNotificationsFunc(0),
		  m_processClientCommandsFunc(0),
		  m_getRendererFunc(0),
		  m_getCollisionFunc(0),
		  m_getFileIOFunc(0),
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
		m_processNotificationsFunc = 0;
		m_processClientCommandsFunc = 0;
		m_getRendererFunc = 0;
		m_getCollisionFunc = 0;
		m_getFileIOFunc = 0;
		m_userPointer = 0;
		m_isInitialized = false;
	}
};

typedef b3PoolBodyHandle<b3Plugin> b3PluginHandle;

struct b3PluginManagerInternalData
{
	b3ResizablePool<b3PluginHandle> m_plugins;
	b3HashMap<b3HashString, int> m_pluginMap;
	PhysicsDirect* m_physicsDirect;
	PhysicsCommandProcessorInterface* m_rpcCommandProcessorInterface;
	b3AlignedObjectArray<b3KeyboardEvent> m_keyEvents;
	b3AlignedObjectArray<b3VRControllerEvent> m_vrEvents;
	b3AlignedObjectArray<b3MouseEvent> m_mouseEvents;
	b3AlignedObjectArray<b3Notification> m_notifications[2];
	int m_activeNotificationsBufferIndex;
	int m_activeRendererPluginUid;
	int m_activeCollisionPluginUid;
	int m_numNotificationPlugins;
	int m_activeFileIOPluginUid;
	b3BulletDefaultFileIO m_defaultFileIO;
	
	b3PluginManagerInternalData()
		: m_rpcCommandProcessorInterface(0), m_activeNotificationsBufferIndex(0), m_activeRendererPluginUid(-1), m_activeCollisionPluginUid(-1), m_numNotificationPlugins(0), m_activeFileIOPluginUid(-1)
	{
	}
};

b3PluginManager::b3PluginManager(class PhysicsCommandProcessorInterface* physSdk)
{
	m_data = new b3PluginManagerInternalData;
	m_data->m_rpcCommandProcessorInterface = physSdk;
	m_data->m_physicsDirect = new PhysicsDirect(physSdk, false);
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
	if (m_data->m_numNotificationPlugins > 0)
	{
		m_data->m_notifications[m_data->m_activeNotificationsBufferIndex].push_back(notification);
	}
}

int b3PluginManager::loadPlugin(const char* pluginPath, const char* postFixStr)
{
	int pluginUniqueId = -1;

	int* pluginUidPtr = m_data->m_pluginMap.find(pluginPath);
	if (pluginUidPtr)
	{
		//already loaded
		pluginUniqueId = *pluginUidPtr;
		b3PluginHandle* plugin = m_data->m_plugins.getHandle(pluginUniqueId);
		if (!plugin->m_isInitialized)
		{
			b3PluginContext context = {0};
			context.m_userPointer = 0;
			context.m_physClient = (b3PhysicsClientHandle)m_data->m_physicsDirect;
			context.m_rpcCommandProcessorInterface = m_data->m_rpcCommandProcessorInterface;
			int result = plugin->m_initFunc(&context);
			plugin->m_isInitialized = true;
			plugin->m_userPointer = context.m_userPointer;
		}
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
			std::string processClientCommandsStr = std::string("processClientCommands") + postFix;
			std::string getRendererStr = std::string("getRenderInterface") + postFix;
			std::string getCollisionStr = std::string("getCollisionInterface") + postFix;
			std::string getFileIOStr = std::string("getFileIOInterface") + postFix;
			
			plugin->m_initFunc = (PFN_INIT)B3_DYNLIB_IMPORT(pluginHandle, initStr.c_str());
			plugin->m_exitFunc = (PFN_EXIT)B3_DYNLIB_IMPORT(pluginHandle, exitStr.c_str());
			plugin->m_executeCommandFunc = (PFN_EXECUTE)B3_DYNLIB_IMPORT(pluginHandle, executePluginCommandStr.c_str());
			plugin->m_preTickFunc = (PFN_TICK)B3_DYNLIB_IMPORT(pluginHandle, preTickPluginCallbackStr.c_str());
			plugin->m_postTickFunc = (PFN_TICK)B3_DYNLIB_IMPORT(pluginHandle, postTickPluginCallback.c_str());
			plugin->m_processNotificationsFunc = (PFN_TICK)B3_DYNLIB_IMPORT(pluginHandle, processNotificationsStr.c_str());

			if (plugin->m_processNotificationsFunc)
			{
				m_data->m_numNotificationPlugins++;
			}
			plugin->m_processClientCommandsFunc = (PFN_TICK)B3_DYNLIB_IMPORT(pluginHandle, processClientCommandsStr.c_str());

			plugin->m_getRendererFunc = (PFN_GET_RENDER_INTERFACE)B3_DYNLIB_IMPORT(pluginHandle, getRendererStr.c_str());
			plugin->m_getCollisionFunc = (PFN_GET_COLLISION_INTERFACE)B3_DYNLIB_IMPORT(pluginHandle, getCollisionStr.c_str());
			plugin->m_getFileIOFunc = (PFN_GET_FILEIO_INTERFACE)B3_DYNLIB_IMPORT(pluginHandle, getFileIOStr.c_str());
			

			if (plugin->m_initFunc && plugin->m_exitFunc && plugin->m_executeCommandFunc)
			{
				b3PluginContext context;
				context.m_userPointer = plugin->m_userPointer;
				context.m_physClient = (b3PhysicsClientHandle)m_data->m_physicsDirect;
				context.m_rpcCommandProcessorInterface = m_data->m_rpcCommandProcessorInterface;
				int version = plugin->m_initFunc(&context);
				plugin->m_isInitialized = true;
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
#ifdef _WIN32
#else
			b3Warning("Error: %s\n", dlerror());
#endif
		}
		if (!ok)
		{
			m_data->m_plugins.freeHandle(pluginUniqueId);
			pluginUniqueId = -1;
		}
	}

	//for now, automatically select the loaded plugin as active renderer.
	if (pluginUniqueId >= 0)
	{
		b3PluginHandle* plugin = m_data->m_plugins.getHandle(pluginUniqueId);
		if (plugin && plugin->m_getRendererFunc)
		{
			selectPluginRenderer(pluginUniqueId);
		}
	}

	//for now, automatically select the loaded plugin as active collision plugin.
	if (pluginUniqueId >= 0)
	{
		b3PluginHandle* plugin = m_data->m_plugins.getHandle(pluginUniqueId);
		if (plugin && plugin->m_getCollisionFunc)
		{
			selectCollisionPlugin(pluginUniqueId);
		}
	}
	//for now, automatically select the loaded plugin as active fileIO plugin.
	if (pluginUniqueId >= 0)
	{
		b3PluginHandle* plugin = m_data->m_plugins.getHandle(pluginUniqueId);
		if (plugin && plugin->m_getFileIOFunc)
		{
			selectFileIOPlugin(pluginUniqueId);
		}
	}

	return pluginUniqueId;
}

void b3PluginManager::unloadPlugin(int pluginUniqueId)
{
	b3PluginHandle* plugin = m_data->m_plugins.getHandle(pluginUniqueId);
	if (plugin)
	{
		if (plugin->m_processNotificationsFunc)
		{
			m_data->m_numNotificationPlugins--;
		}
		b3PluginContext context = {0};
		context.m_userPointer = plugin->m_userPointer;
		context.m_physClient = (b3PhysicsClientHandle)m_data->m_physicsDirect;

		if (plugin->m_isInitialized)
		{
			plugin->m_exitFunc(&context);
			plugin->m_userPointer = 0;
			plugin->m_isInitialized = false;
		}
		m_data->m_pluginMap.remove(plugin->m_pluginPath.c_str());
		m_data->m_plugins.freeHandle(pluginUniqueId);
	}
}

void b3PluginManager::tickPlugins(double timeStep, b3PluginManagerTickMode tickMode)
{
	for (int i = 0; i < m_data->m_pluginMap.size(); i++)
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

		PFN_TICK tick = 0;
		switch (tickMode)
		{
			case B3_PRE_TICK_MODE:
			{
				tick = plugin->m_preTickFunc;
				break;
			}
			case B3_POST_TICK_MODE:
			{
				tick = plugin->m_postTickFunc;
				break;
			}
			case B3_PROCESS_CLIENT_COMMANDS_TICK:
			{
				tick = plugin->m_processClientCommandsFunc;
				break;
			}
			default:
			{
			}
		}

		if (tick)
		{
			b3PluginContext context = {0};
			context.m_userPointer = plugin->m_userPointer;
			context.m_physClient = (b3PhysicsClientHandle)m_data->m_physicsDirect;
			context.m_numMouseEvents = m_data->m_mouseEvents.size();
			context.m_mouseEvents = m_data->m_mouseEvents.size() ? &m_data->m_mouseEvents[0] : 0;
			context.m_numKeyEvents = m_data->m_keyEvents.size();
			context.m_keyEvents = m_data->m_keyEvents.size() ? &m_data->m_keyEvents[0] : 0;
			context.m_numVRControllerEvents = m_data->m_vrEvents.size();
			context.m_vrControllerEvents = m_data->m_vrEvents.size() ? &m_data->m_vrEvents[0] : 0;
			if (tickMode == B3_PROCESS_CLIENT_COMMANDS_TICK)
			{
				context.m_rpcCommandProcessorInterface = m_data->m_rpcCommandProcessorInterface;
			}
			int result = tick(&context);
			plugin->m_userPointer = context.m_userPointer;
		}
	}
}

void b3PluginManager::reportNotifications()
{
	b3AlignedObjectArray<b3Notification>& notifications = m_data->m_notifications[m_data->m_activeNotificationsBufferIndex];
	if (notifications.size() == 0)
	{
		return;
	}

	// Swap notification buffers.
	m_data->m_activeNotificationsBufferIndex = 1 - m_data->m_activeNotificationsBufferIndex;

	for (int i = 0; i < m_data->m_pluginMap.size(); i++)
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

		if (plugin->m_processNotificationsFunc)
		{
			b3PluginContext context = {0};
			context.m_userPointer = plugin->m_userPointer;
			context.m_physClient = (b3PhysicsClientHandle)m_data->m_physicsDirect;
			context.m_numNotifications = notifications.size();
			context.m_notifications = notifications.size() ? &notifications[0] : 0;
			plugin->m_processNotificationsFunc(&context);
		}
	}
	notifications.resize(0);
}

int b3PluginManager::executePluginCommand(int pluginUniqueId, const b3PluginArguments* arguments)
{
	int result = -1;

	b3PluginHandle* plugin = m_data->m_plugins.getHandle(pluginUniqueId);
	if (plugin)
	{
		b3PluginContext context = {0};
		context.m_userPointer = plugin->m_userPointer;
		context.m_physClient = (b3PhysicsClientHandle)m_data->m_physicsDirect;
		context.m_rpcCommandProcessorInterface = m_data->m_rpcCommandProcessorInterface;
		result = plugin->m_executeCommandFunc(&context, arguments);
		plugin->m_userPointer = context.m_userPointer;
	}
	return result;
}



int b3PluginManager::registerStaticLinkedPlugin(const char* pluginPath,  b3PluginFunctions& functions, bool initPlugin)
{
	b3Plugin orgPlugin;

	int pluginUniqueId = m_data->m_plugins.allocHandle();
	b3PluginHandle* pluginHandle = m_data->m_plugins.getHandle(pluginUniqueId);
	pluginHandle->m_pluginHandle = 0;
	pluginHandle->m_ownsPluginHandle = false;
	pluginHandle->m_pluginUniqueId = pluginUniqueId;
	pluginHandle->m_executeCommandFunc = functions.m_executeCommandFunc;
	pluginHandle->m_exitFunc = functions.m_exitFunc;
	pluginHandle->m_initFunc = functions.m_initFunc;
	pluginHandle->m_preTickFunc = functions.m_preTickFunc;
	pluginHandle->m_postTickFunc = functions.m_postTickFunc;
	pluginHandle->m_getRendererFunc = functions.m_getRendererFunc;
	pluginHandle->m_getCollisionFunc = functions.m_getCollisionFunc;
	pluginHandle->m_processClientCommandsFunc = functions.m_processClientCommandsFunc;
	pluginHandle->m_getFileIOFunc = functions.m_fileIoFunc;
	pluginHandle->m_pluginHandle = 0;
	pluginHandle->m_pluginPath = pluginPath;
	pluginHandle->m_userPointer = 0;

	if (pluginHandle->m_processNotificationsFunc)
	{
		m_data->m_numNotificationPlugins++;
	}

	m_data->m_pluginMap.insert(pluginPath, pluginUniqueId);

	if (initPlugin)
	{
		b3PluginContext context = {0};
		context.m_userPointer = 0;
		context.m_physClient = (b3PhysicsClientHandle)m_data->m_physicsDirect;
		context.m_rpcCommandProcessorInterface = m_data->m_rpcCommandProcessorInterface;
		int result = pluginHandle->m_initFunc(&context);
		pluginHandle->m_isInitialized = true;
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

	if (m_data->m_activeRendererPluginUid >= 0)
	{
		b3PluginHandle* plugin = m_data->m_plugins.getHandle(m_data->m_activeRendererPluginUid);
		if (plugin && plugin->m_getRendererFunc)
		{
			b3PluginContext context = {0};
			context.m_userPointer = plugin->m_userPointer;
			context.m_physClient = (b3PhysicsClientHandle)m_data->m_physicsDirect;
			renderer = plugin->m_getRendererFunc(&context);
		}
	}
	return renderer;
}

void b3PluginManager::selectFileIOPlugin(int pluginUniqueId)
{
	m_data->m_activeFileIOPluginUid = pluginUniqueId;
}

struct CommonFileIOInterface* b3PluginManager::getFileIOInterface()
{
	CommonFileIOInterface* fileIOInterface = 0;
	if (m_data->m_activeFileIOPluginUid >= 0)
	{
		b3PluginHandle* plugin = m_data->m_plugins.getHandle(m_data->m_activeFileIOPluginUid);
		if (plugin && plugin->m_getFileIOFunc)
		{
			b3PluginContext context = {0};
			context.m_userPointer = plugin->m_userPointer;
			context.m_physClient = (b3PhysicsClientHandle)m_data->m_physicsDirect;
			fileIOInterface = plugin->m_getFileIOFunc(&context);
		}
	}
	if (fileIOInterface==0)
	{
		return &m_data->m_defaultFileIO;
	}
	return fileIOInterface;
}

void b3PluginManager::selectCollisionPlugin(int pluginUniqueId)
{
	m_data->m_activeCollisionPluginUid = pluginUniqueId;
}

struct b3PluginCollisionInterface* b3PluginManager::getCollisionInterface()
{
	b3PluginCollisionInterface* collisionInterface = 0;
	if (m_data->m_activeCollisionPluginUid >= 0)
	{
		b3PluginHandle* plugin = m_data->m_plugins.getHandle(m_data->m_activeCollisionPluginUid);
		if (plugin && plugin->m_getCollisionFunc)
		{
			b3PluginContext context = {0};
			context.m_userPointer = plugin->m_userPointer;
			context.m_physClient = (b3PhysicsClientHandle)m_data->m_physicsDirect;
			collisionInterface = plugin->m_getCollisionFunc(&context);
		}
	}
	return collisionInterface;
}
