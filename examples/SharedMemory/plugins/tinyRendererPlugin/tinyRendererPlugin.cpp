
//tinyRenderer plugin

/*
import pybullet as p
p.connect(p.GUI)
plugin = p.loadPlugin("e:/develop/bullet3/bin/pybullet_tinyRendererPlugin_vs2010_x64_debug.dll","_tinyRendererPlugin")
print("plugin=",plugin)
p.loadURDF("r2d2.urdf")
while (1):
	p.getCameraImage(320,200)
*/

#include "tinyRendererPlugin.h"
#include "TinyRendererVisualShapeConverter.h"

#include "../../SharedMemoryPublic.h"
#include "../b3PluginContext.h"
#include <stdio.h>

struct MyRendererPluginClass
{
	TinyRendererVisualShapeConverter m_renderer;
	b3UserDataValue* m_returnData;
	MyRendererPluginClass()
		:m_returnData(0)
	{
	}
	virtual ~MyRendererPluginClass()
	{
		if (m_returnData)
		{
			delete[] m_returnData->m_data1;
		}
		delete m_returnData;
	}
};

B3_SHARED_API int initPlugin_tinyRendererPlugin(struct b3PluginContext* context)
{
	MyRendererPluginClass* obj = new MyRendererPluginClass();
	context->m_userPointer = obj;
	return SHARED_MEMORY_MAGIC_NUMBER;
}

B3_SHARED_API int executePluginCommand_tinyRendererPlugin(struct b3PluginContext* context, const struct b3PluginArguments* arguments)
{
	MyRendererPluginClass* obj = (MyRendererPluginClass*)context->m_userPointer;
	if (obj->m_returnData==0)
	{
		obj->m_returnData = new b3UserDataValue();
		obj->m_returnData->m_type = 1;
		obj->m_returnData->m_length = 123;
		char* data = new char[obj->m_returnData->m_length];
		for (int i = 0; i < obj->m_returnData->m_length; i++)
		{
			data[i] = i;
		}
		obj->m_returnData->m_data1 = data;
	}
	context->m_returnData = obj->m_returnData;
	return -1;
}

B3_SHARED_API void exitPlugin_tinyRendererPlugin(struct b3PluginContext* context)
{
	MyRendererPluginClass* obj = (MyRendererPluginClass*)context->m_userPointer;
	delete obj;
	context->m_userPointer = 0;
}

//all the APIs below are optional
B3_SHARED_API struct UrdfRenderingInterface* getRenderInterface_tinyRendererPlugin(struct b3PluginContext* context)
{
	MyRendererPluginClass* obj = (MyRendererPluginClass*)context->m_userPointer;
	return &obj->m_renderer;
}
