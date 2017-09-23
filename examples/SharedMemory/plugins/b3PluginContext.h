#ifndef B3_PLUGIN_CONTEXT_H
#define B3_PLUGIN_CONTEXT_H

#include "../PhysicsClientC_API.h"

struct b3PluginContext
{
	const char* m_arguments;

	b3PhysicsClientHandle	m_physClient;
};

#endif //B3_PLUGIN_CONTEXT_H