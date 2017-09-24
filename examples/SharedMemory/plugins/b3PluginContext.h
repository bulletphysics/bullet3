#ifndef B3_PLUGIN_CONTEXT_H
#define B3_PLUGIN_CONTEXT_H

#include "../PhysicsClientC_API.h"

struct b3PluginContext
{

	b3PhysicsClientHandle	m_physClient;

	//plugin can modify the m_userPointer to store persistent object pointer (class or struct instance etc)
	void* m_userPointer;

};






#endif //B3_PLUGIN_CONTEXT_H