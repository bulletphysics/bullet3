#ifndef B3_PLUGIN_CONTEXT_H
#define B3_PLUGIN_CONTEXT_H

#include "../PhysicsClientC_API.h"

struct b3PluginContext
{
	b3PhysicsClientHandle	m_physClient;

	//plugin can modify the m_userPointer to store persistent object pointer (class or struct instance etc)
	void* m_userPointer;
	
	const struct b3VRControllerEvent* m_vrControllerEvents;
	int m_numVRControllerEvents;
	const struct b3KeyboardEvent* m_keyEvents;
	int m_numKeyEvents;
	const struct b3MouseEvent* m_mouseEvents;
	int m_numMouseEvents;
	const struct b3Notification* m_notifications;
	int m_numNotifications;
};






#endif //B3_PLUGIN_CONTEXT_H