#ifndef BT_PHYSICS_CLIENT_API_H
#define BT_PHYSICS_CLIENT_API_H

#include "SharedMemoryCommands.h"


class PhysicsClientSharedMemory  //: public CommonPhysicsClientInterface
{
	struct PhysicsClientSharedMemoryInternalData*	m_data;
protected:

public:

	PhysicsClientSharedMemory();
	virtual ~PhysicsClientSharedMemory();

	//todo: implement 'allocateSharedMemory' from client side in 'connect' call
	virtual bool	connect(bool allowSharedMemoryInitialization = true);

	virtual bool	isConnected() const;

	virtual void	processServerStatus();

	virtual bool getLastServerStatus(ServerStatus& status)
	{
		return false;
	}
	
	virtual bool	canSubmitCommand() const;
	
	virtual bool	submitClientCommand(const SharedMemoryCommand& command);

};

#endif //BT_PHYSICS_CLIENT_API_H
