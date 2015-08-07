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

	//return true if connection succesfull, can also check 'isConnected'
	virtual bool	connect();
    
    virtual void disconnectSharedMemory ();

	virtual bool	isConnected() const;

	// return true if there is a status, and fill in 'serverStatus'
	virtual bool	processServerStatus(SharedMemoryStatus& serverStatus);
	
	virtual bool	canSubmitCommand() const;
	
	virtual bool	submitClientCommand(const SharedMemoryCommand& command);

	virtual int		getNumJoints() const;
	
	virtual void	getJointInfo(int index, b3JointInfo& info) const;
	
	virtual void setSharedMemoryKey(int key);
};

#endif //BT_PHYSICS_CLIENT_API_H
