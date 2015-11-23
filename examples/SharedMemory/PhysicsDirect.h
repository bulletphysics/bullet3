#ifndef PHYSICS_DIRECT_H
#define PHYSICS_DIRECT_H

//#include "SharedMemoryCommands.h"


#include "PhysicsClient.h"
#include "LinearMath/btVector3.h"

///todo: the PhysicsClient API was designed with shared memory in mind, 
///now it become more general we need to move out the shared memory specifics away
///for example naming [disconnectSharedMemory -> disconnect] [ move setSharedMemoryKey to shared memory specific subclass ]
///PhysicsDirect executes the commands directly, without transporting them or having a separate server executing commands
class PhysicsDirect : public PhysicsClient 
{
protected:

	struct PhysicsDirectInternalData* m_data;

	bool processDebugLines(const struct SharedMemoryCommand& orgCommand);

public:

    PhysicsDirect();
    
    virtual ~PhysicsDirect();

	  // return true if connection succesfull, can also check 'isConnected'
    virtual bool connect();

	////todo: rename to 'disconnect'
    virtual void disconnectSharedMemory();

    virtual bool isConnected() const;

    // return non-null if there is a status, nullptr otherwise
    virtual const  SharedMemoryStatus* processServerStatus();

    virtual  SharedMemoryCommand* getAvailableSharedMemoryCommand();

    virtual bool canSubmitCommand() const;

    virtual bool submitClientCommand(const struct SharedMemoryCommand& command);

    virtual int getNumJoints(int bodyIndex) const;

    virtual void getJointInfo(int bodyIndex, int jointIndex, struct b3JointInfo& info) const;

	///todo: move this out of the
    virtual void setSharedMemoryKey(int key);

    void uploadBulletFileToSharedMemory(const char* data, int len);

    virtual int getNumDebugLines() const;

    virtual const float* getDebugLinesFrom() const;
    virtual const float* getDebugLinesTo() const;
    virtual const float* getDebugLinesColor() const;

};

#endif //PHYSICS_DIRECT_H
