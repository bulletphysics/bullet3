#ifndef BT_PHYSICS_CLIENT_SHARED_MEMORY_API_H
#define BT_PHYSICS_CLIENT_SHARED_MEMORY_API_H

#include "PhysicsClient.h"

//#include "SharedMemoryCommands.h"
#include "LinearMath/btVector3.h"

class PhysicsClientSharedMemory : public PhysicsClient {
    struct PhysicsClientSharedMemoryInternalData* m_data;

protected:
public:
    PhysicsClientSharedMemory();
    virtual ~PhysicsClientSharedMemory();

    // return true if connection succesfull, can also check 'isConnected'
    virtual bool connect();

    virtual void disconnectSharedMemory();

    virtual bool isConnected() const;

    // return non-null if there is a status, nullptr otherwise
    virtual const struct SharedMemoryStatus* processServerStatus();

    virtual struct SharedMemoryCommand* getAvailableSharedMemoryCommand();

    virtual bool canSubmitCommand() const;

    virtual bool submitClientCommand(const struct SharedMemoryCommand& command);

    virtual int getNumJoints(int bodyIndex) const;

    virtual void getJointInfo(int bodyIndex, int jointIndex, struct b3JointInfo& info) const;

    virtual void setSharedMemoryKey(int key);

    virtual void uploadBulletFileToSharedMemory(const char* data, int len);

    virtual int getNumDebugLines() const;

    virtual const float* getDebugLinesFrom() const;
    virtual const float* getDebugLinesTo() const;
    virtual const float* getDebugLinesColor() const;
};

#endif  // BT_PHYSICS_CLIENT_API_H
