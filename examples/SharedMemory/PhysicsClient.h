#ifndef BT_PHYSICS_CLIENT_API_H
#define BT_PHYSICS_CLIENT_API_H

//#include "SharedMemoryCommands.h"
#include "LinearMath/btVector3.h"

class PhysicsClient {
public:
    virtual ~PhysicsClient();

    // return true if connection succesfull, can also check 'isConnected'
    virtual bool connect() = 0;

    virtual void disconnectSharedMemory() = 0;

    virtual bool isConnected() const = 0;

    // return non-null if there is a status, nullptr otherwise
    virtual const struct SharedMemoryStatus* processServerStatus() = 0;

    virtual struct SharedMemoryCommand* getAvailableSharedMemoryCommand() = 0;

    virtual bool canSubmitCommand() const = 0;

    virtual bool submitClientCommand(const struct SharedMemoryCommand& command) = 0;

    virtual int getNumJoints(int bodyIndex) const = 0;

    virtual void getJointInfo(int bodyIndex, int jointIndex, struct b3JointInfo& info) const = 0;

    virtual void setSharedMemoryKey(int key) = 0;

    virtual void uploadBulletFileToSharedMemory(const char* data, int len) = 0;

    virtual int getNumDebugLines() const = 0;

    virtual const float* getDebugLinesFrom() const = 0;
    virtual const float* getDebugLinesTo() const = 0;
    virtual const float* getDebugLinesColor() const = 0;
};

#endif  // BT_PHYSICS_CLIENT_API_H
