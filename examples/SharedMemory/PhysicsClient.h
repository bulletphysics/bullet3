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

	virtual int getNumBodies() const = 0;

	virtual int getBodyUniqueId(int serialIndex) const = 0;

	virtual bool getBodyInfo(int bodyUniqueId, struct b3BodyInfo& info) const = 0;
	
    virtual int getNumJoints(int bodyUniqueId) const = 0;

    virtual bool getJointInfo(int bodyUniqueId, int jointIndex, struct b3JointInfo& info) const = 0;

    virtual int getNumUserConstraints() const = 0;
    
    virtual int getUserConstraintInfo(int constraintUniqueId, struct b3UserConstraint& info) const = 0;
	
	virtual int getUserConstraintId(int serialIndex) const = 0;
    
    virtual void setSharedMemoryKey(int key) = 0;

    virtual void uploadBulletFileToSharedMemory(const char* data, int len) = 0;

    virtual int getNumDebugLines() const = 0;

    virtual const float* getDebugLinesFrom() const = 0;
    virtual const float* getDebugLinesTo() const = 0;
    virtual const float* getDebugLinesColor() const = 0;

	virtual void getCachedCameraImage(struct b3CameraImageData* cameraData)=0;
	
	virtual void getCachedContactPointInformation(struct b3ContactInformation* contactPointData)=0;
	
	virtual void getCachedOverlappingObjects(struct b3AABBOverlapData* overlappingObjects) = 0;

	virtual void getCachedVisualShapeInformation(struct b3VisualShapeInformation* visualShapesInfo) = 0;

	virtual void getCachedVREvents(struct b3VREventsData* vrEventsData) = 0;

	virtual void getCachedKeyboardEvents(struct b3KeyboardEventsData* keyboardEventsData) = 0;

	virtual void getCachedMouseEvents(struct b3MouseEventsData* mouseEventsData) = 0;

	virtual void getCachedRaycastHits(struct b3RaycastInformation* raycastHits) = 0;

	virtual void setTimeOut(double timeOutInSeconds) = 0;
	virtual double getTimeOut() const  = 0;

};

#endif  // BT_PHYSICS_CLIENT_API_H
