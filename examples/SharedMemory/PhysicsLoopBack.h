#ifndef PHYSICS_LOOP_BACK_H
#define PHYSICS_LOOP_BACK_H

//#include "SharedMemoryCommands.h"


#include "PhysicsClient.h"
#include "LinearMath/btVector3.h"

///todo: the PhysicsClient API was designed with shared memory in mind, 
///now it become more general we need to move out the shared memory specifics away
///for example naming [disconnectSharedMemory -> disconnect] [ move setSharedMemoryKey to shared memory specific subclass ]

class PhysicsLoopBack : public PhysicsClient 
{
	struct PhysicsLoopBackInternalData* m_data;

public:

	PhysicsLoopBack();

	virtual ~PhysicsLoopBack();

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

	virtual int getNumBodies() const;

	virtual int getBodyUniqueId(int serialIndex) const;

	virtual bool getBodyInfo(int bodyUniqueId, struct b3BodyInfo& info) const;

    virtual int getNumJoints(int bodyIndex) const;

    virtual bool getJointInfo(int bodyIndex, int jointIndex, struct b3JointInfo& info) const;

    virtual int getNumUserConstraints() const;
    
    virtual int getUserConstraintInfo(int constraintUniqueId, struct b3UserConstraint&info) const;
    
	///todo: move this out of the
    virtual void setSharedMemoryKey(int key);

    void uploadBulletFileToSharedMemory(const char* data, int len);

    virtual int getNumDebugLines() const;

    virtual const float* getDebugLinesFrom() const;
    virtual const float* getDebugLinesTo() const;
    virtual const float* getDebugLinesColor() const;
	virtual void getCachedCameraImage(struct b3CameraImageData* cameraData);
	
	virtual void getCachedContactPointInformation(struct b3ContactInformation* contactPointData);

	virtual void getCachedOverlappingObjects(struct b3AABBOverlapData* overlappingObjects);

	virtual void getCachedVisualShapeInformation(struct b3VisualShapeInformation* visualShapesInfo);

	virtual void getCachedVREvents(struct b3VREventsData* vrEventsData);

	virtual void getCachedRaycastHits(struct b3RaycastInformation* raycastHits);

};

#endif //PHYSICS_LOOP_BACK_H
