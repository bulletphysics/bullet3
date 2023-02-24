#ifndef BT_PHYSICS_CLIENT_SHARED_MEMORY_API_H
#define BT_PHYSICS_CLIENT_SHARED_MEMORY_API_H

#include "PhysicsClient.h"

//#include "SharedMemoryCommands.h"
#include "LinearMath/btVector3.h"

class PhysicsClientSharedMemory : public PhysicsClient
{
	

protected:
	struct PhysicsClientSharedMemoryInternalData* m_data;
	virtual void setSharedMemoryInterface(class SharedMemoryInterface* sharedMem);
	void processBodyJointInfo(int bodyUniqueId, const struct SharedMemoryStatus& serverCmd);
	void resetData();
	void removeCachedBody(int bodyUniqueId);
	void clearCachedBodies();
	virtual void renderSceneInternal(){};

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

	virtual int getNumBodies() const;

	virtual int getBodyUniqueId(int serialIndex) const;

	virtual bool getBodyInfo(int bodyUniqueId, struct b3BodyInfo& info) const;

	virtual int getNumJoints(int bodyUniqueId) const;

	virtual int getNumDofs(int bodyUniqueId) const;

	virtual bool getJointInfo(int bodyUniqueId, int jointIndex, struct b3JointInfo& info) const;

	virtual int getNumUserConstraints() const;

	virtual int getUserConstraintInfo(int constraintUniqueId, struct b3UserConstraint& info) const;

	virtual int getUserConstraintId(int serialIndex) const;

	virtual void setSharedMemoryKey(int key);

	virtual void uploadBulletFileToSharedMemory(const char* data, int len);

	virtual void uploadRaysToSharedMemory(struct SharedMemoryCommand& command, const double* rayFromWorldArray, const double* rayToWorldArray, int numRays);

	virtual int getNumDebugLines() const;

	virtual const float* getDebugLinesFrom() const;
	virtual const float* getDebugLinesTo() const;
	virtual const float* getDebugLinesColor() const;
	virtual void getCachedCameraImage(struct b3CameraImageData* cameraData);

	virtual void getCachedContactPointInformation(struct b3ContactInformation* contactPointData);

	virtual void getCachedOverlappingObjects(struct b3AABBOverlapData* overlappingObjects);

	virtual void getCachedVisualShapeInformation(struct b3VisualShapeInformation* visualShapesInfo);

	virtual void getCachedCollisionShapeInformation(struct b3CollisionShapeInformation* collisionShapesInfo);

	virtual void getCachedMeshData(struct b3MeshData* meshData);

	virtual void getCachedTetraMeshData(struct b3TetraMeshData* meshData);

	virtual void getCachedVREvents(struct b3VREventsData* vrEventsData);

	virtual void getCachedKeyboardEvents(struct b3KeyboardEventsData* keyboardEventsData);

	virtual void getCachedMouseEvents(struct b3MouseEventsData* mouseEventsData);

	virtual void getCachedRaycastHits(struct b3RaycastInformation* raycastHits);

	virtual void getCachedMassMatrix(int dofCountCheck, double* massMatrix);

	virtual bool getCachedReturnData(b3UserDataValue* returnData);

	virtual void setTimeOut(double timeOutInSeconds);
	virtual double getTimeOut() const;

	virtual bool getCachedUserData(int userDataId, struct b3UserDataValue& valueOut) const;
	virtual int getCachedUserDataId(int bodyUniqueId, int linkIndex, int visualShapeIndex, const char* key) const;
	virtual int getNumUserData(int bodyUniqueId) const;
	virtual void getUserDataInfo(int bodyUniqueId, int userDataIndex, const char** keyOut, int* userDataIdOut, int* linkIndexOut, int* visualShapeIndexOut) const;

	virtual void pushProfileTiming(const char* timingName);
	virtual void popProfileTiming();
};

#endif  // BT_PHYSICS_CLIENT_API_H
