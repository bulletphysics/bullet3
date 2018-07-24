#ifndef PHYSICS_DIRECT_H
#define PHYSICS_DIRECT_H

//#include "SharedMemoryCommands.h"


#include "PhysicsClient.h"
#include "LinearMath/btVector3.h"

///PhysicsDirect executes the commands directly, without transporting them or having a separate server executing commands
class PhysicsDirect : public PhysicsClient 
{
protected:

	struct PhysicsDirectInternalData* m_data;

	bool processDebugLines(const struct SharedMemoryCommand& orgCommand);

	bool processCamera(const struct SharedMemoryCommand& orgCommand);

    bool processContactPointData(const struct SharedMemoryCommand& orgCommand);

	bool processOverlappingObjects(const struct SharedMemoryCommand& orgCommand);

	bool processVisualShapeData(const struct SharedMemoryCommand& orgCommand);
	
    void processBodyJointInfo(int bodyUniqueId, const struct SharedMemoryStatus& serverCmd);

	void processAddUserData(const struct SharedMemoryStatus& serverCmd);

	void postProcessStatus(const struct SharedMemoryStatus& serverCmd);

	void resetData();

	void removeCachedBody(int bodyUniqueId);

public:

	PhysicsDirect(class PhysicsCommandProcessorInterface* physSdk, bool passSdkOwnership);
    
    virtual ~PhysicsDirect();

	// return true if connection succesfull, can also check 'isConnected'
	//it is OK to pass a null pointer for the gui helper
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
    
    virtual int getUserConstraintInfo(int constraintUniqueId, struct b3UserConstraint& info) const;
	
	virtual int getUserConstraintId(int serialIndex) const;
    
	///todo: move this out of the
    virtual void setSharedMemoryKey(int key);

    void uploadBulletFileToSharedMemory(const char* data, int len);
    
	virtual void uploadRaysToSharedMemory(struct SharedMemoryCommand& command, const double* rayFromWorldArray, const double* rayToWorldArray, int numRays);

    virtual int getNumDebugLines() const;

    virtual const float* getDebugLinesFrom() const;
    virtual const float* getDebugLinesTo() const;
    virtual const float* getDebugLinesColor() const;

	virtual void getCachedCameraImage(b3CameraImageData* cameraData);

    virtual void getCachedContactPointInformation(struct b3ContactInformation* contactPointData);

	virtual void getCachedOverlappingObjects(struct b3AABBOverlapData* overlappingObjects);

	virtual void getCachedVisualShapeInformation(struct b3VisualShapeInformation* visualShapesInfo);
	
	virtual void getCachedCollisionShapeInformation(struct b3CollisionShapeInformation* collisionShapesInfo);

	virtual void getCachedVREvents(struct b3VREventsData* vrEventsData);

	virtual void getCachedKeyboardEvents(struct b3KeyboardEventsData* keyboardEventsData);

	virtual void getCachedMouseEvents(struct b3MouseEventsData* mouseEventsData);

	virtual void getCachedRaycastHits(struct b3RaycastInformation* raycastHits);

	virtual void getCachedMassMatrix(int dofCountCheck, double* massMatrix);

	//the following APIs are for internal use for visualization:
	virtual bool connect(struct GUIHelperInterface* guiHelper);
	virtual void renderScene();
	virtual void debugDraw(int debugDrawMode);

	virtual void setTimeOut(double timeOutInSeconds);
	virtual double getTimeOut() const;

	virtual bool getCachedUserData(int userDataId, struct b3UserDataValue &valueOut) const;
	virtual int getCachedUserDataId(int bodyUniqueId, int linkIndex, int visualShapeIndex, const char *key) const;
	virtual int getNumUserData(int bodyUniqueId) const;
	virtual void getUserDataInfo(int bodyUniqueId, int userDataIndex, const char **keyOut, int *userDataIdOut, int *linkIndexOut, int *visualShapeIndexOut) const;
	
	virtual void pushProfileTiming(const char* timingName);
	virtual void popProfileTiming();
};

#endif //PHYSICS_DIRECT_H
