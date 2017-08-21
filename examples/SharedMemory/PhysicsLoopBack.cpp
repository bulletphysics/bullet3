#include "PhysicsLoopBack.h"
#include "PhysicsServerSharedMemory.h"
#include "PhysicsClientSharedMemory.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "PhysicsServerCommandProcessor.h"
#include "../CommonInterfaces/CommonExampleInterface.h"
struct PhysicsLoopBackInternalData
{
	CommandProcessorInterface* m_commandProcessor;
	PhysicsClientSharedMemory* m_physicsClient;
	PhysicsServerSharedMemory* m_physicsServer;
	DummyGUIHelper m_noGfx;
	

	PhysicsLoopBackInternalData()
		:m_commandProcessor(0),
		m_physicsClient(0),
		m_physicsServer(0)
	{
	}
};

struct Bullet2CommandProcessorCreation2 : public CommandProcessorCreationInterface
{
	virtual class CommandProcessorInterface* createCommandProcessor()
	{
		PhysicsServerCommandProcessor* proc = new PhysicsServerCommandProcessor;
		return proc;
	}

	virtual void deleteCommandProcessor(CommandProcessorInterface* proc)
	{
		delete proc;
	}
};

static Bullet2CommandProcessorCreation2 sB2Proc;

PhysicsLoopBack::PhysicsLoopBack()
{
	m_data = new PhysicsLoopBackInternalData;
	m_data->m_physicsServer = new PhysicsServerSharedMemory(&sB2Proc, 0,0);
	m_data->m_physicsClient = new PhysicsClientSharedMemory();
}

PhysicsLoopBack::~PhysicsLoopBack()
{
	delete m_data->m_physicsClient;
	delete m_data->m_physicsServer;
	delete m_data->m_commandProcessor;
	delete m_data;
}


// return true if connection succesfull, can also check 'isConnected'
bool PhysicsLoopBack::connect()
{
	m_data->m_physicsServer->connectSharedMemory(&m_data->m_noGfx);
	m_data->m_physicsClient->connect();
	return m_data->m_physicsClient->isConnected();
}

////todo: rename to 'disconnect'
void PhysicsLoopBack::disconnectSharedMemory()
{
	m_data->m_physicsClient->disconnectSharedMemory();
	m_data->m_physicsServer->disconnectSharedMemory(true);
}

bool PhysicsLoopBack::isConnected() const
{
	return m_data->m_physicsClient->isConnected();
}

// return non-null if there is a status, nullptr otherwise
const  SharedMemoryStatus* PhysicsLoopBack::processServerStatus()
{
	m_data->m_physicsServer->processClientCommands();
	return m_data->m_physicsClient->processServerStatus();
}

SharedMemoryCommand* PhysicsLoopBack::getAvailableSharedMemoryCommand()
{
	return m_data->m_physicsClient->getAvailableSharedMemoryCommand();
}

bool PhysicsLoopBack::canSubmitCommand() const
{
	return m_data->m_physicsClient->canSubmitCommand();
}

bool PhysicsLoopBack::submitClientCommand(const struct SharedMemoryCommand& command)
{
	return  m_data->m_physicsClient->submitClientCommand(command);
}

int PhysicsLoopBack::getNumBodies() const
{
	return m_data->m_physicsClient->getNumBodies();
}

int PhysicsLoopBack::getBodyUniqueId(int serialIndex) const
{
	return m_data->m_physicsClient->getBodyUniqueId(serialIndex);
}

bool PhysicsLoopBack::getBodyInfo(int bodyUniqueId, struct b3BodyInfo& info) const
{
	return m_data->m_physicsClient->getBodyInfo(bodyUniqueId, info);
}

int PhysicsLoopBack::getNumJoints(int bodyIndex) const
{
	return m_data->m_physicsClient->getNumJoints(bodyIndex);
}

bool PhysicsLoopBack::getJointInfo(int bodyIndex, int jointIndex, struct b3JointInfo& info) const
{
	return m_data->m_physicsClient->getJointInfo(bodyIndex,jointIndex,info);
}

int PhysicsLoopBack::getNumUserConstraints() const
{
    return m_data->m_physicsClient->getNumUserConstraints();
}
int PhysicsLoopBack::getUserConstraintInfo(int constraintUniqueId, struct b3UserConstraint&info) const
{
    return m_data->m_physicsClient->getUserConstraintInfo( constraintUniqueId, info);
}

int PhysicsLoopBack::getUserConstraintId(int serialIndex) const
{
	return m_data->m_physicsClient->getUserConstraintId(serialIndex);
}

///todo: move this out of the interface
void PhysicsLoopBack::setSharedMemoryKey(int key)
{
	m_data->m_physicsServer->setSharedMemoryKey(key);
	m_data->m_physicsClient->setSharedMemoryKey(key);
}

void PhysicsLoopBack::uploadBulletFileToSharedMemory(const char* data, int len)
{
	m_data->m_physicsClient->uploadBulletFileToSharedMemory(data,len);
}

int PhysicsLoopBack::getNumDebugLines() const
{
	return m_data->m_physicsClient->getNumDebugLines();
}

const float* PhysicsLoopBack::getDebugLinesFrom() const
{
	return m_data->m_physicsClient->getDebugLinesFrom();
}

const float* PhysicsLoopBack::getDebugLinesTo() const
{
	return m_data->m_physicsClient->getDebugLinesTo();
}

const float* PhysicsLoopBack::getDebugLinesColor() const
{
	return m_data->m_physicsClient->getDebugLinesColor();
}

void PhysicsLoopBack::getCachedCameraImage(struct b3CameraImageData* cameraData)
{
	return m_data->m_physicsClient->getCachedCameraImage(cameraData);
}

void PhysicsLoopBack::getCachedContactPointInformation(struct b3ContactInformation* contactPointData)
{
    return m_data->m_physicsClient->getCachedContactPointInformation(contactPointData);
}

void PhysicsLoopBack::getCachedVisualShapeInformation(struct b3VisualShapeInformation* visualShapesInfo)
{
	return m_data->m_physicsClient->getCachedVisualShapeInformation(visualShapesInfo);
}

void PhysicsLoopBack::getCachedVREvents(struct b3VREventsData* vrEventsData)
{
	return m_data->m_physicsClient->getCachedVREvents(vrEventsData);
}

void PhysicsLoopBack::getCachedKeyboardEvents(struct b3KeyboardEventsData* keyboardEventsData)
{
	return m_data->m_physicsClient->getCachedKeyboardEvents(keyboardEventsData);
}

void PhysicsLoopBack::getCachedMouseEvents(struct b3MouseEventsData* mouseEventsData)
{
	return m_data->m_physicsClient->getCachedMouseEvents(mouseEventsData);
}

void PhysicsLoopBack::getCachedOverlappingObjects(struct b3AABBOverlapData* overlappingObjects)
{
	return m_data->m_physicsClient->getCachedOverlappingObjects(overlappingObjects);
}

void PhysicsLoopBack::getCachedRaycastHits(struct b3RaycastInformation* raycastHits)
{
	return m_data->m_physicsClient->getCachedRaycastHits(raycastHits);
}

void PhysicsLoopBack::setTimeOut(double timeOutInSeconds)
{
	m_data->m_physicsClient->setTimeOut(timeOutInSeconds);
}
double PhysicsLoopBack::getTimeOut() const
{
	return m_data->m_physicsClient->getTimeOut();
}

