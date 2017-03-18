#include "b3RobotSimAPI.h"

//#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
//#include "../CommonInterfaces/CommonExampleInterface.h"
#include "../SharedMemory/PhysicsServerCommandProcessor.h"

#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../SharedMemory/PhysicsServerSharedMemory.h"
#include "../SharedMemory/PhysicsServerSharedMemory.h"
#include "../SharedMemory/PhysicsClientC_API.h"
#include "../SharedMemory/PhysicsDirectC_API.h"
#include "../SharedMemory/PhysicsDirect.h"

#include <string>
#include "../Utils/b3Clock.h"


#include "../MultiThreading/b3ThreadSupportInterface.h"


void	RobotThreadFunc(void* userPtr,void* lsMemory);
void*	RobotlsMemoryFunc();
#define MAX_ROBOT_NUM_THREADS 1
enum
	{
		numCubesX = 20,
		numCubesY = 20
	};


enum TestRobotSimCommunicationEnums
{
	eRequestTerminateRobotSim= 13,
	eRobotSimIsUnInitialized,
	eRobotSimIsInitialized,
	eRobotSimInitializationFailed,
	eRobotSimHasTerminated
};

enum MultiThreadedGUIHelperCommunicationEnums
{
	eRobotSimGUIHelperIdle= 13,
	eRobotSimGUIHelperRegisterTexture,
	eRobotSimGUIHelperRegisterGraphicsShape,
	eRobotSimGUIHelperRegisterGraphicsInstance,
	eRobotSimGUIHelperCreateCollisionShapeGraphicsObject,
	eRobotSimGUIHelperCreateCollisionObjectGraphicsObject,
	eRobotSimGUIHelperRemoveAllGraphicsInstances,
	eRobotSimGUIHelperCopyCameraImageData,
};

#include <stdio.h>
//#include "BulletMultiThreaded/PlatformDefinitions.h"

#ifndef _WIN32
#include "../MultiThreading/b3PosixThreadSupport.h"

b3ThreadSupportInterface* createRobotSimThreadSupport(int numThreads)
{
	b3PosixThreadSupport::ThreadConstructionInfo constructionInfo("RobotSimThreads",
                                                                RobotThreadFunc,
                                                                RobotlsMemoryFunc,
                                                                numThreads);
    b3ThreadSupportInterface* threadSupport = new b3PosixThreadSupport(constructionInfo);

	return threadSupport;

}


#elif defined( _WIN32)
#include "../MultiThreading/b3Win32ThreadSupport.h"

b3ThreadSupportInterface* createRobotSimThreadSupport(int numThreads)
{
	b3Win32ThreadSupport::Win32ThreadConstructionInfo threadConstructionInfo("RobotSimThreads",RobotThreadFunc,RobotlsMemoryFunc,numThreads);
	b3Win32ThreadSupport* threadSupport = new b3Win32ThreadSupport(threadConstructionInfo);
	return threadSupport;

}
#endif



struct	RobotSimArgs
{
	RobotSimArgs()
		:m_physicsServerPtr(0)
	{
	}
	b3CriticalSection* m_cs;
	
	PhysicsServerSharedMemory*	m_physicsServerPtr;
	b3AlignedObjectArray<b3Vector3> m_positions;
};

struct RobotSimThreadLocalStorage
{
	int threadId;
};


void	RobotThreadFunc(void* userPtr,void* lsMemory)
{
	printf("RobotThreadFunc thread started\n");
//	RobotSimThreadLocalStorage* localStorage = (RobotSimThreadLocalStorage*) lsMemory;

	RobotSimArgs* args = (RobotSimArgs*) userPtr;
//	int workLeft = true;
	b3Clock clock;
	clock.reset();
	bool init = true;
	if (init)
	{

		args->m_cs->lock();
		args->m_cs->setSharedParam(0,eRobotSimIsInitialized);
		args->m_cs->unlock();

		do
		{
//todo(erwincoumans): do we want some sleep to reduce CPU resources in this thread?
#if 0
			double deltaTimeInSeconds = double(clock.getTimeMicroseconds())/1000000.;
			if (deltaTimeInSeconds<(1./260.))
			{
				if (deltaTimeInSeconds<.001)
					continue;
			}

			clock.reset();
#endif //
			args->m_physicsServerPtr->processClientCommands();
			
		} while (args->m_cs->getSharedParam(0)!=eRequestTerminateRobotSim);
	} else
	{
		args->m_cs->lock();
		args->m_cs->setSharedParam(0,eRobotSimInitializationFailed);
		args->m_cs->unlock();
	}
	//do nothing
}



void*	RobotlsMemoryFunc()
{
	//don't create local store memory, just return 0
	return new RobotSimThreadLocalStorage;
}



ATTRIBUTE_ALIGNED16(class) MultiThreadedOpenGLGuiHelper2 : public GUIHelperInterface
{
//	CommonGraphicsApp* m_app;
	
	b3CriticalSection* m_cs;

	
public:
	
	BT_DECLARE_ALIGNED_ALLOCATOR();

	GUIHelperInterface* m_childGuiHelper;

	const unsigned char* m_texels;
	int m_textureWidth;
	int m_textureHeight;


	int m_shapeIndex;
	const float* m_position;
	const float* m_quaternion;
	const float* m_color;
	const float* m_scaling;

	const float* m_vertices;
	int m_numvertices;
	const int* m_indices;
	int m_numIndices;
	int m_primitiveType;
	int m_textureId;
	int m_instanceId;
	

	MultiThreadedOpenGLGuiHelper2( GUIHelperInterface* guiHelper)
		:	m_cs(0),
		m_texels(0),
		m_textureId(-1)
	{
		m_childGuiHelper = guiHelper;;

	}

	virtual ~MultiThreadedOpenGLGuiHelper2()
	{
		delete m_childGuiHelper;
	}

	void setCriticalSection(b3CriticalSection* cs)
	{
		m_cs = cs;
	}

	b3CriticalSection* getCriticalSection()
	{
		return m_cs;
	}

	virtual void createRigidBodyGraphicsObject(btRigidBody* body,const btVector3& color)
    {
        createCollisionObjectGraphicsObject((btCollisionObject*)body, color);
    }

	btCollisionObject* m_obj;
	btVector3 m_color2;

	virtual void createCollisionObjectGraphicsObject(btCollisionObject* obj,const btVector3& color)
	{
		m_obj = obj;
		m_color2 = color;
		m_cs->lock();
		m_cs->setSharedParam(1,eRobotSimGUIHelperCreateCollisionObjectGraphicsObject);
		m_cs->unlock();
		while (m_cs->getSharedParam(1)!=eRobotSimGUIHelperIdle)
		{
		}

	}

	btCollisionShape* m_colShape;
	virtual void createCollisionShapeGraphicsObject(btCollisionShape* collisionShape)
	{
		m_colShape = collisionShape;
		m_cs->lock();
		m_cs->setSharedParam(1,eRobotSimGUIHelperCreateCollisionShapeGraphicsObject);
		m_cs->unlock();
		while (m_cs->getSharedParam(1)!=eRobotSimGUIHelperIdle)
		{
		}

	}

	virtual void syncPhysicsToGraphics(const btDiscreteDynamicsWorld* rbWorld)
	{
	    //this check is to prevent a crash, in case we removed all graphics instances, but there are still physics objects.
	    //the check will be obsolete, once we have a better/safer way of synchronizing physics->graphics transforms
        if ( m_childGuiHelper->getRenderInterface()->getTotalNumInstances()>0)
        {
            m_childGuiHelper->syncPhysicsToGraphics(rbWorld);
        }
	}

	virtual void render(const btDiscreteDynamicsWorld* rbWorld) 
	{
		m_childGuiHelper->render(0);
	}

	virtual void createPhysicsDebugDrawer( btDiscreteDynamicsWorld* rbWorld){}

	virtual int	registerTexture(const unsigned char* texels, int width, int height)
	{
		m_texels = texels;
		m_textureWidth = width;
		m_textureHeight = height;

		m_cs->lock();
		m_cs->setSharedParam(1,eRobotSimGUIHelperRegisterTexture);
		m_cs->unlock();
		while (m_cs->getSharedParam(1)!=eRobotSimGUIHelperIdle)
		{
		}
		return m_textureId;
	}
	virtual int registerGraphicsShape(const float* vertices, int numvertices, const int* indices, int numIndices,int primitiveType, int textureId)
	{
		m_vertices = vertices;
		m_numvertices = numvertices;
		m_indices = indices;
		m_numIndices = numIndices;
		m_primitiveType = primitiveType;
		m_textureId = textureId;

		m_cs->lock();
		m_cs->setSharedParam(1,eRobotSimGUIHelperRegisterGraphicsShape);
		m_cs->unlock();
		while (m_cs->getSharedParam(1)!=eRobotSimGUIHelperIdle)
		{
		}
		return m_shapeIndex;
	}
	virtual int registerGraphicsInstance(int shapeIndex, const float* position, const float* quaternion, const float* color, const float* scaling) 
	{
		m_shapeIndex = shapeIndex;
		m_position = position;
		m_quaternion = quaternion;
		m_color = color;
		m_scaling = scaling;

		m_cs->lock();
		m_cs->setSharedParam(1,eRobotSimGUIHelperRegisterGraphicsInstance);
		m_cs->unlock();
		while (m_cs->getSharedParam(1)!=eRobotSimGUIHelperIdle)
		{
		}
		return m_instanceId;
	}

    virtual void removeAllGraphicsInstances()
    {
        m_cs->lock();
		m_cs->setSharedParam(1,eRobotSimGUIHelperRemoveAllGraphicsInstances);
		m_cs->unlock();
		while (m_cs->getSharedParam(1)!=eRobotSimGUIHelperIdle)
		{
		}
    }

	virtual Common2dCanvasInterface* get2dCanvasInterface()
	{
		return 0;
	}
	
	virtual CommonParameterInterface* getParameterInterface()
	{
		return 0;
	}

	virtual CommonRenderInterface* getRenderInterface()
	{
		return 0;
	}
	
	virtual CommonGraphicsApp* getAppInterface()
	{
		return m_childGuiHelper->getAppInterface();
	}


	virtual void setUpAxis(int axis)
	{
		m_childGuiHelper->setUpAxis(axis);
	}
	virtual void resetCamera(float camDist, float pitch, float yaw, float camPosX,float camPosY, float camPosZ)
	{
	}

    float m_viewMatrix[16];
    float m_projectionMatrix[16];
    unsigned char* m_pixelsRGBA;
    int m_rgbaBufferSizeInPixels;
    float* m_depthBuffer;
    int m_depthBufferSizeInPixels;
    int* m_segmentationMaskBuffer;
    int m_segmentationMaskBufferSizeInPixels;
    int m_startPixelIndex;
    int m_destinationWidth;
    int m_destinationHeight;
    int* m_numPixelsCopied;

	virtual void copyCameraImageData(const float viewMatrix[16], const float projectionMatrix[16], 
                                  unsigned char* pixelsRGBA, int rgbaBufferSizeInPixels, 
                                  float* depthBuffer, int depthBufferSizeInPixels, 
                                  int* segmentationMaskBuffer, int segmentationMaskBufferSizeInPixels,
                                  int startPixelIndex, int destinationWidth, 
                                  int destinationHeight, int* numPixelsCopied)
	{
	    m_cs->lock();
	    for (int i=0;i<16;i++)
        {
            m_viewMatrix[i] = viewMatrix[i];
            m_projectionMatrix[i] = projectionMatrix[i];
        }
	    m_pixelsRGBA = pixelsRGBA;
        m_rgbaBufferSizeInPixels = rgbaBufferSizeInPixels;
        m_depthBuffer = depthBuffer;
        m_depthBufferSizeInPixels = depthBufferSizeInPixels;
        m_segmentationMaskBuffer = segmentationMaskBuffer;
        m_segmentationMaskBufferSizeInPixels = segmentationMaskBufferSizeInPixels;
        m_startPixelIndex = startPixelIndex;
        m_destinationWidth = destinationWidth;
        m_destinationHeight = destinationHeight;
        m_numPixelsCopied = numPixelsCopied;
	    
		m_cs->setSharedParam(1,eRobotSimGUIHelperCopyCameraImageData);
		m_cs->unlock();
		while (m_cs->getSharedParam(1)!=eRobotSimGUIHelperIdle)
		{
		}
	}

	virtual void autogenerateGraphicsObjects(btDiscreteDynamicsWorld* rbWorld) 
	{
	}
    
	virtual void drawText3D( const char* txt, float posX, float posZY, float posZ, float size)
	{
	}

		virtual int		addUserDebugText3D( const char* txt, const double positionXYZ[3], const double	textColorRGB[3], double size, double lifeTime)
	{
		return -1;
	}
	virtual int		addUserDebugLine(const double	debugLineFromXYZ[3], const double	debugLineToXYZ[3], const double	debugLineColorRGB[3], double lineWidth, double lifeTime )
	{
		return -1;
	}
	virtual void	removeUserDebugItem( int debugItemUniqueId)
	{
	}
	virtual void	removeAllUserDebugItems( )
	{
	}
};





struct b3RobotSimAPI_InternalData
{
	//GUIHelperInterface* m_guiHelper;
	PhysicsServerSharedMemory	m_physicsServer;
	b3PhysicsClientHandle m_physicsClient;


	b3ThreadSupportInterface* m_threadSupport;
	RobotSimArgs m_args[MAX_ROBOT_NUM_THREADS];
	MultiThreadedOpenGLGuiHelper2* m_multiThreadedHelper;
	PhysicsDirect* m_clientServerDirect;

	bool m_useMultiThreading;
	bool m_connected;

	b3RobotSimAPI_InternalData()
		:
	m_physicsClient(0),
	m_threadSupport(0),
	m_multiThreadedHelper(0),
	
	m_clientServerDirect(0),
	m_useMultiThreading(false),
		m_connected(false)
	{
	}
};

b3RobotSimAPI::b3RobotSimAPI()
{
	m_data = new b3RobotSimAPI_InternalData;
}

void b3RobotSimAPI::stepSimulation()
{
	b3SharedMemoryStatusHandle statusHandle;
	int statusType;
	b3Assert(b3CanSubmitCommand(m_data->m_physicsClient));
    if (b3CanSubmitCommand(m_data->m_physicsClient))
    {
        statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClient, b3InitStepSimulationCommand(m_data->m_physicsClient));
        statusType = b3GetStatusType(statusHandle);
        b3Assert(statusType==CMD_STEP_FORWARD_SIMULATION_COMPLETED);
    }
}

void b3RobotSimAPI::setGravity(const b3Vector3& gravityAcceleration)
{
    b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(m_data->m_physicsClient);
    b3SharedMemoryStatusHandle statusHandle;
	b3PhysicsParamSetGravity(command,  gravityAcceleration[0],gravityAcceleration[1],gravityAcceleration[2]);
	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClient, command);
    b3Assert(b3GetStatusType(statusHandle)==CMD_CLIENT_COMMAND_COMPLETED);

}

void b3RobotSimAPI::setNumSolverIterations(int numIterations)
{
    b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(m_data->m_physicsClient);
    b3SharedMemoryStatusHandle statusHandle;
    b3PhysicsParamSetNumSolverIterations(command, numIterations);
    statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClient, command);
    b3Assert(b3GetStatusType(statusHandle)==CMD_CLIENT_COMMAND_COMPLETED);

}

void b3RobotSimAPI::setNumSimulationSubSteps(int numSubSteps)
{
    b3SharedMemoryCommandHandle command = b3InitPhysicsParamCommand(m_data->m_physicsClient);
    b3SharedMemoryStatusHandle statusHandle;
    b3PhysicsParamSetNumSubSteps(command, numSubSteps);
    statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClient, command);
    b3Assert(b3GetStatusType(statusHandle)==CMD_CLIENT_COMMAND_COMPLETED);
}

/*
b3SharedMemoryCommandHandle	b3CalculateInverseKinematicsCommandInit(b3PhysicsClientHandle physClient, int bodyIndex,
	const double* jointPositionsQ, double targetPosition[3]);

int b3GetStatusInverseKinematicsJointPositions(b3SharedMemoryStatusHandle statusHandle,
	int* bodyUniqueId,
	int* dofCount,
	double* jointPositions);
*/
bool b3RobotSimAPI::calculateInverseKinematics(const struct b3RobotSimInverseKinematicArgs& args, struct b3RobotSimInverseKinematicsResults& results)
{
	btAssert(args.m_endEffectorLinkIndex>=0);
	btAssert(args.m_bodyUniqueId>=0);
	

	b3SharedMemoryCommandHandle command = b3CalculateInverseKinematicsCommandInit(m_data->m_physicsClient,args.m_bodyUniqueId);
	if ((args.m_flags & B3_HAS_IK_TARGET_ORIENTATION) && (args.m_flags & B3_HAS_NULL_SPACE_VELOCITY))
    {
        b3CalculateInverseKinematicsPosOrnWithNullSpaceVel(command, args.m_numDegreeOfFreedom, args.m_endEffectorLinkIndex, args.m_endEffectorTargetPosition, args.m_endEffectorTargetOrientation, &args.m_lowerLimits[0], &args.m_upperLimits[0], &args.m_jointRanges[0], &args.m_restPoses[0]);
    } else if (args.m_flags & B3_HAS_IK_TARGET_ORIENTATION)
	{
		b3CalculateInverseKinematicsAddTargetPositionWithOrientation(command,args.m_endEffectorLinkIndex,args.m_endEffectorTargetPosition,args.m_endEffectorTargetOrientation);
	} else if (args.m_flags & B3_HAS_NULL_SPACE_VELOCITY)
    {
        b3CalculateInverseKinematicsPosWithNullSpaceVel(command, args.m_numDegreeOfFreedom, args.m_endEffectorLinkIndex, args.m_endEffectorTargetPosition, &args.m_lowerLimits[0], &args.m_upperLimits[0], &args.m_jointRanges[0], &args.m_restPoses[0]);
    } else
	{
		b3CalculateInverseKinematicsAddTargetPurePosition(command,args.m_endEffectorLinkIndex,args.m_endEffectorTargetPosition);
	}
    
    if (args.m_flags & B3_HAS_JOINT_DAMPING)
    {
        b3CalculateInverseKinematicsSetJointDamping(command, args.m_numDegreeOfFreedom, &args.m_jointDamping[0]);
    }

    b3SharedMemoryStatusHandle statusHandle;
	statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClient, command);

	

    int numPos=0;
    
	bool result = b3GetStatusInverseKinematicsJointPositions(statusHandle,
		&results.m_bodyUniqueId,
		&numPos,
		0);
    if (result && numPos)
    {
        results.m_calculatedJointPositions.resize(numPos);
        result = b3GetStatusInverseKinematicsJointPositions(statusHandle,
                                                                 &results.m_bodyUniqueId,
                                                                 &numPos,
                                &results.m_calculatedJointPositions[0]);
        
    }
	return result;
}


b3RobotSimAPI::~b3RobotSimAPI()
{
	delete m_data;
}

void b3RobotSimAPI::processMultiThreadedGraphicsRequests()
{
	if (0==m_data->m_multiThreadedHelper)
		return;

	switch (m_data->m_multiThreadedHelper->getCriticalSection()->getSharedParam(1))
	{
	case eRobotSimGUIHelperCreateCollisionShapeGraphicsObject:
	{
		m_data->m_multiThreadedHelper->m_childGuiHelper->createCollisionShapeGraphicsObject(m_data->m_multiThreadedHelper->m_colShape);
		m_data->m_multiThreadedHelper->getCriticalSection()->lock();
		m_data->m_multiThreadedHelper->getCriticalSection()->setSharedParam(1,eRobotSimGUIHelperIdle);
		m_data->m_multiThreadedHelper->getCriticalSection()->unlock();

		break;
	}
	case eRobotSimGUIHelperCreateCollisionObjectGraphicsObject:
	{
		m_data->m_multiThreadedHelper->m_childGuiHelper->createCollisionObjectGraphicsObject(m_data->m_multiThreadedHelper->m_obj,
			m_data->m_multiThreadedHelper->m_color2);
		m_data->m_multiThreadedHelper->getCriticalSection()->lock();
		m_data->m_multiThreadedHelper->getCriticalSection()->setSharedParam(1,eRobotSimGUIHelperIdle);
		m_data->m_multiThreadedHelper->getCriticalSection()->unlock();
		break;
	}
	case eRobotSimGUIHelperRegisterTexture:
	{
		
		m_data->m_multiThreadedHelper->m_textureId = m_data->m_multiThreadedHelper->m_childGuiHelper->registerTexture(m_data->m_multiThreadedHelper->m_texels,
						m_data->m_multiThreadedHelper->m_textureWidth,m_data->m_multiThreadedHelper->m_textureHeight);

		m_data->m_multiThreadedHelper->getCriticalSection()->lock();
		m_data->m_multiThreadedHelper->getCriticalSection()->setSharedParam(1,eRobotSimGUIHelperIdle);
		m_data->m_multiThreadedHelper->getCriticalSection()->unlock();
		
		break;
	}
	case eRobotSimGUIHelperRegisterGraphicsShape:
	{
		m_data->m_multiThreadedHelper->m_shapeIndex = m_data->m_multiThreadedHelper->m_childGuiHelper->registerGraphicsShape(
				m_data->m_multiThreadedHelper->m_vertices,
				m_data->m_multiThreadedHelper->m_numvertices,
				m_data->m_multiThreadedHelper->m_indices,
				m_data->m_multiThreadedHelper->m_numIndices,
				m_data->m_multiThreadedHelper->m_primitiveType,
				m_data->m_multiThreadedHelper->m_textureId);

		m_data->m_multiThreadedHelper->getCriticalSection()->lock();
		m_data->m_multiThreadedHelper->getCriticalSection()->setSharedParam(1,eRobotSimGUIHelperIdle);
		m_data->m_multiThreadedHelper->getCriticalSection()->unlock();
		break;
	}
	case eRobotSimGUIHelperRegisterGraphicsInstance:
	{
		m_data->m_multiThreadedHelper->m_instanceId = m_data->m_multiThreadedHelper->m_childGuiHelper->registerGraphicsInstance(
				m_data->m_multiThreadedHelper->m_shapeIndex,
				m_data->m_multiThreadedHelper->m_position,
				m_data->m_multiThreadedHelper->m_quaternion,
				m_data->m_multiThreadedHelper->m_color,
				m_data->m_multiThreadedHelper->m_scaling);

		m_data->m_multiThreadedHelper->getCriticalSection()->lock();
		m_data->m_multiThreadedHelper->getCriticalSection()->setSharedParam(1,eRobotSimGUIHelperIdle);
		m_data->m_multiThreadedHelper->getCriticalSection()->unlock();
		break;
	}
	case eRobotSimGUIHelperRemoveAllGraphicsInstances:
        {
            m_data->m_multiThreadedHelper->m_childGuiHelper->removeAllGraphicsInstances();
			int numRenderInstances;
			numRenderInstances = m_data->m_multiThreadedHelper->m_childGuiHelper->getRenderInterface()->getTotalNumInstances();
			b3Assert(numRenderInstances==0);

            m_data->m_multiThreadedHelper->getCriticalSection()->lock();
            m_data->m_multiThreadedHelper->getCriticalSection()->setSharedParam(1,eRobotSimGUIHelperIdle);
            m_data->m_multiThreadedHelper->getCriticalSection()->unlock();
            break;
        }
    case eRobotSimGUIHelperCopyCameraImageData:
        {
            m_data->m_multiThreadedHelper->m_childGuiHelper->copyCameraImageData(m_data->m_multiThreadedHelper->m_viewMatrix,
                                                                                 m_data->m_multiThreadedHelper->m_projectionMatrix,
                                                                                 m_data->m_multiThreadedHelper->m_pixelsRGBA,
                                                                                 m_data->m_multiThreadedHelper->m_rgbaBufferSizeInPixels,
                                                                                 m_data->m_multiThreadedHelper->m_depthBuffer,
                                                                                 m_data->m_multiThreadedHelper->m_depthBufferSizeInPixels,
                                                                                 m_data->m_multiThreadedHelper->m_segmentationMaskBuffer,
                                                                                 m_data->m_multiThreadedHelper->m_segmentationMaskBufferSizeInPixels,
                                                                                 m_data->m_multiThreadedHelper->m_startPixelIndex, 
                                                                                 m_data->m_multiThreadedHelper->m_destinationWidth, 
                                                                                 m_data->m_multiThreadedHelper->m_destinationHeight, 
                                                                                 m_data->m_multiThreadedHelper->m_numPixelsCopied);
            m_data->m_multiThreadedHelper->getCriticalSection()->lock();
            m_data->m_multiThreadedHelper->getCriticalSection()->setSharedParam(1,eRobotSimGUIHelperIdle);
            m_data->m_multiThreadedHelper->getCriticalSection()->unlock();

            break;
        }
	case eRobotSimGUIHelperIdle:
	default:
		{
			
		}
	}
	
}

b3SharedMemoryStatusHandle b3RobotSimAPI::submitClientCommandAndWaitStatusMultiThreaded(b3PhysicsClientHandle physClient, b3SharedMemoryCommandHandle commandHandle)
{
	int timeout = 1024*1024*1024;
    b3SharedMemoryStatusHandle statusHandle=0;
    
    b3SubmitClientCommand(physClient,commandHandle);
    
    while ((statusHandle==0) && (timeout-- > 0))
    {
        statusHandle =b3ProcessServerStatus(physClient);
		processMultiThreadedGraphicsRequests();
    }
    return (b3SharedMemoryStatusHandle) statusHandle;
}

int b3RobotSimAPI::getNumJoints(int bodyUniqueId) const
{
	return b3GetNumJoints(m_data->m_physicsClient,bodyUniqueId);
}

bool b3RobotSimAPI::getJointInfo(int bodyUniqueId, int jointIndex, b3JointInfo* jointInfo)
{
	return (b3GetJointInfo(m_data->m_physicsClient,bodyUniqueId, jointIndex,jointInfo)!=0);
}

void b3RobotSimAPI::createJoint(int parentBodyIndex, int parentJointIndex, int childBodyIndex, int childJointIndex, b3JointInfo* jointInfo)
{
    b3SharedMemoryStatusHandle statusHandle;
    b3Assert(b3CanSubmitCommand(m_data->m_physicsClient));
    if (b3CanSubmitCommand(m_data->m_physicsClient))
    {
        statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClient, b3InitCreateUserConstraintCommand(m_data->m_physicsClient, parentBodyIndex, parentJointIndex, childBodyIndex, childJointIndex, jointInfo));
    }
}


bool b3RobotSimAPI::getJointStates(int bodyUniqueId, b3JointStates& state)
{
    b3SharedMemoryCommandHandle command = b3RequestActualStateCommandInit(m_data->m_physicsClient,bodyUniqueId);
    b3SharedMemoryStatusHandle statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClient, command);
    
    if (statusHandle)
    {
     //   double rootInertialFrame[7];
        const double* rootLocalInertialFrame;
        const double* actualStateQ;
        const double* actualStateQdot;
        const double* jointReactionForces;
        
        int stat = b3GetStatusActualState(statusHandle,
                        &state.m_bodyUniqueId,
                        &state.m_numDegreeOfFreedomQ,
                        &state.m_numDegreeOfFreedomU,
                        &rootLocalInertialFrame,
                        &actualStateQ,
                        &actualStateQdot,
                        &jointReactionForces);
        if (stat)
        {
            state.m_actualStateQ.resize(state.m_numDegreeOfFreedomQ);
            state.m_actualStateQdot.resize(state.m_numDegreeOfFreedomU);

            for (int i=0;i<state.m_numDegreeOfFreedomQ;i++)
            {
                state.m_actualStateQ[i] = actualStateQ[i];
            }
            for (int i=0;i<state.m_numDegreeOfFreedomU;i++)
            {
                state.m_actualStateQdot[i] = actualStateQdot[i];
            }
            int numJoints = getNumJoints(bodyUniqueId);
            state.m_jointReactionForces.resize(6*numJoints);
            for (int i=0;i<numJoints*6;i++)
            {
                state.m_jointReactionForces[i] = jointReactionForces[i];
            }
            
            return true;
        }
        //rootInertialFrame,
          //              &state.m_actualStateQ[0],
            //            &state.m_actualStateQdot[0],
              //          &state.m_jointReactionForces[0]);
        
     
    }
    return false;
}

void b3RobotSimAPI::setJointMotorControl(int bodyUniqueId, int jointIndex, const b3JointMotorArgs& args)
{
	b3SharedMemoryStatusHandle statusHandle;
	switch (args.m_controlMode)
	{
		case CONTROL_MODE_VELOCITY:
		{
			b3SharedMemoryCommandHandle command = b3JointControlCommandInit2( m_data->m_physicsClient, bodyUniqueId, CONTROL_MODE_VELOCITY);
			b3JointInfo jointInfo;
			b3GetJointInfo(m_data->m_physicsClient, bodyUniqueId, jointIndex, &jointInfo);
			int uIndex = jointInfo.m_uIndex;
            b3JointControlSetKd(command,uIndex,args.m_kd);
			b3JointControlSetDesiredVelocity(command,uIndex,args.m_targetVelocity);
			b3JointControlSetMaximumForce(command,uIndex,args.m_maxTorqueValue);
			statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClient, command);
			break;
		}
		case CONTROL_MODE_POSITION_VELOCITY_PD:
		{
			b3SharedMemoryCommandHandle command = b3JointControlCommandInit2( m_data->m_physicsClient, bodyUniqueId, CONTROL_MODE_POSITION_VELOCITY_PD);
			b3JointInfo jointInfo;
			b3GetJointInfo(m_data->m_physicsClient, bodyUniqueId, jointIndex, &jointInfo);
			int uIndex = jointInfo.m_uIndex;
			int qIndex = jointInfo.m_qIndex;

			b3JointControlSetDesiredPosition(command,qIndex,args.m_targetPosition);
			b3JointControlSetKp(command,uIndex,args.m_kp);
			b3JointControlSetDesiredVelocity(command,uIndex,args.m_targetVelocity);
			b3JointControlSetKd(command,uIndex,args.m_kd);
			b3JointControlSetMaximumForce(command,uIndex,args.m_maxTorqueValue);
			statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClient, command);
			break;
		}
		case CONTROL_MODE_TORQUE:
		{
			b3SharedMemoryCommandHandle command = b3JointControlCommandInit2( m_data->m_physicsClient, bodyUniqueId, CONTROL_MODE_TORQUE);
			b3JointInfo jointInfo;
			b3GetJointInfo(m_data->m_physicsClient, bodyUniqueId, jointIndex, &jointInfo);
			int uIndex = jointInfo.m_uIndex;
			b3JointControlSetDesiredForceTorque(command,uIndex,args.m_maxTorqueValue);
			statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClient, command);
			break;
		}
		default:
		{
			b3Error("Unknown control command in b3RobotSimAPI::setJointMotorControl");
		}
	}
}

bool b3RobotSimAPI::loadFile(const struct b3RobotSimLoadFileArgs& args, b3RobotSimLoadFileResults& results)
{
	bool statusOk = false;

	int robotUniqueId = -1;
	b3Assert(m_data->m_connected);
	switch (args.m_fileType)
	 {
		case B3_URDF_FILE:
		{
				b3SharedMemoryStatusHandle statusHandle;
				int statusType;
				b3SharedMemoryCommandHandle command = b3LoadUrdfCommandInit(m_data->m_physicsClient, args.m_fileName.c_str());
			
				//setting the initial position, orientation and other arguments are optional
            
				b3LoadUrdfCommandSetStartPosition(command, args.m_startPosition[0],
																args.m_startPosition[1],
																args.m_startPosition[2]);
				b3LoadUrdfCommandSetStartOrientation(command,args.m_startOrientation[0]
														,args.m_startOrientation[1]
														,args.m_startOrientation[2]
														,args.m_startOrientation[3]);
				if (args.m_forceOverrideFixedBase)
				{
					b3LoadUrdfCommandSetUseFixedBase(command,true);
				}
                b3LoadUrdfCommandSetUseMultiBody(command, args.m_useMultiBody);
				statusHandle = submitClientCommandAndWaitStatusMultiThreaded(m_data->m_physicsClient, command);
				statusType = b3GetStatusType(statusHandle);

				b3Assert(statusType==CMD_URDF_LOADING_COMPLETED);
				robotUniqueId = b3GetStatusBodyIndex(statusHandle);
				results.m_uniqueObjectIds.push_back(robotUniqueId);
				statusOk = true;
			break;
		}
		case B3_SDF_FILE:
		{
			b3SharedMemoryStatusHandle statusHandle;
			int statusType;
			b3SharedMemoryCommandHandle command = b3LoadSdfCommandInit(m_data->m_physicsClient, args.m_fileName.c_str());
            b3LoadSdfCommandSetUseMultiBody(command, args.m_useMultiBody);
			statusHandle = submitClientCommandAndWaitStatusMultiThreaded(m_data->m_physicsClient, command);
			statusType = b3GetStatusType(statusHandle);
			b3Assert(statusType == CMD_SDF_LOADING_COMPLETED);
			if (statusType == CMD_SDF_LOADING_COMPLETED)
			{
				int numBodies = b3GetStatusBodyIndices(statusHandle, 0,0);
				if (numBodies)
				{
					results.m_uniqueObjectIds.resize(numBodies);
					int numBodies;
					numBodies = b3GetStatusBodyIndices(statusHandle, &results.m_uniqueObjectIds[0],results.m_uniqueObjectIds.size());

				}
				statusOk = true;
			}

			break;
		}
		default:
		{
			b3Warning("Unknown file type in b3RobotSimAPI::loadFile");
		}

	}

	return statusOk;
}

bool b3RobotSimAPI::connect(GUIHelperInterface* guiHelper)
{
	if (m_data->m_useMultiThreading)
	{
		m_data->m_multiThreadedHelper = new MultiThreadedOpenGLGuiHelper2(guiHelper);


		m_data->m_threadSupport = createRobotSimThreadSupport(MAX_ROBOT_NUM_THREADS);

		for (int i = 0; i < m_data->m_threadSupport->getNumTasks(); i++)
		{
			RobotSimThreadLocalStorage* storage = (RobotSimThreadLocalStorage*)m_data->m_threadSupport->getThreadLocalMemory(i);
			b3Assert(storage);
			storage->threadId = i;
			//storage->m_sharedMem = data->m_sharedMem;
		}




		for (int w = 0; w < MAX_ROBOT_NUM_THREADS; w++)
		{
			m_data->m_args[w].m_cs = m_data->m_threadSupport->createCriticalSection();
			m_data->m_args[w].m_cs->setSharedParam(0, eRobotSimIsUnInitialized);
			int numMoving = 0;
			m_data->m_args[w].m_positions.resize(numMoving);
			m_data->m_args[w].m_physicsServerPtr = &m_data->m_physicsServer;
			//int index = 0;

			m_data->m_threadSupport->runTask(B3_THREAD_SCHEDULE_TASK, (void*)&m_data->m_args[w], w);
			while (m_data->m_args[w].m_cs->getSharedParam(0) == eRobotSimIsUnInitialized)
			{
			}
		}

		m_data->m_args[0].m_cs->setSharedParam(1, eRobotSimGUIHelperIdle);
		m_data->m_multiThreadedHelper->setCriticalSection(m_data->m_args[0].m_cs);

		bool serverConnected;
		serverConnected = m_data->m_physicsServer.connectSharedMemory(m_data->m_multiThreadedHelper);
		b3Assert(serverConnected);

		m_data->m_physicsClient = b3ConnectSharedMemory(SHARED_MEMORY_KEY);
	}
	else
	{
		PhysicsServerCommandProcessor* sdk = new PhysicsServerCommandProcessor;
		m_data->m_clientServerDirect = new PhysicsDirect(sdk,true);
		bool connected;
		connected = m_data->m_clientServerDirect->connect(guiHelper);
		m_data->m_physicsClient = (b3PhysicsClientHandle)m_data->m_clientServerDirect;

	}

	//client connected
	m_data->m_connected = b3CanSubmitCommand(m_data->m_physicsClient);
	
	b3Assert(m_data->m_connected);
	return m_data->m_connected && m_data->m_connected;
}

void b3RobotSimAPI::disconnect()
{
	if (m_data->m_useMultiThreading)
	{
		for (int i = 0; i < MAX_ROBOT_NUM_THREADS; i++)
		{
			m_data->m_args[i].m_cs->lock();
			m_data->m_args[i].m_cs->setSharedParam(0, eRequestTerminateRobotSim);
			m_data->m_args[i].m_cs->unlock();
		}
		int numActiveThreads = MAX_ROBOT_NUM_THREADS;

		while (numActiveThreads)
		{
			int arg0, arg1;
			if (m_data->m_threadSupport->isTaskCompleted(&arg0, &arg1, 0))
			{
				numActiveThreads--;
				printf("numActiveThreads = %d\n", numActiveThreads);

			}
			else
			{
			}
		};

		printf("stopping threads\n");

		delete m_data->m_threadSupport;
		m_data->m_threadSupport = 0;
	}
	b3DisconnectSharedMemory(m_data->m_physicsClient);
	m_data->m_physicsServer.disconnectSharedMemory(true);
	
	m_data->m_connected = false;
}


void b3RobotSimAPI::debugDraw(int debugDrawMode)
{
	if (m_data->m_clientServerDirect)
	{
		m_data->m_clientServerDirect->debugDraw(debugDrawMode);
	}
}
void b3RobotSimAPI::renderScene()
{
	if (m_data->m_useMultiThreading)
	{
		if (m_data->m_multiThreadedHelper->m_childGuiHelper->getRenderInterface())
		{
			m_data->m_multiThreadedHelper->m_childGuiHelper->getRenderInterface()->writeTransforms();
		}
	}

    
	if (m_data->m_clientServerDirect)
	{
		m_data->m_clientServerDirect->renderScene();
	}
    
	m_data->m_physicsServer.renderScene();
}

void b3RobotSimAPI::getBodyJacobian(int bodyUniqueId, int linkIndex, const double* localPosition, const double* jointPositions, const double* jointVelocities, const double* jointAccelerations, double* linearJacobian, double* angularJacobian)
{
    b3SharedMemoryCommandHandle command = b3CalculateJacobianCommandInit(m_data->m_physicsClient, bodyUniqueId, linkIndex, localPosition, jointPositions, jointVelocities, jointAccelerations);
    b3SharedMemoryStatusHandle statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClient, command);
    
    if (b3GetStatusType(statusHandle) == CMD_CALCULATED_JACOBIAN_COMPLETED)
    {
        b3GetStatusJacobian(statusHandle, linearJacobian, angularJacobian);
    }
}

void b3RobotSimAPI::getLinkState(int bodyUniqueId, int linkIndex, double* worldPosition, double* worldOrientation)
{
    b3SharedMemoryCommandHandle command = b3RequestActualStateCommandInit(m_data->m_physicsClient,bodyUniqueId);
    b3SharedMemoryStatusHandle statusHandle = b3SubmitClientCommandAndWaitStatus(m_data->m_physicsClient, command);
    
    if (b3GetStatusType(statusHandle) == CMD_ACTUAL_STATE_UPDATE_COMPLETED)
    {
        b3LinkState linkState;
        b3GetLinkState(m_data->m_physicsClient, statusHandle, linkIndex, &linkState);
        worldPosition[0] = linkState.m_worldPosition[0];
        worldPosition[1] = linkState.m_worldPosition[1];
        worldPosition[2] = linkState.m_worldPosition[2];
        worldOrientation[0] = linkState.m_worldOrientation[0];
        worldOrientation[1] = linkState.m_worldOrientation[1];
        worldOrientation[2] = linkState.m_worldOrientation[2];
        worldOrientation[3] = linkState.m_worldOrientation[3];
    }
}

void b3RobotSimAPI::loadBunny(double scale, double mass, double collisionMargin)
{
    b3SharedMemoryCommandHandle command = b3LoadBunnyCommandInit(m_data->m_physicsClient);
    b3LoadBunnySetScale(command, scale);
    b3LoadBunnySetMass(command, mass);
    b3LoadBunnySetCollisionMargin(command, collisionMargin);
    b3SubmitClientCommand(m_data->m_physicsClient, command);
}
