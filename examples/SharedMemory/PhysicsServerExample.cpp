

//todo(erwincoumans): re-use the upcoming b3RobotSimAPI here

#include "PhysicsServerExample.h"



#include "PhysicsServerSharedMemory.h"
#include "Bullet3Common/b3CommandLineArgs.h"
#include "SharedMemoryCommon.h"
#include "Bullet3Common/b3Matrix3x3.h"
#include "../Utils/b3Clock.h"
#include "../MultiThreading/b3ThreadSupportInterface.h"
#ifdef BT_ENABLE_VR
#include "../RenderingExamples/TinyVRGui.h"
#endif//BT_ENABLE_VR


#include "../CommonInterfaces/CommonParameterInterface.h"

#define MAX_VR_CONTROLLERS 8


//@todo(erwincoumans) those globals are hacks for a VR demo, move this to Python/pybullet!
extern btVector3 gLastPickPos;
btVector3 gVRTeleportPos(0,0,0);
btQuaternion gVRTeleportOrn(0, 0, 0,1);
extern btVector3 gVRGripperPos;
extern btQuaternion gVRGripperOrn;
extern btVector3 gVRController2Pos;
extern btQuaternion gVRController2Orn;
extern btScalar gVRGripperAnalog;
extern btScalar gVRGripper2Analog;
extern bool gCloseToKuka;
extern bool gEnableRealTimeSimVR;
extern bool gCreateSamuraiRobotAssets;
extern int gCreateObjectSimVR;
static int gGraspingController = -1;
extern btScalar simTimeScalingFactor;

extern bool gVRGripperClosed;

bool gDebugRenderToggle  = false;
void	MotionThreadFunc(void* userPtr,void* lsMemory);
void*	MotionlsMemoryFunc();
#define MAX_MOTION_NUM_THREADS 1
enum
	{
		numCubesX = 20,
		numCubesY = 20
	};


enum TestExampleBrowserCommunicationEnums
{
	eRequestTerminateMotion= 13,
	eMotionIsUnInitialized,
	eMotionIsInitialized,
	eMotionInitializationFailed,
	eMotionHasTerminated
};

enum MultiThreadedGUIHelperCommunicationEnums
{
	eGUIHelperIdle= 13,
	eGUIHelperRegisterTexture,
	eGUIHelperRegisterGraphicsShape,
	eGUIHelperRegisterGraphicsInstance,
	eGUIHelperCreateCollisionShapeGraphicsObject,
	eGUIHelperCreateCollisionObjectGraphicsObject,
	eGUIHelperCreateRigidBodyGraphicsObject,
	eGUIHelperRemoveAllGraphicsInstances,
	eGUIHelperCopyCameraImageData,
};

#include <stdio.h>
//#include "BulletMultiThreaded/PlatformDefinitions.h"

#ifndef _WIN32
#include "../MultiThreading/b3PosixThreadSupport.h"

b3ThreadSupportInterface* createMotionThreadSupport(int numThreads)
{
	b3PosixThreadSupport::ThreadConstructionInfo constructionInfo("MotionThreads",
                                                                MotionThreadFunc,
                                                                MotionlsMemoryFunc,
                                                                numThreads);
    b3ThreadSupportInterface* threadSupport = new b3PosixThreadSupport(constructionInfo);

	return threadSupport;

}


#elif defined( _WIN32)
#include "../MultiThreading/b3Win32ThreadSupport.h"

b3ThreadSupportInterface* createMotionThreadSupport(int numThreads)
{
	b3Win32ThreadSupport::Win32ThreadConstructionInfo threadConstructionInfo("MotionThreads",MotionThreadFunc,MotionlsMemoryFunc,numThreads);
	b3Win32ThreadSupport* threadSupport = new b3Win32ThreadSupport(threadConstructionInfo);
	return threadSupport;

}
#endif



struct	MotionArgs
{
	MotionArgs()
		:m_physicsServerPtr(0)
	{
		for (int i=0;i<MAX_VR_CONTROLLERS;i++)
		{
			m_isVrControllerPicking[i] = false;
			m_isVrControllerDragging[i] = false;
			m_isVrControllerReleasing[i] = false;
			m_isVrControllerTeleporting[i] = false;
		}
	}
	b3CriticalSection* m_cs;
	
	PhysicsServerSharedMemory*	m_physicsServerPtr;
	b3AlignedObjectArray<b3Vector3> m_positions;

	btVector3 m_vrControllerPos[MAX_VR_CONTROLLERS];
	btQuaternion m_vrControllerOrn[MAX_VR_CONTROLLERS];
	bool m_isVrControllerPicking[MAX_VR_CONTROLLERS];
	bool m_isVrControllerDragging[MAX_VR_CONTROLLERS];
	bool m_isVrControllerReleasing[MAX_VR_CONTROLLERS];
	bool m_isVrControllerTeleporting[MAX_VR_CONTROLLERS];
	
};

struct MotionThreadLocalStorage
{
	int threadId;
};

int skip = 0;
int skip1 = 0;
float clampedDeltaTime  = 0.2;

void	MotionThreadFunc(void* userPtr,void* lsMemory)
{
	printf("MotionThreadFunc thread started\n");
	MotionThreadLocalStorage* localStorage = (MotionThreadLocalStorage*) lsMemory;

	MotionArgs* args = (MotionArgs*) userPtr;
	int workLeft = true;
	b3Clock clock;
	clock.reset();
	bool init = true;
	if (init)
	{

		args->m_cs->lock();
		args->m_cs->setSharedParam(0,eMotionIsInitialized);
		args->m_cs->unlock();


		double deltaTimeInSeconds = 0;

		do
		{
			deltaTimeInSeconds+= double(clock.getTimeMicroseconds())/1000000.;

			if (deltaTimeInSeconds<(1./5000.))
			{

				skip++;
				skip1++;
				if (skip1>5)
				{
					b3Clock::usleep(250);
				}
			} else
			{
				skip1=0;
				
				//process special controller commands, such as
				//VR controller button press/release and controller motion

				for (int c=0;c<MAX_VR_CONTROLLERS;c++)
				{
				
					btVector3 from = args->m_vrControllerPos[c];
					btMatrix3x3 mat(args->m_vrControllerOrn[c]);
				
					btScalar pickDistance = 1000.;
					btVector3 toX = from+mat.getColumn(0);
					btVector3 toY = from+mat.getColumn(1);
					btVector3 toZ = from+mat.getColumn(2)*pickDistance;

					if (args->m_isVrControllerTeleporting[c])
					{
						args->m_isVrControllerTeleporting[c] = false;
						args->m_physicsServerPtr->pickBody(from,-toZ);
						args->m_physicsServerPtr->removePickingConstraint();
					}

					if (!gCloseToKuka)
					{
						if (args->m_isVrControllerPicking[c])
						{
							args->m_isVrControllerPicking[c]  = false;
							args->m_isVrControllerDragging[c] = true;
							args->m_physicsServerPtr->pickBody(from,-toZ);
							//printf("PICK!\n");
						}
					}

					 if (args->m_isVrControllerDragging[c])
					 {
						 args->m_physicsServerPtr->movePickedBody(from,-toZ);
						// printf(".");
					 }
				
					if (args->m_isVrControllerReleasing[c])
					{
						args->m_isVrControllerDragging[c] = false;
						args->m_isVrControllerReleasing[c] = false;
						args->m_physicsServerPtr->removePickingConstraint();
						//printf("Release pick\n");
					}
				}

				//don't simulate over a huge timestep if we had some interruption (debugger breakpoint etc)
				if (deltaTimeInSeconds>clampedDeltaTime)
				{
					deltaTimeInSeconds = clampedDeltaTime;
					b3Warning("Clamp deltaTime from %f to %f",deltaTimeInSeconds, clampedDeltaTime);
				}
				
				clock.reset();
				args->m_physicsServerPtr->stepSimulationRealTime(deltaTimeInSeconds);
				deltaTimeInSeconds = 0;
				
			}

			args->m_physicsServerPtr->processClientCommands();
			
		} while (args->m_cs->getSharedParam(0)!=eRequestTerminateMotion);
	} else
	{
		args->m_cs->lock();
		args->m_cs->setSharedParam(0,eMotionInitializationFailed);
		args->m_cs->unlock();
	}


	printf("finished, #skip = %d, skip1 = %d\n",skip,skip1);
	skip=0;
	skip1=0;
	//do nothing

}



void*	MotionlsMemoryFunc()
{
	//don't create local store memory, just return 0
	return new MotionThreadLocalStorage;
}



class MultiThreadedOpenGLGuiHelper : public GUIHelperInterface
{
	CommonGraphicsApp* m_app;
	
	b3CriticalSection* m_cs;

	
public:

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
	

	MultiThreadedOpenGLGuiHelper(CommonGraphicsApp* app, GUIHelperInterface* guiHelper)
		:m_app(app)
		,m_cs(0),
		m_texels(0),
		m_textureId(-1)
	{
		m_childGuiHelper = guiHelper;;

	}

	virtual ~MultiThreadedOpenGLGuiHelper()
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

	btRigidBody* m_body;
	btVector3 m_color3;
	virtual void createRigidBodyGraphicsObject(btRigidBody* body,const btVector3& color)
	{
		m_body = body;
		m_color3 = color;
		m_cs->lock();
		m_cs->setSharedParam(1,eGUIHelperCreateRigidBodyGraphicsObject);
		m_cs->unlock();
		while (m_cs->getSharedParam(1)!=eGUIHelperIdle)
		{
			b3Clock::usleep(1000);
		}
	}

	btCollisionObject* m_obj;
	btVector3 m_color2;

	virtual void createCollisionObjectGraphicsObject(btCollisionObject* obj,const btVector3& color)
	{
		m_obj = obj;
		m_color2 = color;
		m_cs->lock();
		m_cs->setSharedParam(1,eGUIHelperCreateCollisionObjectGraphicsObject);
		m_cs->unlock();
		while (m_cs->getSharedParam(1)!=eGUIHelperIdle)
		{
			b3Clock::usleep(1000);
		}

	}

	btCollisionShape* m_colShape;
	virtual void createCollisionShapeGraphicsObject(btCollisionShape* collisionShape)
	{
		m_colShape = collisionShape;
		m_cs->lock();
		m_cs->setSharedParam(1,eGUIHelperCreateCollisionShapeGraphicsObject);
		m_cs->unlock();
		while (m_cs->getSharedParam(1)!=eGUIHelperIdle)
		{
			b3Clock::usleep(1000);
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

	virtual void createPhysicsDebugDrawer( btDiscreteDynamicsWorld* rbWorld)
	{
		m_childGuiHelper->createPhysicsDebugDrawer(rbWorld);
	}

	virtual int	registerTexture(const unsigned char* texels, int width, int height)
	{
		m_texels = texels;
		m_textureWidth = width;
		m_textureHeight = height;

		m_cs->lock();
		m_cs->setSharedParam(1,eGUIHelperRegisterTexture);
		m_cs->unlock();
		while (m_cs->getSharedParam(1)!=eGUIHelperIdle)
		{
			b3Clock::usleep(1000);
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
		m_cs->setSharedParam(1,eGUIHelperRegisterGraphicsShape);
		m_cs->unlock();
		while (m_cs->getSharedParam(1)!=eGUIHelperIdle)
		{
			b3Clock::usleep(1000);
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
		m_cs->setSharedParam(1,eGUIHelperRegisterGraphicsInstance);
		m_cs->unlock();
		while (m_cs->getSharedParam(1)!=eGUIHelperIdle)
		{
			b3Clock::usleep(1000);
		}
		return m_instanceId;
	}

    virtual void removeAllGraphicsInstances()
    {
        m_cs->lock();
		m_cs->setSharedParam(1,eGUIHelperRemoveAllGraphicsInstances);
		m_cs->unlock();
		while (m_cs->getSharedParam(1)!=eGUIHelperIdle)
		{
			b3Clock::usleep(1000);
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
	    m_childGuiHelper->resetCamera(camDist,pitch,yaw,camPosX,camPosY,camPosZ);
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
	    
		m_cs->setSharedParam(1,eGUIHelperCopyCameraImageData);
		m_cs->unlock();
		while (m_cs->getSharedParam(1)!=eGUIHelperIdle)
		{
			b3Clock::usleep(1000);
		}
	}
	

	virtual void autogenerateGraphicsObjects(btDiscreteDynamicsWorld* rbWorld) 
	{
	}
    
	virtual void drawText3D( const char* txt, float posX, float posZY, float posZ, float size)
	{
	}
};



class PhysicsServerExample : public SharedMemoryCommon
{
	PhysicsServerSharedMemory	m_physicsServer;
	b3ThreadSupportInterface* m_threadSupport;
	MotionArgs m_args[MAX_MOTION_NUM_THREADS];
	MultiThreadedOpenGLGuiHelper* m_multiThreadedHelper;
    bool m_wantsShutdown;

    bool m_isConnected;
    btClock m_clock;
	bool m_replay;
	int m_options;
	
#ifdef BT_ENABLE_VR
	TinyVRGui* m_tinyVrGui;
#endif

public:

	PhysicsServerExample(MultiThreadedOpenGLGuiHelper* helper, SharedMemoryInterface* sharedMem=0, int options=0);

	virtual ~PhysicsServerExample();

	virtual void	initPhysics();

	virtual void	stepSimulation(float deltaTime);

    void enableCommandLogging()
	{
		m_physicsServer.enableCommandLogging(true,"BulletPhysicsCommandLog.bin");
	}

	void replayFromLogFile()
	{
		m_replay = true;
		m_physicsServer.replayFromLogFile("BulletPhysicsCommandLog.bin");
	}



	virtual void resetCamera()
	{
		float dist = 5;
		float pitch = 50;
		float yaw = 35;
		float targetPos[3]={0,0,0};//-3,2.8,-2.5};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}

    virtual bool wantsTermination();
    virtual bool isConnected();
	virtual void	renderScene();
	virtual void    exitPhysics();

	virtual void	physicsDebugDraw(int debugFlags);

	btVector3	getRayTo(int x,int y);

	virtual void	vrControllerButtonCallback(int controllerId, int button, int state, float pos[4], float orientation[4]);
	virtual void	vrControllerMoveCallback(int controllerId, float pos[4], float orientation[4], float analogAxis);

	virtual bool	mouseMoveCallback(float x,float y)
	{
		if (m_replay)
			return false;

		CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();

		if (!renderer)
		{
			return false;
		}

		btVector3 rayTo = getRayTo(int(x), int(y));
		btVector3 rayFrom;
		renderer->getActiveCamera()->getCameraPosition(rayFrom);
		m_physicsServer.movePickedBody(rayFrom,rayTo);
		return false;
	};

	virtual bool	mouseButtonCallback(int button, int state, float x, float y)
	{
		if (m_replay)
			return false;

		CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();

		if (!renderer)
		{
			return false;
		}

		CommonWindowInterface* window = m_guiHelper->getAppInterface()->m_window;


		if (state==1)
		{
			if(button==0 && (!window->isModifierKeyPressed(B3G_ALT) && !window->isModifierKeyPressed(B3G_CONTROL) ))
			{
				btVector3 camPos;
				renderer->getActiveCamera()->getCameraPosition(camPos);

				btVector3 rayFrom = camPos;
				btVector3 rayTo = getRayTo(int(x),int(y));

				m_physicsServer.pickBody(rayFrom, rayTo);


			}
		} else
		{
			if (button==0)
			{
				m_physicsServer.removePickingConstraint();
				//remove p2p
			}
		}

		//printf("button=%d, state=%d\n",button,state);
		return false;
	}
	virtual bool	keyboardCallback(int key, int state){return false;}

	virtual void setSharedMemoryKey(int key)
	{
		m_physicsServer.setSharedMemoryKey(key);
	}

	virtual void	processCommandLineArgs(int argc, char* argv[])
	{
		b3CommandLineArgs args(argc,argv);
		if (args.CheckCmdLineFlag("emptyworld"))
		{
			gCreateSamuraiRobotAssets = false;
		}
	}

	

};

PhysicsServerExample::PhysicsServerExample(MultiThreadedOpenGLGuiHelper* helper, SharedMemoryInterface* sharedMem, int options)
:SharedMemoryCommon(helper),
m_physicsServer(sharedMem),
m_wantsShutdown(false),
m_isConnected(false),
m_replay(false),
m_options(options)
#ifdef BT_ENABLE_VR
,m_tinyVrGui(0)
#endif
{
	m_multiThreadedHelper = helper;
	b3Printf("Started PhysicsServer\n");
}



PhysicsServerExample::~PhysicsServerExample()
{
#ifdef BT_ENABLE_VR
	delete m_tinyVrGui;
#endif
	bool deInitializeSharedMemory = true;
	m_physicsServer.disconnectSharedMemory(deInitializeSharedMemory);
    m_isConnected = false;
}

bool PhysicsServerExample::isConnected()
{
    return m_isConnected;
}

void	PhysicsServerExample::initPhysics()
{
	///for this testing we use Z-axis up
	int upAxis = 2;
	m_guiHelper->setUpAxis(upAxis);


	


	m_threadSupport = createMotionThreadSupport(MAX_MOTION_NUM_THREADS);
		
		

		for (int i=0;i<m_threadSupport->getNumTasks();i++)
		{
			MotionThreadLocalStorage* storage = (MotionThreadLocalStorage*) m_threadSupport->getThreadLocalMemory(i);
			b3Assert(storage);
			storage->threadId = i;
			//storage->m_sharedMem = data->m_sharedMem;
		}


		

		for (int w=0;w<MAX_MOTION_NUM_THREADS;w++)
		{
			m_args[w].m_cs = m_threadSupport->createCriticalSection();
			m_args[w].m_cs->setSharedParam(0,eMotionIsUnInitialized);
			int numMoving = 0;
 			m_args[w].m_positions.resize(numMoving);
			m_args[w].m_physicsServerPtr = &m_physicsServer;
			int index = 0;
			
			m_threadSupport->runTask(B3_THREAD_SCHEDULE_TASK, (void*) &this->m_args[w], w);
			
			while (m_args[w].m_cs->getSharedParam(0)==eMotionIsUnInitialized)
			{
				b3Clock::usleep(1000);
			}
		}

		m_args[0].m_cs->setSharedParam(1,eGUIHelperIdle);
		m_multiThreadedHelper->setCriticalSection(m_args[0].m_cs);
		m_isConnected = m_physicsServer.connectSharedMemory( m_guiHelper);
}


void    PhysicsServerExample::exitPhysics()
{
		for (int i=0;i<MAX_MOTION_NUM_THREADS;i++)
		{
			m_args[i].m_cs->lock();
			m_args[i].m_cs->setSharedParam(0,eRequestTerminateMotion);
			m_args[i].m_cs->unlock();
		}
		int numActiveThreads = MAX_MOTION_NUM_THREADS;

		while (numActiveThreads)
                {
			int arg0,arg1;
                        if (m_threadSupport->isTaskCompleted(&arg0,&arg1,0))
                        {
                                numActiveThreads--;
                                printf("numActiveThreads = %d\n",numActiveThreads);

                        } else
                        {
							b3Clock::usleep(1000);
                        }
                };

		printf("stopping threads\n");

		delete m_threadSupport;   
		m_threadSupport = 0;

		//m_physicsServer.resetDynamicsWorld();
		
}



bool PhysicsServerExample::wantsTermination()
{
    return m_wantsShutdown;
}



void	PhysicsServerExample::stepSimulation(float deltaTime)
{
	//this->m_physicsServer.processClientCommands();

	//check if any graphics related tasks are requested
	
	switch (m_multiThreadedHelper->getCriticalSection()->getSharedParam(1))
	{
	case eGUIHelperCreateCollisionShapeGraphicsObject:
	{
		m_multiThreadedHelper->m_childGuiHelper->createCollisionShapeGraphicsObject(m_multiThreadedHelper->m_colShape);
		m_multiThreadedHelper->getCriticalSection()->lock();
		m_multiThreadedHelper->getCriticalSection()->setSharedParam(1,eGUIHelperIdle);
		m_multiThreadedHelper->getCriticalSection()->unlock();

		break;
	}
	case eGUIHelperCreateCollisionObjectGraphicsObject:
	{
		m_multiThreadedHelper->m_childGuiHelper->createCollisionObjectGraphicsObject(m_multiThreadedHelper->m_obj,
			m_multiThreadedHelper->m_color2);
		m_multiThreadedHelper->getCriticalSection()->lock();
		m_multiThreadedHelper->getCriticalSection()->setSharedParam(1,eGUIHelperIdle);
		m_multiThreadedHelper->getCriticalSection()->unlock();
		break;
	}
	case eGUIHelperCreateRigidBodyGraphicsObject:
	{
		m_multiThreadedHelper->m_childGuiHelper->createRigidBodyGraphicsObject(m_multiThreadedHelper->m_body,m_multiThreadedHelper->m_color3);
		m_multiThreadedHelper->getCriticalSection()->lock();
		m_multiThreadedHelper->getCriticalSection()->setSharedParam(1,eGUIHelperIdle);
		m_multiThreadedHelper->getCriticalSection()->unlock();
		break;
	}
	case eGUIHelperRegisterTexture:
	{
		
		m_multiThreadedHelper->m_textureId = m_multiThreadedHelper->m_childGuiHelper->registerTexture(m_multiThreadedHelper->m_texels,
						m_multiThreadedHelper->m_textureWidth,m_multiThreadedHelper->m_textureHeight);

		m_multiThreadedHelper->getCriticalSection()->lock();
		m_multiThreadedHelper->getCriticalSection()->setSharedParam(1,eGUIHelperIdle);
		m_multiThreadedHelper->getCriticalSection()->unlock();
		
		break;
	}
	case eGUIHelperRegisterGraphicsShape:
	{
		m_multiThreadedHelper->m_shapeIndex = m_multiThreadedHelper->m_childGuiHelper->registerGraphicsShape(
				m_multiThreadedHelper->m_vertices,
				m_multiThreadedHelper->m_numvertices,
				m_multiThreadedHelper->m_indices,
				m_multiThreadedHelper->m_numIndices,
				m_multiThreadedHelper->m_primitiveType,
				m_multiThreadedHelper->m_textureId);

		m_multiThreadedHelper->getCriticalSection()->lock();
		m_multiThreadedHelper->getCriticalSection()->setSharedParam(1,eGUIHelperIdle);
		m_multiThreadedHelper->getCriticalSection()->unlock();
		break;
	}
	case eGUIHelperRegisterGraphicsInstance:
	{
		m_multiThreadedHelper->m_instanceId = m_multiThreadedHelper->m_childGuiHelper->registerGraphicsInstance(
				m_multiThreadedHelper->m_shapeIndex,
				m_multiThreadedHelper->m_position,
				m_multiThreadedHelper->m_quaternion,
				m_multiThreadedHelper->m_color,
				m_multiThreadedHelper->m_scaling);

		m_multiThreadedHelper->getCriticalSection()->lock();
		m_multiThreadedHelper->getCriticalSection()->setSharedParam(1,eGUIHelperIdle);
		m_multiThreadedHelper->getCriticalSection()->unlock();
		break;
	}
	case eGUIHelperRemoveAllGraphicsInstances:
        {
            m_multiThreadedHelper->m_childGuiHelper->removeAllGraphicsInstances();
			int numRenderInstances = m_multiThreadedHelper->m_childGuiHelper->getRenderInterface()->getTotalNumInstances();
			b3Assert(numRenderInstances==0);

            m_multiThreadedHelper->getCriticalSection()->lock();
            m_multiThreadedHelper->getCriticalSection()->setSharedParam(1,eGUIHelperIdle);
            m_multiThreadedHelper->getCriticalSection()->unlock();
            break;
        }
        
    case eGUIHelperCopyCameraImageData:
        {
             m_multiThreadedHelper->m_childGuiHelper->copyCameraImageData(m_multiThreadedHelper->m_viewMatrix,
                                                                                 m_multiThreadedHelper->m_projectionMatrix,
                                                                                 m_multiThreadedHelper->m_pixelsRGBA,
                                                                                 m_multiThreadedHelper->m_rgbaBufferSizeInPixels,
                                                                                 m_multiThreadedHelper->m_depthBuffer,
                                                                                 m_multiThreadedHelper->m_depthBufferSizeInPixels,
                                                                                 m_multiThreadedHelper->m_segmentationMaskBuffer,
                                                                                 m_multiThreadedHelper->m_segmentationMaskBufferSizeInPixels,
                                                                                 m_multiThreadedHelper->m_startPixelIndex, 
                                                                                 m_multiThreadedHelper->m_destinationWidth, 
                                                                                 m_multiThreadedHelper->m_destinationHeight, 
                                                                                 m_multiThreadedHelper->m_numPixelsCopied);
            m_multiThreadedHelper->getCriticalSection()->lock();
            m_multiThreadedHelper->getCriticalSection()->setSharedParam(1,eGUIHelperIdle);
            m_multiThreadedHelper->getCriticalSection()->unlock();
            break;
        }
	case eGUIHelperIdle:
	default:
		{
			
		}
	}
	



	#if 0
	if (m_options == PHYSICS_SERVER_USE_RTC_CLOCK)
	{
		btClock rtc;
		btScalar endTime = rtc.getTimeMilliseconds() + deltaTime*btScalar(800);

		while (rtc.getTimeMilliseconds()<endTime)
		{
			m_physicsServer.processClientCommands();
		}
	} else
	{
        //for (int i=0;i<10;i++)
			m_physicsServer.processClientCommands();
	}
	#endif

	{
		if (m_multiThreadedHelper->m_childGuiHelper->getRenderInterface())
		{
			m_multiThreadedHelper->m_childGuiHelper->getRenderInterface()->writeTransforms();
		}
	}
}

static float vrOffset[16]={1,0,0,0,
							0,1,0,0,
							0,0,1,0,
							0,0,0,0};


extern int gDroppedSimulationSteps;
extern int gNumSteps;
extern double gDtInSec;
extern double gSubStep;


void PhysicsServerExample::renderScene()
{
	B3_PROFILE("PhysicsServerExample::RenderScene");
	static char line0[1024];
		static char line1[1024];

	if (gEnableRealTimeSimVR)
	{
		
		static int frameCount=0;
		static btScalar prevTime = m_clock.getTimeSeconds();
		frameCount++;
		
		static btScalar worseFps = 1000000;
		int numFrames = 200;
		static int count = 0;
		count++;

		if (0 == (count & 1))
		{
			btScalar curTime = m_clock.getTimeSeconds();
			btScalar fps = 1. / (curTime - prevTime);
			prevTime = curTime;
			if (fps < worseFps)
			{
				worseFps = fps;
			}

			if (count > numFrames)
			{
				count = 0;
				sprintf(line0, "fps:%f frame:%d", worseFps, frameCount / 2);
				sprintf(line1, "drop:%d tscale:%f dt:%f, substep %f)", gDroppedSimulationSteps, simTimeScalingFactor,gDtInSec, gSubStep);
				gDroppedSimulationSteps = 0;

				worseFps = 1000000;
			}
		}

#ifdef BT_ENABLE_VR
		if (m_tinyVrGui==0)
		{
			ComboBoxParams comboParams;
        comboParams.m_comboboxId = 0;
        comboParams.m_numItems = 0;
        comboParams.m_startItem = 0;
        comboParams.m_callback = 0;//MyComboBoxCallback;
        comboParams.m_userPointer = 0;//this;
        
			m_tinyVrGui = new TinyVRGui(comboParams,this->m_multiThreadedHelper->m_childGuiHelper->getRenderInterface());
			m_tinyVrGui->init();
		}

		if (m_tinyVrGui)
		{

			b3Transform tr;tr.setIdentity();
			tr.setOrigin(b3MakeVector3(gVRController2Pos[0],gVRController2Pos[1],gVRController2Pos[2]));
			tr.setRotation(b3Quaternion(gVRController2Orn[0],gVRController2Orn[1],gVRController2Orn[2],gVRController2Orn[3]));
			tr = tr*b3Transform(b3Quaternion(0,0,-SIMD_HALF_PI),b3MakeVector3(0,0,0));
			b3Scalar dt = 0.01;
			m_tinyVrGui->clearTextArea();
			
			m_tinyVrGui->grapicalPrintf(line0,0,0,0,0,0,255);
			m_tinyVrGui->grapicalPrintf(line1,0,16,255,255,255,255);

			m_tinyVrGui->tick(dt,tr);
		}
#endif//BT_ENABLE_VR
	}
	///debug rendering
	//m_args[0].m_cs->lock();
	
	//gVRTeleportPos[0] += 0.01;
	vrOffset[12]=-gVRTeleportPos[0];
	vrOffset[13]=-gVRTeleportPos[1];
	vrOffset[14]=-gVRTeleportPos[2];

	this->m_multiThreadedHelper->m_childGuiHelper->getRenderInterface()->
		getActiveCamera()->setVRCameraOffsetTransform(vrOffset);

	m_physicsServer.renderScene();
	
	for (int i=0;i<MAX_VR_CONTROLLERS;i++)
	{
		if (m_args[0].m_isVrControllerPicking[i] || m_args[0].m_isVrControllerDragging[i])
		{
			btVector3 from = m_args[0].m_vrControllerPos[i];
			btMatrix3x3 mat(m_args[0].m_vrControllerOrn[i]);
	
			btVector3 toX = from+mat.getColumn(0);
			btVector3 toY = from+mat.getColumn(1);
			btVector3 toZ = from+mat.getColumn(2);
	
			int width = 2;

	
			btVector4 color;
			color=btVector4(1,0,0,1);
			m_guiHelper->getAppInterface()->m_renderer->drawLine(from,toX,color,width);
			color=btVector4(0,1,0,1);
			m_guiHelper->getAppInterface()->m_renderer->drawLine(from,toY,color,width);
			color=btVector4(0,0,1,1);
			m_guiHelper->getAppInterface()->m_renderer->drawLine(from,toZ,color,width);
	
		}
	}

	if (m_guiHelper->getAppInterface()->m_renderer->getActiveCamera()->isVRCamera())
	{
		gEnableRealTimeSimVR = true;
	}

	if (gDebugRenderToggle)
	if (m_guiHelper->getAppInterface()->m_renderer->getActiveCamera()->isVRCamera())
	{
		
		B3_PROFILE("Draw Debug HUD");
		//some little experiment to add text/HUD to a VR camera (HTC Vive/Oculus Rift)


		float pos[4];
		m_guiHelper->getAppInterface()->m_renderer->getActiveCamera()->getCameraTargetPosition(pos);
		pos[0]+=gVRTeleportPos[0];
		pos[1]+=gVRTeleportPos[1];
		pos[2]+=gVRTeleportPos[2];

		btTransform viewTr;
		btScalar m[16];
		float mf[16];
		m_guiHelper->getAppInterface()->m_renderer->getActiveCamera()->getCameraViewMatrix(mf);
		for (int i=0;i<16;i++)
		{
			m[i] = mf[i];
		}
		m[12]=+gVRTeleportPos[0];
		m[13]=+gVRTeleportPos[1];
		m[14]=+gVRTeleportPos[2];
		viewTr.setFromOpenGLMatrix(m);
		btTransform viewTrInv = viewTr.inverse();
		
		btVector3 side = viewTrInv.getBasis().getColumn(0);
		btVector3 up = viewTrInv.getBasis().getColumn(1);
		btVector3 fwd = viewTrInv.getBasis().getColumn(2);

		
		float upMag = 0;
		float sideMag = 2.2;
		float fwdMag = -4;

		m_guiHelper->getAppInterface()->drawText3D(line0,pos[0]+upMag*up[0]-sideMag*side[0]+fwdMag*fwd[0],pos[1]+upMag*up[1]-sideMag*side[1]+fwdMag*fwd[1],pos[2]+upMag*up[2]-sideMag*side[2]+fwdMag*fwd[2],1);
		//btVector3 fwd = viewTrInv.getBasis().getColumn(2);
		
		up = viewTrInv.getBasis().getColumn(1);
		upMag = -0.3;
		
		
		
		m_guiHelper->getAppInterface()->drawText3D(line1,pos[0]+upMag*up[0]-sideMag*side[0]+fwdMag*fwd[0],pos[1]+upMag*up[1]-sideMag*side[1]+fwdMag*fwd[1],pos[2]+upMag*up[2]-sideMag*side[2]+fwdMag*fwd[2],1);
	}

	//m_args[0].m_cs->unlock();
}

void    PhysicsServerExample::physicsDebugDraw(int debugDrawFlags)
{
	///debug rendering
	m_physicsServer.physicsDebugDraw(debugDrawFlags);

}



btVector3	PhysicsServerExample::getRayTo(int x,int y)
{
	CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();

	if (!renderer)
	{
		btAssert(0);
		return btVector3(0,0,0);
	}

	float top = 1.f;
	float bottom = -1.f;
	float nearPlane = 1.f;
	float tanFov = (top-bottom)*0.5f / nearPlane;
	float fov = btScalar(2.0) * btAtan(tanFov);

	btVector3 camPos,camTarget;
	renderer->getActiveCamera()->getCameraPosition(camPos);
	renderer->getActiveCamera()->getCameraTargetPosition(camTarget);
	
	btVector3	rayFrom = camPos;
	btVector3 rayForward = (camTarget-camPos);
	rayForward.normalize();
	float farPlane = 10000.f;
	rayForward*= farPlane;

	btVector3 rightOffset;
	btVector3 cameraUp=btVector3(0,0,0);
	cameraUp[m_guiHelper->getAppInterface()->getUpAxis()]=1;

	btVector3 vertical = cameraUp;

	btVector3 hor;
	hor = rayForward.cross(vertical);
	hor.normalize();
	vertical = hor.cross(rayForward);
	vertical.normalize();

	float tanfov = tanf(0.5f*fov);


	hor *= 2.f * farPlane * tanfov;
	vertical *= 2.f * farPlane * tanfov;

	btScalar aspect;
	float width = float(renderer->getScreenWidth());
	float height = float (renderer->getScreenHeight());

	aspect =  width / height;

	hor*=aspect;


	btVector3 rayToCenter = rayFrom + rayForward;
	btVector3 dHor = hor * 1.f/width;
	btVector3 dVert = vertical * 1.f/height;


	btVector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
	rayTo += btScalar(x) * dHor;
	rayTo -= btScalar(y) * dVert;
	return rayTo;
}


extern int gSharedMemoryKey;

class CommonExampleInterface*    PhysicsServerCreateFunc(struct CommonExampleOptions& options)
{

	MultiThreadedOpenGLGuiHelper* guiHelperWrapper = new MultiThreadedOpenGLGuiHelper(options.m_guiHelper->getAppInterface(),options.m_guiHelper);

  	PhysicsServerExample* example = new PhysicsServerExample(guiHelperWrapper, 
		options.m_sharedMem, 
		options.m_option);

	if (gSharedMemoryKey>=0)
	{
		example->setSharedMemoryKey(gSharedMemoryKey);
	}
	if (options.m_option & PHYSICS_SERVER_ENABLE_COMMAND_LOGGING)
	{
		example->enableCommandLogging();
	}
	if (options.m_option & PHYSICS_SERVER_REPLAY_FROM_COMMAND_LOG)
	{
		example->replayFromLogFile();
	}
	return example;

}



void	PhysicsServerExample::vrControllerButtonCallback(int controllerId, int button, int state, float pos[4], float orn[4])
{
	//printf("controllerId %d, button=%d\n",controllerId, button);
	
	if (controllerId<0 || controllerId>=MAX_VR_CONTROLLERS)
		return;

	if (gGraspingController < 0)
		gGraspingController = controllerId;

	if (controllerId != gGraspingController)
	{
		if (button == 1 && state == 0)
		{
			gVRTeleportPos = gLastPickPos;
		}
	} else
	{
		if (button == 1)
		{
			if (state == 1)
			{
				gDebugRenderToggle = 1;
			} else
			{
				gDebugRenderToggle = 0;
				
				if (simTimeScalingFactor==0)
				{
					simTimeScalingFactor = 1;
				} else
				{
					if (simTimeScalingFactor==1)
					{
						simTimeScalingFactor = 0.25;
					}
					else
					{
						simTimeScalingFactor = 0;
					}
				}
			}
		} else
		{
			
		}
	}
	if (button==32 && state==0)
	{
		gCreateObjectSimVR = 1;
	}
	

	if (button==1)
	{
		m_args[0].m_isVrControllerTeleporting[controllerId] = true;
	}

	if (controllerId == gGraspingController && (button == 33))
	{
		gVRGripperClosed =state;
	}
	else
	{

		if (button == 33)
		{
			m_args[0].m_isVrControllerPicking[controllerId] = (state != 0);
			m_args[0].m_isVrControllerReleasing[controllerId] = (state == 0);
		}
		if ((button == 33) || (button == 1))
		{
			m_args[0].m_vrControllerPos[controllerId].setValue(pos[0] + gVRTeleportPos[0], pos[1] + gVRTeleportPos[1], pos[2] + gVRTeleportPos[2]);
			m_args[0].m_vrControllerOrn[controllerId].setValue(orn[0], orn[1], orn[2], orn[3]);
		}
	}
}



void	PhysicsServerExample::vrControllerMoveCallback(int controllerId, float pos[4], float orn[4], float analogAxis)
{

	gEnableRealTimeSimVR = true;

	if (controllerId <= 0 || controllerId >= MAX_VR_CONTROLLERS)
	{
		printf("Controller Id exceeds max: %d > %d", controllerId, MAX_VR_CONTROLLERS);
		return;
	}
	if (controllerId == gGraspingController)
	{
		gVRGripperAnalog = analogAxis;
		gVRGripperPos.setValue(pos[0] + gVRTeleportPos[0], pos[1] + gVRTeleportPos[1], pos[2] + gVRTeleportPos[2]);
		btQuaternion orgOrn(orn[0], orn[1], orn[2], orn[3]);
		gVRGripperOrn = orgOrn*btQuaternion(btVector3(0, 0, 1), SIMD_HALF_PI)*btQuaternion(btVector3(0, 1, 0), SIMD_HALF_PI);
	}
	else
	{
		gVRGripper2Analog = analogAxis;
		gVRController2Pos.setValue(pos[0] + gVRTeleportPos[0], pos[1] + gVRTeleportPos[1], pos[2] + gVRTeleportPos[2]);
		btQuaternion orgOrn(orn[0], orn[1], orn[2], orn[3]);
		gVRController2Orn = orgOrn*btQuaternion(btVector3(0, 0, 1), SIMD_HALF_PI)*btQuaternion(btVector3(0, 1, 0), SIMD_HALF_PI);
		
		m_args[0].m_vrControllerPos[controllerId].setValue(pos[0] + gVRTeleportPos[0], pos[1] + gVRTeleportPos[1], pos[2] + gVRTeleportPos[2]);
		m_args[0].m_vrControllerOrn[controllerId].setValue(orn[0], orn[1], orn[2], orn[3]);
	}

}
B3_STANDALONE_EXAMPLE(PhysicsServerCreateFunc)
