

//todo(erwincoumans): re-use the upcoming b3RobotSimAPI here

#include "PhysicsServerExample.h"

#ifdef B3_USE_MIDI
#include "RtMidi.h"
#endif//B3_USE_MIDI

#include "PhysicsServerSharedMemory.h"
#include "Bullet3Common/b3CommandLineArgs.h"
#include "SharedMemoryCommon.h"
#include "Bullet3Common/b3Matrix3x3.h"
#include "../Utils/b3Clock.h"
#include "../MultiThreading/b3ThreadSupportInterface.h"
#include "SharedMemoryPublic.h"
#ifdef BT_ENABLE_VR
#include "../RenderingExamples/TinyVRGui.h"
#endif//BT_ENABLE_VR


#include "../CommonInterfaces/CommonParameterInterface.h"



//@todo(erwincoumans) those globals are hacks for a VR demo, move this to Python/pybullet!
extern btVector3 gLastPickPos;

btVector3 gVRTeleportPosLocal(0,0,0);
btQuaternion gVRTeleportOrnLocal(0,0,0,1);

extern btVector3 gVRTeleportPos1;
extern btQuaternion gVRTeleportOrn;
btScalar gVRTeleportRotZ = 0;

extern btVector3 gVRGripperPos;
extern btQuaternion gVRGripperOrn;
extern btVector3 gVRController2Pos;
extern btQuaternion gVRController2Orn;
extern btScalar gVRGripperAnalog;
extern btScalar gVRGripper2Analog;
extern bool gCloseToKuka;
extern bool gEnableRealTimeSimVR;
extern bool gCreateDefaultRobotAssets;
extern int gInternalSimFlags;
extern int gCreateObjectSimVR;
extern bool gResetSimulation;
extern int gEnableKukaControl;
int gGraspingController = -1;
extern btScalar simTimeScalingFactor;
bool gBatchUserDebugLines = true;
extern bool gVRGripperClosed;

const char* startFileNameVR = "0_VRDemoSettings.txt";

#include <vector>

static void loadCurrentSettingsVR(b3CommandLineArgs& args)
{
	//int currentEntry = 0;
	FILE* f = fopen(startFileNameVR, "r");
	if (f)
	{
		char oneline[1024];
		char* argv[] = { 0,&oneline[0] };
		
		while (fgets(oneline, 1024, f) != NULL)
		{
			char *pos;
			if ((pos = strchr(oneline, '\n')) != NULL)
				*pos = '\0';
			args.addArgs(2, argv);
		}
		fclose(f);
	}
	
};

//remember the settings (you don't want to re-tune again and again...)
static void saveCurrentSettingsVR()
{
	FILE* f = fopen(startFileNameVR, "w");
	if (f)
	{
		fprintf(f, "--camPosX= %f\n", gVRTeleportPos1[0]);
		fprintf(f, "--camPosY= %f\n", gVRTeleportPos1[1]);
		fprintf(f, "--camPosZ= %f\n", gVRTeleportPos1[2]);
		fprintf(f, "--camRotZ= %f\n", gVRTeleportRotZ);
		fclose(f);
	}
};

#if B3_USE_MIDI



static float getParamf(float rangeMin, float rangeMax, int midiVal)
{
	float v = rangeMin + (rangeMax - rangeMin)* (float(midiVal / 127.));
	return v;
}
void midiCallback(double deltatime, std::vector< unsigned char > *message, void *userData)
{
	unsigned int nBytes = message->size();
	for (unsigned int i = 0; i<nBytes; i++)
		std::cout << "Byte " << i << " = " << (int)message->at(i) << ", ";
	if (nBytes > 0)
		std::cout << "stamp = " << deltatime << std::endl;
	
	if (nBytes > 2)
	{
		
		if (message->at(0) == 176)
		{
			if (message->at(1) == 16)
			{
				gVRTeleportRotZ= getParamf(-3.1415, 3.1415, message->at(2));
				gVRTeleportOrn = btQuaternion(btVector3(0, 0, 1), gVRTeleportRotZ);
				saveCurrentSettingsVR();
//				b3Printf("gVRTeleportOrnLocal rotZ = %f\n", gVRTeleportRotZ);
			}

			if (message->at(1) == 32)
			{
				gCreateDefaultRobotAssets = 1;
			}

			for (int i = 0; i < 3; i++)
			{
				if (message->at(1) == i)
				{
					gVRTeleportPos1[i] = getParamf(-2, 2, message->at(2));
					saveCurrentSettingsVR();
//					b3Printf("gVRTeleportPos[%d] =  %f\n", i,gVRTeleportPosLocal[i]);

				}
			}
		}
	}
}

#endif //B3_USE_MIDI

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
	eGUIHelperAutogenerateGraphicsObjects,
	eGUIUserDebugAddText,
	eGUIUserDebugAddLine,
	eGUIUserDebugAddParameter,
	eGUIUserDebugRemoveItem,
	eGUIUserDebugRemoveAllItems,
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

enum MyMouseCommandType
{
	MyMouseMove = 1,
	MyMouseButtonDown,
	MyMouseButtonUp

};
struct MyMouseCommand
{
	btVector3 m_rayFrom;
	btVector3 m_rayTo;
	int m_type;
};

struct	MotionArgs
{
	MotionArgs()
		:m_physicsServerPtr(0)
	{
		for (int i=0;i<MAX_VR_CONTROLLERS;i++)
		{
			m_vrControllerEvents[i].m_numButtonEvents = 0;
			m_vrControllerEvents[i].m_numMoveEvents = 0;
			for (int b=0;b<MAX_VR_BUTTONS;b++)
			{
				m_vrControllerEvents[i].m_buttons[b]=0;
			}

			m_isVrControllerPicking[i] = false;
			m_isVrControllerDragging[i] = false;
			m_isVrControllerReleasing[i] = false;
			m_isVrControllerTeleporting[i] = false;
		}
	}
	b3CriticalSection* m_cs;
	b3CriticalSection* m_cs2;
	b3CriticalSection* m_cs3;
	b3CriticalSection* m_csGUI;


	btAlignedObjectArray<MyMouseCommand> m_mouseCommands;

	b3VRControllerEvent m_vrControllerEvents[MAX_VR_CONTROLLERS];
	
	b3VRControllerEvent m_sendVrControllerEvents[MAX_VR_CONTROLLERS];

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


float clampedDeltaTime  = 0.2;
float sleepTimeThreshold = 8./1000.;


void	MotionThreadFunc(void* userPtr,void* lsMemory)
{
	printf("MotionThreadFunc thread started\n");
	//MotionThreadLocalStorage* localStorage = (MotionThreadLocalStorage*) lsMemory;

	MotionArgs* args = (MotionArgs*) userPtr;
	//int workLeft = true;
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
			BT_PROFILE("loop");
			
			{
				BT_PROFILE("usleep(0)");
				b3Clock::usleep(0);
			}
			double dt = double(clock.getTimeMicroseconds())/1000000.;
			clock.reset();
			deltaTimeInSeconds+= dt;
			
			{
				
				//process special controller commands, such as
				//VR controller button press/release and controller motion

				for (int c=0;c<MAX_VR_CONTROLLERS;c++)
				{
				
					btVector3 from = args->m_vrControllerPos[c];
					btMatrix3x3 mat(args->m_vrControllerOrn[c]);
				
					btScalar pickDistance = 1000.;
					btVector3 to = from+mat.getColumn(0)*pickDistance;
//					btVector3 toY = from+mat.getColumn(1)*pickDistance;
//					btVector3 toZ = from+mat.getColumn(2)*pickDistance;

					if (args->m_isVrControllerTeleporting[c])
					{
						args->m_isVrControllerTeleporting[c] = false;
						args->m_physicsServerPtr->pickBody(from, to);
						args->m_physicsServerPtr->removePickingConstraint();
					}

//					if (!gEnableKukaControl)
					{
						if (args->m_isVrControllerPicking[c])
						{
							args->m_isVrControllerPicking[c]  = false;
							args->m_isVrControllerDragging[c] = true;
							args->m_physicsServerPtr->pickBody(from, to);
							//printf("PICK!\n");
						}
					}

					 if (args->m_isVrControllerDragging[c])
					 {
						 args->m_physicsServerPtr->movePickedBody(from, to);
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
					//b3Warning("Clamp deltaTime from %f to %f",deltaTimeInSeconds, clampedDeltaTime);
				}
				
				

				args->m_csGUI->lock();

				int numSendVrControllers = 0;
				for (int i=0;i<MAX_VR_CONTROLLERS;i++)
				{
					if (args->m_vrControllerEvents[i].m_numButtonEvents+args->m_vrControllerEvents[i].m_numMoveEvents)
					{
						args->m_sendVrControllerEvents[numSendVrControllers++] =
							args->m_vrControllerEvents[i];


						if (args->m_vrControllerEvents[i].m_numButtonEvents)
						{
							for (int b=0;b<MAX_VR_BUTTONS;b++)
							{
								args->m_vrControllerEvents[i].m_buttons[b] &= eButtonIsDown;
							}
						}
						args->m_vrControllerEvents[i].m_numMoveEvents = 0;
						args->m_vrControllerEvents[i].m_numButtonEvents = 0;
					}
				}

				args->m_csGUI->unlock();
				{
					BT_PROFILE("stepSimulationRealTime");
					args->m_physicsServerPtr->stepSimulationRealTime(deltaTimeInSeconds, args->m_sendVrControllerEvents,numSendVrControllers);
				}
				deltaTimeInSeconds = 0;
				
			}

			args->m_csGUI->lock();
			for (int i = 0; i < args->m_mouseCommands.size(); i++)
			{
				switch (args->m_mouseCommands[i].m_type)
				{
				case MyMouseMove:
				{
					args->m_physicsServerPtr->movePickedBody(args->m_mouseCommands[i].m_rayFrom, args->m_mouseCommands[i].m_rayTo);
					break;
				};
				case MyMouseButtonDown:
				{
					args->m_physicsServerPtr->pickBody(args->m_mouseCommands[i].m_rayFrom, args->m_mouseCommands[i].m_rayTo);
					break;
				}
				case MyMouseButtonUp:
				{
					args->m_physicsServerPtr->removePickingConstraint();
					break;
				}

				default:
				{
				}
				
				}
			}
			args->m_mouseCommands.clear();
			args->m_csGUI->unlock();

			{
				BT_PROFILE("processClientCommands");
				args->m_physicsServerPtr->processClientCommands();
			}
			
		} while (args->m_cs->getSharedParam(0)!=eRequestTerminateMotion);
	} else
	{
		args->m_cs->lock();
		args->m_cs->setSharedParam(0,eMotionInitializationFailed);
		args->m_cs->unlock();
	}


	//do nothing

}



void*	MotionlsMemoryFunc()
{
	//don't create local store memory, just return 0
	return new MotionThreadLocalStorage;
}



struct UserDebugDrawLine
{
	double	m_debugLineFromXYZ[3];
	double	m_debugLineToXYZ[3];
	double	m_debugLineColorRGB[3];
	double	m_lineWidth;
	
	double m_lifeTime;
	int m_itemUniqueId;
};

struct UserDebugParameter
{
	char m_text[1024];
	double m_rangeMin;
	double m_rangeMax;
	btScalar m_value;
	int m_itemUniqueId;
};

struct UserDebugText
{
	char m_text[1024];
	double m_textPositionXYZ[3];
	double m_textColorRGB[3];
	double textSize;

	double m_lifeTime;
	int m_itemUniqueId;
};



class MultiThreadedOpenGLGuiHelper : public GUIHelperInterface
{
//	CommonGraphicsApp* m_app;
	
	b3CriticalSection* m_cs;
	b3CriticalSection* m_cs2;
	b3CriticalSection* m_cs3;
	b3CriticalSection* m_csGUI;


public:

	GUIHelperInterface* m_childGuiHelper;

	int m_uidGenerator;
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
	
	void mainThreadRelease()
	{
		BT_PROFILE("mainThreadRelease");

		getCriticalSection()->setSharedParam(1,eGUIHelperIdle);
		getCriticalSection3()->lock();
		getCriticalSection2()->unlock();
		getCriticalSection()->lock();
		getCriticalSection2()->lock();
		getCriticalSection()->unlock();
		getCriticalSection3()->unlock();

	}

	void workerThreadWait()
	{
		BT_PROFILE("workerThreadWait");
		m_cs2->lock();
		m_cs->unlock();
		m_cs2->unlock();
		m_cs3->lock();
		m_cs3->unlock();

		
		
		while (m_cs->getSharedParam(1)!=eGUIHelperIdle)
		{
			b3Clock::usleep(0);
		}
	}

	MultiThreadedOpenGLGuiHelper(CommonGraphicsApp* app, GUIHelperInterface* guiHelper)
		:
	//m_app(app),
		m_cs(0),
		m_cs2(0),
		m_cs3(0),
		m_csGUI(0),
		m_uidGenerator(0),
		m_texels(0),
		m_textureId(-1)
	{
		m_childGuiHelper = guiHelper;;

	}

	virtual ~MultiThreadedOpenGLGuiHelper()
	{
		//delete m_childGuiHelper;
	}

	void setCriticalSection(b3CriticalSection* cs)
	{
		m_cs = cs;
	}

	b3CriticalSection* getCriticalSection()
	{
		return m_cs;
	}


	void setCriticalSection2(b3CriticalSection* cs)
	{
		m_cs2 = cs;
	}

	b3CriticalSection* getCriticalSection2()
	{
		return m_cs2;
	}

	void setCriticalSection3(b3CriticalSection* cs)
	{
		m_cs3 = cs;
	}

	void setCriticalSectionGUI(b3CriticalSection* cs)
	{
		m_csGUI  = cs;
	}


	b3CriticalSection* getCriticalSection3()
	{
		return m_cs3;
	}
	btRigidBody* m_body;
	btVector3 m_color3;
	virtual void createRigidBodyGraphicsObject(btRigidBody* body,const btVector3& color)
	{
		m_body = body;
		m_color3 = color;
		m_cs->lock();
		m_cs->setSharedParam(1,eGUIHelperCreateRigidBodyGraphicsObject);
		workerThreadWait();

	}

	btCollisionObject* m_obj;
	btVector3 m_color2;

	virtual void createCollisionObjectGraphicsObject(btCollisionObject* obj,const btVector3& color)
	{
		m_obj = obj;
		m_color2 = color;
		m_cs->lock();
		m_cs->setSharedParam(1,eGUIHelperCreateCollisionObjectGraphicsObject);
		workerThreadWait();

	}

	btCollisionShape* m_colShape;
	virtual void createCollisionShapeGraphicsObject(btCollisionShape* collisionShape)
	{
		m_colShape = collisionShape;
		m_cs->lock();
		m_cs->setSharedParam(1,eGUIHelperCreateCollisionShapeGraphicsObject);
		workerThreadWait();

	}

	virtual void syncPhysicsToGraphics(const btDiscreteDynamicsWorld* rbWorld)
	{
	    //this check is to prevent a crash, in case we removed all graphics instances, but there are still physics objects.
	    //the check will be obsolete, once we have a better/safer way of synchronizing physics->graphics transforms
        if ( m_childGuiHelper->getRenderInterface() && m_childGuiHelper->getRenderInterface()->getTotalNumInstances()>0)
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
		
		workerThreadWait();


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
		workerThreadWait();

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
		workerThreadWait();
		return m_instanceId;
	}

    virtual void removeAllGraphicsInstances()
    {
        m_cs->lock();
		m_cs->setSharedParam(1,eGUIHelperRemoveAllGraphicsInstances);
		workerThreadWait();
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
		return m_childGuiHelper->getRenderInterface();
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
		workerThreadWait();
	}
	

	btDiscreteDynamicsWorld* m_dynamicsWorld;

	virtual void autogenerateGraphicsObjects(btDiscreteDynamicsWorld* rbWorld) 
	{
		m_dynamicsWorld = rbWorld;
		m_cs->lock();
		m_cs->setSharedParam(1, eGUIHelperAutogenerateGraphicsObjects);
		workerThreadWait();
	}
    
	virtual void drawText3D( const char* txt, float posX, float posZY, float posZ, float size)
	{
	}


	btAlignedObjectArray<UserDebugText> m_userDebugText;
	
	UserDebugText m_tmpText;

	virtual int		addUserDebugText3D( const char* txt, const double positionXYZ[3], const double	textColorRGB[3], double size, double lifeTime)
	{
		
		m_tmpText.m_itemUniqueId = m_uidGenerator++;
		m_tmpText.m_lifeTime = lifeTime;
		m_tmpText.textSize = size;
		//int len = strlen(txt);
		strcpy(m_tmpText.m_text,txt);
		m_tmpText.m_textPositionXYZ[0] = positionXYZ[0];
		m_tmpText.m_textPositionXYZ[1] = positionXYZ[1];
		m_tmpText.m_textPositionXYZ[2] = positionXYZ[2];
		m_tmpText.m_textColorRGB[0] = textColorRGB[0];
		m_tmpText.m_textColorRGB[1] = textColorRGB[1];
		m_tmpText.m_textColorRGB[2] = textColorRGB[2];

		m_cs->lock();
		m_cs->setSharedParam(1, eGUIUserDebugAddText);
		workerThreadWait();

		return m_userDebugText[m_userDebugText.size()-1].m_itemUniqueId;
	}

	btAlignedObjectArray<UserDebugParameter*> m_userDebugParams;
	UserDebugParameter m_tmpParam;

	virtual int		readUserDebugParameter(int itemUniqueId, double* value) 
	{ 
		for (int i=0;i<m_userDebugParams.size();i++)
		{
			if (m_userDebugParams[i]->m_itemUniqueId == itemUniqueId)
			{
				*value = m_userDebugParams[i]->m_value;
				return 1;
			}
		}
		return 0;
	}

	virtual int		addUserDebugParameter(const char* txt, double	rangeMin, double	rangeMax, double startValue)
	{
		strcpy(m_tmpParam.m_text,txt);
		m_tmpParam.m_rangeMin = rangeMin;
		m_tmpParam.m_rangeMax = rangeMax;
		m_tmpParam.m_value = startValue;
		m_tmpParam.m_itemUniqueId = m_uidGenerator++;

		m_cs->lock();
		m_cs->setSharedParam(1, eGUIUserDebugAddParameter);
		workerThreadWait();

		return (*m_userDebugParams[m_userDebugParams.size()-1]).m_itemUniqueId;
	}


	btAlignedObjectArray<UserDebugDrawLine> m_userDebugLines;
	UserDebugDrawLine m_tmpLine;

	virtual int		addUserDebugLine(const double	debugLineFromXYZ[3], const double	debugLineToXYZ[3], const double	debugLineColorRGB[3], double lineWidth, double lifeTime )
	{
		m_tmpLine.m_lifeTime = lifeTime;
		m_tmpLine.m_lineWidth = lineWidth;
		m_tmpLine.m_itemUniqueId = m_uidGenerator++;
		m_tmpLine.m_debugLineFromXYZ[0] = debugLineFromXYZ[0];
		m_tmpLine.m_debugLineFromXYZ[1] = debugLineFromXYZ[1];
		m_tmpLine.m_debugLineFromXYZ[2] = debugLineFromXYZ[2];

		m_tmpLine.m_debugLineToXYZ[0] = debugLineToXYZ[0];
		m_tmpLine.m_debugLineToXYZ[1] = debugLineToXYZ[1];
		m_tmpLine.m_debugLineToXYZ[2] = debugLineToXYZ[2];
		
		m_tmpLine.m_debugLineColorRGB[0] = debugLineColorRGB[0];
		m_tmpLine.m_debugLineColorRGB[1] = debugLineColorRGB[1];
		m_tmpLine.m_debugLineColorRGB[2] = debugLineColorRGB[2];
		
		m_cs->lock();
		m_cs->setSharedParam(1, eGUIUserDebugAddLine);
		workerThreadWait();
		return m_userDebugLines[m_userDebugLines.size()-1].m_itemUniqueId;
	}

	int m_removeDebugItemUid;

	virtual void	removeUserDebugItem( int debugItemUniqueId)
	{
		m_removeDebugItemUid = debugItemUniqueId;
		m_cs->lock();
		m_cs->setSharedParam(1, eGUIUserDebugRemoveItem);
		workerThreadWait();

	}	
	virtual void	removeAllUserDebugItems( )
	{
		m_cs->lock();
		m_cs->setSharedParam(1, eGUIUserDebugRemoveAllItems);
		workerThreadWait();
	
	}

};



class PhysicsServerExample : public SharedMemoryCommon
{
	PhysicsServerSharedMemory	m_physicsServer;
	b3ThreadSupportInterface* m_threadSupport;
	MotionArgs m_args[MAX_MOTION_NUM_THREADS];
	MultiThreadedOpenGLGuiHelper* m_multiThreadedHelper;
    bool m_wantsShutdown;
#ifdef B3_USE_MIDI
	RtMidiIn* m_midi;
#endif
    bool m_isConnected;
    btClock m_clock;
	bool m_replay;
//	int m_options;
	
#ifdef BT_ENABLE_VR
	TinyVRGui* m_tinyVrGui;
#endif

public:

	PhysicsServerExample(MultiThreadedOpenGLGuiHelper* helper, SharedMemoryInterface* sharedMem=0, int options=0);

	virtual ~PhysicsServerExample();

	virtual void	initPhysics();

	virtual void	stepSimulation(float deltaTime);

	virtual void updateGraphics();

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
	void drawUserDebugLines();
	virtual void    exitPhysics();

	virtual void	physicsDebugDraw(int debugFlags);

	btVector3	getRayTo(int x,int y);

	virtual void	vrControllerButtonCallback(int controllerId, int button, int state, float pos[4], float orientation[4]);
	virtual void	vrControllerMoveCallback(int controllerId, float pos[4], float orientation[4], float analogAxis);
	

	virtual bool	mouseMoveCallback(float x,float y)
	{
		if (m_replay)
			return false;

		CommonRenderInterface* renderer = m_multiThreadedHelper->m_childGuiHelper->getRenderInterface();// m_guiHelper->getRenderInterface();
	
		if (!renderer)
		{
			return false;
		}

		btVector3 rayTo = getRayTo(int(x), int(y));
		btVector3 rayFrom;
		renderer->getActiveCamera()->getCameraPosition(rayFrom);

		MyMouseCommand cmd;
		cmd.m_rayFrom = rayFrom;
		cmd.m_rayTo = rayTo;
		cmd.m_type = MyMouseMove;
		m_args[0].m_csGUI->lock();
		m_args[0].m_mouseCommands.push_back(cmd);
		m_args[0].m_csGUI->unlock();
		
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

				MyMouseCommand cmd;
				cmd.m_rayFrom = rayFrom;
				cmd.m_rayTo = rayTo;
				cmd.m_type = MyMouseButtonDown;
				
				m_args[0].m_csGUI->lock();
				m_args[0].m_mouseCommands.push_back(cmd);
				m_args[0].m_csGUI->unlock();
				

			}
		} else
		{
			if (button==0)
			{
				//m_physicsServer.removePickingConstraint();
				MyMouseCommand cmd;
				cmd.m_rayFrom.setValue(0,0,0);
				cmd.m_rayTo.setValue(0, 0, 0);
				cmd.m_type = MyMouseButtonUp;
				
				m_args[0].m_csGUI->lock();
				m_args[0].m_mouseCommands.push_back(cmd);
				m_args[0].m_csGUI->unlock();
				//remove p2p
			}
		}

		//printf("button=%d, state=%d\n",button,state);
		return false;
	}
	virtual bool	keyboardCallback(int key, int state){
		if (key=='w' && state)
		{
			gVRTeleportPos1[0]+=0.1;
			saveCurrentSettingsVR();
		}
		if (key=='s' && state)
		{
			gVRTeleportPos1[0]-=0.1;
			saveCurrentSettingsVR();
		}
		if (key=='a' && state)
		{
			gVRTeleportPos1[1]-=0.1;
			saveCurrentSettingsVR();
		}
		if (key=='d' && state)
		{
			gVRTeleportPos1[1]+=0.1;
			saveCurrentSettingsVR();
		}
		if (key=='q' && state)
		{
			gVRTeleportPos1[2]+=0.1;
			saveCurrentSettingsVR();
		}
		if (key=='e' && state)
		{
			gVRTeleportPos1[2]-=0.1;
			saveCurrentSettingsVR();
		}
		if (key=='z' && state)
		{
			gVRTeleportRotZ+=0.1;
			gVRTeleportOrn = btQuaternion(btVector3(0, 0, 1), gVRTeleportRotZ);
			saveCurrentSettingsVR();
		}
		
		return false;
	}

	virtual void setSharedMemoryKey(int key)
	{
		m_physicsServer.setSharedMemoryKey(key);
	}

	virtual void	processCommandLineArgs(int argc, char* argv[])
	{
		b3CommandLineArgs args(argc,argv);
		loadCurrentSettingsVR(args);
		int shmemKey;
		
		if (args.GetCmdLineArgument("sharedMemoryKey", shmemKey))
		{
			setSharedMemoryKey(shmemKey);
		}

		if (args.GetCmdLineArgument("camPosX", gVRTeleportPos1[0]))
		{
			printf("camPosX=%f\n", gVRTeleportPos1[0]);
		}

		if (args.GetCmdLineArgument("camPosY", gVRTeleportPos1[1]))
		{
			printf("camPosY=%f\n", gVRTeleportPos1[1]);
		}

		if (args.GetCmdLineArgument("camPosZ", gVRTeleportPos1[2]))
		{
			printf("camPosZ=%f\n", gVRTeleportPos1[2]);
		}

		float camRotZ = 0.f;
		if (args.GetCmdLineArgument("camRotZ", camRotZ))
		{
			printf("camRotZ = %f\n", camRotZ);
			btQuaternion ornZ(btVector3(0, 0, 1), camRotZ);
			gVRTeleportOrn = ornZ;
		}

		if (args.CheckCmdLineFlag("robotassets"))
		{
			gCreateDefaultRobotAssets = true;
		}

		if (args.CheckCmdLineFlag("norobotassets"))
		{
			gCreateDefaultRobotAssets = false;
		}


	}

};

#ifdef B3_USE_MIDI
static bool chooseMidiPort(RtMidiIn *rtmidi)
{
	/*

	std::cout << "\nWould you like to open a virtual input port? [y/N] ";

	std::string keyHit;
	std::getline( std::cin, keyHit );
	if ( keyHit == "y" ) {
	rtmidi->openVirtualPort();
	return true;
	}
	*/

	std::string portName;
	unsigned int i = 0, nPorts = rtmidi->getPortCount();
	if (nPorts == 0) {
		std::cout << "No midi input ports available!" << std::endl;
		return false;
	}

	if (nPorts > 0) {
		std::cout << "\nOpening midi input port " << rtmidi->getPortName() << std::endl;
	}

	//  std::getline( std::cin, keyHit );  // used to clear out stdin
	rtmidi->openPort(i);

	return true;
}
#endif //B3_USE_MIDI


PhysicsServerExample::PhysicsServerExample(MultiThreadedOpenGLGuiHelper* helper, SharedMemoryInterface* sharedMem, int options)
:SharedMemoryCommon(helper),
m_physicsServer(sharedMem),
m_wantsShutdown(false),
m_isConnected(false),
m_replay(false)
//m_options(options)
#ifdef BT_ENABLE_VR
,m_tinyVrGui(0)
#endif
{
#ifdef B3_USE_MIDI
	m_midi = new   RtMidiIn();
	chooseMidiPort(m_midi);
	m_midi->setCallback(&midiCallback);
	// Don't ignore sysex, timing, or active sensing messages.
	m_midi->ignoreTypes(false, false, false);

#endif
	m_multiThreadedHelper = helper;
//	b3Printf("Started PhysicsServer\n");
}



PhysicsServerExample::~PhysicsServerExample()
{
#ifdef B3_USE_MIDI
	delete m_midi;
	m_midi = 0;
#endif
#ifdef BT_ENABLE_VR
	delete m_tinyVrGui;
#endif


	bool deInitializeSharedMemory = true;
	m_physicsServer.disconnectSharedMemory(deInitializeSharedMemory);
    m_isConnected = false;
	delete m_multiThreadedHelper;
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
			m_args[w].m_cs2 = m_threadSupport->createCriticalSection();
			m_args[w].m_cs3 = m_threadSupport->createCriticalSection();
			m_args[w].m_csGUI = m_threadSupport->createCriticalSection();

			m_args[w].m_cs->setSharedParam(0,eMotionIsUnInitialized);
			int numMoving = 0;
 			m_args[w].m_positions.resize(numMoving);
			m_args[w].m_physicsServerPtr = &m_physicsServer;
			//int index = 0;
			
			m_threadSupport->runTask(B3_THREAD_SCHEDULE_TASK, (void*) &this->m_args[w], w);
			
			while (m_args[w].m_cs->getSharedParam(0)==eMotionIsUnInitialized)
			{
				b3Clock::usleep(1000);
			}
		}

		m_args[0].m_cs->setSharedParam(1,eGUIHelperIdle);
		m_multiThreadedHelper->setCriticalSection(m_args[0].m_cs);
		m_multiThreadedHelper->setCriticalSection2(m_args[0].m_cs2);
		m_multiThreadedHelper->setCriticalSection3(m_args[0].m_cs3);
		m_multiThreadedHelper->setCriticalSectionGUI(m_args[0].m_csGUI);
		
		m_args[0].m_cs2->lock();


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
						//we need to call 'stepSimulation' to make sure that
						//other threads get out of blocking state (workerThreadWait)
						stepSimulation(0);
                };

		printf("stopping threads\n");

		m_threadSupport->deleteCriticalSection(m_args[0].m_cs);
		m_threadSupport->deleteCriticalSection(m_args[0].m_cs2);
		m_threadSupport->deleteCriticalSection(m_args[0].m_cs3);
		m_threadSupport->deleteCriticalSection(m_args[0].m_csGUI);

		delete m_threadSupport;   
		m_threadSupport = 0;

		//m_physicsServer.resetDynamicsWorld();
		
}



bool PhysicsServerExample::wantsTermination()
{
    return m_wantsShutdown;
}

void	PhysicsServerExample::updateGraphics()
{
	//check if any graphics related tasks are requested
	
	switch (m_multiThreadedHelper->getCriticalSection()->getSharedParam(1))
	{
	case eGUIHelperCreateCollisionShapeGraphicsObject:
	{
		m_multiThreadedHelper->m_childGuiHelper->createCollisionShapeGraphicsObject(m_multiThreadedHelper->m_colShape);
		m_multiThreadedHelper->mainThreadRelease();
		break;
	}
	case eGUIHelperCreateCollisionObjectGraphicsObject:
	{
		m_multiThreadedHelper->m_childGuiHelper->createCollisionObjectGraphicsObject(m_multiThreadedHelper->m_obj,
			m_multiThreadedHelper->m_color2);
		m_multiThreadedHelper->mainThreadRelease();

		break;
	}
	case eGUIHelperCreateRigidBodyGraphicsObject:
	{
		m_multiThreadedHelper->m_childGuiHelper->createRigidBodyGraphicsObject(m_multiThreadedHelper->m_body,m_multiThreadedHelper->m_color3);
		m_multiThreadedHelper->mainThreadRelease();
		break;
	}
	case eGUIHelperRegisterTexture:
	{
		
		m_multiThreadedHelper->m_textureId = m_multiThreadedHelper->m_childGuiHelper->registerTexture(m_multiThreadedHelper->m_texels,
						m_multiThreadedHelper->m_textureWidth,m_multiThreadedHelper->m_textureHeight);
		m_multiThreadedHelper->mainThreadRelease();
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
		m_multiThreadedHelper->mainThreadRelease();
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
		m_multiThreadedHelper->mainThreadRelease();
		break;
	}
	case eGUIHelperRemoveAllGraphicsInstances:
        {
            m_multiThreadedHelper->m_childGuiHelper->removeAllGraphicsInstances();
			if (m_multiThreadedHelper->m_childGuiHelper->getRenderInterface())
			{
				int numRenderInstances;
				numRenderInstances = m_multiThreadedHelper->m_childGuiHelper->getRenderInterface()->getTotalNumInstances();
				b3Assert(numRenderInstances==0);
			}
			m_multiThreadedHelper->mainThreadRelease();

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
 		m_multiThreadedHelper->mainThreadRelease();
            break;
        }
	case eGUIHelperAutogenerateGraphicsObjects:
	{
		m_multiThreadedHelper->m_childGuiHelper->autogenerateGraphicsObjects(m_multiThreadedHelper->m_dynamicsWorld);
		m_multiThreadedHelper->mainThreadRelease();
		break;
	}

	case eGUIUserDebugAddText:
	{
		m_multiThreadedHelper->m_userDebugText.push_back(m_multiThreadedHelper->m_tmpText);
		m_multiThreadedHelper->mainThreadRelease();
		break;
	}
	case eGUIUserDebugAddParameter:
	{
		UserDebugParameter* param = new UserDebugParameter(m_multiThreadedHelper->m_tmpParam);
		m_multiThreadedHelper->m_userDebugParams.push_back(param);

		{
        SliderParams slider(param->m_text,&param->m_value);
        slider.m_minVal=param->m_rangeMin;
        slider.m_maxVal=param->m_rangeMax;
		
		if (m_multiThreadedHelper->m_childGuiHelper->getParameterInterface())
	        m_multiThreadedHelper->m_childGuiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
		}

		//also add actual menu
		m_multiThreadedHelper->mainThreadRelease();
		break;
	}
	case eGUIUserDebugAddLine:
	{
		m_multiThreadedHelper->m_userDebugLines.push_back(m_multiThreadedHelper->m_tmpLine);
		m_multiThreadedHelper->mainThreadRelease();
			break;
	}
	case eGUIUserDebugRemoveItem:
	{
		for (int i=0;i<m_multiThreadedHelper->m_userDebugLines.size();i++)
		{
			if (m_multiThreadedHelper->m_userDebugLines[i].m_itemUniqueId == m_multiThreadedHelper->m_removeDebugItemUid)
			{
				m_multiThreadedHelper->m_userDebugLines.swap(i,m_multiThreadedHelper->m_userDebugLines.size()-1);
				m_multiThreadedHelper->m_userDebugLines.pop_back();
				break;
			}
		}


		for (int i=0;i<m_multiThreadedHelper->m_userDebugText.size();i++)
		{
			if (m_multiThreadedHelper->m_userDebugText[i].m_itemUniqueId == m_multiThreadedHelper->m_removeDebugItemUid)
			{
				m_multiThreadedHelper->m_userDebugText.swap(i,m_multiThreadedHelper->m_userDebugText.size()-1);
				m_multiThreadedHelper->m_userDebugText.pop_back();
				break;
			}
		}

		m_multiThreadedHelper->mainThreadRelease();
			break;
	}
	case eGUIUserDebugRemoveAllItems:
	{
		m_multiThreadedHelper->m_userDebugLines.clear();
		m_multiThreadedHelper->m_userDebugText.clear();
		m_multiThreadedHelper->m_uidGenerator = 0;
		m_multiThreadedHelper->mainThreadRelease();
			break;
	}
	case eGUIHelperIdle:
		{
			break;
		}
	default:
		{
			btAssert(0);
		}
	}
	

}

void	PhysicsServerExample::stepSimulation(float deltaTime)
{
	BT_PROFILE("PhysicsServerExample::stepSimulation");

	//this->m_physicsServer.processClientCommands();

	for (int i = m_multiThreadedHelper->m_userDebugLines.size()-1;i>=0;i--)
	{
		if (m_multiThreadedHelper->m_userDebugLines[i].m_lifeTime)
		{
			m_multiThreadedHelper->m_userDebugLines[i].m_lifeTime -= deltaTime;
			if (m_multiThreadedHelper->m_userDebugLines[i].m_lifeTime<=0)
			{
				m_multiThreadedHelper->m_userDebugLines.swap(i,m_multiThreadedHelper->m_userDebugLines.size()-1);
				m_multiThreadedHelper->m_userDebugLines.pop_back();
			}
		}
	}

	for (int i = m_multiThreadedHelper->m_userDebugText.size()-1;i>=0;i--)
	{
		if (m_multiThreadedHelper->m_userDebugText[i].m_lifeTime)
		{
			m_multiThreadedHelper->m_userDebugText[i].m_lifeTime -= deltaTime;
			if (m_multiThreadedHelper->m_userDebugText[i].m_lifeTime<=0)
			{
				m_multiThreadedHelper->m_userDebugText.swap(i,m_multiThreadedHelper->m_userDebugText.size()-1);
				m_multiThreadedHelper->m_userDebugText.pop_back();
			}
		}
	}
	updateGraphics();


	
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
extern int gVRTrackingObjectUniqueId;
extern btTransform gVRTrackingObjectTr;

  struct LineSegment
                {
                        btVector3 m_from;
                        btVector3 m_to;
                };

                struct ColorWidth
                {
                        btVector3FloatData m_color;
                        int width;
                        int getHash() const
                        {
                                unsigned char r = (unsigned char) m_color.m_floats[0]*255;
                                unsigned char g = (unsigned char) m_color.m_floats[1]*255;
                                unsigned char b = (unsigned char) m_color.m_floats[2]*255;
                                unsigned char w = width;
                                return r+(256*g)+(256*256*b)+(256*256*256*w);
                        }
                        bool equals(const ColorWidth& other) const
                        {
                                bool same = ((width == other.width) && (m_color.m_floats[0] == other.m_color.m_floats[0]) &&
                                        (m_color.m_floats[1] == other.m_color.m_floats[1]) &&
                                        (m_color.m_floats[2] == other.m_color.m_floats[2]));
                                return same;
                        }
                };

void PhysicsServerExample::drawUserDebugLines()
{
	//static char line0[1024];
	//static char line1[1024];

	//draw all user-debug-lines

	//add array of lines

	//draw all user- 'text3d' messages
	if (m_multiThreadedHelper)
	{


		//if gBatchUserDebugLines is true, batch lines based on color+width, to reduce line draw calls

		btAlignedObjectArray< btAlignedObjectArray<unsigned int> > sortedIndices;
		btAlignedObjectArray< btAlignedObjectArray<btVector3FloatData> > sortedLines;

		btHashMap<ColorWidth,int> hashedLines;

		for (int i = 0; i<m_multiThreadedHelper->m_userDebugLines.size(); i++)
		{
			btVector3 from;
			from.setValue(m_multiThreadedHelper->m_userDebugLines[i].m_debugLineFromXYZ[0],
				m_multiThreadedHelper->m_userDebugLines[i].m_debugLineFromXYZ[1],
				m_multiThreadedHelper->m_userDebugLines[i].m_debugLineFromXYZ[2]);
			btVector3 toX;
			toX.setValue(m_multiThreadedHelper->m_userDebugLines[i].m_debugLineToXYZ[0],
				m_multiThreadedHelper->m_userDebugLines[i].m_debugLineToXYZ[1],
				m_multiThreadedHelper->m_userDebugLines[i].m_debugLineToXYZ[2]);

			btVector3 color;
			color.setValue(m_multiThreadedHelper->m_userDebugLines[i].m_debugLineColorRGB[0],
				m_multiThreadedHelper->m_userDebugLines[i].m_debugLineColorRGB[1],
				m_multiThreadedHelper->m_userDebugLines[i].m_debugLineColorRGB[2]);
			ColorWidth cw;
			color.serializeFloat(cw.m_color);
			cw.width = m_multiThreadedHelper->m_userDebugLines[i].m_lineWidth;
			int index = -1;

			if (gBatchUserDebugLines)
			{
				int* indexPtr = hashedLines.find(cw);
				if (indexPtr)
				{
					index = *indexPtr;
				} else
				{
					index = sortedLines.size();
					sortedLines.expand();
					sortedIndices.expand();
					hashedLines.insert(cw,index);
				}
				btAssert(index>=0);
				if (index>=0)
				{
					btVector3FloatData from1,toX1;
					sortedIndices[index].push_back(sortedLines[index].size());
					from.serializeFloat(from1);
					sortedLines[index].push_back(from1);
					sortedIndices[index].push_back(sortedLines[index].size());
					toX.serializeFloat(toX1);
					sortedLines[index].push_back(toX1);
				}
			}
			else
			{
				m_guiHelper->getAppInterface()->m_renderer->drawLine(from, toX, color, m_multiThreadedHelper->m_userDebugLines[i].m_lineWidth);
			}
		}


		if (gBatchUserDebugLines)
		{
			for (int i=0;i<hashedLines.size();i++)
			{
				ColorWidth cw = hashedLines.getKeyAtIndex(i);
				int index = *hashedLines.getAtIndex(i);
				int stride = sizeof(btVector3FloatData);
				const float* positions = &sortedLines[index][0].m_floats[0];
				int numPoints = sortedLines[index].size();
				const unsigned int* indices = &sortedIndices[index][0];
				int numIndices = sortedIndices[index].size();
				m_guiHelper->getAppInterface()->m_renderer->drawLines(positions,cw.m_color.m_floats,numPoints, stride, indices,numIndices,cw.width);
			}
		}

		for (int i = 0; i<m_multiThreadedHelper->m_userDebugText.size(); i++)
		{
			m_guiHelper->getAppInterface()->drawText3D(m_multiThreadedHelper->m_userDebugText[i].m_text,
				m_multiThreadedHelper->m_userDebugText[i].m_textPositionXYZ[0],
				m_multiThreadedHelper->m_userDebugText[i].m_textPositionXYZ[1],
				m_multiThreadedHelper->m_userDebugText[i].m_textPositionXYZ[2],
				m_multiThreadedHelper->m_userDebugText[i].textSize);

		}
	}

}

void PhysicsServerExample::renderScene()
{
	btTransform vrTrans;
	//gVRTeleportPos1 = gVRTeleportPosLocal;
	//gVRTeleportOrn = gVRTeleportOrnLocal;

	///little VR test to follow/drive Husky vehicle
	if (gVRTrackingObjectUniqueId >= 0)
	{
		btTransform vrTrans;
		vrTrans.setOrigin(gVRTeleportPosLocal);
		vrTrans.setRotation(gVRTeleportOrnLocal);
			
		vrTrans = vrTrans * gVRTrackingObjectTr;

		gVRTeleportPos1 = vrTrans.getOrigin();
		gVRTeleportOrn = vrTrans.getRotation();
	}
		

	B3_PROFILE("PhysicsServerExample::RenderScene");

	drawUserDebugLines();

	if (gEnableRealTimeSimVR)
	{
		
		static int frameCount=0;
		//static btScalar prevTime = m_clock.getTimeSeconds();
		frameCount++;

#if 0

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
#endif

#ifdef BT_ENABLE_VR
		if ((gInternalSimFlags&2 ) && m_tinyVrGui==0)
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
			static char line0[1024];
			static char line1[1024];

			m_tinyVrGui->grapicalPrintf(line0,0,0,0,0,0,255);
			m_tinyVrGui->grapicalPrintf(line1,0,16,255,255,255,255);

			m_tinyVrGui->tick(dt,tr);
		}
#endif//BT_ENABLE_VR
	}
	///debug rendering
	//m_args[0].m_cs->lock();
	
	//gVRTeleportPos[0] += 0.01;
	btTransform tr2a, tr2;
	tr2a.setIdentity();
	tr2.setIdentity();
	tr2.setOrigin(gVRTeleportPos1);
	tr2a.setRotation(gVRTeleportOrn);
	btTransform trTotal = tr2*tr2a;
	btTransform trInv = trTotal.inverse();

	btMatrix3x3 vrOffsetRot;
	vrOffsetRot.setRotation(trInv.getRotation());
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			vrOffset[i + 4 * j] = vrOffsetRot[i][j];
		}
	}

	vrOffset[12]= trInv.getOrigin()[0];
	vrOffset[13]= trInv.getOrigin()[1];
	vrOffset[14]= trInv.getOrigin()[2];

	if (m_multiThreadedHelper->m_childGuiHelper->getRenderInterface())
	{
		m_multiThreadedHelper->m_childGuiHelper->getRenderInterface()->
		getActiveCamera()->setVRCameraOffsetTransform(vrOffset);
	}
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
		if (!gEnableRealTimeSimVR)
		{
			gEnableRealTimeSimVR = true;
			m_physicsServer.enableRealTimeSimulation(1);
		}
	}



	//m_args[0].m_cs->unlock();
}

void    PhysicsServerExample::physicsDebugDraw(int debugDrawFlags)
{
	drawUserDebugLines();

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
	{
		gGraspingController = controllerId;
		gEnableKukaControl = true;
	}

	btTransform trLocal;
	trLocal.setIdentity();
	trLocal.setRotation(btQuaternion(btVector3(0, 0, 1), SIMD_HALF_PI)*btQuaternion(btVector3(0, 1, 0), SIMD_HALF_PI));

	btTransform trOrg;
	trOrg.setIdentity();
	trOrg.setOrigin(btVector3(pos[0], pos[1], pos[2]));
	trOrg.setRotation(btQuaternion(orn[0], orn[1], orn[2], orn[3]));

	btTransform tr2a;
	tr2a.setIdentity();
	btTransform tr2;
	tr2.setIdentity();



	tr2.setOrigin(gVRTeleportPos1);
	tr2a.setRotation(gVRTeleportOrn);


	btTransform trTotal = tr2*tr2a*trOrg*trLocal;


	if (controllerId != gGraspingController)
	{
		if (button == 1 && state == 0)
		{
			//gResetSimulation = true;
			gVRTeleportPos1 = gLastPickPos;
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
#if 0//it confuses people, make it into a debug option in a VR GUI?
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
#endif
			}
		} else
		{
			
		}
	}


	if (button==32 && state==0)
	{

		if (controllerId == gGraspingController)
		{
			gCreateObjectSimVR = 1;
		}
		else
		{
//			gEnableKukaControl = !gEnableKukaControl;
		}
	}
	

	if (button==1)
	{
		m_args[0].m_isVrControllerTeleporting[controllerId] = true;
	}

	if (controllerId == gGraspingController && (button == 33))
	{
		gVRGripperClosed =(state!=0);
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
//			m_args[0].m_vrControllerPos[controllerId].setValue(pos[0] + gVRTeleportPos[0], pos[1] + gVRTeleportPos[1], pos[2] + gVRTeleportPos[2]);
	//		m_args[0].m_vrControllerOrn[controllerId].setValue(orn[0], orn[1], orn[2], orn[3]);
			m_args[0].m_vrControllerPos[controllerId] = trTotal.getOrigin();
			m_args[0].m_vrControllerOrn[controllerId] = trTotal.getRotation();
		}
		
	}

	m_args[0].m_csGUI->lock();
	m_args[0].m_vrControllerEvents[controllerId].m_controllerId = controllerId;
	m_args[0].m_vrControllerEvents[controllerId].m_pos[0] = trTotal.getOrigin()[0];
	m_args[0].m_vrControllerEvents[controllerId].m_pos[1] = trTotal.getOrigin()[1];
	m_args[0].m_vrControllerEvents[controllerId].m_pos[2] = trTotal.getOrigin()[2];
	m_args[0].m_vrControllerEvents[controllerId].m_orn[0] = trTotal.getRotation()[0];
	m_args[0].m_vrControllerEvents[controllerId].m_orn[1] = trTotal.getRotation()[1];
	m_args[0].m_vrControllerEvents[controllerId].m_orn[2] = trTotal.getRotation()[2];
	m_args[0].m_vrControllerEvents[controllerId].m_orn[3] = trTotal.getRotation()[3];
	m_args[0].m_vrControllerEvents[controllerId].m_numButtonEvents++;
	if (state)
	{
		m_args[0].m_vrControllerEvents[controllerId].m_buttons[button]|=eButtonIsDown+eButtonTriggered;
	} else
	{
		m_args[0].m_vrControllerEvents[controllerId].m_buttons[button]|=eButtonReleased;
		m_args[0].m_vrControllerEvents[controllerId].m_buttons[button] &= ~eButtonIsDown;
	}
	m_args[0].m_csGUI->unlock();
}


void	PhysicsServerExample::vrControllerMoveCallback(int controllerId, float pos[4], float orn[4], float analogAxis)
{

	if (controllerId <= 0 || controllerId >= MAX_VR_CONTROLLERS)
	{
		printf("Controller Id exceeds max: %d > %d", controllerId, MAX_VR_CONTROLLERS);
		return;
	}

	btTransform trLocal;
	trLocal.setIdentity();
	trLocal.setRotation(btQuaternion(btVector3(0, 0, 1), SIMD_HALF_PI)*btQuaternion(btVector3(0, 1, 0), SIMD_HALF_PI));

	btTransform trOrg;
	trOrg.setIdentity();
	trOrg.setOrigin(btVector3(pos[0], pos[1], pos[2]));
	trOrg.setRotation(btQuaternion(orn[0], orn[1], orn[2], orn[3]));

	btTransform tr2a;
	tr2a.setIdentity();
	btTransform tr2;
	tr2.setIdentity();

	

	tr2.setOrigin(gVRTeleportPos1);
	tr2a.setRotation(gVRTeleportOrn);


	btTransform trTotal = tr2*tr2a*trOrg*trLocal;

	if (controllerId == gGraspingController)
	{
		gVRGripperAnalog = analogAxis;

		gVRGripperPos = trTotal.getOrigin();
		gVRGripperOrn = trTotal.getRotation();
	}
	else
	{
		gVRGripper2Analog = analogAxis;
		gVRController2Pos = trTotal.getOrigin();
		gVRController2Orn = trTotal.getRotation();
		
		m_args[0].m_vrControllerPos[controllerId] = trTotal.getOrigin();
		m_args[0].m_vrControllerOrn[controllerId] = trTotal.getRotation();
	}

	m_args[0].m_csGUI->lock();
	m_args[0].m_vrControllerEvents[controllerId].m_controllerId = controllerId;
	m_args[0].m_vrControllerEvents[controllerId].m_pos[0] = trTotal.getOrigin()[0];
	m_args[0].m_vrControllerEvents[controllerId].m_pos[1] = trTotal.getOrigin()[1];
	m_args[0].m_vrControllerEvents[controllerId].m_pos[2] = trTotal.getOrigin()[2];
	m_args[0].m_vrControllerEvents[controllerId].m_orn[0] = trTotal.getRotation()[0];
	m_args[0].m_vrControllerEvents[controllerId].m_orn[1] = trTotal.getRotation()[1];
	m_args[0].m_vrControllerEvents[controllerId].m_orn[2] = trTotal.getRotation()[2];
	m_args[0].m_vrControllerEvents[controllerId].m_orn[3] = trTotal.getRotation()[3];
	m_args[0].m_vrControllerEvents[controllerId].m_numMoveEvents++;
	m_args[0].m_vrControllerEvents[controllerId].m_analogAxis = analogAxis;
	m_args[0].m_csGUI->unlock();

}
B3_STANDALONE_EXAMPLE(PhysicsServerCreateFunc)
