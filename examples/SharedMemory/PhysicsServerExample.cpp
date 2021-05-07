

//todo(erwincoumans): re-use the upcoming b3RobotSimAPI here

#include "PhysicsServerExample.h"

#include "../CommonInterfaces/Common2dCanvasInterface.h"
#include "PhysicsServerSharedMemory.h"
#include "Bullet3Common/b3CommandLineArgs.h"
#include "SharedMemoryCommon.h"
#include "Bullet3Common/b3Matrix3x3.h"
#include "../Utils/b3Clock.h"
#include "../MultiThreading/b3ThreadSupportInterface.h"
#include "SharedMemoryPublic.h"
//#define BT_ENABLE_VR
#define SYNC_CAMERA_USING_GUI_CS
#ifdef BT_ENABLE_VR
#include "../RenderingExamples/TinyVRGui.h"
#endif  //BT_ENABLE_VR

#include "../CommonInterfaces/CommonParameterInterface.h"
#include "../Importers/ImportURDFDemo/urdfStringSplit.h"

//@todo(erwincoumans) those globals are hacks for a VR demo, move this to Python/pybullet!
bool gEnablePicking = true;
bool gEnableTeleporting = true;
bool gEnableRendering = true;
bool gActivedVRRealTimeSimulation = false;

bool gEnableSyncPhysicsRendering = true;
static int gCamVisualizerWidth = 228;
static int gCamVisualizerHeight = 192;

static bool gEnableDefaultKeyboardShortcuts = true;
static bool gEnableDefaultMousePicking = true;

btScalar gVRTeleportRotZ = 0;

extern int gInternalSimFlags;
extern bool gResetSimulation;
int gGraspingController = -1;
extern btScalar simTimeScalingFactor;
bool gBatchUserDebugLines = true;

const char* startFileNameVR = "0_VRDemoSettings.txt";

#include <vector>

static void loadCurrentSettingsVR(b3CommandLineArgs& args)
{
	//int currentEntry = 0;
	FILE* f = fopen(startFileNameVR, "r");
	if (f)
	{
		char oneline[1024];
		char* argv[] = {0, &oneline[0]};

		while (fgets(oneline, 1024, f) != NULL)
		{
			char* pos;
			if ((pos = strchr(oneline, '\n')) != NULL)
				*pos = '\0';
			args.addArgs(2, argv);
		}
		fclose(f);
	}
};

//remember the settings (you don't want to re-tune again and again...)

static void saveCurrentSettingsVR(const btVector3& VRTeleportPos1)
{
	FILE* f = fopen(startFileNameVR, "w");
	if (f)
	{
		fprintf(f, "--camPosX= %f\n", VRTeleportPos1[0]);
		fprintf(f, "--camPosY= %f\n", VRTeleportPos1[1]);
		fprintf(f, "--camPosZ= %f\n", VRTeleportPos1[2]);
		fprintf(f, "--camRotZ= %f\n", gVRTeleportRotZ);
		fclose(f);
	}
};
bool gDebugRenderToggle = false;
void MotionThreadFunc(void* userPtr, void* lsMemory);
void* MotionlsMemoryFunc();
void MotionlsMemoryReleaseFunc(void* ptr);
#define MAX_MOTION_NUM_THREADS 1
enum
{
	numCubesX = 20,
	numCubesY = 20
};

enum TestExampleBrowserCommunicationEnums
{
	eRequestTerminateMotion = 13,
	eMotionIsUnInitialized,
	eMotionIsInitialized,
	eMotionInitializationFailed,
	eMotionHasTerminated
};

enum MultiThreadedGUIHelperCommunicationEnums
{
	eGUIHelperIdle = 13,
	eGUIHelperRegisterTexture,
	eGUIHelperRegisterGraphicsShape,
	eGUIHelperRegisterGraphicsInstance,
	eGUIHelperCreateCollisionShapeGraphicsObject,
	eGUIHelperCreateCollisionObjectGraphicsObject,
	eGUIHelperCreateRigidBodyGraphicsObject,
	eGUIHelperRemoveAllGraphicsInstances,
	eGUIHelperCopyCameraImageData,
	eGUIHelperDisplayCameraImageData,
	eGUIHelperAutogenerateGraphicsObjects,
	eGUIUserDebugAddText,
	eGUIUserDebugAddLine,
	eGUIUserDebugAddParameter,
	eGUIUserDebugRemoveItem,
	eGUIUserDebugRemoveAllItems,
	eGUIDumpFramesToVideo,
	eGUIHelperRemoveGraphicsInstance,
	eGUIHelperChangeGraphicsInstanceRGBAColor,
	eGUIHelperChangeGraphicsInstanceSpecularColor,
	eGUIHelperSetVisualizerFlag,
	eGUIHelperChangeGraphicsInstanceTextureId,
	eGUIHelperGetShapeIndexFromInstance,
	eGUIHelperChangeTexture,
	eGUIHelperRemoveTexture,
	eGUIHelperSetVisualizerFlagCheckRenderedFrame,
	eGUIHelperUpdateShape,
	eGUIHelperChangeGraphicsInstanceScaling,
	eGUIUserDebugRemoveAllParameters,
	eGUIHelperResetCamera,
	eGUIHelperChangeGraphicsInstanceFlags,
	eGUIHelperSetRgbBackground,
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
																  MotionlsMemoryReleaseFunc,
																  numThreads);
	b3ThreadSupportInterface* threadSupport = new b3PosixThreadSupport(constructionInfo);

	return threadSupport;
}

#elif defined(_WIN32)
#include "../MultiThreading/b3Win32ThreadSupport.h"

b3ThreadSupportInterface* createMotionThreadSupport(int numThreads)
{
	b3Win32ThreadSupport::Win32ThreadConstructionInfo threadConstructionInfo("MotionThreads", MotionThreadFunc, MotionlsMemoryFunc, MotionlsMemoryReleaseFunc, numThreads);
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

struct MotionArgs
{
	MotionArgs()
		: m_debugDrawFlags(0),
		  m_enableUpdateDebugDrawLines(true),
		  m_physicsServerPtr(0)
	{
		for (int i = 0; i < MAX_VR_CONTROLLERS; i++)
		{
			m_vrControllerEvents[i].m_numButtonEvents = 0;
			m_vrControllerEvents[i].m_numMoveEvents = 0;
			for (int b = 0; b < MAX_VR_BUTTONS; b++)
			{
				m_vrControllerEvents[i].m_buttons[b] = 0;
			}
			m_vrControllerPos[i].setValue(0, 0, 0);
			m_vrControllerOrn[i].setValue(0, 0, 0, 1);
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

	int m_debugDrawFlags;
	bool m_enableUpdateDebugDrawLines;
	btAlignedObjectArray<MyMouseCommand> m_mouseCommands;

	b3VRControllerEvent m_vrControllerEvents[MAX_VR_CONTROLLERS];
	b3VRControllerEvent m_sendVrControllerEvents[MAX_VR_CONTROLLERS];

	btAlignedObjectArray<b3KeyboardEvent> m_keyboardEvents;
	btAlignedObjectArray<b3KeyboardEvent> m_sendKeyEvents;
	btAlignedObjectArray<b3MouseEvent> m_allMouseEvents;
	btAlignedObjectArray<b3MouseEvent> m_sendMouseEvents;
	PhysicsServerSharedMemory* m_physicsServerPtr;
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

float clampedDeltaTime = 0.2;

void MotionThreadFunc(void* userPtr, void* lsMemory)
{
	printf("MotionThreadFunc thread started\n");
	//MotionThreadLocalStorage* localStorage = (MotionThreadLocalStorage*) lsMemory;

	MotionArgs* args = (MotionArgs*)userPtr;
	//int workLeft = true;
	b3Clock clock;
	clock.reset();
	b3Clock sleepClock;
	bool init = true;
	if (init)
	{
		unsigned int cachedSharedParam = eMotionIsInitialized;

		args->m_cs->lock();
		args->m_cs->setSharedParam(0, eMotionIsInitialized);
		args->m_cs->unlock();

		double deltaTimeInSeconds = 0;
		int numCmdSinceSleep1ms = 0;
		unsigned long long int prevTime = clock.getTimeMicroseconds();

		do
		{
			{
				b3Clock::usleep(0);
			}

			{
				if (sleepClock.getTimeMilliseconds() > 1)
				{
					sleepClock.reset();
					numCmdSinceSleep1ms = 0;
				}
			}

			unsigned long long int curTime = clock.getTimeMicroseconds();
			unsigned long long int dtMicro = curTime - prevTime;
			prevTime = curTime;
#if 1
			double dt = double(dtMicro) / 1000000.;
#else
			double dt = double(clock.getTimeMicroseconds()) / 1000000.;
			clock.reset();
#endif
			deltaTimeInSeconds += dt;

			{
				//process special controller commands, such as
				//VR controller button press/release and controller motion

				for (int c = 0; c < MAX_VR_CONTROLLERS; c++)
				{
					btVector3 from = args->m_vrControllerPos[c];
					btMatrix3x3 mat(args->m_vrControllerOrn[c]);

					btScalar pickDistance = 1000.;
					btVector3 to = from + mat.getColumn(0) * pickDistance;
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
							args->m_isVrControllerPicking[c] = false;
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
				if (deltaTimeInSeconds > clampedDeltaTime)
				{
					deltaTimeInSeconds = clampedDeltaTime;
					//b3Warning("Clamp deltaTime from %f to %f",deltaTimeInSeconds, clampedDeltaTime);
				}

				args->m_csGUI->lock();

				int numSendVrControllers = 0;
				for (int i = 0; i < MAX_VR_CONTROLLERS; i++)
				{
					if (args->m_vrControllerEvents[i].m_numButtonEvents + args->m_vrControllerEvents[i].m_numMoveEvents)
					{
						args->m_sendVrControllerEvents[numSendVrControllers++] =
							args->m_vrControllerEvents[i];

						if (args->m_vrControllerEvents[i].m_numButtonEvents)
						{
							for (int b = 0; b < MAX_VR_BUTTONS; b++)
							{
								args->m_vrControllerEvents[i].m_buttons[b] &= eButtonIsDown;
							}
						}
						args->m_vrControllerEvents[i].m_numMoveEvents = 0;
						args->m_vrControllerEvents[i].m_numButtonEvents = 0;
					}
				}

				args->m_sendKeyEvents.resize(args->m_keyboardEvents.size());
				for (int i = 0; i < args->m_keyboardEvents.size(); i++)
				{
					args->m_sendKeyEvents[i] = args->m_keyboardEvents[i];
					if (args->m_keyboardEvents[i].m_keyState & eButtonReleased)
					{
						args->m_keyboardEvents[i].m_keyState = 0;
					}
					else
					{
						args->m_keyboardEvents[i].m_keyState &= ~eButtonTriggered;
					}
				}
				//remove the 'released' events
				for (int i = args->m_keyboardEvents.size() - 1; i >= 0; i--)
				{
					if (args->m_keyboardEvents[i].m_keyState == 0)
					{
						args->m_keyboardEvents.removeAtIndex(i);
					}
				}

				b3KeyboardEvent* keyEvents = args->m_sendKeyEvents.size() ? &args->m_sendKeyEvents[0] : 0;

				args->m_csGUI->unlock();

				args->m_csGUI->lock();
				if (gEnableDefaultMousePicking)
				{
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
				}

				args->m_sendMouseEvents.resize(args->m_allMouseEvents.size());
				for (int i = 0; i < args->m_allMouseEvents.size(); i++)
				{
					args->m_sendMouseEvents[i] = args->m_allMouseEvents[i];
				}
				b3MouseEvent* mouseEvents = args->m_sendMouseEvents.size() ? &args->m_sendMouseEvents[0] : 0;

				args->m_allMouseEvents.clear();
				args->m_mouseCommands.clear();
				args->m_csGUI->unlock();

				{
					args->m_physicsServerPtr->stepSimulationRealTime(deltaTimeInSeconds, args->m_sendVrControllerEvents, numSendVrControllers, keyEvents, args->m_sendKeyEvents.size(), mouseEvents, args->m_sendMouseEvents.size());
				}
				{
					args->m_csGUI->lock();
					if (args->m_enableUpdateDebugDrawLines)
					{
						args->m_physicsServerPtr->physicsDebugDraw(args->m_debugDrawFlags);
						args->m_enableUpdateDebugDrawLines = false;
					}
					args->m_csGUI->unlock();
				}
				deltaTimeInSeconds = 0;
			}

			{
				args->m_physicsServerPtr->processClientCommands();
				numCmdSinceSleep1ms++;
			}

			args->m_physicsServerPtr->reportNotifications();

			args->m_cs->lock();
			cachedSharedParam = args->m_cs->getSharedParam(0);
			args->m_cs->unlock();

		} while (cachedSharedParam != eRequestTerminateMotion);
	}
	else
	{
		args->m_cs->lock();
		args->m_cs->setSharedParam(0, eMotionInitializationFailed);
		args->m_cs->unlock();
	}

	args->m_physicsServerPtr->disconnectSharedMemory(true);
	//do nothing
}

void* MotionlsMemoryFunc()
{
	//don't create local store memory, just return 0
	return new MotionThreadLocalStorage;
}

void MotionlsMemoryReleaseFunc(void* ptr)
{
	MotionThreadLocalStorage* p = (MotionThreadLocalStorage*)ptr;
	delete p;
}

struct UserDebugDrawLine
{
	double m_debugLineFromXYZ[3];
	double m_debugLineToXYZ[3];
	double m_debugLineColorRGB[3];
	double m_lineWidth;

	double m_lifeTime;
	int m_itemUniqueId;
	int m_trackingVisualShapeIndex;
	int m_replaceItemUid;
};

struct UserDebugParameter
{
	char m_text[1024];
	double m_rangeMin;
	double m_rangeMax;
	btScalar m_value;
	int m_itemUniqueId;
};

static void UserButtonToggle(int buttonId, bool buttonState, void* userPointer)
{
	UserDebugParameter* param = (UserDebugParameter*)userPointer;
	param->m_value += 1;
}


struct UserDebugText
{
	char m_text[1024];
	double m_textPositionXYZ1[3];
	double m_textColorRGB[3];
	double textSize;

	double m_lifeTime;
	int m_itemUniqueId;
	double m_textOrientation[4];
	int m_trackingVisualShapeIndex;
	int m_optionFlags;
};

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
		unsigned char r = (unsigned char)m_color.m_floats[0] * 255;
		unsigned char g = (unsigned char)m_color.m_floats[1] * 255;
		unsigned char b = (unsigned char)m_color.m_floats[2] * 255;
		unsigned char w = width;
		return r + (256 * g) + (256 * 256 * b) + (256 * 256 * 256 * w);
	}
	bool equals(const ColorWidth& other) const
	{
		bool same = ((width == other.width) && (m_color.m_floats[0] == other.m_color.m_floats[0]) &&
					 (m_color.m_floats[1] == other.m_color.m_floats[1]) &&
					 (m_color.m_floats[2] == other.m_color.m_floats[2]));
		return same;
	}
};

ATTRIBUTE_ALIGNED16(class)
MultithreadedDebugDrawer : public btIDebugDraw
{
	struct GUIHelperInterface* m_guiHelper;
	int m_debugMode;

	btAlignedObjectArray<btAlignedObjectArray<unsigned int> > m_sortedIndices;
	btAlignedObjectArray<btAlignedObjectArray<btVector3FloatData> > m_sortedLines;
	btHashMap<ColorWidth, int> m_hashedLines;

public:
	virtual void drawDebugDrawerLines()
	{
		if (m_hashedLines.size())
		{
			for (int i = 0; i < m_hashedLines.size(); i++)
			{
				ColorWidth cw = m_hashedLines.getKeyAtIndex(i);
				int index = *m_hashedLines.getAtIndex(i);
				int stride = sizeof(btVector3FloatData);
				const float* positions = &m_sortedLines[index][0].m_floats[0];
				int numPoints = m_sortedLines[index].size();
				const unsigned int* indices = &m_sortedIndices[index][0];
				int numIndices = m_sortedIndices[index].size();
				m_guiHelper->getRenderInterface()->drawLines(positions, cw.m_color.m_floats, numPoints, stride, indices, numIndices, cw.width);
			}
		}
	}
	MultithreadedDebugDrawer(GUIHelperInterface * guiHelper)
		: m_guiHelper(guiHelper),
		  m_debugMode(0)
	{
	}
	virtual ~MultithreadedDebugDrawer()
	{
	}
	virtual void drawLine(const btVector3& from, const btVector3& to, const btVector3& color)
	{
		{
			ColorWidth cw;
			color.serializeFloat(cw.m_color);
			cw.width = 1;
			int index = -1;

			int* indexPtr = m_hashedLines.find(cw);
			if (indexPtr)
			{
				index = *indexPtr;
			}
			else
			{
				index = m_sortedLines.size();
				m_sortedLines.expand();
				m_sortedIndices.expand();
				m_hashedLines.insert(cw, index);
			}
			btAssert(index >= 0);
			if (index >= 0)
			{
				btVector3FloatData from1, toX1;
				m_sortedIndices[index].push_back(m_sortedLines[index].size());
				from.serializeFloat(from1);
				m_sortedLines[index].push_back(from1);
				m_sortedIndices[index].push_back(m_sortedLines[index].size());
				to.serializeFloat(toX1);
				m_sortedLines[index].push_back(toX1);
			}
		}
	}

	virtual void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color)
	{
		drawLine(PointOnB, PointOnB + normalOnB * distance, color);
		btVector3 ncolor(0, 0, 0);
		drawLine(PointOnB, PointOnB + normalOnB * 0.01, ncolor);
	}

	virtual void reportErrorWarning(const char* warningString)
	{
	}
	virtual void draw3dText(const btVector3& location, const char* textString)
	{
	}
	virtual void setDebugMode(int debugMode)
	{
		m_debugMode = debugMode;
	}

	virtual int getDebugMode() const
	{
		return m_debugMode;
	}

	virtual void clearLines() override
	{
		m_hashedLines.clear();
		m_sortedIndices.clear();
		m_sortedLines.clear();
	}
	virtual void flushLines()
	{
	}
};

class MultiThreadedOpenGLGuiHelper : public GUIHelperInterface
{
	//	CommonGraphicsApp* m_app;

	b3CriticalSection* m_cs;
	b3CriticalSection* m_cs2;
	b3CriticalSection* m_cs3;
	b3CriticalSection* m_csGUI;

public:
	MultithreadedDebugDrawer* m_debugDraw;
	virtual void drawDebugDrawerLines()
	{
		if (m_debugDraw)
		{
			m_csGUI->lock();
			//draw stuff and flush?
			m_debugDraw->drawDebugDrawerLines();
			m_csGUI->unlock();
		}
	}
        virtual void clearLines()
        {
			m_csGUI->lock();
			if (m_debugDraw)
			{
				m_debugDraw->clearLines();
			}
			m_csGUI->unlock();
		}
        
	GUIHelperInterface* m_childGuiHelper;

	btHashMap<btHashPtr, int> m_cachedTextureIds;
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
	bool m_skipGraphicsUpdate;

	void mainThreadRelease()
	{
		BT_PROFILE("mainThreadRelease");

		setSharedParam(1, eGUIHelperIdle);
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

		if (m_skipGraphicsUpdate)
		{
			this->m_csGUI->lock();
			getCriticalSection()->setSharedParam(1, eGUIHelperIdle);
			this->m_csGUI->unlock();
			m_cs->unlock();
			return;
		}
		m_cs2->lock();
		m_cs->unlock();
		m_cs2->unlock();
		m_cs3->lock();
		m_cs3->unlock();


		m_csGUI->lock();
		unsigned int cachedSharedParam = m_cs->getSharedParam(1);
		m_csGUI->unlock();

		
		while (cachedSharedParam != eGUIHelperIdle)
		{
			b3Clock::usleep(0);
			m_csGUI->lock();
			cachedSharedParam = m_cs->getSharedParam(1);
			m_csGUI->unlock();
		}
	}

	MultiThreadedOpenGLGuiHelper(CommonGraphicsApp* app, GUIHelperInterface* guiHelper, int skipGraphicsUpdate)
		:  //m_app(app),
		m_cs(0),
		m_cs2(0),
		m_cs3(0),
		m_csGUI(0),
		m_debugDraw(0),
		m_uidGenerator(0),
		m_texels(0),
		m_shapeIndex(-1),
		m_textureId(-1),
		m_instanceId(-1),
		m_skipGraphicsUpdate(skipGraphicsUpdate)
	{
		m_cameraUpdated = 0;
		m_childGuiHelper = guiHelper;
	}

	virtual ~MultiThreadedOpenGLGuiHelper()
	{
		//delete m_childGuiHelper;
		if (m_debugDraw)
		{
			delete m_debugDraw;
			m_debugDraw = 0;
		}

		for (int i = 0; i < m_userDebugParams.size(); i++)
		{
			delete m_userDebugParams[i];
		}
		m_userDebugParams.clear();
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
		m_csGUI = cs;
	}

	b3CriticalSection* getCriticalSection3()
	{
		return m_cs3;
	}
	
	b3CriticalSection* getCriticalSectionGUI()
	{
		return m_csGUI;
	}
	
	btRigidBody* m_body;
	btVector3 m_color3;
	virtual void createRigidBodyGraphicsObject(btRigidBody* body, const btVector3& color)
	{
		m_cs->lock();

    m_body = body;
		m_color3 = color;
		setSharedParam(1, eGUIHelperCreateRigidBodyGraphicsObject);
		workerThreadWait();
	}

	btCollisionObject* m_obj;
	btVector3 m_color2;

	virtual void createCollisionObjectGraphicsObject(btCollisionObject* obj, const btVector3& color)
	{
		m_cs->lock();

    m_obj = obj;
		m_color2 = color;
		setSharedParam(1, eGUIHelperCreateCollisionObjectGraphicsObject);
		workerThreadWait();
	}

	btCollisionShape* m_colShape;
	virtual void createCollisionShapeGraphicsObject(btCollisionShape* collisionShape)
	{
		m_cs->lock();

    m_colShape = collisionShape;
    setSharedParam(1, eGUIHelperCreateCollisionShapeGraphicsObject);
		workerThreadWait();
	}

	virtual void syncPhysicsToGraphics(const btDiscreteDynamicsWorld* rbWorld)
	{
		//this check is to prevent a crash, in case we removed all graphics instances, but there are still physics objects.
		//the check will be obsolete, once we have a better/safer way of synchronizing physics->graphics transforms
		if (m_childGuiHelper->getRenderInterface() && m_childGuiHelper->getRenderInterface()->getTotalNumInstances() > 0)
		{
			m_childGuiHelper->syncPhysicsToGraphics(rbWorld);
		}
	}
	
	virtual void syncPhysicsToGraphics2(const btDiscreteDynamicsWorld* rbWorld)
	{
		 m_childGuiHelper->syncPhysicsToGraphics2(rbWorld);
	}

	virtual void syncPhysicsToGraphics2(const GUISyncPosition* positions, int numPositions)
	{
		m_childGuiHelper->syncPhysicsToGraphics2(positions, numPositions);
	}

	virtual void render(const btDiscreteDynamicsWorld* rbWorld)
	{
		m_childGuiHelper->render(0);
	}

	virtual void createPhysicsDebugDrawer(btDiscreteDynamicsWorld* rbWorld)
	{
		btAssert(rbWorld);
		if (m_debugDraw)
		{
			delete m_debugDraw;
			m_debugDraw = 0;
		}
                m_debugDraw = new MultithreadedDebugDrawer(this);
                rbWorld->setDebugDrawer(m_debugDraw);

		//m_childGuiHelper->createPhysicsDebugDrawer(rbWorld);
	}

	int m_removeTextureUid;

	virtual void removeTexture(int textureUid)
	{
		m_cs->lock();

    m_removeTextureUid = textureUid;
		setSharedParam(1, eGUIHelperRemoveTexture);

		workerThreadWait();
	}

	int m_updateShapeIndex;
	float* m_updateShapeVertices;
	int m_updateNumShapeVertices;
	virtual void updateShape(int shapeIndex, float* vertices, int numVertices)
	{
		m_cs->lock();

    m_updateShapeIndex = shapeIndex;
		m_updateShapeVertices = vertices;
		m_updateNumShapeVertices = numVertices;
		setSharedParam(1, eGUIHelperUpdateShape);
		workerThreadWait();
	}
	virtual int registerTexture(const unsigned char* texels, int width, int height)
	{
		int* cachedTexture = m_cachedTextureIds[texels];
		if (cachedTexture)
		{
			return *cachedTexture;
		}
		m_cs->lock();

    m_texels = texels;
		m_textureWidth = width;
		m_textureHeight = height;

		setSharedParam(1, eGUIHelperRegisterTexture);

		workerThreadWait();
		m_cachedTextureIds.insert(texels, m_textureId);
		return m_textureId;
	}
	virtual int registerGraphicsShape(const float* vertices, int numvertices, const int* indices, int numIndices, int primitiveType, int textureId)
	{
		m_cs->lock();
    m_csGUI->lock();
    m_vertices = vertices;
		m_numvertices = numvertices;
		m_indices = indices;
		m_numIndices = numIndices;
		m_primitiveType = primitiveType;
		m_textureId = textureId;
    m_csGUI->unlock();
		setSharedParam(1, eGUIHelperRegisterGraphicsShape);
		workerThreadWait();

    m_csGUI->lock();
    int shapeIndex = m_shapeIndex;
    m_csGUI->unlock();


		return shapeIndex;
	}

	int m_visualizerFlag;
	int m_visualizerEnable;
	int m_renderedFrames;

  void setSharedParam(int slot, int param)
  {
    m_csGUI->lock();
    m_cs->setSharedParam(slot, param);
    m_csGUI->unlock();
  }
	void setVisualizerFlag(int flag, int enable)
	{
		m_cs->lock();

    m_visualizerFlag = flag;
		m_visualizerEnable = enable;

		setSharedParam(1, eGUIHelperSetVisualizerFlag);
		workerThreadWait();
	}

	void setVisualizerFlagCallback(VisualizerFlagCallback callback)
	{
		m_childGuiHelper->setVisualizerFlagCallback(callback);
	}

	virtual int registerGraphicsInstance(int shapeIndex, const float* position, const float* quaternion, const float* color, const float* scaling)
	{
		m_shapeIndex = shapeIndex;
		m_position = position;
		m_quaternion = quaternion;
		m_color = color;
		m_scaling = scaling;

		m_cs->lock();
		setSharedParam(1, eGUIHelperRegisterGraphicsInstance);
		workerThreadWait();
		return m_instanceId;
	}

	virtual void removeAllGraphicsInstances()
	{
    m_cs->lock();
		m_cachedTextureIds.clear();
    setSharedParam(1, eGUIHelperRemoveAllGraphicsInstances);
		workerThreadWait();
	}

	int m_graphicsInstanceRemove;
	virtual void removeGraphicsInstance(int graphicsUid)
	{
		m_graphicsInstanceRemove = graphicsUid;
		m_cs->lock();
		setSharedParam(1, eGUIHelperRemoveGraphicsInstance);
		workerThreadWait();
	}

	int m_getShapeIndex_instance;
	int getShapeIndex_shapeIndex;

	virtual int getShapeIndexFromInstance(int instance)
	{
		m_getShapeIndex_instance = instance;
		m_cs->lock();
		setSharedParam(1, eGUIHelperGetShapeIndexFromInstance);
		getShapeIndex_shapeIndex = -1;
		workerThreadWait();
		return getShapeIndex_shapeIndex;
	}

	int m_graphicsInstanceChangeTextureId;
	int m_graphicsInstanceChangeTextureShapeIndex;
	virtual void replaceTexture(int shapeIndex, int textureUid)
	{
		m_graphicsInstanceChangeTextureShapeIndex = shapeIndex;
		m_graphicsInstanceChangeTextureId = textureUid;
		m_cs->lock();
		setSharedParam(1, eGUIHelperChangeGraphicsInstanceTextureId);
		workerThreadWait();
	}

	int m_changeTextureUniqueId;
	const unsigned char* m_changeTextureRgbTexels;
	int m_changeTextureWidth;
	int m_changeTextureHeight;

	virtual void changeTexture(int textureUniqueId, const unsigned char* rgbTexels, int width, int height)
	{
		m_changeTextureUniqueId = textureUniqueId;
		m_changeTextureRgbTexels = rgbTexels;
		m_changeTextureWidth = width;
		m_changeTextureHeight = height;
		m_cs->lock();
	  setSharedParam(1, eGUIHelperChangeTexture);
		workerThreadWait();
	}

	double m_rgbaColor[4];
	int m_graphicsInstanceChangeColor;
	virtual void changeRGBAColor(int instanceUid, const double rgbaColor[4])
	{
		m_graphicsInstanceChangeColor = instanceUid;
		m_rgbaColor[0] = rgbaColor[0];
		m_rgbaColor[1] = rgbaColor[1];
		m_rgbaColor[2] = rgbaColor[2];
		m_rgbaColor[3] = rgbaColor[3];
		m_cs->lock();
		setSharedParam(1, eGUIHelperChangeGraphicsInstanceRGBAColor);
		workerThreadWait();
	}

	
	int m_graphicsInstanceFlagsInstanceUid;
	int m_graphicsInstanceFlags;
	virtual void changeInstanceFlags(int instanceUid, int flags)
	{
		m_graphicsInstanceFlagsInstanceUid = instanceUid;
		m_graphicsInstanceFlags = flags;
		m_cs->lock();
		setSharedParam(1, eGUIHelperChangeGraphicsInstanceFlags);
		workerThreadWait();
	}

	double m_rgbBackground[3];
	virtual void setBackgroundColor(const double rgbBackground[3])
	{
		m_cs->lock();
		m_rgbBackground[0] = rgbBackground[0];
		m_rgbBackground[1] = rgbBackground[1];
		m_rgbBackground[2] = rgbBackground[2];
		
		setSharedParam(1, eGUIHelperSetRgbBackground);
		workerThreadWait();
		
	}


	int m_graphicsInstanceChangeScaling;
	double m_baseScaling[3];
	virtual void changeScaling(int instanceUid, const double scaling[3])
	{
		m_graphicsInstanceChangeScaling = instanceUid;
		m_baseScaling[0] = scaling[0];
		m_baseScaling[1] = scaling[1];
		m_baseScaling[2] = scaling[2];
		m_cs->lock();
		setSharedParam(1, eGUIHelperChangeGraphicsInstanceScaling);
		workerThreadWait();
	}

	double m_specularColor[3];
	int m_graphicsInstanceChangeSpecular;
	virtual void changeSpecularColor(int instanceUid, const double specularColor[3])
	{
		m_graphicsInstanceChangeSpecular = instanceUid;
		m_specularColor[0] = specularColor[0];
		m_specularColor[1] = specularColor[1];
		m_specularColor[2] = specularColor[2];
		m_cs->lock();
		setSharedParam(1, eGUIHelperChangeGraphicsInstanceSpecularColor);
		workerThreadWait();
	}

	virtual Common2dCanvasInterface* get2dCanvasInterface()
	{
		return m_childGuiHelper->get2dCanvasInterface();
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
	
	bool  m_cameraUpdated;
	float m_resetCameraCamDist;
	float m_resetCameraYaw;
	float m_resetCameraPitch;
	float m_resetCameraCamPosX;
	float m_resetCameraCamPosY;
	float m_resetCameraCamPosZ;

	virtual void resetCamera(float camDist, float yaw, float pitch, float camPosX, float camPosY, float camPosZ)
	{
		m_csGUI->lock();
		m_cameraUpdated = true;
		m_resetCameraCamDist = camDist;
		m_resetCameraYaw = yaw;
		m_resetCameraPitch = pitch;
		m_resetCameraCamPosX = camPosX;
		m_resetCameraCamPosY = camPosY;
		m_resetCameraCamPosZ = camPosZ;

#ifdef SYNC_CAMERA_USING_GUI_CS
		m_csGUI->unlock();
#else
		setSharedParam(1, eGUIHelperResetCamera);
		workerThreadWait();
		m_childGuiHelper->resetCamera(camDist, yaw, pitch, camPosX, camPosY, camPosZ);
#endif //SYNC_CAMERA_USING_GUI_CS
	}

	virtual bool getCameraInfo(int* width, int* height, float viewMatrix[16], float projectionMatrix[16], float camUp[3], float camForward[3], float hor[3], float vert[3], float* yaw, float* pitch, float* camDist, float camTarget[3]) const
	{
		return m_childGuiHelper->getCameraInfo(width, height, viewMatrix, projectionMatrix, camUp, camForward, hor, vert, yaw, pitch, camDist, camTarget);
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
		for (int i = 0; i < 16; i++)
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

		setSharedParam(1, eGUIHelperCopyCameraImageData);
		workerThreadWait();
	}

	virtual void debugDisplayCameraImageData(const float viewMatrix[16], const float projectionMatrix[16],
											 unsigned char* pixelsRGBA, int rgbaBufferSizeInPixels,
											 float* depthBuffer, int depthBufferSizeInPixels,
											 int* segmentationMaskBuffer, int segmentationMaskBufferSizeInPixels,
											 int startPixelIndex, int destinationWidth, int destinationHeight, int* numPixelsCopied)
	{
		m_cs->lock();
		for (int i = 0; i < 16; i++)
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

		setSharedParam(1, eGUIHelperDisplayCameraImageData);
		workerThreadWait();
	}

	virtual void setProjectiveTextureMatrices(const float viewMatrix[16], const float projectionMatrix[16])
	{
		if (m_childGuiHelper->getAppInterface() && m_childGuiHelper->getAppInterface()->m_renderer)
		{
			m_childGuiHelper->getAppInterface()->m_renderer->setProjectiveTextureMatrices(viewMatrix, projectionMatrix);
		}
	}

	virtual void setProjectiveTexture(bool useProjectiveTexture)
	{
		if (m_childGuiHelper->getAppInterface() && m_childGuiHelper->getAppInterface()->m_renderer)
		{
			m_childGuiHelper->getAppInterface()->m_renderer->setProjectiveTexture(useProjectiveTexture);
		}
	}

	btDiscreteDynamicsWorld* m_dynamicsWorld;

	virtual void autogenerateGraphicsObjects(btDiscreteDynamicsWorld* rbWorld)
	{
		m_dynamicsWorld = rbWorld;
		m_cs->lock();
		setSharedParam(1, eGUIHelperAutogenerateGraphicsObjects);
		workerThreadWait();
	}

	virtual void drawText3D(const char* txt, float posX, float posZY, float posZ, float size)
	{
	}

	virtual void drawText3D(const char* txt, float position[3], float orientation[4], float color[4], float size, int optionFlag)
	{
	}

	btAlignedObjectArray<UserDebugText> m_userDebugText;

	UserDebugText m_tmpText;
	int m_resultUserDebugTextUid;

	virtual int addUserDebugText3D(const char* txt, const double positionXYZ[3], const double orientation[4], const double textColorRGB[3], double size, double lifeTime, int trackingVisualShapeIndex, int optionFlags, int replaceItemUid)
	{
		if (replaceItemUid >= 0)
		{
			m_tmpText.m_itemUniqueId = replaceItemUid;
		}
		else
		{
			m_tmpText.m_itemUniqueId = m_uidGenerator++;
		}
		m_tmpText.m_lifeTime = lifeTime;
		m_tmpText.textSize = size;
		//int len = strlen(txt);
		strcpy(m_tmpText.m_text, txt);
		m_tmpText.m_textPositionXYZ1[0] = positionXYZ[0];
		m_tmpText.m_textPositionXYZ1[1] = positionXYZ[1];
		m_tmpText.m_textPositionXYZ1[2] = positionXYZ[2];

		m_tmpText.m_textOrientation[0] = orientation[0];
		m_tmpText.m_textOrientation[1] = orientation[1];
		m_tmpText.m_textOrientation[2] = orientation[2];
		m_tmpText.m_textOrientation[3] = orientation[3];

		m_tmpText.m_textColorRGB[0] = textColorRGB[0];
		m_tmpText.m_textColorRGB[1] = textColorRGB[1];
		m_tmpText.m_textColorRGB[2] = textColorRGB[2];

		m_tmpText.m_trackingVisualShapeIndex = trackingVisualShapeIndex;

		m_tmpText.m_optionFlags = optionFlags;
		m_tmpText.m_textOrientation[0] = orientation[0];
		m_tmpText.m_textOrientation[1] = orientation[1];
		m_tmpText.m_textOrientation[2] = orientation[2];
		m_tmpText.m_textOrientation[3] = orientation[3];

		m_cs->lock();
		setSharedParam(1, eGUIUserDebugAddText);
		m_resultUserDebugTextUid = -1;
		workerThreadWait();

		return m_resultUserDebugTextUid;
	}

	btAlignedObjectArray<UserDebugParameter*> m_userDebugParams;
	UserDebugParameter m_tmpParam;

	virtual int readUserDebugParameter(int itemUniqueId, double* value)
	{
		for (int i = 0; i < m_userDebugParams.size(); i++)
		{
			if (m_userDebugParams[i]->m_itemUniqueId == itemUniqueId)
			{
				*value = m_userDebugParams[i]->m_value;
				return 1;
			}
		}
		return 0;
	}
	int m_userDebugParamUid;

	virtual int addUserDebugParameter(const char* txt, double rangeMin, double rangeMax, double startValue)
	{
		strcpy(m_tmpParam.m_text, txt);
		m_tmpParam.m_rangeMin = rangeMin;
		m_tmpParam.m_rangeMax = rangeMax;
		m_tmpParam.m_value = startValue;
		m_tmpParam.m_itemUniqueId = m_uidGenerator++;

		m_cs->lock();
	  setSharedParam(1, eGUIUserDebugAddParameter);
		m_userDebugParamUid = -1;
		workerThreadWait();

		return m_userDebugParamUid;
	}

	btAlignedObjectArray<UserDebugDrawLine> m_userDebugLines;
	UserDebugDrawLine m_tmpLine;
	int m_resultDebugLineUid;

	virtual int addUserDebugLine(const double debugLineFromXYZ[3], const double debugLineToXYZ[3], const double debugLineColorRGB[3], double lineWidth, double lifeTime, int trackingVisualShapeIndex, int replaceItemUid)
	{
		m_tmpLine.m_lifeTime = lifeTime;
		m_tmpLine.m_lineWidth = lineWidth;

		m_tmpLine.m_itemUniqueId = replaceItemUid < 0 ? m_uidGenerator++ : replaceItemUid;
		m_tmpLine.m_debugLineFromXYZ[0] = debugLineFromXYZ[0];
		m_tmpLine.m_debugLineFromXYZ[1] = debugLineFromXYZ[1];
		m_tmpLine.m_debugLineFromXYZ[2] = debugLineFromXYZ[2];

		m_tmpLine.m_debugLineToXYZ[0] = debugLineToXYZ[0];
		m_tmpLine.m_debugLineToXYZ[1] = debugLineToXYZ[1];
		m_tmpLine.m_debugLineToXYZ[2] = debugLineToXYZ[2];

		m_tmpLine.m_debugLineColorRGB[0] = debugLineColorRGB[0];
		m_tmpLine.m_debugLineColorRGB[1] = debugLineColorRGB[1];
		m_tmpLine.m_debugLineColorRGB[2] = debugLineColorRGB[2];
		m_tmpLine.m_trackingVisualShapeIndex = trackingVisualShapeIndex;
		m_tmpLine.m_replaceItemUid = replaceItemUid;

		//don't block when replacing an item
		if (replaceItemUid>=0 && replaceItemUid<m_userDebugLines.size())
		{
			//find the right slot

			int slot=-1;
			for (int i=0;i<m_userDebugLines.size();i++)
			{
				if (replaceItemUid == m_userDebugLines[i].m_itemUniqueId)
				{
						slot = i;
				}
			}

			if (slot>=0)
			{
				m_userDebugLines[slot] = m_tmpLine;
			}
			m_resultDebugLineUid = replaceItemUid;
		}
		else
		{

			m_cs->lock();
			setSharedParam(1, eGUIUserDebugAddLine);
			m_resultDebugLineUid = -1;
			workerThreadWait();
		}
		return m_resultDebugLineUid;
	}

	int m_removeDebugItemUid;

	virtual void removeUserDebugItem(int debugItemUniqueId)
	{
		m_removeDebugItemUid = debugItemUniqueId;
		m_cs->lock();
		setSharedParam(1, eGUIUserDebugRemoveItem);
		workerThreadWait();
	}
	virtual void removeAllUserDebugItems()
	{
		m_cs->lock();
		setSharedParam(1, eGUIUserDebugRemoveAllItems);
		workerThreadWait();
	}

	virtual void removeAllUserParameters()
	{
		m_cs->lock();
		setSharedParam(1, eGUIUserDebugRemoveAllParameters);
		workerThreadWait();
	}

	


	const char* m_mp4FileName;
	virtual void dumpFramesToVideo(const char* mp4FileName)
	{
		m_cs->lock();
		m_mp4FileName = mp4FileName;
		setSharedParam(1, eGUIDumpFramesToVideo);
		workerThreadWait();
		m_mp4FileName = 0;
	}
};

class PhysicsServerExample : public SharedMemoryCommon
{
	PhysicsServerSharedMemory m_physicsServer;
	b3ThreadSupportInterface* m_threadSupport;
	MotionArgs m_args[MAX_MOTION_NUM_THREADS];
	MultiThreadedOpenGLGuiHelper* m_multiThreadedHelper;
	bool m_wantsShutdown;

	bool m_isConnected;
	btClock m_clock;
	bool m_replay;

	struct Common2dCanvasInterface* m_canvas;
	int m_canvasRGBIndex;
	int m_canvasDepthIndex;
	int m_canvasSegMaskIndex;

	//	int m_options;

#ifdef BT_ENABLE_VR
	TinyVRGui* m_tinyVrGui;
#endif

	int m_renderedFrames;

public:
	PhysicsServerExample(MultiThreadedOpenGLGuiHelper* helper, CommandProcessorCreationInterface* commandProcessorCreator, SharedMemoryInterface* sharedMem = 0, int options = 0);

	virtual ~PhysicsServerExample();

	virtual void initPhysics();

	virtual void stepSimulation(float deltaTime);

	virtual void updateGraphics();

	void enableCommandLogging()
	{
		m_physicsServer.enableCommandLogging(true, "BulletPhysicsCommandLog.bin");
	}

	void replayFromLogFile()
	{
		m_replay = true;
		m_physicsServer.replayFromLogFile("BulletPhysicsCommandLog.bin");
	}

	virtual void resetCamera()
	{
		float dist = 5;
		float pitch = -35;
		float yaw = 50;
		float targetPos[3] = {0, 0, 0};  //-3,2.8,-2.5};
		m_multiThreadedHelper->m_childGuiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}

	virtual bool wantsTermination();
	virtual bool isConnected();
	virtual void renderScene();
	void drawUserDebugLines();
	virtual void exitPhysics();

	virtual void physicsDebugDraw(int debugFlags);

	btVector3 getRayTo(int x, int y);

	virtual void vrControllerButtonCallback(int controllerId, int button, int state, float pos[4], float orn[4]);
	virtual void vrControllerMoveCallback(int controllerId, float pos[4], float orn[4], float analogAxis, float auxAnalogAxes[10]);
	virtual void vrHMDMoveCallback(int controllerId, float pos[4], float orientation[4]);
	virtual void vrGenericTrackerMoveCallback(int controllerId, float pos[4], float orientation[4]);

	virtual bool mouseMoveCallback(float x, float y)
	{
		if (m_replay)
			return false;

		CommonRenderInterface* renderer = m_multiThreadedHelper->m_childGuiHelper->getRenderInterface();  // m_guiHelper->getRenderInterface();

		if (!renderer)
		{
			return false;
		}

		b3MouseEvent event;
		event.m_buttonState = 0;
		event.m_buttonIndex = -1;
		event.m_mousePosX = x;
		event.m_mousePosY = y;
		event.m_eventType = MOUSE_MOVE_EVENT;
		m_args[0].m_csGUI->lock();
		m_args[0].m_allMouseEvents.push_back(event);
		m_args[0].m_csGUI->unlock();

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

	virtual bool mouseButtonCallback(int button, int state, float x, float y)
	{
		if (m_replay)
			return false;

		CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();

		if (!renderer)
		{
			return false;
		}

		CommonWindowInterface* window = m_guiHelper->getAppInterface()->m_window;

		b3MouseEvent event;
		event.m_buttonIndex = button;
		event.m_mousePosX = x;
		event.m_mousePosY = y;
		event.m_eventType = MOUSE_BUTTON_EVENT;
		if (state)
		{
			event.m_buttonState = eButtonIsDown + eButtonTriggered;
		}
		else
		{
			event.m_buttonState = eButtonReleased;
		}

		m_args[0].m_csGUI->lock();
		m_args[0].m_allMouseEvents.push_back(event);
		m_args[0].m_csGUI->unlock();

		if (state == 1)
		{
			if (button == 0 && (!window->isModifierKeyPressed(B3G_ALT) && !window->isModifierKeyPressed(B3G_CONTROL)))
			{
				btVector3 camPos;
				renderer->getActiveCamera()->getCameraPosition(camPos);

				btVector3 rayFrom = camPos;
				btVector3 rayTo = getRayTo(int(x), int(y));

				MyMouseCommand cmd;
				cmd.m_rayFrom = rayFrom;
				cmd.m_rayTo = rayTo;
				cmd.m_type = MyMouseButtonDown;

				m_args[0].m_csGUI->lock();
				m_args[0].m_mouseCommands.push_back(cmd);
				m_args[0].m_csGUI->unlock();
			}
		}
		else
		{
			if (button == 0)
			{
				//m_physicsServer.removePickingConstraint();
				MyMouseCommand cmd;
				cmd.m_rayFrom.setValue(0, 0, 0);
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
	virtual bool keyboardCallback(int key, int state)
	{
		//printf("key=%d, state=%d\n", key,state);
		{
			m_args[0].m_csGUI->lock();
			int keyIndex = -1;
			//is already there?
			for (int i = 0; i < m_args[0].m_keyboardEvents.size(); i++)
			{
				if (m_args[0].m_keyboardEvents[i].m_keyCode == key)
				{
					keyIndex = i;
					break;
				}
			}

			if (state)
			{
				b3KeyboardEvent ev;
				ev.m_keyCode = key;
				ev.m_keyState = eButtonIsDown + eButtonTriggered;
				
				if (keyIndex >= 0)
				{
					if (0 == (m_args[0].m_keyboardEvents[keyIndex].m_keyState & eButtonIsDown))
					{
						m_args[0].m_keyboardEvents[keyIndex] = ev;
					}
				}
				else
				{
					m_args[0].m_keyboardEvents.push_back(ev);
				}
				
			}
			else
			{
				b3KeyboardEvent ev;
				ev.m_keyCode = key;
				ev.m_keyState = eButtonReleased;
				if (keyIndex >= 0)
				{
					m_args[0].m_keyboardEvents[keyIndex] = ev;
				}
				else
				{
					m_args[0].m_keyboardEvents.push_back(ev);
				}
				
			}
			m_args[0].m_csGUI->unlock();
		}
		/*printf("m_args[0].m_keyboardEvents.size()=%d\n", m_args[0].m_keyboardEvents.size());
		for (int i=0;i<m_args[0].m_keyboardEvents.size();i++)
		{
			printf("key[%d]=%d state = %d\n",i,m_args[0].m_keyboardEvents[i].m_keyCode,m_args[0].m_keyboardEvents[i].m_keyState);
		}
		*/
		double shift = 0.1;

		CommonWindowInterface* window = m_guiHelper->getAppInterface()->m_window;
		if (window->isModifierKeyPressed(B3G_SHIFT))
			shift = 0.01;

		btVector3 VRTeleportPos = this->m_physicsServer.getVRTeleportPosition();

		if (gEnableDefaultKeyboardShortcuts)
		{
			if (m_guiHelper->getAppInterface()->m_renderer->getActiveCamera()->isVRCamera())
			{
				if (key == 'w' && state)
				{
					VRTeleportPos[0] += shift;
					m_physicsServer.setVRTeleportPosition(VRTeleportPos);
					saveCurrentSettingsVR(VRTeleportPos);
				}
				if (key == 's' && state)
				{
					VRTeleportPos[0] -= shift;
					m_physicsServer.setVRTeleportPosition(VRTeleportPos);
					saveCurrentSettingsVR(VRTeleportPos);
				}
				if (key == 'a' && state)
				{
					VRTeleportPos[1] -= shift;
					m_physicsServer.setVRTeleportPosition(VRTeleportPos);
					saveCurrentSettingsVR(VRTeleportPos);
				}
				if (key == 'd' && state)
				{
					VRTeleportPos[1] += shift;
					m_physicsServer.setVRTeleportPosition(VRTeleportPos);
					saveCurrentSettingsVR(VRTeleportPos);
				}
				if (key == 'q' && state)
				{
					VRTeleportPos[2] += shift;
					m_physicsServer.setVRTeleportPosition(VRTeleportPos);
					saveCurrentSettingsVR(VRTeleportPos);
				}
				if (key == 'e' && state)
				{
					VRTeleportPos[2] -= shift;
					m_physicsServer.setVRTeleportPosition(VRTeleportPos);
					saveCurrentSettingsVR(VRTeleportPos);
				}
				if (key == 'z' && state)
				{
					gVRTeleportRotZ += shift;
					btQuaternion VRTeleportOrn = btQuaternion(btVector3(0, 0, 1), gVRTeleportRotZ);
					m_physicsServer.setVRTeleportOrientation(VRTeleportOrn);
					saveCurrentSettingsVR(VRTeleportPos);
				}
			}
		}

		return false;
	}

	virtual void setSharedMemoryKey(int key)
	{
		m_physicsServer.setSharedMemoryKey(key);
	}

	virtual void processCommandLineArgs(int argc, char* argv[])
	{
		b3CommandLineArgs args(argc, argv);
		loadCurrentSettingsVR(args);
		int shmemKey;

		if (args.GetCmdLineArgument("sharedMemoryKey", shmemKey))
		{
			setSharedMemoryKey(shmemKey);
		}

		btVector3 vrTeleportPos = m_physicsServer.getVRTeleportPosition();

		if (args.GetCmdLineArgument("camPosX", vrTeleportPos[0]))
		{
			printf("camPosX=%f\n", vrTeleportPos[0]);
		}

		if (args.GetCmdLineArgument("camPosY", vrTeleportPos[1]))
		{
			printf("camPosY=%f\n", vrTeleportPos[1]);
		}

		if (args.GetCmdLineArgument("camPosZ", vrTeleportPos[2]))
		{
			printf("camPosZ=%f\n", vrTeleportPos[2]);
		}

		m_physicsServer.setVRTeleportPosition(vrTeleportPos);

		float camRotZ = 0.f;
		if (args.GetCmdLineArgument("camRotZ", camRotZ))
		{
			printf("camRotZ = %f\n", camRotZ);
			btQuaternion ornZ(btVector3(0, 0, 1), camRotZ);
			m_physicsServer.setVRTeleportOrientation(ornZ);
		}

		if (args.CheckCmdLineFlag("realtimesimulation"))
		{
			//gEnableRealTimeSimVR = true;
			m_physicsServer.enableRealTimeSimulation(true);
		}

		if (args.CheckCmdLineFlag("disableDefaultKeyboardShortcuts"))
		{
			gEnableDefaultKeyboardShortcuts = false;
		}
		if (args.CheckCmdLineFlag("enableDefaultKeyboardShortcuts"))
		{
			gEnableDefaultKeyboardShortcuts = true;
		}
		if (args.CheckCmdLineFlag("disableDefaultMousePicking"))
		{
			gEnableDefaultMousePicking = false;
		}
		if (args.CheckCmdLineFlag("enableDefaultMousePicking"))
		{
			gEnableDefaultMousePicking = true;
		}
	}
};

PhysicsServerExample::PhysicsServerExample(MultiThreadedOpenGLGuiHelper* helper, CommandProcessorCreationInterface* commandProcessorCreator, SharedMemoryInterface* sharedMem, int options)
	: SharedMemoryCommon(helper),
	  m_physicsServer(commandProcessorCreator, sharedMem, 0),
	  m_wantsShutdown(false),
	  m_isConnected(false),
	  m_replay(false),
	  m_canvas(0),
	  m_canvasRGBIndex(-1),
	  m_canvasDepthIndex(-1),
	  m_canvasSegMaskIndex(-1)
//m_options(options)
#ifdef BT_ENABLE_VR
	  ,
	  m_tinyVrGui(0)
#endif
	  ,
	  m_renderedFrames(0)
{
	m_multiThreadedHelper = helper;
	//	b3Printf("Started PhysicsServer\n");
}

PhysicsServerExample::~PhysicsServerExample()
{
	if (m_canvas)
	{
		if (m_canvasRGBIndex >= 0)
			m_canvas->destroyCanvas(m_canvasRGBIndex);
		if (m_canvasDepthIndex >= 0)
			m_canvas->destroyCanvas(m_canvasDepthIndex);
		if (m_canvasSegMaskIndex >= 0)
			m_canvas->destroyCanvas(m_canvasSegMaskIndex);
	}

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

void PhysicsServerExample::initPhysics()
{
	///for this testing we use Z-axis up
	int upAxis = 2;
	m_guiHelper->setUpAxis(upAxis);

	m_threadSupport = createMotionThreadSupport(MAX_MOTION_NUM_THREADS);

	m_isConnected = m_physicsServer.connectSharedMemory(m_guiHelper);

	for (int i = 0; i < m_threadSupport->getNumTasks(); i++)
	{
		MotionThreadLocalStorage* storage = (MotionThreadLocalStorage*)m_threadSupport->getThreadLocalMemory(i);
		b3Assert(storage);
		storage->threadId = i;
		//storage->m_sharedMem = data->m_sharedMem;
	}

	for (int w = 0; w < MAX_MOTION_NUM_THREADS; w++)
	{
		m_args[w].m_cs = m_threadSupport->createCriticalSection();
		m_args[w].m_cs2 = m_threadSupport->createCriticalSection();
		m_args[w].m_cs3 = m_threadSupport->createCriticalSection();
		m_args[w].m_csGUI = m_threadSupport->createCriticalSection();
		m_multiThreadedHelper->setCriticalSection(m_args[w].m_cs);
		m_multiThreadedHelper->setCriticalSection2(m_args[w].m_cs2);
		m_multiThreadedHelper->setCriticalSection3(m_args[w].m_cs3);
		m_multiThreadedHelper->setCriticalSectionGUI(m_args[w].m_csGUI);

		m_args[w].m_cs->lock();
		m_args[w].m_cs->setSharedParam(0, eMotionIsUnInitialized);
		m_args[w].m_cs->unlock();
		int numMoving = 0;
		m_args[w].m_positions.resize(numMoving);
		m_args[w].m_physicsServerPtr = &m_physicsServer;
		//int index = 0;

		m_threadSupport->runTask(B3_THREAD_SCHEDULE_TASK, (void*)&this->m_args[w], w);
		bool isUninitialized = true;

		while (isUninitialized)
		{
			m_args[w].m_cs->lock();
			isUninitialized = (m_args[w].m_cs->getSharedParam(0) == eMotionIsUnInitialized);
			m_args[w].m_cs->unlock();
#ifdef _WIN32
			b3Clock::usleep(1000);
#endif
		}
	}
	m_args[0].m_cs->lock();
  m_args[0].m_csGUI->lock();
	m_args[0].m_cs->setSharedParam(1, eGUIHelperIdle);
  m_args[0].m_csGUI->unlock();
  m_args[0].m_cs->unlock();
	m_args[0].m_cs2->lock();

	{
		m_canvas = m_guiHelper->get2dCanvasInterface();
		if (m_canvas)
		{
			m_canvasRGBIndex = m_canvas->createCanvas("Synthetic Camera RGB data", gCamVisualizerWidth, gCamVisualizerHeight, 8, 55);
			m_canvasDepthIndex = m_canvas->createCanvas("Synthetic Camera Depth data", gCamVisualizerWidth, gCamVisualizerHeight, 8, 75 + gCamVisualizerHeight);
			m_canvasSegMaskIndex = m_canvas->createCanvas("Synthetic Camera Segmentation Mask", gCamVisualizerWidth, gCamVisualizerHeight, 8, 95 + gCamVisualizerHeight * 2);

			for (int i = 0; i < gCamVisualizerWidth; i++)
			{
				for (int j = 0; j < gCamVisualizerHeight; j++)
				{
					unsigned char red = 255;
					unsigned char green = 255;
					unsigned char blue = 255;
					unsigned char alpha = 255;
					if (i == j)
					{
						red = 0;
						green = 0;
						blue = 0;
					}
					m_canvas->setPixel(m_canvasRGBIndex, i, j, red, green, blue, alpha);
					if (m_canvasSegMaskIndex >= 0)
						m_canvas->setPixel(m_canvasDepthIndex, i, j, red, green, blue, alpha);
					if (m_canvasSegMaskIndex >= 0)
						m_canvas->setPixel(m_canvasSegMaskIndex, i, j, red, green, blue, alpha);
				}
			}
			m_canvas->refreshImageData(m_canvasRGBIndex);

			if (m_canvasDepthIndex >= 0)
				m_canvas->refreshImageData(m_canvasDepthIndex);
			if (m_canvasSegMaskIndex >= 0)
				m_canvas->refreshImageData(m_canvasSegMaskIndex);
		}
	}
}

void PhysicsServerExample::exitPhysics()
{
	for (int i = 0; i < MAX_MOTION_NUM_THREADS; i++)
	{
		m_args[i].m_cs2->unlock();
		m_args[i].m_cs->lock();
		m_args[i].m_cs->setSharedParam(0, eRequestTerminateMotion);
		m_args[i].m_cs->unlock();
	}
	int numActiveThreads = MAX_MOTION_NUM_THREADS;

	while (numActiveThreads)
	{
		int arg0, arg1;
		if (m_threadSupport->isTaskCompleted(&arg0, &arg1, 0))
		{
			numActiveThreads--;
			printf("numActiveThreads = %d\n", numActiveThreads);
		}
		else
		{
			b3Clock::usleep(0);
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

void PhysicsServerExample::updateGraphics()
{
	//check if any graphics related tasks are requested

#ifdef SYNC_CAMERA_USING_GUI_CS
	m_multiThreadedHelper->getCriticalSectionGUI()->lock();
	if (m_multiThreadedHelper->m_cameraUpdated)
	{
		m_multiThreadedHelper->m_cameraUpdated = false;
		m_multiThreadedHelper->m_childGuiHelper->resetCamera(
			m_multiThreadedHelper->m_resetCameraCamDist,
			m_multiThreadedHelper->m_resetCameraYaw,
			m_multiThreadedHelper->m_resetCameraPitch,
			m_multiThreadedHelper->m_resetCameraCamPosX,
			m_multiThreadedHelper->m_resetCameraCamPosY,
			m_multiThreadedHelper->m_resetCameraCamPosZ);
	}
	m_multiThreadedHelper->getCriticalSectionGUI()->unlock();
#endif

	m_multiThreadedHelper->getCriticalSectionGUI()->lock();
	unsigned int cachedSharedParam = m_multiThreadedHelper->getCriticalSection()->getSharedParam(1);
	m_multiThreadedHelper->getCriticalSectionGUI()->unlock();

	switch (cachedSharedParam)
	{
		case eGUIHelperCreateCollisionShapeGraphicsObject:
		{
			B3_PROFILE("eGUIHelperCreateCollisionShapeGraphicsObject");
			m_multiThreadedHelper->m_childGuiHelper->createCollisionShapeGraphicsObject(m_multiThreadedHelper->m_colShape);
			m_multiThreadedHelper->mainThreadRelease();
			break;
		}
		case eGUIHelperCreateCollisionObjectGraphicsObject:
		{
			B3_PROFILE("eGUIHelperCreateCollisionObjectGraphicsObject");

			m_multiThreadedHelper->m_childGuiHelper->createCollisionObjectGraphicsObject(m_multiThreadedHelper->m_obj,
																						 m_multiThreadedHelper->m_color2);
			m_multiThreadedHelper->mainThreadRelease();

			break;
		}
		case eGUIHelperCreateRigidBodyGraphicsObject:
		{
			B3_PROFILE("eGUIHelperCreateRigidBodyGraphicsObject");
			m_multiThreadedHelper->m_childGuiHelper->createRigidBodyGraphicsObject(m_multiThreadedHelper->m_body, m_multiThreadedHelper->m_color3);
			m_multiThreadedHelper->mainThreadRelease();
			break;
		}
		case eGUIHelperRegisterTexture:
		{
			B3_PROFILE("eGUIHelperRegisterTexture");
			m_multiThreadedHelper->m_textureId = m_multiThreadedHelper->m_childGuiHelper->registerTexture(m_multiThreadedHelper->m_texels,
																										  m_multiThreadedHelper->m_textureWidth, m_multiThreadedHelper->m_textureHeight);
			m_multiThreadedHelper->mainThreadRelease();
			break;
		}
		case eGUIHelperRemoveTexture:
		{
			B3_PROFILE("eGUIHelperRemoveTexture");
			m_multiThreadedHelper->m_childGuiHelper->removeTexture(m_multiThreadedHelper->m_removeTextureUid);
			m_multiThreadedHelper->mainThreadRelease();
			break;
		}

		case eGUIHelperUpdateShape:
		{
			B3_PROFILE("eGUIHelperUpdateShape");
			m_multiThreadedHelper->m_childGuiHelper->updateShape(m_multiThreadedHelper->m_updateShapeIndex, m_multiThreadedHelper->m_updateShapeVertices, m_multiThreadedHelper->m_updateNumShapeVertices);
			m_multiThreadedHelper->mainThreadRelease();
			break;
		}

		case eGUIHelperRegisterGraphicsShape:
		{
			B3_PROFILE("eGUIHelperRegisterGraphicsShape");
			int shapeIndex = m_multiThreadedHelper->m_childGuiHelper->registerGraphicsShape(
				m_multiThreadedHelper->m_vertices,
				m_multiThreadedHelper->m_numvertices,
				m_multiThreadedHelper->m_indices,
				m_multiThreadedHelper->m_numIndices,
				m_multiThreadedHelper->m_primitiveType,
				m_multiThreadedHelper->m_textureId);

       m_multiThreadedHelper->getCriticalSectionGUI()->lock();
       m_multiThreadedHelper->m_shapeIndex = shapeIndex;
       m_multiThreadedHelper->getCriticalSectionGUI()->unlock();


      m_multiThreadedHelper->mainThreadRelease();


			break;
		}

		case eGUIHelperSetVisualizerFlag:
		{
			B3_PROFILE("eGUIHelperSetVisualizerFlag");
			int flag = m_multiThreadedHelper->m_visualizerFlag;
			int enable = m_multiThreadedHelper->m_visualizerEnable;

			if (flag == COV_ENABLE_RGB_BUFFER_PREVIEW)
			{
				if (enable)
				{
					if (m_canvasRGBIndex < 0)
					{
						m_canvasRGBIndex = m_canvas->createCanvas("Synthetic Camera RGB data", gCamVisualizerWidth, gCamVisualizerHeight, 8, 55);
					}
				}
				else
				{
					if (m_canvasRGBIndex >= 0)
					{
						m_canvas->destroyCanvas(m_canvasRGBIndex);
						m_canvasRGBIndex = -1;
					}
				}
			}

			if (flag == COV_ENABLE_DEPTH_BUFFER_PREVIEW)
			{
				if (enable)
				{
					if (m_canvasDepthIndex < 0)
					{
						m_canvasDepthIndex = m_canvas->createCanvas("Synthetic Camera Depth data", gCamVisualizerWidth, gCamVisualizerHeight, 8, 75 + gCamVisualizerHeight);
					}
				}
				else
				{
					if (m_canvasDepthIndex >= 0)
					{
						m_canvas->destroyCanvas(m_canvasDepthIndex);
						m_canvasDepthIndex = -1;
					}
				}
			}

			if (flag == COV_ENABLE_SEGMENTATION_MARK_PREVIEW)
			{
				if (enable)
				{
					if (m_canvasSegMaskIndex < 0)
					{
						m_canvasSegMaskIndex = m_canvas->createCanvas("Synthetic Camera Segmentation Mask", gCamVisualizerWidth, gCamVisualizerHeight, 8, 95 + gCamVisualizerHeight * 2);
					}
				}
				else
				{
					if (m_canvasSegMaskIndex >= 0)
					{
						m_canvas->destroyCanvas(m_canvasSegMaskIndex);
						m_canvasSegMaskIndex = -1;
					}
				}
			}

			if (flag == COV_ENABLE_VR_TELEPORTING)
			{
				gEnableTeleporting = (enable != 0);
			}

			if (flag == COV_ENABLE_VR_PICKING)
			{
				gEnablePicking = (enable != 0);
			}

			if (flag == COV_ENABLE_SYNC_RENDERING_INTERNAL)
			{
				gEnableSyncPhysicsRendering = (enable != 0);
			}

			if (flag == COV_ENABLE_RENDERING)
			{
				gEnableRendering = (enable != 0);
			}

			if (flag == COV_ENABLE_KEYBOARD_SHORTCUTS)
			{
				gEnableDefaultKeyboardShortcuts = (enable != 0);
			}

			if (flag == COV_ENABLE_MOUSE_PICKING)
			{
				gEnableDefaultMousePicking = (enable != 0);
			}

			m_multiThreadedHelper->m_renderedFrames = m_renderedFrames;

			m_multiThreadedHelper->m_childGuiHelper->setVisualizerFlag(m_multiThreadedHelper->m_visualizerFlag, m_multiThreadedHelper->m_visualizerEnable);

			//postpone the release until an actual frame is rendered, unless it is a remote visualizer
			if ((!m_multiThreadedHelper->m_childGuiHelper->isRemoteVisualizer()) && flag == COV_ENABLE_SINGLE_STEP_RENDERING)
			{
				m_multiThreadedHelper->getCriticalSection()->setSharedParam(1, eGUIHelperSetVisualizerFlagCheckRenderedFrame);
			}
			else
			{
				m_multiThreadedHelper->mainThreadRelease();
			}
			break;
		}


		case eGUIHelperSetVisualizerFlagCheckRenderedFrame:
		{
			if (m_renderedFrames != m_multiThreadedHelper->m_renderedFrames)
			{
				m_multiThreadedHelper->mainThreadRelease();
			}
			break;
		}

		case eGUIHelperRegisterGraphicsInstance:
		{
			B3_PROFILE("eGUIHelperRegisterGraphicsInstance");
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
			B3_PROFILE("eGUIHelperRemoveAllGraphicsInstances");
#ifdef BT_ENABLE_VR
			if (m_tinyVrGui)
			{
				delete m_tinyVrGui;
				m_tinyVrGui = 0;
			}
#endif  //BT_ENABLE_VR
			m_multiThreadedHelper->m_childGuiHelper->removeAllGraphicsInstances();
			if (m_multiThreadedHelper->m_childGuiHelper->getRenderInterface())
			{
				int numRenderInstances;
				numRenderInstances = m_multiThreadedHelper->m_childGuiHelper->getRenderInterface()->getTotalNumInstances();
				b3Assert(numRenderInstances == 0);
			}
			m_multiThreadedHelper->mainThreadRelease();

			break;
		}
		case eGUIHelperRemoveGraphicsInstance:
		{
			B3_PROFILE("eGUIHelperRemoveGraphicsInstance");
			m_multiThreadedHelper->m_childGuiHelper->removeGraphicsInstance(m_multiThreadedHelper->m_graphicsInstanceRemove);
			m_multiThreadedHelper->mainThreadRelease();
			break;
		}

		case eGUIHelperGetShapeIndexFromInstance:
		{
			B3_PROFILE("eGUIHelperGetShapeIndexFromInstance");
			m_multiThreadedHelper->getShapeIndex_shapeIndex = m_multiThreadedHelper->m_childGuiHelper->getShapeIndexFromInstance(m_multiThreadedHelper->m_getShapeIndex_instance);
			m_multiThreadedHelper->mainThreadRelease();
			break;
		}

		case eGUIHelperChangeGraphicsInstanceTextureId:
		{
			B3_PROFILE("eGUIHelperChangeGraphicsInstanceTextureId");

			m_multiThreadedHelper->m_childGuiHelper->replaceTexture(
				m_multiThreadedHelper->m_graphicsInstanceChangeTextureShapeIndex,
				m_multiThreadedHelper->m_graphicsInstanceChangeTextureId);
			m_multiThreadedHelper->mainThreadRelease();
			break;
		}

		case eGUIHelperChangeTexture:
		{
			B3_PROFILE("eGUIHelperChangeTexture");

			m_multiThreadedHelper->m_childGuiHelper->changeTexture(
				m_multiThreadedHelper->m_changeTextureUniqueId,
				m_multiThreadedHelper->m_changeTextureRgbTexels,
				m_multiThreadedHelper->m_changeTextureWidth,
				m_multiThreadedHelper->m_changeTextureHeight);
			m_multiThreadedHelper->mainThreadRelease();
			break;
		}

		case eGUIHelperChangeGraphicsInstanceRGBAColor:
		{
			B3_PROFILE("eGUIHelperChangeGraphicsInstanceRGBAColor");

			m_multiThreadedHelper->m_childGuiHelper->changeRGBAColor(m_multiThreadedHelper->m_graphicsInstanceChangeColor, m_multiThreadedHelper->m_rgbaColor);
			m_multiThreadedHelper->mainThreadRelease();
			break;
		}

		case eGUIHelperChangeGraphicsInstanceFlags:
		{
			m_multiThreadedHelper->m_childGuiHelper->changeInstanceFlags(m_multiThreadedHelper->m_graphicsInstanceFlagsInstanceUid, m_multiThreadedHelper->m_graphicsInstanceFlags);
			m_multiThreadedHelper->mainThreadRelease();
			break;
		}

		case eGUIHelperSetRgbBackground:
		{
			m_multiThreadedHelper->m_childGuiHelper->setBackgroundColor(m_multiThreadedHelper->m_rgbBackground);
			m_multiThreadedHelper->mainThreadRelease();
			break;
		}

		case eGUIHelperChangeGraphicsInstanceScaling:
		{
			B3_PROFILE("eGUIHelperChangeGraphicsInstanceScaling");

			m_multiThreadedHelper->m_childGuiHelper->changeScaling(m_multiThreadedHelper->m_graphicsInstanceChangeScaling, m_multiThreadedHelper->m_baseScaling);
			m_multiThreadedHelper->mainThreadRelease();
			break;
		}

		case eGUIHelperChangeGraphicsInstanceSpecularColor:
		{
			B3_PROFILE("eGUIHelperChangeGraphicsInstanceSpecularColor");

			m_multiThreadedHelper->m_childGuiHelper->changeSpecularColor(m_multiThreadedHelper->m_graphicsInstanceChangeSpecular, m_multiThreadedHelper->m_specularColor);
			m_multiThreadedHelper->mainThreadRelease();
			break;
		}
		case eGUIHelperDisplayCameraImageData:
		{
			B3_PROFILE("eGUIHelperDisplayCameraImageData");

			if (m_canvas)
			{
				int numBytesPerPixel = 4;
				int startRGBIndex = m_multiThreadedHelper->m_startPixelIndex * numBytesPerPixel;
				int endRGBIndex = startRGBIndex + (*m_multiThreadedHelper->m_numPixelsCopied * numBytesPerPixel);

				int startDepthIndex = m_multiThreadedHelper->m_startPixelIndex;
				int endDepthIndex = startDepthIndex + (*m_multiThreadedHelper->m_numPixelsCopied);

				int startSegIndex = m_multiThreadedHelper->m_startPixelIndex;
				int endSegIndex = startSegIndex + (*m_multiThreadedHelper->m_numPixelsCopied);

				//btScalar frustumZNear = m_multiThreadedHelper->m_projectionMatrix[14]/(m_multiThreadedHelper->m_projectionMatrix[10]-1);
				//btScalar frustumZFar = m_multiThreadedHelper->m_projectionMatrix[14]/(m_multiThreadedHelper->m_projectionMatrix[10]+1);

				for (int i = 0; i < gCamVisualizerWidth; i++)
				{
					for (int j = 0; j < gCamVisualizerHeight; j++)
					{
						int xIndex = int(float(i) * (float(m_multiThreadedHelper->m_destinationWidth) / float(gCamVisualizerWidth)));
						int yIndex = int(float(j) * (float(m_multiThreadedHelper->m_destinationHeight) / float(gCamVisualizerHeight)));
						btClamp(xIndex, 0, m_multiThreadedHelper->m_destinationWidth);
						btClamp(yIndex, 0, m_multiThreadedHelper->m_destinationHeight);
						int bytesPerPixel = 4;  //RGBA

						if (m_canvasRGBIndex >= 0)
						{
							int rgbPixelIndex = (xIndex + yIndex * m_multiThreadedHelper->m_destinationWidth) * bytesPerPixel;
							if (rgbPixelIndex >= startRGBIndex && rgbPixelIndex < endRGBIndex)
							{
								m_canvas->setPixel(m_canvasRGBIndex, i, j,
												   m_multiThreadedHelper->m_pixelsRGBA[rgbPixelIndex - startRGBIndex],
												   m_multiThreadedHelper->m_pixelsRGBA[rgbPixelIndex + 1 - startRGBIndex],
												   m_multiThreadedHelper->m_pixelsRGBA[rgbPixelIndex + 2 - startRGBIndex],
												   255);  //alpha set to 255
							}
						}
						if (m_canvasDepthIndex >= 0 && 0 != m_multiThreadedHelper->m_depthBuffer)
						{
							int depthPixelIndex = (xIndex + yIndex * m_multiThreadedHelper->m_destinationWidth);
							if (depthPixelIndex >= startDepthIndex && depthPixelIndex < endDepthIndex)
							{
								float depthValue = m_multiThreadedHelper->m_depthBuffer[depthPixelIndex - startDepthIndex];
								//todo: rescale the depthValue to [0..255]
								if (depthValue > -1e20)
								{
									int rgb = 0;
									btScalar frustumZNear = 0.1;
									btScalar frustumZFar = 30;
									btScalar minDepthValue = frustumZNear;  //todo: compute more reasonably min/max depth range
									btScalar maxDepthValue = frustumZFar;

									float depth = depthValue;
									btScalar linearDepth = 255. * (2.0 * frustumZNear) / (frustumZFar + frustumZNear - depth * (frustumZFar - frustumZNear));
									btClamp(linearDepth, btScalar(0), btScalar(255));
									rgb = linearDepth;

									m_canvas->setPixel(m_canvasDepthIndex, i, j,
													   rgb,
													   rgb,
													   255, 255);  //alpha set to 255
								}
								else
								{
									m_canvas->setPixel(m_canvasDepthIndex, i, j,
													   0,
													   0,
													   0, 255);  //alpha set to 255
								}
							}
						}

						if (m_canvasSegMaskIndex >= 0 && (0 != m_multiThreadedHelper->m_segmentationMaskBuffer))
						{
							int segmentationMaskPixelIndex = (xIndex + yIndex * m_multiThreadedHelper->m_destinationWidth);

							if (segmentationMaskPixelIndex >= startSegIndex && segmentationMaskPixelIndex < endSegIndex)
							{
								int segmentationMask = m_multiThreadedHelper->m_segmentationMaskBuffer[segmentationMaskPixelIndex - startSegIndex];
								btVector4 palette[4] = {btVector4(32, 255, 32, 255),
														btVector4(32, 32, 255, 255),
														btVector4(255, 255, 32, 255),
														btVector4(32, 255, 255, 255)};
								if (segmentationMask >= 0)
								{
									int obIndex = segmentationMask & ((1 << 24) - 1);
									int linkIndex = (segmentationMask >> 24) - 1;

									btVector4 rgb = palette[(obIndex + linkIndex) & 3];

									m_canvas->setPixel(m_canvasSegMaskIndex, i, j,
													   rgb.x(),
													   rgb.y(),
													   rgb.z(), 255);  //alpha set to 255
								}
								else
								{
									m_canvas->setPixel(m_canvasSegMaskIndex, i, j,
													   0,
													   0,
													   0, 255);  //alpha set to 255
								}
							}
						}
					}
				}

				if (m_canvasRGBIndex >= 0)
					m_canvas->refreshImageData(m_canvasRGBIndex);

				if (m_canvasDepthIndex >= 0)
					m_canvas->refreshImageData(m_canvasDepthIndex);
				if (m_canvasSegMaskIndex >= 0)
					m_canvas->refreshImageData(m_canvasSegMaskIndex);
			}
			m_multiThreadedHelper->mainThreadRelease();
			break;
		}
		case eGUIHelperCopyCameraImageData:
		{
			B3_PROFILE("eGUIHelperCopyCameraImageData");

			if (m_multiThreadedHelper->m_startPixelIndex == 0)
			{
				m_physicsServer.syncPhysicsToGraphics();
			}

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
		case eGUIHelperResetCamera:
		{
			m_multiThreadedHelper->m_childGuiHelper->resetCamera(
				m_multiThreadedHelper->m_resetCameraCamDist,
				m_multiThreadedHelper->m_resetCameraYaw,
				m_multiThreadedHelper->m_resetCameraPitch,
				m_multiThreadedHelper->m_resetCameraCamPosX,
				m_multiThreadedHelper->m_resetCameraCamPosY,
				m_multiThreadedHelper->m_resetCameraCamPosZ);
			m_multiThreadedHelper->mainThreadRelease();
			break;
		}

		case eGUIHelperAutogenerateGraphicsObjects:
		{
			B3_PROFILE("eGUIHelperAutogenerateGraphicsObjects");

			m_multiThreadedHelper->m_childGuiHelper->autogenerateGraphicsObjects(m_multiThreadedHelper->m_dynamicsWorld);
			m_multiThreadedHelper->mainThreadRelease();
			break;
		}

		case eGUIUserDebugAddText:
		{
			B3_PROFILE("eGUIUserDebugAddText");

			bool replaced = false;

			for (int i = 0; i < m_multiThreadedHelper->m_userDebugText.size(); i++)
			{
				if (m_multiThreadedHelper->m_userDebugText[i].m_itemUniqueId == m_multiThreadedHelper->m_tmpText.m_itemUniqueId)
				{
					m_multiThreadedHelper->m_userDebugText[i] = m_multiThreadedHelper->m_tmpText;
					m_multiThreadedHelper->m_resultUserDebugTextUid = m_multiThreadedHelper->m_tmpText.m_itemUniqueId;
					replaced = true;
				}
			}

			if (!replaced)
			{
				m_multiThreadedHelper->m_userDebugText.push_back(m_multiThreadedHelper->m_tmpText);
				m_multiThreadedHelper->m_resultUserDebugTextUid = m_multiThreadedHelper->m_userDebugText[m_multiThreadedHelper->m_userDebugText.size() - 1].m_itemUniqueId;
			}
			m_multiThreadedHelper->mainThreadRelease();

			break;
		}
		case eGUIUserDebugAddParameter:
		{
			B3_PROFILE("eGUIUserDebugAddParameter");

			UserDebugParameter* param = new UserDebugParameter(m_multiThreadedHelper->m_tmpParam);
			m_multiThreadedHelper->m_userDebugParams.push_back(param);

			if (param->m_rangeMin<= param->m_rangeMax)
			{
				SliderParams slider(param->m_text, &param->m_value);
				slider.m_minVal = param->m_rangeMin;
				slider.m_maxVal = param->m_rangeMax;

				if (m_multiThreadedHelper->m_childGuiHelper->getParameterInterface())
					m_multiThreadedHelper->m_childGuiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
			}
			else
			{
				int buttonId = -1;
				bool isTrigger = false;
				ButtonParams button(param->m_text, buttonId, isTrigger);
				button.m_callback = UserButtonToggle;
				button.m_userPointer = param;
				button.m_initialState = false;
				
				//create a button
				if (m_multiThreadedHelper->m_childGuiHelper->getParameterInterface())
					m_multiThreadedHelper->m_childGuiHelper->getParameterInterface()->registerButtonParameter(button);
			}

			m_multiThreadedHelper->m_userDebugParamUid = (*m_multiThreadedHelper->m_userDebugParams[m_multiThreadedHelper->m_userDebugParams.size() - 1]).m_itemUniqueId;

			//also add actual menu
			m_multiThreadedHelper->mainThreadRelease();
			break;
		}
		case eGUIUserDebugAddLine:
		{
			B3_PROFILE("eGUIUserDebugAddLine");

			if (m_multiThreadedHelper->m_tmpLine.m_replaceItemUid >= 0)
			{
				for (int i = 0; i < m_multiThreadedHelper->m_userDebugLines.size(); i++)
				{
					if (m_multiThreadedHelper->m_userDebugLines[i].m_itemUniqueId == m_multiThreadedHelper->m_tmpLine.m_replaceItemUid)
					{
						m_multiThreadedHelper->m_userDebugLines[i] = m_multiThreadedHelper->m_tmpLine;
						m_multiThreadedHelper->m_resultDebugLineUid = m_multiThreadedHelper->m_tmpLine.m_replaceItemUid;
					}
				}
			}
			else
			{
				m_multiThreadedHelper->m_userDebugLines.push_back(m_multiThreadedHelper->m_tmpLine);
				m_multiThreadedHelper->m_resultDebugLineUid = m_multiThreadedHelper->m_userDebugLines[m_multiThreadedHelper->m_userDebugLines.size() - 1].m_itemUniqueId;
			}
			m_multiThreadedHelper->mainThreadRelease();
			break;
		}
		case eGUIUserDebugRemoveItem:
		{
			B3_PROFILE("eGUIUserDebugRemoveItem");

			for (int i = 0; i < m_multiThreadedHelper->m_userDebugLines.size(); i++)
			{
				if (m_multiThreadedHelper->m_userDebugLines[i].m_itemUniqueId == m_multiThreadedHelper->m_removeDebugItemUid)
				{
					m_multiThreadedHelper->m_userDebugLines.swap(i, m_multiThreadedHelper->m_userDebugLines.size() - 1);
					m_multiThreadedHelper->m_userDebugLines.pop_back();
					break;
				}
			}

			for (int i = 0; i < m_multiThreadedHelper->m_userDebugText.size(); i++)
			{
				if (m_multiThreadedHelper->m_userDebugText[i].m_itemUniqueId == m_multiThreadedHelper->m_removeDebugItemUid)
				{
					m_multiThreadedHelper->m_userDebugText.swap(i, m_multiThreadedHelper->m_userDebugText.size() - 1);
					m_multiThreadedHelper->m_userDebugText.pop_back();
					break;
				}
			}

			m_multiThreadedHelper->mainThreadRelease();
			break;
		}
		case eGUIUserDebugRemoveAllParameters:
		{
			B3_PROFILE("eGUIUserDebugRemoveAllParameters");
			if (m_multiThreadedHelper->m_childGuiHelper->getParameterInterface())
				m_multiThreadedHelper->m_childGuiHelper->getParameterInterface()->removeAllParameters();
			for (int i = 0; i < m_multiThreadedHelper->m_userDebugParams.size(); i++)
			{
				delete m_multiThreadedHelper->m_userDebugParams[i];
			}
			m_multiThreadedHelper->m_userDebugParams.clear();
			m_multiThreadedHelper->mainThreadRelease();
			break;
		}

		case eGUIUserDebugRemoveAllItems:
		{
			B3_PROFILE("eGUIUserDebugRemoveAllItems");
			m_multiThreadedHelper->m_userDebugLines.clear();
			m_multiThreadedHelper->m_userDebugText.clear();
			m_multiThreadedHelper->m_uidGenerator = 0;
			m_multiThreadedHelper->mainThreadRelease();
			break;
		}

		case eGUIDumpFramesToVideo:
		{
			B3_PROFILE("eGUIDumpFramesToVideo");

			m_multiThreadedHelper->m_childGuiHelper->dumpFramesToVideo(m_multiThreadedHelper->m_mp4FileName);
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

void PhysicsServerExample::stepSimulation(float deltaTime)
{
	BT_PROFILE("PhysicsServerExample::stepSimulation");

	//this->m_physicsServer.processClientCommands();

	for (int i = m_multiThreadedHelper->m_userDebugLines.size() - 1; i >= 0; i--)
	{
		if (m_multiThreadedHelper->m_userDebugLines[i].m_lifeTime)
		{
			m_multiThreadedHelper->m_userDebugLines[i].m_lifeTime -= deltaTime;
			if (m_multiThreadedHelper->m_userDebugLines[i].m_lifeTime <= 0)
			{
				m_multiThreadedHelper->m_userDebugLines.swap(i, m_multiThreadedHelper->m_userDebugLines.size() - 1);
				m_multiThreadedHelper->m_userDebugLines.pop_back();
			}
		}
	}

	for (int i = m_multiThreadedHelper->m_userDebugText.size() - 1; i >= 0; i--)
	{
		if (m_multiThreadedHelper->m_userDebugText[i].m_lifeTime)
		{
			m_multiThreadedHelper->m_userDebugText[i].m_lifeTime -= deltaTime;
			if (m_multiThreadedHelper->m_userDebugText[i].m_lifeTime <= 0)
			{
				m_multiThreadedHelper->m_userDebugText.swap(i, m_multiThreadedHelper->m_userDebugText.size() - 1);
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

static float vrOffset[16] = {1, 0, 0, 0,
							 0, 1, 0, 0,
							 0, 0, 1, 0,
							 0, 0, 0, 0};

extern int gDroppedSimulationSteps;
extern int gNumSteps;
extern double gDtInSec;
extern double gSubStep;
extern btTransform gVRTrackingObjectTr;

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

		btAlignedObjectArray<btAlignedObjectArray<unsigned int> > sortedIndices;
		btAlignedObjectArray<btAlignedObjectArray<btVector3FloatData> > sortedLines;

		btHashMap<ColorWidth, int> hashedLines;

		for (int i = 0; i < m_multiThreadedHelper->m_userDebugLines.size(); i++)
		{
			btVector3 from;
			from.setValue(m_multiThreadedHelper->m_userDebugLines[i].m_debugLineFromXYZ[0],
						  m_multiThreadedHelper->m_userDebugLines[i].m_debugLineFromXYZ[1],
						  m_multiThreadedHelper->m_userDebugLines[i].m_debugLineFromXYZ[2]);
			btVector3 toX;
			toX.setValue(m_multiThreadedHelper->m_userDebugLines[i].m_debugLineToXYZ[0],
						 m_multiThreadedHelper->m_userDebugLines[i].m_debugLineToXYZ[1],
						 m_multiThreadedHelper->m_userDebugLines[i].m_debugLineToXYZ[2]);

			int graphicsIndex = m_multiThreadedHelper->m_userDebugLines[i].m_trackingVisualShapeIndex;
			if (graphicsIndex >= 0)
			{
				CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();
				if (renderer)
				{
					float parentPos[3];
					float parentOrn[4];

					if (renderer->readSingleInstanceTransformToCPU(parentPos, parentOrn, graphicsIndex))
					{
						btTransform parentTrans;
						parentTrans.setOrigin(btVector3(parentPos[0], parentPos[1], parentPos[2]));
						parentTrans.setRotation(btQuaternion(parentOrn[0], parentOrn[1], parentOrn[2], parentOrn[3]));
						from = parentTrans * from;
						toX = parentTrans * toX;
					}
				}
			}

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
				}
				else
				{
					index = sortedLines.size();
					sortedLines.expand();
					sortedIndices.expand();
					hashedLines.insert(cw, index);
				}
				btAssert(index >= 0);
				if (index >= 0)
				{
					btVector3FloatData from1, toX1;
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
			for (int i = 0; i < hashedLines.size(); i++)
			{
				ColorWidth cw = hashedLines.getKeyAtIndex(i);
				int index = *hashedLines.getAtIndex(i);
				int stride = sizeof(btVector3FloatData);
				const float* positions = &sortedLines[index][0].m_floats[0];
				int numPoints = sortedLines[index].size();
				const unsigned int* indices = &sortedIndices[index][0];
				int numIndices = sortedIndices[index].size();
				m_guiHelper->getAppInterface()->m_renderer->drawLines(positions, cw.m_color.m_floats, numPoints, stride, indices, numIndices, cw.width);
			}
		}

		for (int i = 0; i < m_multiThreadedHelper->m_userDebugText.size(); i++)
		{
			//int optionFlag = 0;//CommonGraphicsApp::eDrawText3D_OrtogonalFaceCamera|CommonGraphicsApp::eDrawText3D_TrueType;
			//int optionFlag = CommonGraphicsApp::eDrawText3D_TrueType;
			float orientation[4] = {0, 0, 0, 1};

			//int optionFlag = CommonGraphicsApp::eDrawText3D_OrtogonalFaceCamera;
			int optionFlag = 0;

			if (m_multiThreadedHelper->m_userDebugText[i].m_optionFlags & CommonGraphicsApp::eDrawText3D_OrtogonalFaceCamera)
			{
				optionFlag |= CommonGraphicsApp::eDrawText3D_OrtogonalFaceCamera;
			}
			else
			{
				orientation[0] = (float)m_multiThreadedHelper->m_userDebugText[i].m_textOrientation[0];
				orientation[1] = (float)m_multiThreadedHelper->m_userDebugText[i].m_textOrientation[1];
				orientation[2] = (float)m_multiThreadedHelper->m_userDebugText[i].m_textOrientation[2];
				orientation[3] = (float)m_multiThreadedHelper->m_userDebugText[i].m_textOrientation[3];
				optionFlag |= CommonGraphicsApp::eDrawText3D_TrueType;
			}

			float colorRGBA[4] = {
				(float)m_multiThreadedHelper->m_userDebugText[i].m_textColorRGB[0],
				(float)m_multiThreadedHelper->m_userDebugText[i].m_textColorRGB[1],
				(float)m_multiThreadedHelper->m_userDebugText[i].m_textColorRGB[2],
				(float)1.};

			float pos[3] = {(float)m_multiThreadedHelper->m_userDebugText[i].m_textPositionXYZ1[0],
							(float)m_multiThreadedHelper->m_userDebugText[i].m_textPositionXYZ1[1],
							(float)m_multiThreadedHelper->m_userDebugText[i].m_textPositionXYZ1[2]};

			int graphicsIndex = m_multiThreadedHelper->m_userDebugText[i].m_trackingVisualShapeIndex;
			if (graphicsIndex >= 0)
			{
				CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();
				if (renderer)
				{
					float parentPos[3];
					float parentOrn[4];

					if (renderer->readSingleInstanceTransformToCPU(parentPos, parentOrn, graphicsIndex))
					{
						btTransform parentTrans;
						parentTrans.setOrigin(btVector3(parentPos[0], parentPos[1], parentPos[2]));
						parentTrans.setRotation(btQuaternion(parentOrn[0], parentOrn[1], parentOrn[2], parentOrn[3]));
						btTransform childTr;
						childTr.setOrigin(btVector3(pos[0], pos[1], pos[2]));
						childTr.setRotation(btQuaternion(orientation[0], orientation[1], orientation[2], orientation[3]));

						btTransform siteTr = parentTrans * childTr;
						pos[0] = siteTr.getOrigin()[0];
						pos[1] = siteTr.getOrigin()[1];
						pos[2] = siteTr.getOrigin()[2];
						btQuaternion siteOrn = siteTr.getRotation();
						orientation[0] = siteOrn[0];
						orientation[1] = siteOrn[1];
						orientation[2] = siteOrn[2];
						orientation[3] = siteOrn[3];
					}
				}
			}

			{
				btAlignedObjectArray<std::string> pieces;
				btAlignedObjectArray<std::string> separators;
				separators.push_back("\n");
				urdfStringSplit(pieces, m_multiThreadedHelper->m_userDebugText[i].m_text, separators);

				double sz = m_multiThreadedHelper->m_userDebugText[i].textSize;

				btTransform tr;
				tr.setIdentity();
				tr.setOrigin(btVector3(pos[0], pos[1], pos[2]));
				tr.setRotation(btQuaternion(orientation[0], orientation[1], orientation[2], orientation[3]));

				//float newpos[3]={pos[0]-float(t)*sz,pos[1],pos[2]};

				for (int t = 0; t < pieces.size(); t++)
				{
					btTransform offset;
					offset.setIdentity();
					offset.setOrigin(btVector3(0, -float(t) * sz, 0));
					btTransform result = tr * offset;
					float newpos[3] = {(float)result.getOrigin()[0],
									   (float)result.getOrigin()[1],
									   (float)result.getOrigin()[2]};

					m_guiHelper->getAppInterface()->drawText3D(pieces[t].c_str(),
															   newpos, orientation, colorRGBA,
															   sz, optionFlag);
				}
			}

			/*m_guiHelper->getAppInterface()->drawText3D(m_multiThreadedHelper->m_userDebugText[i].m_text,
				m_multiThreadedHelper->m_userDebugText[i].m_textPositionXYZ[0],
				m_multiThreadedHelper->m_userDebugText[i].m_textPositionXYZ[1],
				m_multiThreadedHelper->m_userDebugText[i].m_textPositionXYZ[2],
				m_multiThreadedHelper->m_userDebugText[i].textSize);
				*/
		}
	}
}

void PhysicsServerExample::renderScene()
{
	m_renderedFrames++;

	btTransform vrTrans;

	B3_PROFILE("PhysicsServerExample::RenderScene");

	if (m_physicsServer.isRealTimeSimulationEnabled())
	{
		static int frameCount = 0;
		static btScalar prevTime = m_clock.getTimeSeconds();
		frameCount++;

		static char line0[1024] = {0};
		static char line1[1024] = {0};

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
		if ((gInternalSimFlags & 2) && m_tinyVrGui == 0)
		{
			ComboBoxParams comboParams;
			comboParams.m_comboboxId = 0;
			comboParams.m_numItems = 0;
			comboParams.m_startItem = 0;
			comboParams.m_callback = 0;     //MyComboBoxCallback;
			comboParams.m_userPointer = 0;  //this;

			m_tinyVrGui = new TinyVRGui(comboParams, this->m_multiThreadedHelper->m_childGuiHelper->getRenderInterface());
			m_tinyVrGui->init();
		}

		if (m_tinyVrGui)
		{
			b3Transform tr;
			tr.setIdentity();
			btVector3 VRController2Pos = m_physicsServer.getVRTeleportPosition();
			btQuaternion VRController2Orn = m_physicsServer.getVRTeleportOrientation();
			tr.setOrigin(b3MakeVector3(VRController2Pos[0], VRController2Pos[1], VRController2Pos[2]));
			tr.setRotation(b3Quaternion(VRController2Orn[0], VRController2Orn[1], VRController2Orn[2], VRController2Orn[3]));
			tr = tr * b3Transform(b3Quaternion(0, 0, -SIMD_HALF_PI), b3MakeVector3(0, 0, 0));
			b3Scalar dt = 0.01;
			m_tinyVrGui->clearTextArea();
			m_tinyVrGui->grapicalPrintf(line0, 0, 0, 0, 0, 0, 255);
			m_tinyVrGui->grapicalPrintf(line1, 0, 16, 255, 255, 255, 255);

			m_tinyVrGui->tick(dt, tr);
		}
#endif  //BT_ENABLE_VR
	}
	///debug rendering
	//m_args[0].m_cs->lock();

	//gVRTeleportPos[0] += 0.01;
	btTransform tr2a, tr2;
	tr2a.setIdentity();
	tr2.setIdentity();
	tr2.setOrigin(m_physicsServer.getVRTeleportPosition());
	tr2a.setRotation(m_physicsServer.getVRTeleportOrientation());
	btTransform trTotal = tr2 * tr2a;
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

	vrOffset[12] = trInv.getOrigin()[0];
	vrOffset[13] = trInv.getOrigin()[1];
	vrOffset[14] = trInv.getOrigin()[2];

	if (m_multiThreadedHelper->m_childGuiHelper->getRenderInterface())
	{
		m_multiThreadedHelper->m_childGuiHelper->getRenderInterface()->getActiveCamera()->setVRCameraOffsetTransform(vrOffset);
	}
	if (gEnableRendering)
	{
		int renderFlags = 0;
		if (!gEnableSyncPhysicsRendering)
		{
			renderFlags |= 1;  //COV_DISABLE_SYNC_RENDERING;
		}
		m_physicsServer.renderScene(renderFlags);
	}

	if (gEnablePicking)
	{
		for (int i = 0; i < MAX_VR_CONTROLLERS; i++)
		{
			if (m_args[0].m_isVrControllerPicking[i] || m_args[0].m_isVrControllerDragging[i])
			{
				btVector3 from = m_args[0].m_vrControllerPos[i];
				btMatrix3x3 mat(m_args[0].m_vrControllerOrn[i]);

				btVector3 toX = from + mat.getColumn(0);
				btVector3 toY = from + mat.getColumn(1);
				btVector3 toZ = from + mat.getColumn(2);

				int width = 2;

				btVector4 color;
				color = btVector4(1, 0, 0, 1);
				m_guiHelper->getAppInterface()->m_renderer->drawLine(from, toX, color, width);
				color = btVector4(0, 1, 0, 1);
				m_guiHelper->getAppInterface()->m_renderer->drawLine(from, toY, color, width);
				color = btVector4(0, 0, 1, 1);
				m_guiHelper->getAppInterface()->m_renderer->drawLine(from, toZ, color, width);
			}
		}
	}

	if (m_guiHelper->getAppInterface()->m_renderer->getActiveCamera()->isVRCamera())
	{
		if (!m_physicsServer.isRealTimeSimulationEnabled() && !gActivedVRRealTimeSimulation)
		{
			//only activate real-time simulation once (for backward compatibility)
			gActivedVRRealTimeSimulation = true;
			m_physicsServer.enableRealTimeSimulation(1);
		}
	}

	drawUserDebugLines();

	//m_args[0].m_cs->unlock();
}

void PhysicsServerExample::physicsDebugDraw(int debugDrawFlags)
{
	m_renderedFrames++;

	if (gEnableSyncPhysicsRendering)
	{
		m_physicsServer.syncPhysicsToGraphics();
	}

	drawUserDebugLines();

	if (gEnableRendering)
	{
		///debug rendering
		//m_physicsServer.physicsDebugDraw(debugDrawFlags);
		m_args[0].m_csGUI->lock();
		//draw stuff and flush?
		this->m_multiThreadedHelper->m_debugDraw->drawDebugDrawerLines();
		m_args[0].m_debugDrawFlags = debugDrawFlags;
		m_args[0].m_enableUpdateDebugDrawLines = true;
		m_args[0].m_csGUI->unlock();
	}
}

btVector3 PhysicsServerExample::getRayTo(int x, int y)
{
	CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();

	if (!renderer)
	{
		btAssert(0);
		return btVector3(0, 0, 0);
	}

	float top = 1.f;
	float bottom = -1.f;
	float nearPlane = 1.f;
	float tanFov = (top - bottom) * 0.5f / nearPlane;
	float fov = btScalar(2.0) * btAtan(tanFov);

	btVector3 camPos, camTarget;
	renderer->getActiveCamera()->getCameraPosition(camPos);
	renderer->getActiveCamera()->getCameraTargetPosition(camTarget);

	btVector3 rayFrom = camPos;
	btVector3 rayForward = (camTarget - camPos);
	rayForward.normalize();
	float farPlane = 10000.f;
	rayForward *= farPlane;

	btVector3 rightOffset;
	btVector3 cameraUp = btVector3(0, 0, 0);
	cameraUp[m_guiHelper->getAppInterface()->getUpAxis()] = 1;

	btVector3 vertical = cameraUp;

	btVector3 hor;
	hor = rayForward.cross(vertical);
	hor.safeNormalize();
	vertical = hor.cross(rayForward);
	vertical.safeNormalize();

	float tanfov = tanf(0.5f * fov);

	hor *= 2.f * farPlane * tanfov;
	vertical *= 2.f * farPlane * tanfov;

	btScalar aspect;
	float width = float(renderer->getScreenWidth());
	float height = float(renderer->getScreenHeight());

	aspect = width / height;

	hor *= aspect;

	btVector3 rayToCenter = rayFrom + rayForward;
	btVector3 dHor = hor * 1.f / width;
	btVector3 dVert = vertical * 1.f / height;

	btVector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * vertical;
	rayTo += btScalar(x) * dHor;
	rayTo -= btScalar(y) * dVert;
	return rayTo;
}

void PhysicsServerExample::vrControllerButtonCallback(int controllerId, int button, int state, float pos[4], float orn[4])
{
	//printf("controllerId %d, button=%d\n",controllerId, button);

	if (controllerId < 0 || controllerId >= MAX_VR_CONTROLLERS)
		return;

	if (gGraspingController < 0)
	{
		gGraspingController = controllerId;
	}

	btTransform trLocal;
	trLocal.setIdentity();
	trLocal.setRotation(btQuaternion(btVector3(0, 0, 1), SIMD_HALF_PI) * btQuaternion(btVector3(0, 1, 0), SIMD_HALF_PI));

	btTransform trOrg;
	trOrg.setIdentity();
	trOrg.setOrigin(btVector3(pos[0], pos[1], pos[2]));
	trOrg.setRotation(btQuaternion(orn[0], orn[1], orn[2], orn[3]));

	btTransform tr2a;
	tr2a.setIdentity();
	btTransform tr2;
	tr2.setIdentity();

	tr2.setOrigin(m_physicsServer.getVRTeleportPosition());
	tr2a.setRotation(m_physicsServer.getVRTeleportOrientation());

	btTransform trTotal = tr2 * tr2a * trOrg * trLocal;

	if (controllerId != gGraspingController)
	{
		if (button == 1 && state == 0)
		{
		}
	}
	else
	{
		if (button == 1)
		{
			if (state == 1)
			{
				gDebugRenderToggle = 1;
			}
			else
			{
				gDebugRenderToggle = 0;
#if 0  //it confuses people, make it into a debug option in a VR GUI?
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
		}
		else
		{
		}
	}

	if (button == 32 && state == 0)
	{
		if (controllerId == gGraspingController)
		{
		}
		else
		{
			//			gEnableKukaControl = !gEnableKukaControl;
		}
	}

	if (button == 1 && gEnableTeleporting)
	{
		m_args[0].m_isVrControllerTeleporting[controllerId] = true;
	}

	if (controllerId == gGraspingController && (button == 33))
	{
	}
	else
	{
		if (button == 33 && gEnablePicking)
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
	m_args[0].m_vrControllerEvents[controllerId].m_deviceType = VR_DEVICE_CONTROLLER;
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
		m_args[0].m_vrControllerEvents[controllerId].m_buttons[button] |= eButtonIsDown + eButtonTriggered;
	}
	else
	{
		m_args[0].m_vrControllerEvents[controllerId].m_buttons[button] |= eButtonReleased;
		m_args[0].m_vrControllerEvents[controllerId].m_buttons[button] &= ~eButtonIsDown;
	}
	m_args[0].m_csGUI->unlock();
}

void PhysicsServerExample::vrControllerMoveCallback(int controllerId, float pos[4], float orn[4], float analogAxis, float auxAnalogAxes[10])
{
	if (controllerId < 0 || controllerId >= MAX_VR_CONTROLLERS)
	{
		printf("Controller Id exceeds max: %d > %d", controllerId, MAX_VR_CONTROLLERS);
		return;
	}

	btTransform trLocal;
	trLocal.setIdentity();
	trLocal.setRotation(btQuaternion(btVector3(0, 0, 1), SIMD_HALF_PI) * btQuaternion(btVector3(0, 1, 0), SIMD_HALF_PI));

	btTransform trOrg;
	trOrg.setIdentity();
	trOrg.setOrigin(btVector3(pos[0], pos[1], pos[2]));
	trOrg.setRotation(btQuaternion(orn[0], orn[1], orn[2], orn[3]));

	btTransform tr2a;
	tr2a.setIdentity();
	btTransform tr2;
	tr2.setIdentity();

	tr2.setOrigin(m_physicsServer.getVRTeleportPosition());
	tr2a.setRotation(m_physicsServer.getVRTeleportOrientation());

	btTransform trTotal = tr2 * tr2a * trOrg * trLocal;

	if (controllerId == gGraspingController)
	{
	}
	else
	{
		m_args[0].m_vrControllerPos[controllerId] = trTotal.getOrigin();
		m_args[0].m_vrControllerOrn[controllerId] = trTotal.getRotation();
	}

	m_args[0].m_csGUI->lock();
	m_args[0].m_vrControllerEvents[controllerId].m_controllerId = controllerId;
	m_args[0].m_vrControllerEvents[controllerId].m_deviceType = VR_DEVICE_CONTROLLER;
	m_args[0].m_vrControllerEvents[controllerId].m_pos[0] = trTotal.getOrigin()[0];
	m_args[0].m_vrControllerEvents[controllerId].m_pos[1] = trTotal.getOrigin()[1];
	m_args[0].m_vrControllerEvents[controllerId].m_pos[2] = trTotal.getOrigin()[2];
	m_args[0].m_vrControllerEvents[controllerId].m_orn[0] = trTotal.getRotation()[0];
	m_args[0].m_vrControllerEvents[controllerId].m_orn[1] = trTotal.getRotation()[1];
	m_args[0].m_vrControllerEvents[controllerId].m_orn[2] = trTotal.getRotation()[2];
	m_args[0].m_vrControllerEvents[controllerId].m_orn[3] = trTotal.getRotation()[3];
	m_args[0].m_vrControllerEvents[controllerId].m_numMoveEvents++;
	m_args[0].m_vrControllerEvents[controllerId].m_analogAxis = analogAxis;
	for (int i = 0; i < 10; i++)
	{
		m_args[0].m_vrControllerEvents[controllerId].m_auxAnalogAxis[i] = auxAnalogAxes[i];
	}

	m_args[0].m_csGUI->unlock();
}

void PhysicsServerExample::vrHMDMoveCallback(int controllerId, float pos[4], float orn[4])
{
	if (controllerId < 0 || controllerId >= MAX_VR_CONTROLLERS)
	{
		printf("Controller Id exceeds max: %d > %d", controllerId, MAX_VR_CONTROLLERS);
		return;
	}

	//we may need to add some trLocal transform, to align the camera to our preferences
	btTransform trLocal;
	trLocal.setIdentity();
	//	trLocal.setRotation(btQuaternion(btVector3(0, 0, 1), SIMD_HALF_PI)*btQuaternion(btVector3(0, 1, 0), SIMD_HALF_PI));

	btTransform trOrg;
	trOrg.setIdentity();
	trOrg.setOrigin(btVector3(pos[0], pos[1], pos[2]));
	trOrg.setRotation(btQuaternion(orn[0], orn[1], orn[2], orn[3]));

	btTransform tr2a;
	tr2a.setIdentity();
	btTransform tr2;
	tr2.setIdentity();
	tr2.setOrigin(m_physicsServer.getVRTeleportPosition());
	tr2a.setRotation(m_physicsServer.getVRTeleportOrientation());
	btTransform trTotal = tr2 * tr2a * trOrg * trLocal;

	m_args[0].m_csGUI->lock();
	m_args[0].m_vrControllerEvents[controllerId].m_controllerId = controllerId;
	m_args[0].m_vrControllerEvents[controllerId].m_deviceType = VR_DEVICE_HMD;
	m_args[0].m_vrControllerEvents[controllerId].m_pos[0] = trTotal.getOrigin()[0];
	m_args[0].m_vrControllerEvents[controllerId].m_pos[1] = trTotal.getOrigin()[1];
	m_args[0].m_vrControllerEvents[controllerId].m_pos[2] = trTotal.getOrigin()[2];
	m_args[0].m_vrControllerEvents[controllerId].m_orn[0] = trTotal.getRotation()[0];
	m_args[0].m_vrControllerEvents[controllerId].m_orn[1] = trTotal.getRotation()[1];
	m_args[0].m_vrControllerEvents[controllerId].m_orn[2] = trTotal.getRotation()[2];
	m_args[0].m_vrControllerEvents[controllerId].m_orn[3] = trTotal.getRotation()[3];
	m_args[0].m_vrControllerEvents[controllerId].m_numMoveEvents++;
	m_args[0].m_csGUI->unlock();
}
void PhysicsServerExample::vrGenericTrackerMoveCallback(int controllerId, float pos[4], float orn[4])
{
	if (controllerId < 0 || controllerId >= MAX_VR_CONTROLLERS)
	{
		printf("Controller Id exceeds max: %d > %d", controllerId, MAX_VR_CONTROLLERS);
		return;
	}

	//we may need to add some trLocal transform, to align the camera to our preferences
	btTransform trLocal;
	trLocal.setIdentity();
	trLocal.setRotation(btQuaternion(btVector3(0, 0, 1), SIMD_HALF_PI) * btQuaternion(btVector3(0, 1, 0), SIMD_HALF_PI));

	btTransform trOrg;
	trOrg.setIdentity();
	trOrg.setOrigin(btVector3(pos[0], pos[1], pos[2]));
	trOrg.setRotation(btQuaternion(orn[0], orn[1], orn[2], orn[3]));

	btTransform tr2a;
	tr2a.setIdentity();
	btTransform tr2;
	tr2.setIdentity();
	tr2.setOrigin(m_physicsServer.getVRTeleportPosition());
	tr2a.setRotation(m_physicsServer.getVRTeleportOrientation());
	btTransform trTotal = tr2 * tr2a * trOrg * trLocal;

	m_args[0].m_csGUI->lock();
	m_args[0].m_vrControllerEvents[controllerId].m_controllerId = controllerId;
	m_args[0].m_vrControllerEvents[controllerId].m_deviceType = VR_DEVICE_GENERIC_TRACKER;
	m_args[0].m_vrControllerEvents[controllerId].m_pos[0] = trTotal.getOrigin()[0];
	m_args[0].m_vrControllerEvents[controllerId].m_pos[1] = trTotal.getOrigin()[1];
	m_args[0].m_vrControllerEvents[controllerId].m_pos[2] = trTotal.getOrigin()[2];
	m_args[0].m_vrControllerEvents[controllerId].m_orn[0] = trTotal.getRotation()[0];
	m_args[0].m_vrControllerEvents[controllerId].m_orn[1] = trTotal.getRotation()[1];
	m_args[0].m_vrControllerEvents[controllerId].m_orn[2] = trTotal.getRotation()[2];
	m_args[0].m_vrControllerEvents[controllerId].m_orn[3] = trTotal.getRotation()[3];
	m_args[0].m_vrControllerEvents[controllerId].m_numMoveEvents++;
	m_args[0].m_csGUI->unlock();
}

extern int gSharedMemoryKey;

class CommonExampleInterface* PhysicsServerCreateFuncInternal(struct CommonExampleOptions& options)
{
	MultiThreadedOpenGLGuiHelper* guiHelperWrapper = new MultiThreadedOpenGLGuiHelper(options.m_guiHelper->getAppInterface(), options.m_guiHelper, options.m_skipGraphicsUpdate);

	PhysicsServerExample* example = new PhysicsServerExample(guiHelperWrapper,
															 options.m_commandProcessorCreation,
															 options.m_sharedMem,
															 options.m_option);

	if (gSharedMemoryKey >= 0)
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
