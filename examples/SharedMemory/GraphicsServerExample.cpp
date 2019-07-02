
#include "GraphicsServerExample.h"
#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "PosixSharedMemory.h"
#include "Win32SharedMemory.h"
#include "../CommonInterfaces/CommonExampleInterface.h"
#include "LinearMath/btTransform.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "GraphicsSharedMemoryBlock.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "SharedMemoryPublic.h"

#define MAX_GRAPHICS_SHARED_MEMORY_BLOCKS 1


class GraphicsServerExample : public CommonExampleInterface
{
	CommonGraphicsApp* m_app;
	GUIHelperInterface* m_guiHelper;
	SharedMemoryInterface* m_sharedMemory;
	int m_sharedMemoryKey;
	bool m_verboseOutput;
	bool m_areConnected[MAX_GRAPHICS_SHARED_MEMORY_BLOCKS];
	GraphicsSharedMemoryBlock* m_testBlocks[MAX_GRAPHICS_SHARED_MEMORY_BLOCKS];
	b3AlignedObjectArray< b3AlignedObjectArray<unsigned char> > m_dataSlots;

	float m_x;
	float m_y;
	float m_z;

public:
	GraphicsServerExample(GUIHelperInterface* guiHelper)
		: m_guiHelper(guiHelper),
		  m_x(0),
		  m_y(0),
		  m_z(0)
	{
		m_verboseOutput = true;
		m_sharedMemoryKey = GRAPHICS_SHARED_MEMORY_KEY;
		m_app = guiHelper->getAppInterface();
		m_app->setUpAxis(2);


		for (int i = 0; i < MAX_GRAPHICS_SHARED_MEMORY_BLOCKS; i++)
		{
			m_areConnected[i] = false;
		}

#ifdef _WIN32
		m_sharedMemory = new Win32SharedMemoryServer();
#else
		m_sharedMemory = new PosixSharedMemory();
#endif

#if 0
		{
			int boxId = m_app->registerCubeShape(0.1, 0.1, 0.1);
			btVector3 pos(0, 0, 0);
			btQuaternion orn(0, 0, 0, 1);
			btVector4 color(0.3, 0.3, 0.3, 1);
			btVector3 scaling(1, 1, 1);
			m_app->m_renderer->registerGraphicsInstance(boxId, pos, orn, color, scaling);
		}

		m_app->m_renderer->writeTransforms();
#endif
		connectSharedMemory(m_guiHelper, m_sharedMemoryKey);

	}
	virtual ~GraphicsServerExample()
	{
		disconnectSharedMemory();
	}

	virtual void initPhysics()
	{
	}
	virtual void exitPhysics()
	{
	}
	
	GraphicsSharedMemoryStatus& createServerStatus(int statusType, int sequenceNumber, int timeStamp, int blockIndex)
	{
		GraphicsSharedMemoryStatus& serverCmd = m_testBlocks[blockIndex]->m_serverCommands[0];
		serverCmd.m_type = statusType;
		serverCmd.m_sequenceNumber = sequenceNumber;
		serverCmd.m_timeStamp = timeStamp;
		return serverCmd;
	}
	void submitServerStatus(GraphicsSharedMemoryStatus& status, int blockIndex)
	{
		m_testBlocks[blockIndex]->m_numServerCommands++;
	}

	bool processCommand(const struct GraphicsSharedMemoryCommand& clientCmd, struct GraphicsSharedMemoryStatus& serverStatusOut, char* bufferServerToClient, int bufferSizeInBytes)
	{
		//printf("processed command of type:%d\n", clientCmd.m_type);
		B3_PROFILE("processCommand");
		switch (clientCmd.m_type)
		{
			case GFX_CMD_0:
			{
				//either Y or Z can be up axis
				int upAxis = (clientCmd.m_upAxisYCommand.m_enableUpAxisY) ? 1 : 2;
				m_guiHelper->setUpAxis(upAxis);
				break;
			}
			case GFX_CMD_SET_VISUALIZER_FLAG:
			{
				if (clientCmd.m_visualizerFlagCommand.m_visualizerFlag != COV_ENABLE_RENDERING)
				{
					//printf("clientCmd.m_visualizerFlag.m_visualizerFlag: %d, clientCmd.m_visualizerFlag.m_enable %d\n",
					//	clientCmd.m_visualizerFlagCommand.m_visualizerFlag, clientCmd.m_visualizerFlagCommand.m_enable);

					this->m_guiHelper->setVisualizerFlag(clientCmd.m_visualizerFlagCommand.m_visualizerFlag, clientCmd.m_visualizerFlagCommand.m_enable);
				}
				break;

			}

			case GFX_CMD_UPLOAD_DATA:
			{
				//printf("uploadData command: curSize=%d, offset=%d, slot=%d", clientCmd.m_uploadDataCommand.m_numBytes, clientCmd.m_uploadDataCommand.m_dataOffset, clientCmd.m_uploadDataCommand.m_dataSlot);
				int dataSlot = clientCmd.m_uploadDataCommand.m_dataSlot;
				int dataOffset = clientCmd.m_uploadDataCommand.m_dataOffset;
				m_dataSlots.resize(dataSlot + 1);
				btAssert(m_dataSlots[dataSlot].size() >= dataOffset);
				m_dataSlots[dataSlot].resize(clientCmd.m_uploadDataCommand.m_numBytes + clientCmd.m_uploadDataCommand.m_dataOffset);
				for (int i = 0; i < clientCmd.m_uploadDataCommand.m_numBytes; i++)
				{
					m_dataSlots[dataSlot][dataOffset + i] = bufferServerToClient[i];
				}
				break;
			}
			case GFX_CMD_REGISTER_TEXTURE:
			{
				int dataSlot = 0;
				int sizeData = m_dataSlots[dataSlot].size();
				btAssert(sizeData > 0);
				serverStatusOut.m_type = GFX_CMD_REGISTER_TEXTURE_FAILED;
				if (sizeData)
				{
					unsigned char* texels = &m_dataSlots[dataSlot][0];
					int textureId = this->m_guiHelper->registerTexture(texels, clientCmd.m_registerTextureCommand.m_width, clientCmd.m_registerTextureCommand.m_height);
					serverStatusOut.m_type = GFX_CMD_REGISTER_TEXTURE_COMPLETED;
					serverStatusOut.m_registerTextureStatus.m_textureId = textureId;
				}
				break;
			}

			case GFX_CMD_REGISTER_GRAPHICS_SHAPE:
			{
				int verticesSlot = 0;
				int indicesSlot = 1;
				serverStatusOut.m_type = GFX_CMD_REGISTER_GRAPHICS_SHAPE_FAILED;
				const float* vertices = (const float*)&m_dataSlots[verticesSlot][0];
				const int* indices = (const int*)&m_dataSlots[indicesSlot][0];
				int numVertices = clientCmd.m_registerGraphicsShapeCommand.m_numVertices;
				int numIndices = clientCmd.m_registerGraphicsShapeCommand.m_numIndices;
				int primitiveType = clientCmd.m_registerGraphicsShapeCommand.m_primitiveType;
				int textureId = clientCmd.m_registerGraphicsShapeCommand.m_textureId;
				int shapeId = this->m_guiHelper->registerGraphicsShape(vertices, numVertices, indices, numIndices, primitiveType, textureId);
				serverStatusOut.m_registerGraphicsShapeStatus.m_shapeId = shapeId;
				serverStatusOut.m_type = GFX_CMD_REGISTER_GRAPHICS_SHAPE_COMPLETED;
				break;
			}

			case GFX_CMD_REGISTER_GRAPHICS_INSTANCE:
			{
				int graphicsInstanceId = m_guiHelper->registerGraphicsInstance(clientCmd.m_registerGraphicsInstanceCommand.m_shapeIndex,
					clientCmd.m_registerGraphicsInstanceCommand.m_position,
					clientCmd.m_registerGraphicsInstanceCommand.m_quaternion,
					clientCmd.m_registerGraphicsInstanceCommand.m_color,
					clientCmd.m_registerGraphicsInstanceCommand.m_scaling);
				serverStatusOut.m_registerGraphicsInstanceStatus.m_graphicsInstanceId = graphicsInstanceId;
				serverStatusOut.m_type = GFX_CMD_REGISTER_GRAPHICS_INSTANCE_COMPLETED;

				break;
			}
			case GFX_CMD_SYNCHRONIZE_TRANSFORMS:
			{
				GUISyncPosition* positions = (GUISyncPosition*)bufferServerToClient;
				for (int i = 0; i < clientCmd.m_syncTransformsCommand.m_numPositions; i++)
				{
					m_app->m_renderer->writeSingleInstanceTransformToCPU(positions[i].m_pos, positions[i].m_orn, positions[i].m_graphicsInstanceId);
				}
				break;
			}
			case GFX_CMD_REMOVE_ALL_GRAPHICS_INSTANCES:
			{
				m_guiHelper->removeAllGraphicsInstances();
				break;
			}
			case GFX_CMD_REMOVE_SINGLE_GRAPHICS_INSTANCE:
			{
				m_app->m_renderer->removeGraphicsInstance(clientCmd.m_removeGraphicsInstanceCommand.m_graphicsUid);
				break;
			}
			case GFX_CMD_CHANGE_RGBA_COLOR:
			{
				m_guiHelper->changeRGBAColor(clientCmd.m_changeRGBAColorCommand.m_graphicsUid, clientCmd.m_changeRGBAColorCommand.m_rgbaColor);
				break;
			}
			case GFX_CMD_GET_CAMERA_INFO:
			{
				serverStatusOut.m_type = GFX_CMD_GET_CAMERA_INFO_FAILED;
				
				if (m_guiHelper->getCameraInfo(
					&serverStatusOut.m_getCameraInfoStatus.width,
					&serverStatusOut.m_getCameraInfoStatus.height,
					serverStatusOut.m_getCameraInfoStatus.viewMatrix,
					serverStatusOut.m_getCameraInfoStatus.projectionMatrix,
					serverStatusOut.m_getCameraInfoStatus.camUp,
					serverStatusOut.m_getCameraInfoStatus.camForward,
					serverStatusOut.m_getCameraInfoStatus.hor,
					serverStatusOut.m_getCameraInfoStatus.vert,
					&serverStatusOut.m_getCameraInfoStatus.yaw,
					&serverStatusOut.m_getCameraInfoStatus.pitch,
					&serverStatusOut.m_getCameraInfoStatus.camDist,
					serverStatusOut.m_getCameraInfoStatus.camTarget))
				{
					serverStatusOut.m_type = GFX_CMD_GET_CAMERA_INFO_COMPLETED;
				}

				break;
			}
			default:
			{
				printf("unsupported command:%d\n", clientCmd.m_type);
			}
		}
		return true;
	}

	void processClientCommands()
	{
		B3_PROFILE("processClientCommands");
		for (int block = 0; block < MAX_GRAPHICS_SHARED_MEMORY_BLOCKS; block++)
		{
			if (m_areConnected[block] && m_testBlocks[block])
			{
				///we ignore overflow of integer for now
				if (m_testBlocks[block]->m_numClientCommands > m_testBlocks[block]->m_numProcessedClientCommands)
				{
					//BT_PROFILE("processClientCommand");

					//until we implement a proper ring buffer, we assume always maximum of 1 outstanding commands
					btAssert(m_testBlocks[block]->m_numClientCommands == m_testBlocks[block]->m_numProcessedClientCommands + 1);

					const GraphicsSharedMemoryCommand& clientCmd = m_testBlocks[block]->m_clientCommands[0];

					m_testBlocks[block]->m_numProcessedClientCommands++;
					//todo, timeStamp
					int timeStamp = 0;
					GraphicsSharedMemoryStatus& serverStatusOut = createServerStatus(GFX_CMD_CLIENT_COMMAND_FAILED, clientCmd.m_sequenceNumber, timeStamp, block);
					bool hasStatus = processCommand(clientCmd, serverStatusOut, &m_testBlocks[block]->m_bulletStreamData[0], GRAPHICS_SHARED_MEMORY_MAX_STREAM_CHUNK_SIZE);
					if (hasStatus)
					{
						submitServerStatus(serverStatusOut, block);
					}
				}
			}
		}
	}

	virtual void stepSimulation(float deltaTime)
	{
		B3_PROFILE("stepSimulation");
		processClientCommands();
		m_x += 0.01f;
		m_y += 0.01f;
		m_z += 0.01f;
	}


	virtual void renderScene()
	{
		B3_PROFILE("renderScene");
		{

			B3_PROFILE("writeTransforms");
			m_app->m_renderer->writeTransforms();
		}
		{
			B3_PROFILE("m_renderer->renderScene");
			m_app->m_renderer->renderScene();
		}
		
	}

	

	virtual void physicsDebugDraw(int debugDrawFlags)
	{
		
	}

	virtual bool mouseMoveCallback(float x, float y)
	{
		return false;
	}
	virtual bool mouseButtonCallback(int button, int state, float x, float y)
	{
		return false;
	}
	virtual bool keyboardCallback(int key, int state)
	{
		return false;
	}

	virtual void resetCamera()
	{
		float dist = 3.5;
		float pitch = -32;
		float yaw = 136;
		float targetPos[3] = {0, 0, 0};
		if (m_app->m_renderer && m_app->m_renderer->getActiveCamera())
		{
			m_app->m_renderer->getActiveCamera()->setCameraDistance(dist);
			m_app->m_renderer->getActiveCamera()->setCameraPitch(pitch);
			m_app->m_renderer->getActiveCamera()->setCameraYaw(yaw);
			m_app->m_renderer->getActiveCamera()->setCameraTargetPosition(targetPos[0], targetPos[1], targetPos[2]);
		}
	}
	
	bool connectSharedMemory(struct GUIHelperInterface* guiHelper, int sharedMemoryKey);
	void disconnectSharedMemory(bool deInitializeSharedMemory=true);
};



bool GraphicsServerExample::connectSharedMemory(struct GUIHelperInterface* guiHelper, int sharedMemoryKey)
{
	
	bool allowCreation = true;
	bool allConnected = false;
	int numConnected = 0;

	int counter = 0;
	for (int block = 0; block < MAX_GRAPHICS_SHARED_MEMORY_BLOCKS; block++)
	{
		if (m_areConnected[block])
		{
			allConnected = true;
			numConnected++;
			b3Warning("connectSharedMemory, while already connected");
			continue;
		}
		do
		{
			m_testBlocks[block] = (GraphicsSharedMemoryBlock*)m_sharedMemory->allocateSharedMemory(m_sharedMemoryKey + block, GRAPHICS_SHARED_MEMORY_SIZE, allowCreation);
			if (m_testBlocks[block])
			{
				int magicId = m_testBlocks[block]->m_magicId;
				if (m_verboseOutput)
				{
					b3Printf("magicId = %d\n", magicId);
				}

				if (m_testBlocks[block]->m_magicId != GRAPHICS_SHARED_MEMORY_MAGIC_NUMBER)
				{
					InitSharedMemoryBlock(m_testBlocks[block]);
					if (m_verboseOutput)
					{
						b3Printf("Created and initialized shared memory block\n");
					}
					m_areConnected[block] = true;
					numConnected++;
				}
				else
				{
					m_sharedMemory->releaseSharedMemory(m_sharedMemoryKey + block, GRAPHICS_SHARED_MEMORY_SIZE);
					m_testBlocks[block] = 0;
					m_areConnected[block] = false;
				}
			}
			else
			{
				//b3Error("Cannot connect to shared memory");
				m_areConnected[block] = false;
			}
		} while (counter++ < 10 && !m_areConnected[block]);
		if (!m_areConnected[block])
		{
			b3Error("Server cannot connect to shared memory.\n");
		}
	}

	allConnected = (numConnected == MAX_GRAPHICS_SHARED_MEMORY_BLOCKS);

	return allConnected;
}

void GraphicsServerExample::disconnectSharedMemory(bool deInitializeSharedMemory)
{
	//m_data->m_commandProcessor->deleteDynamicsWorld();

	
	if (m_verboseOutput)
	{
		b3Printf("releaseSharedMemory1\n");
	}
	for (int block = 0; block < MAX_GRAPHICS_SHARED_MEMORY_BLOCKS; block++)
	{
		if (m_testBlocks[block])
		{
			if (m_verboseOutput)
			{
				b3Printf("m_testBlock1\n");
			}
			if (deInitializeSharedMemory)
			{
				m_testBlocks[block]->m_magicId = 0;
				if (m_verboseOutput)
				{
					b3Printf("De-initialized shared memory, magic id = %d\n", m_testBlocks[block]->m_magicId);
				}
			}
			btAssert(m_sharedMemory);
			m_sharedMemory->releaseSharedMemory(m_sharedMemoryKey + block, GRAPHICS_SHARED_MEMORY_SIZE);
		}
		m_testBlocks[block] = 0;
		m_areConnected[block] = false;
	}
}

CommonExampleInterface* GraphicsServerCreateFuncBullet(struct CommonExampleOptions& options)
{
	return new GraphicsServerExample(options.m_guiHelper);
}
