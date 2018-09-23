#include "GpuConvexScene.h"

#include "GpuRigidBodyDemo.h"
#include "../OpenGLWindow/ShapeData.h"

#include "../OpenGLWindow/GLInstancingRenderer.h"
#include "Bullet3Common/b3Quaternion.h"
#include "../CommonInterfaces/CommonWindowInterface.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3GpuSapBroadphase.h"
#include "../CommonOpenCL/GpuDemoInternalData.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "../OpenGLWindow/OpenGLInclude.h"
#include "../OpenGLWindow/GLInstanceRendererInternalData.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3LauncherCL.h"
#include "Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipeline.h"
#include "Bullet3OpenCL/RigidBody/b3GpuNarrowPhase.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3Config.h"
#include "GpuRigidBodyDemoInternalData.h"

#include "Bullet3Dynamics/ConstraintSolver/b3Point2PointConstraint.h"
#include "../OpenGLWindow/GLPrimitiveRenderer.h"
#include "Bullet3OpenCL/Raycast/b3GpuRaycast.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3ConvexUtility.h"
#include "Bullet3Dynamics/ConstraintSolver/b3FixedConstraint.h"

#include "../OpenGLWindow/GLRenderToTexture.h"

static bool gUseInstancedCollisionShapes = true;
extern int gGpuArraySizeX;
extern int gGpuArraySizeY;
extern int gGpuArraySizeZ;

#include "GpuRigidBodyDemo.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3RaycastInfo.h"

class GpuConvexScene : public GpuRigidBodyDemo
{
protected:
	class b3GpuRaycast* m_raycaster;

public:
	GpuConvexScene(GUIHelperInterface* helper)
		: GpuRigidBodyDemo(helper), m_raycaster(0)
	{
	}
	virtual ~GpuConvexScene() {}
	virtual const char* getName()
	{
		return "Tetrahedra";
	}

	virtual void setupScene();

	virtual void destroyScene();

	virtual int createDynamicsObjects();

	virtual int createDynamicsObjects2(const float* vertices, int numVertices, const int* indices, int numIndices);

	virtual void createStaticEnvironment();
};

class GpuConvexPlaneScene : public GpuConvexScene
{
public:
	GpuConvexPlaneScene(GUIHelperInterface* helper)
		: GpuConvexScene(helper) {}
	virtual ~GpuConvexPlaneScene() {}
	virtual const char* getName()
	{
		return "ConvexOnPlane";
	}

	virtual void createStaticEnvironment();
};

class GpuBoxPlaneScene : public GpuConvexPlaneScene
{
public:
	GpuBoxPlaneScene(GUIHelperInterface* helper) : GpuConvexPlaneScene(helper) {}
	virtual ~GpuBoxPlaneScene() {}
	virtual const char* getName()
	{
		return "BoxBox";
	}

	virtual int createDynamicsObjects();
};

class GpuTetraScene : public GpuConvexScene
{
protected:
	void createFromTetGenData(const char* ele, const char* node);

public:
	virtual const char* getName()
	{
		return "TetraBreakable";
	}

	virtual int createDynamicsObjects();
};

b3Vector4 colors[4] =
	{
		b3MakeVector4(1, 0, 0, 1),
		b3MakeVector4(0, 1, 0, 1),
		b3MakeVector4(0, 1, 1, 1),
		b3MakeVector4(1, 1, 0, 1),
};

void GpuConvexScene::setupScene()
{
	m_raycaster = new b3GpuRaycast(m_clData->m_clContext, m_clData->m_clDevice, m_clData->m_clQueue);

	int index = 0;
	createStaticEnvironment();

	index += createDynamicsObjects();

	m_data->m_rigidBodyPipeline->writeAllInstancesToGpu();

	float camPos[4] = {0, 0, 0, 0};  //ci.arraySizeX,ci.arraySizeY/2,ci.arraySizeZ,0};
	//float camPos[4]={1,12.5,1.5,0};

	m_guiHelper->getRenderInterface()->getActiveCamera()->setCameraTargetPosition(camPos[0], camPos[1], camPos[2]);
	m_guiHelper->getRenderInterface()->getActiveCamera()->setCameraDistance(150);
	//m_instancingRenderer->setCameraYaw(85);
	m_guiHelper->getRenderInterface()->getActiveCamera()->setCameraYaw(225);
	m_guiHelper->getRenderInterface()->getActiveCamera()->setCameraPitch(-30);

	m_guiHelper->getRenderInterface()->updateCamera(1);  //>updateCamera();

	char msg[1024];
	int numInstances = index;
	sprintf(msg, "Num objects = %d", numInstances);
	b3Printf(msg);

	//if (ci.m_gui)
	//	ci.m_gui->setStatusBarMessage(msg,true);
}

void GpuConvexScene::destroyScene()
{
	delete m_raycaster;
	m_raycaster = 0;
}

int GpuConvexScene::createDynamicsObjects()
{
	int strideInBytes = 9 * sizeof(float);
	/*int numVertices = sizeof(barrel_vertices)/strideInBytes;
	int numIndices = sizeof(barrel_indices)/sizeof(int);
	return createDynamicsObjects2(ci,barrel_vertices,numVertices,barrel_indices,numIndices);
	*/

	int numVertices = sizeof(tetra_vertices) / strideInBytes;
	int numIndices = sizeof(tetra_indices) / sizeof(int);
	return createDynamicsObjects2(tetra_vertices, numVertices, tetra_indices, numIndices);
}

int GpuBoxPlaneScene::createDynamicsObjects()
{
	int strideInBytes = 9 * sizeof(float);
	int numVertices = sizeof(cube_vertices) / strideInBytes;
	int numIndices = sizeof(cube_indices) / sizeof(int);
	return createDynamicsObjects2(cube_vertices_textured, numVertices, cube_indices, numIndices);
}

int GpuConvexScene::createDynamicsObjects2(const float* vertices, int numVertices, const int* indices, int numIndices)
{
	int strideInBytes = 9 * sizeof(float);
	int textureIndex = -1;
	if (0)
	{
		int width, height, n;

		const char* filename = "data/cube.png";
		const unsigned char* image = 0;

		const char* prefix[] = {"./", "../", "../../", "../../../", "../../../../"};
		int numprefix = sizeof(prefix) / sizeof(const char*);

		for (int i = 0; !image && i < numprefix; i++)
		{
			char relativeFileName[1024];
			sprintf(relativeFileName, "%s%s", prefix[i], filename);
			image = loadImage(relativeFileName, width, height, n);
		}

		b3Assert(image);
		if (image)
		{
			textureIndex = m_instancingRenderer->registerTexture(image, width, height);
		}
	}

	int shapeId = m_guiHelper->getRenderInterface()->registerShape(&vertices[0], numVertices, indices, numIndices, B3_GL_TRIANGLES, textureIndex);
	//int group=1;
	//int mask=1;
	int index = 0;

	{
		int curColor = 0;
		float scaling[4] = {1, 1, 1, 1};
		int prevBody = -1;
		//int insta = 0;

		b3ConvexUtility* utilPtr = new b3ConvexUtility();

		{
			b3AlignedObjectArray<b3Vector3> verts;

			unsigned char* vts = (unsigned char*)vertices;
			for (int i = 0; i < numVertices; i++)
			{
				float* vertex = (float*)&vts[i * strideInBytes];
				verts.push_back(b3MakeVector3(vertex[0] * scaling[0], vertex[1] * scaling[1], vertex[2] * scaling[2]));
			}

			bool merge = true;
			if (numVertices)
			{
				utilPtr->initializePolyhedralFeatures(&verts[0], verts.size(), merge);
			}
		}

		int colIndex = -1;
		if (gUseInstancedCollisionShapes)
			colIndex = m_data->m_np->registerConvexHullShape(utilPtr);

		//int colIndex = m_data->m_np->registerSphereShape(1);
		for (int i = 0; i < gGpuArraySizeX; i++)
		{
			//printf("%d of %d\n", i, ci.arraySizeX);
			for (int j = 0; j < gGpuArraySizeY; j++)
			{
				for (int k = 0; k < gGpuArraySizeZ; k++)
				{
					//int colIndex = m_data->m_np->registerConvexHullShape(&vertices[0],strideInBytes,numVertices, scaling);
					if (!gUseInstancedCollisionShapes)
						colIndex = m_data->m_np->registerConvexHullShape(utilPtr);

					float mass = 1.f;
					if (j == 0)  //ci.arraySizeY-1)
					{
						//mass=0.f;
					}
					b3Vector3 position = b3MakeVector3(((j + 1) & 1) + i * 2.2, 1 + j * 2., ((j + 1) & 1) + k * 2.2);
					//b3Vector3 position = b3MakeVector3(i*2,1+j*2,k*2);
					//b3Vector3 position=b3MakeVector3(1,0.9,1);
					b3Quaternion orn(0, 0, 0, 1);

					b3Vector4 color = colors[curColor];
					curColor++;
					curColor &= 3;
					//					b3Vector4 scaling=b3MakeVector4(1,1,1,1);
					int id;
					id = m_guiHelper->getRenderInterface()->registerGraphicsInstance(shapeId, position, orn, color, scaling);
					int pid;
					pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(mass, position, orn, colIndex, index, false);

					if (prevBody >= 0)
					{
						//b3Point2PointConstraint* p2p = new b3Point2PointConstraint(pid,prevBody,b3Vector3(0,-1.1,0),b3Vector3(0,1.1,0));
						//						 m_data->m_rigidBodyPipeline->addConstraint(p2p);//,false);
					}
					prevBody = pid;

					index++;
				}
			}
		}
		delete utilPtr;
	}
	return index;
}

void GpuConvexScene::createStaticEnvironment()
{
	int strideInBytes = 9 * sizeof(float);
	int numVertices = sizeof(cube_vertices) / strideInBytes;
	int numIndices = sizeof(cube_indices) / sizeof(int);
	//int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	int shapeId = m_instancingRenderer->registerShape(&cube_vertices[0], numVertices, cube_indices, numIndices);
	//int group=1;
	//int mask=1;
	int index = 0;

	{
		b3Vector4 scaling = b3MakeVector4(400, 400, 400, 1);
		int colIndex = m_data->m_np->registerConvexHullShape(&cube_vertices[0], strideInBytes, numVertices, scaling);
		b3Vector3 position = b3MakeVector3(0, -400, 0);
		b3Quaternion orn(0, 0, 0, 1);

		b3Vector4 color = b3MakeVector4(0, 0, 1, 1);

		int id;
		id = m_instancingRenderer->registerGraphicsInstance(shapeId, position, orn, color, scaling);
		int pid;
		pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(0.f, position, orn, colIndex, index, false);
	}
}

void GpuConvexPlaneScene::createStaticEnvironment()
{
	int strideInBytes = 9 * sizeof(float);
	int numVertices = sizeof(cube_vertices) / strideInBytes;
	int numIndices = sizeof(cube_indices) / sizeof(int);
	//int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	int shapeId = m_guiHelper->getRenderInterface()->registerShape(&cube_vertices[0], numVertices, cube_indices, numIndices);
	//	int group=1;
	//	int mask=1;
	int index = 0;

	{
		b3Vector4 scaling = b3MakeVector4(400, 400, 400, 1);
		int colIndex = m_data->m_np->registerConvexHullShape(&cube_vertices[0], strideInBytes, numVertices, scaling);
		b3Vector3 position = b3MakeVector3(0, -400, 0);
		b3Quaternion orn(0, 0, 0, 1);

		b3Vector4 color = b3MakeVector4(0, 0, 1, 1);

		int id;
		id = m_guiHelper->getRenderInterface()->registerGraphicsInstance(shapeId, position, orn, color, scaling);
		int pid;
		pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(0.f, position, orn, colIndex, index, false);
	}
}

/*
void GpuConvexPlaneScene::createStaticEnvironment(const ConstructionInfo& ci)
{
	int strideInBytes = 9*sizeof(float);
	int numVertices = sizeof(cube_vertices)/strideInBytes;
	int numIndices = sizeof(cube_indices)/sizeof(int);
	//int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	int group=1;
	int mask=1;
	int index=0;

	
	{
		b3Vector4 scaling=b3MakeVector4(100,0.001,100,1);
		
	
		//int colIndex = m_data->m_np->registerConvexHullShape(&cube_vertices[0],strideInBytes,numVertices, scaling);
		b3Vector3 normal=b3MakeVector3(0,1,0);
		float constant=0.f;
		int colIndex = m_data->m_np->registerPlaneShape(normal,constant);//>registerConvexHullShape(&cube_vertices[0],strideInBytes,numVertices, scaling);
		b3Vector3 position=b3MakeVector3(0,0,0);
		

		
		b3Quaternion orn(0,0,0,1);

		b3Vector4 color=b3MakeVector4(0,0,1,1);

		int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId,position,orn,color,scaling);
		int pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(0.f,position,orn,colIndex,index,false);

	}

}
*/

struct TetraBunny
{
#include "bunny.inl"
};

struct TetraCube
{
#include "cube.inl"
};

static int nextLine(const char* buffer)
{
	int numBytesRead = 0;

	while (*buffer != '\n')
	{
		buffer++;
		numBytesRead++;
	}

	if (buffer[0] == 0x0a)
	{
		buffer++;
		numBytesRead++;
	}
	return numBytesRead;
}

static float mytetra_vertices[] =
	{
		-1.f, 0, -1.f, 0.5f, 0, 1, 0, 0, 0,
		-1.f, 0, 1.f, 0.5f, 0, 1, 0, 1, 0,
		1.f, 0, 1.f, 0.5f, 0, 1, 0, 1, 1,
		1.f, 0, -1.f, 0.5f, 0, 1, 0, 0, 1};

static int mytetra_indices[] =
	{
		0, 1, 2,
		3, 1, 2, 3, 2, 0,
		3, 0, 1};

/* Create from TetGen .ele, .face, .node data */
void GpuTetraScene::createFromTetGenData(const char* ele,
										 const char* node)
{
	b3Scalar scaling(10);

	b3AlignedObjectArray<b3Vector3> pos;
	int nnode = 0;
	int ndims = 0;
	int nattrb = 0;
	int hasbounds = 0;
	int result = sscanf(node, "%d %d %d %d", &nnode, &ndims, &nattrb, &hasbounds);
	result = sscanf(node, "%d %d %d %d", &nnode, &ndims, &nattrb, &hasbounds);
	node += nextLine(node);

	//b3AlignedObjectArray<b3Vector3> rigidBodyPositions;
	//b3AlignedObjectArray<int> rigidBodyIds;

	pos.resize(nnode);
	for (int i = 0; i < pos.size(); ++i)
	{
		int index = 0;
		//int			bound=0;
		float x, y, z;
		sscanf(node, "%d %f %f %f", &index, &x, &y, &z);

		//	sn>>index;
		//	sn>>x;sn>>y;sn>>z;
		node += nextLine(node);

		//for(int j=0;j<nattrb;++j)
		//	sn>>a;

		//if(hasbounds)
		//	sn>>bound;

		pos[index].setX(b3Scalar(x) * scaling);
		pos[index].setY(b3Scalar(y) * scaling);
		pos[index].setZ(b3Scalar(z) * scaling);
	}

	if (ele && ele[0])
	{
		int ntetra = 0;
		int ncorner = 0;
		int neattrb = 0;
		sscanf(ele, "%d %d %d", &ntetra, &ncorner, &neattrb);
		ele += nextLine(ele);

		//se>>ntetra;se>>ncorner;se>>neattrb;
		for (int i = 0; i < ntetra; ++i)
		{
			int index = 0;
			int ni[4];

			//se>>index;
			//se>>ni[0];se>>ni[1];se>>ni[2];se>>ni[3];
			sscanf(ele, "%d %d %d %d %d", &index, &ni[0], &ni[1], &ni[2], &ni[3]);
			ele += nextLine(ele);

			b3Vector3 average = b3MakeVector3(0, 0, 0);

			for (int v = 0; v < 4; v++)
			{
				average += pos[ni[v]];
			}
			average /= 4;

			for (int v = 0; v < 4; v++)
			{
				b3Vector3 shiftedPos = pos[ni[v]] - average;
				mytetra_vertices[0 + v * 9] = shiftedPos.getX();
				mytetra_vertices[1 + v * 9] = shiftedPos.getY();
				mytetra_vertices[2 + v * 9] = shiftedPos.getZ();
			}
			//todo: subtract average

			int strideInBytes = 9 * sizeof(float);
			int numVertices = sizeof(mytetra_vertices) / strideInBytes;
			int numIndices = sizeof(mytetra_indices) / sizeof(int);
			int shapeId = m_instancingRenderer->registerShape(&mytetra_vertices[0], numVertices, mytetra_indices, numIndices);
			//	int group=1;
			//	int mask=1;

			{
				b3Vector4 scaling = b3MakeVector4(1, 1, 1, 1);
				int colIndex = m_data->m_np->registerConvexHullShape(&mytetra_vertices[0], strideInBytes, numVertices, scaling);
				b3Vector3 position = b3MakeVector3(0, 150, 0);
				//				position+=average;//*1.2;//*2;
				position += average * 1.2;  //*2;
				//rigidBodyPositions.push_back(position);
				b3Quaternion orn(0, 0, 0, 1);

				static int curColor = 0;
				b3Vector4 color = colors[curColor++];
				curColor &= 3;

				int id;
				id = m_instancingRenderer->registerGraphicsInstance(shapeId, position, orn, color, scaling);
				int pid;
				pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(1.f, position, orn, colIndex, 0, false);
				//rigidBodyIds.push_back(pid);
			}

			//for(int j=0;j<neattrb;++j)
			//	se>>a;
			//psb->appendTetra(ni[0],ni[1],ni[2],ni[3]);
		}
		//	printf("Nodes:  %u\r\n",psb->m_nodes.size());
		//	printf("Links:  %u\r\n",psb->m_links.size());
		//	printf("Faces:  %u\r\n",psb->m_faces.size());
		//	printf("Tetras: %u\r\n",psb->m_tetras.size());
	}

	m_data->m_rigidBodyPipeline->writeAllInstancesToGpu();
	m_data->m_np->writeAllBodiesToGpu();
	m_data->m_bp->writeAabbsToGpu();
	m_data->m_rigidBodyPipeline->setupGpuAabbsFull();
	m_data->m_bp->calculateOverlappingPairs(m_data->m_config.m_maxBroadphasePairs);

	int numPairs = m_data->m_bp->getNumOverlap();
	cl_mem pairs = m_data->m_bp->getOverlappingPairBuffer();
	b3OpenCLArray<b3Int2> clPairs(m_clData->m_clContext, m_clData->m_clQueue);
	clPairs.setFromOpenCLBuffer(pairs, numPairs);
	b3AlignedObjectArray<b3Int2> allPairs;
	clPairs.copyToHost(allPairs);

	for (int p = 0; p < allPairs.size(); p++)
	{
		b3Vector3 posA, posB;
		b3Quaternion ornA, ornB;
		int bodyIndexA = allPairs[p].x;
		int bodyIndexB = allPairs[p].y;

		m_data->m_np->getObjectTransformFromCpu(posA, ornA, bodyIndexA);
		m_data->m_np->getObjectTransformFromCpu(posB, ornB, bodyIndexB);

		b3Vector3 pivotWorld = (posA + posB) * 0.5f;
		b3Transform transA, transB;
		transA.setIdentity();
		transA.setOrigin(posA);
		transA.setRotation(ornA);
		transB.setIdentity();
		transB.setOrigin(posB);
		transB.setRotation(ornB);
		b3Vector3 pivotInA = transA.inverse() * pivotWorld;
		b3Vector3 pivotInB = transB.inverse() * pivotWorld;

		b3Transform frameInA, frameInB;
		frameInA.setIdentity();
		frameInB.setIdentity();
		frameInA.setOrigin(pivotInA);
		frameInB.setOrigin(pivotInB);
		b3Quaternion relTargetAB = frameInA.getRotation() * frameInB.getRotation().inverse();

		//c = new b3FixedConstraint(pid,prevBody,frameInA,frameInB);
		float breakingThreshold = 45;  //37.f;
		//c->setBreakingImpulseThreshold(37.1);
		bool useGPU = true;
		if (useGPU)
		{
			int cid;
			cid = m_data->m_rigidBodyPipeline->createFixedConstraint(bodyIndexA, bodyIndexB, pivotInA, pivotInB, relTargetAB, breakingThreshold);
		}
		else
		{
			b3FixedConstraint* c = new b3FixedConstraint(bodyIndexA, bodyIndexB, frameInA, frameInB);
			c->setBreakingImpulseThreshold(breakingThreshold);
			m_data->m_rigidBodyPipeline->addConstraint(c);
		}
	}

	printf("numPairs = %d\n", numPairs);
}

int GpuTetraScene::createDynamicsObjects()
{
	//createFromTetGenData(TetraCube::getElements(),TetraCube::getNodes());
	createFromTetGenData(TetraBunny::getElements(), TetraBunny::getNodes());

	return 0;
}

class CommonExampleInterface* OpenCLBoxBoxCreateFunc(struct CommonExampleOptions& options)
{
	return new GpuBoxPlaneScene(options.m_guiHelper);
}
