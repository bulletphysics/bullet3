#include "GpuCompoundScene.h"
#include "GpuRigidBodyDemo.h"
#include "OpenGLWindow/ShapeData.h"

#include "OpenGLWindow/GLInstancingRenderer.h"
#include "Bullet3Common/b3Quaternion.h"
#include "OpenGLWindow/b3gWindowInterface.h"
#include "Bullet3OpenCL/BroadphaseCollision/b3GpuSapBroadphase.h"
#include "../GpuDemoInternalData.h"
#include "Bullet3OpenCL/Initialize/b3OpenCLUtils.h"
#include "OpenGLWindow/OpenGLInclude.h"
#include "OpenGLWindow/GLInstanceRendererInternalData.h"
#include "Bullet3OpenCL/ParallelPrimitives/b3LauncherCL.h"
#include "Bullet3OpenCL/RigidBody/b3GpuRigidBodyPipeline.h"
#include "Bullet3OpenCL/RigidBody/b3GpuNarrowPhase.h"
#include "Bullet3Collision/NarrowPhaseCollision/b3Config.h"
#include "GpuRigidBodyDemoInternalData.h"
#include "Bullet3Common/b3Transform.h"

#include "OpenGLWindow/GLInstanceGraphicsShape.h"

#define NUM_COMPOUND_CHILDREN_X 4
#define NUM_COMPOUND_CHILDREN_Y 4
#define NUM_COMPOUND_CHILDREN_Z 4

void GpuCompoundScene::setupScene(const ConstructionInfo& ci)
{
	createStaticEnvironment(ci);

	int strideInBytes = 9 * sizeof(float);
	int numVertices = sizeof(cube_vertices) / strideInBytes;
	int numIndices = sizeof(cube_indices) / sizeof(int);
	float scaling[4] = {1, 1, 1, 1};

	GLInstanceVertex* cubeVerts = (GLInstanceVertex*)&cube_vertices[0];
	int stride2 = sizeof(GLInstanceVertex);
	b3Assert(stride2 == strideInBytes);
	int index = 0;
	int colIndex = -1;
	b3AlignedObjectArray<GLInstanceVertex> vertexArray;
	b3AlignedObjectArray<int> indexArray;
	{
		int childColIndex = m_data->m_np->registerConvexHullShape(&cube_vertices[0], strideInBytes, numVertices, scaling);

		/*		b3Vector3 childPositions[3] = {
			b3Vector3(0,-2,0),
			b3Vector3(0,0,0),
			b3Vector3(0,0,2)
		};
		*/

		b3AlignedObjectArray<b3GpuChildShape> childShapes;

		for (int x = 0; x < NUM_COMPOUND_CHILDREN_X; x++)
			for (int y = 0; y < NUM_COMPOUND_CHILDREN_Y; y++)
				for (int z = 0; z < NUM_COMPOUND_CHILDREN_Z; z++)
				{
					int blax = x != 0 ? 1 : 0;
					int blay = y != 0 ? 1 : 0;
					int blaz = z != 0 ? 1 : 0;
					int bla = blax + blay + blaz;
					if (bla != 1)
						continue;

					//for now, only support polyhedral child shapes
					b3GpuChildShape child;
					child.m_shapeIndex = childColIndex;
					b3Vector3 pos = b3MakeVector3((x - NUM_COMPOUND_CHILDREN_X / 2.f) * 2, (y - NUM_COMPOUND_CHILDREN_X / 2.f) * 2, (z - NUM_COMPOUND_CHILDREN_X / 2.f) * 2);  //childPositions[i];
					b3Quaternion orn(0, 0, 0, 1);
					for (int v = 0; v < 4; v++)
					{
						child.m_childPosition[v] = pos[v];
						child.m_childOrientation[v] = orn[v];
					}
					childShapes.push_back(child);
					b3Transform tr;
					tr.setIdentity();
					tr.setOrigin(pos);
					tr.setRotation(orn);

					int baseIndex = vertexArray.size();
					for (int j = 0; j < numIndices; j++)
						indexArray.push_back(cube_indices[j] + baseIndex);

					//add transformed graphics vertices and indices
					for (int v = 0; v < numVertices; v++)
					{
						GLInstanceVertex vert = cubeVerts[v];
						b3Vector3 vertPos = b3MakeVector3(vert.xyzw[0], vert.xyzw[1], vert.xyzw[2]);
						b3Vector3 newPos = tr * vertPos;
						vert.xyzw[0] = newPos[0];
						vert.xyzw[1] = newPos[1];
						vert.xyzw[2] = newPos[2];
						vert.xyzw[3] = 0.f;
						vertexArray.push_back(vert);
					}
				}
		colIndex = m_data->m_np->registerCompoundShape(&childShapes);
	}

	//int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	int shapeId = ci.m_instancingRenderer->registerShape(&vertexArray[0].xyzw[0], vertexArray.size(), &indexArray[0], indexArray.size());

	b3Vector4 colors[4] =
		{
			b3MakeVector4(1, 0, 0, 1),
			b3MakeVector4(0, 1, 0, 1),
			b3MakeVector4(0, 0, 1, 1),
			b3MakeVector4(0, 1, 1, 1),
		};

	int curColor = 0;
	for (int i = 0; i < ci.arraySizeX; i++)
	{
		for (int j = 0; j < ci.arraySizeY; j++)
		{
			for (int k = 0; k < ci.arraySizeZ; k++)
			{
				float mass = 1;  //j==0? 0.f : 1.f;

				b3Vector3 position = b3MakeVector3((i - ci.arraySizeX / 2.) * ci.gapX, 35 + j * 3 * ci.gapY, (k - ci.arraySizeZ / 2.f) * ci.gapZ);
				//b3Quaternion orn(0,0,0,1);
				b3Quaternion orn(b3MakeVector3(1, 0, 0), 0.7);

				b3Vector4 color = colors[curColor];
				curColor++;
				curColor &= 3;
				b3Vector4 scaling = b3MakeVector4(1, 1, 1, 1);
				int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId, position, orn, color, scaling);
				int pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(mass, position, orn, colIndex, index, false);

				index++;
			}
		}
	}

	m_data->m_rigidBodyPipeline->writeAllInstancesToGpu();

	float camPos[4] = {0, 0, 0};  //65.5,4.5,65.5,0};
	//float camPos[4]={1,12.5,1.5,0};
	m_instancingRenderer->setCameraTargetPosition(camPos);
	m_instancingRenderer->setCameraDistance(320);
}

void GpuCompoundScene::createStaticEnvironment(const ConstructionInfo& ci)
{
	int strideInBytes = 9 * sizeof(float);

	//int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	int group = 1;
	int mask = 1;
	int index = 0;
	int colIndex = 0;

	{
		if (1)
		{
			float radius = 41;
			int prevGraphicsShapeIndex = -1;
			{
				if (radius >= 100)
				{
					int numVertices = sizeof(detailed_sphere_vertices) / strideInBytes;
					int numIndices = sizeof(detailed_sphere_indices) / sizeof(int);
					prevGraphicsShapeIndex = ci.m_instancingRenderer->registerShape(&detailed_sphere_vertices[0], numVertices, detailed_sphere_indices, numIndices);
				}
				else
				{
					bool usePointSprites = false;
					if (usePointSprites)
					{
						int numVertices = sizeof(point_sphere_vertices) / strideInBytes;
						int numIndices = sizeof(point_sphere_indices) / sizeof(int);
						prevGraphicsShapeIndex = ci.m_instancingRenderer->registerShape(&point_sphere_vertices[0], numVertices, point_sphere_indices, numIndices, B3_GL_POINTS);
					}
					else
					{
						if (radius >= 10)
						{
							int numVertices = sizeof(medium_sphere_vertices) / strideInBytes;
							int numIndices = sizeof(medium_sphere_indices) / sizeof(int);
							prevGraphicsShapeIndex = ci.m_instancingRenderer->registerShape(&medium_sphere_vertices[0], numVertices, medium_sphere_indices, numIndices);
						}
						else
						{
							int numVertices = sizeof(low_sphere_vertices) / strideInBytes;
							int numIndices = sizeof(low_sphere_indices) / sizeof(int);
							prevGraphicsShapeIndex = ci.m_instancingRenderer->registerShape(&low_sphere_vertices[0], numVertices, low_sphere_indices, numIndices);
						}
					}
				}
			}
			b3Vector4 colors[4] =
				{
					b3MakeVector4(1, 0, 0, 1),
					b3MakeVector4(0, 1, 0, 1),
					b3MakeVector4(0, 1, 1, 1),
					b3MakeVector4(1, 1, 0, 1),
				};

			int curColor = 1;

			//int colIndex = m_data->m_np->registerConvexHullShape(&cube_vertices[0],strideInBytes,numVertices, scaling);
			int colIndex = m_data->m_np->registerSphereShape(radius);  //>registerConvexHullShape(&cube_vertices[0],strideInBytes,numVertices, scaling);
			float mass = 0.f;

			//b3Vector3 position((j&1)+i*2.2,1+j*2.,(j&1)+k*2.2);
			b3Vector3 position = b3MakeVector3(0, -41, 0);

			b3Quaternion orn(0, 0, 0, 1);

			b3Vector4 color = colors[curColor];
			curColor++;
			curColor &= 3;
			b3Vector4 scaling = b3MakeVector4(radius, radius, radius, 1);
			int id = ci.m_instancingRenderer->registerGraphicsInstance(prevGraphicsShapeIndex, position, orn, color, scaling);
			int pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(mass, position, orn, colIndex, index, false);

			index++;
		}
	}
}

void GpuCompoundPlaneScene::createStaticEnvironment(const ConstructionInfo& ci)
{
	int index = 0;
	b3Vector3 normal = b3MakeVector3(0, 1, 0);
	float constant = 0.f;
	int strideInBytes = 9 * sizeof(float);
	int numVertices = sizeof(cube_vertices) / strideInBytes;
	int numIndices = sizeof(cube_indices) / sizeof(int);

	b3Vector4 scaling = b3MakeVector4(400, 1., 400, 1);

	//int colIndex = m_data->m_np->registerPlaneShape(normal,constant);//>registerConvexHullShape(&cube_vertices[0],strideInBytes,numVertices, scaling);
	int colIndex = m_data->m_np->registerConvexHullShape(&cube_vertices[0], strideInBytes, numVertices, scaling);
	b3Vector3 position = b3MakeVector3(0, 0, 0);
	b3Quaternion orn(0, 0, 0, 1);
	//		b3Quaternion orn(b3Vector3(1,0,0),0.3);
	b3Vector4 color = b3MakeVector4(0, 0, 1, 1);

	int shapeId = ci.m_instancingRenderer->registerShape(&cube_vertices[0], numVertices, cube_indices, numIndices);

	int id = ci.m_instancingRenderer->registerGraphicsInstance(shapeId, position, orn, color, scaling);
	int pid = m_data->m_rigidBodyPipeline->registerPhysicsInstance(0.f, position, orn, colIndex, index, false);
}