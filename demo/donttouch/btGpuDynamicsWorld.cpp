#include "btGpuDynamicsWorld.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

#include "../../../opencl/gpu_rigidbody_pipeline2/CLPhysicsDemo.h"
#include "../../../opencl/gpu_rigidbody_pipeline/btGpuNarrowPhaseAndSolver.h"
#include "BulletCollision/CollisionShapes/btPolyhedralConvexShape.h"
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btStaticPlaneShape.h"

#include "LinearMath/b3Quickprof.h"


#ifdef _WIN32
	#include <wiNdOws.h>
#endif



btGpuDynamicsWorld::btGpuDynamicsWorld(int preferredOpenCLPlatformIndex,int preferredOpenCLDeviceIndex)
:btDynamicsWorld(0,0,0),
m_gravity(0,-10,0),
m_once(true)
{
	m_gpuPhysics = new CLPhysicsDemo(512*1024, MAX_CONVEX_BODIES_CL);
	bool useInterop = false;
	///platform and device are swapped, todo: fix this and make it consistent
	m_gpuPhysics->init(preferredOpenCLDeviceIndex,preferredOpenCLPlatformIndex,useInterop);
}

btGpuDynamicsWorld::~btGpuDynamicsWorld()
{
	delete m_gpuPhysics;
}

void btGpuDynamicsWorld::exitOpenCL()
{
}






int		btGpuDynamicsWorld::stepSimulation( b3Scalar timeStep,int maxSubSteps, b3Scalar fixedTimeStep)
{
#ifndef BT_NO_PROFILE
//	CProfileManager::Reset();
#endif //BT_NO_PROFILE

	BT_PROFILE("stepSimulation");

	//convert all shapes now, and if any change, reset all (todo)
	
	if (m_once)
	{
		m_once = false;
		m_gpuPhysics->writeBodiesToGpu();
	}

	m_gpuPhysics->stepSimulation();

	{
		{
			BT_PROFILE("readbackBodiesToCpu");
			//now copy info back to rigid bodies....
			m_gpuPhysics->readbackBodiesToCpu();
		}

		
		{
			BT_PROFILE("scatter transforms into rigidbody (CPU)");
			for (int i=0;i<this->m_collisionObjects.size();i++)
			{
				b3Vector3 pos;
				b3Quaternion orn;
				m_gpuPhysics->getObjectTransformFromCpu(&pos[0],&orn[0],i);
				b3Transform newTrans;
				newTrans.setOrigin(pos);
				newTrans.setRotation(orn);
				this->m_collisionObjects[i]->setWorldTransform(newTrans);
			}
		}
	}


#ifndef BT_NO_PROFILE
	//CProfileManager::Increment_Frame_Counter();
#endif //BT_NO_PROFILE


	return 1;
}


void	btGpuDynamicsWorld::setGravity(const b3Vector3& gravity)
{
}

int btGpuDynamicsWorld::findOrRegisterCollisionShape(const btCollisionShape* colShape)
{
	int index = m_uniqueShapes.findLinearSearch(colShape);
	if (index==m_uniqueShapes.size())
	{
		if (colShape->isPolyhedral())
		{
			m_uniqueShapes.push_back(colShape);
			
			btPolyhedralConvexShape* convex = (btPolyhedralConvexShape*)colShape;
			int numVertices=convex->getNumVertices();
			
			int strideInBytes=sizeof(b3Vector3);
			b3AlignedObjectArray<b3Vector3> tmpVertices;
			tmpVertices.resize(numVertices);
			for (int i=0;i<numVertices;i++)
				convex->getVertex(i,tmpVertices[i]);
			const float scaling[4]={1,1,1,1};
			bool noHeightField=true;
			
			int gpuShapeIndex = m_gpuPhysics->registerConvexPolyhedron(&tmpVertices[0].getX(), strideInBytes, numVertices, scaling, noHeightField);
			m_uniqueShapeMapping.push_back(gpuShapeIndex);
		} else
		{
			if (colShape->getShapeType()==TRIANGLE_MESH_SHAPE_PROXYTYPE)
			{
				m_uniqueShapes.push_back(colShape);
				
				btBvhTriangleMeshShape* trimesh = (btBvhTriangleMeshShape*) colShape;
				b3StridingMeshInterface* meshInterface = trimesh->getMeshInterface();
				b3AlignedObjectArray<b3Vector3> vertices;
				b3AlignedObjectArray<int> indices;
				
				b3Vector3 trimeshScaling(1,1,1);
				for (int partId=0;partId<meshInterface->getNumSubParts();partId++)
				{
					
					const unsigned char *vertexbase = 0;
					int numverts = 0;
					PHY_ScalarType type = PHY_INTEGER;
					int stride = 0;
					const unsigned char *indexbase = 0;
					int indexstride = 0;
					int numfaces = 0;
					PHY_ScalarType indicestype = PHY_INTEGER;
					//PHY_ScalarType indexType=0;
					
					b3Vector3 triangleVerts[3];
					meshInterface->getLockedReadOnlyVertexIndexBase(&vertexbase,numverts,	type,stride,&indexbase,indexstride,numfaces,indicestype,partId);
					b3Vector3 aabbMin,aabbMax;
					
					for (int triangleIndex = 0 ; triangleIndex < numfaces;triangleIndex++)
					{
						unsigned int* gfxbase = (unsigned int*)(indexbase+triangleIndex*indexstride);
						
						for (int j=2;j>=0;j--)
						{
							
							int graphicsindex = indicestype==PHY_SHORT?((unsigned short*)gfxbase)[j]:gfxbase[j];
							if (type == PHY_FLOAT)
							{
								float* graphicsbase = (float*)(vertexbase+graphicsindex*stride);
								triangleVerts[j] = b3Vector3(
															 graphicsbase[0]*trimeshScaling.getX(),
															 graphicsbase[1]*trimeshScaling.getY(),
															 graphicsbase[2]*trimeshScaling.getZ());
							}
							else
							{
								double* graphicsbase = (double*)(vertexbase+graphicsindex*stride);
								triangleVerts[j] = b3Vector3( b3Scalar(graphicsbase[0]*trimeshScaling.getX()),
															 b3Scalar(graphicsbase[1]*trimeshScaling.getY()),
															 b3Scalar(graphicsbase[2]*trimeshScaling.getZ()));
							}
						}
						vertices.push_back(triangleVerts[0]);
						vertices.push_back(triangleVerts[1]);
						vertices.push_back(triangleVerts[2]);
						indices.push_back(indices.size());
						indices.push_back(indices.size());
						indices.push_back(indices.size());
					}
				}
				//GraphicsShape* gfxShape = 0;//btBulletDataExtractor::createGraphicsShapeFromWavefrontObj(objData);
				
				//GraphicsShape* gfxShape = btBulletDataExtractor::createGraphicsShapeFromConvexHull(&sUnitSpherePoints[0],MY_UNITSPHERE_POINTS);
				float meshScaling[4] = {1,1,1,1};
				//int shapeIndex = renderer.registerShape(gfxShape->m_vertices,gfxShape->m_numvertices,gfxShape->m_indices,gfxShape->m_numIndices);
				float groundPos[4] = {0,0,0,0};
				
				//renderer.registerGraphicsInstance(shapeIndex,groundPos,rotOrn,color,meshScaling);
				if (vertices.size() && indices.size())
				{
					int gpuShapeIndex = m_gpuPhysics->registerConcaveMesh(&vertices,&indices, meshScaling);
					m_uniqueShapeMapping.push_back(gpuShapeIndex);
				} else
				{
					printf("Error: no vertices in mesh in btGpuDynamicsWorld::addRigidBody\n");
					index = -1;
					btAssert(0);
				}
				
				
			} else
			{
				if (colShape->getShapeType()==COMPOUND_SHAPE_PROXYTYPE)
				{
					
					btCompoundShape* compound = (btCompoundShape*) colShape;
					b3AlignedObjectArray<btGpuChildShape> childShapes;
					
					for (int i=0;i<compound->getNumChildShapes();i++)
					{
						//for now, only support polyhedral child shapes
						btAssert(compound->getChildShape(i)->isPolyhedral());
						btGpuChildShape child;
						child.m_shapeIndex = findOrRegisterCollisionShape(compound->getChildShape(i));
						b3Vector3 pos = compound->getChildTransform(i).getOrigin();
						b3Quaternion orn = compound->getChildTransform(i).getRotation();
						for (int v=0;v<4;v++)
						{
							child.m_childPosition[v] = pos[v];
							child.m_childOrientation[v] = orn[v];
						}
						childShapes.push_back(child);
					}
					index = m_uniqueShapes.size();
					m_uniqueShapes.push_back(colShape);
					
					int gpuShapeIndex = m_gpuPhysics->registerCompoundShape(&childShapes);
					m_uniqueShapeMapping.push_back(gpuShapeIndex);

					
					
					
					/*printf("Error: unsupported compound type (%d) in btGpuDynamicsWorld::addRigidBody\n",colShape->getShapeType());
					index = -1;
					btAssert(0);
					 */
				} else
				{
					if (colShape->getShapeType()==SPHERE_SHAPE_PROXYTYPE)
					{
						m_uniqueShapes.push_back(colShape);
						btSphereShape* sphere = (btSphereShape*)colShape;
						
						int gpuShapeIndex = m_gpuPhysics->registerSphereShape(sphere->getRadius());
						m_uniqueShapeMapping.push_back(gpuShapeIndex);
					} else
					{
						if (colShape->getShapeType()==STATIC_PLANE_PROXYTYPE)
						{
							m_uniqueShapes.push_back(colShape);
							btStaticPlaneShape* plane = (btStaticPlaneShape*)colShape;
						
							int gpuShapeIndex = m_gpuPhysics->registerPlaneShape(plane->getPlaneNormal(),plane->getPlaneConstant());
							m_uniqueShapeMapping.push_back(gpuShapeIndex);
						} else
						{
							printf("Error: unsupported shape type (%d) in btGpuDynamicsWorld::addRigidBody\n",colShape->getShapeType());
							index = -1;
							btAssert(0);
						}
					}
				}
			}
		}
		
	}
	
	return index;
}

void	btGpuDynamicsWorld::addRigidBody(btRigidBody* body)
{

	body->setMotionState(0);
	

	int index = findOrRegisterCollisionShape(body->getCollisionShape());

	if (index>=0)
	{
		int gpuShapeIndex= m_uniqueShapeMapping[index];
		float mass = body->getInvMass() ? 1.f/body->getInvMass() : 0.f;
		b3Vector3 pos = body->getWorldTransform().getOrigin();
		b3Quaternion orn = body->getWorldTransform().getRotation();
	
		m_gpuPhysics->registerPhysicsInstance(mass,&pos.getX(),&orn.getX(),gpuShapeIndex,m_collisionObjects.size());

		m_collisionObjects.push_back(body);
	}
}

void	btGpuDynamicsWorld::removeCollisionObject(btCollisionObject* colObj)
{
	btDynamicsWorld::removeCollisionObject(colObj);
}


