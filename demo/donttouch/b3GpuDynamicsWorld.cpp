#include "b3GpuDynamicsWorld.h"
#include "BulletDynamics/Dynamics/b3RigidBody.h"

#include "../../../opencl/gpu_rigidbody_pipeline2/CLPhysicsDemo.h"
#include "../../../opencl/gpu_rigidbody_pipeline/b3GpuNarrowPhaseAndSolver.h"
#include "BulletCollision/CollisionShapes/b3PolyhedralConvexShape.h"
#include "BulletCollision/CollisionShapes/b3BvhTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/b3CompoundShape.h"
#include "BulletCollision/CollisionShapes/b3SphereShape.h"
#include "BulletCollision/CollisionShapes/b3StaticPlaneShape.h"

#include "LinearMath/b3Quickprof.h"


#ifdef _WIN32
	#include <wiNdOws.h>
#endif



b3GpuDynamicsWorld::b3GpuDynamicsWorld(int preferredOpenCLPlatformIndex,int preferredOpenCLDeviceIndex)
:b3DynamicsWorld(0,0,0),
m_gravity(0,-10,0),
m_once(true)
{
	m_gpuPhysics = new CLPhysicsDemo(512*1024, MAX_CONVEX_BODIES_CL);
	bool useInterop = false;
	///platform and device are swapped, todo: fix this and make it consistent
	m_gpuPhysics->init(preferredOpenCLDeviceIndex,preferredOpenCLPlatformIndex,useInterop);
}

b3GpuDynamicsWorld::~b3GpuDynamicsWorld()
{
	delete m_gpuPhysics;
}

void b3GpuDynamicsWorld::exitOpenCL()
{
}






int		b3GpuDynamicsWorld::stepSimulation( b3Scalar timeStep,int maxSubSteps, b3Scalar fixedTimeStep)
{
#ifndef B3_NO_PROFILE
//	b3ProfileManager::Reset();
#endif //B3_NO_PROFILE

	B3_PROFILE("stepSimulation");

	//convert all shapes now, and if any change, reset all (todo)
	
	if (m_once)
	{
		m_once = false;
		m_gpuPhysics->writeBodiesToGpu();
	}

	m_gpuPhysics->stepSimulation();

	{
		{
			B3_PROFILE("readbackBodiesToCpu");
			//now copy info back to rigid bodies....
			m_gpuPhysics->readbackBodiesToCpu();
		}

		
		{
			B3_PROFILE("scatter transforms into rigidbody (CPU)");
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


#ifndef B3_NO_PROFILE
	//b3ProfileManager::Increment_Frame_Counter();
#endif //B3_NO_PROFILE


	return 1;
}


void	b3GpuDynamicsWorld::setGravity(const b3Vector3& gravity)
{
}

int b3GpuDynamicsWorld::findOrRegisterCollisionShape(const b3CollisionShape* colShape)
{
	int index = m_uniqueShapes.findLinearSearch(colShape);
	if (index==m_uniqueShapes.size())
	{
		if (colShape->isPolyhedral())
		{
			m_uniqueShapes.push_back(colShape);
			
			b3PolyhedralConvexShape* convex = (b3PolyhedralConvexShape*)colShape;
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
				
				b3BvhTriangleMeshShape* trimesh = (b3BvhTriangleMeshShape*) colShape;
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
				//GraphicsShape* gfxShape = 0;//b3BulletDataExtractor::createGraphicsShapeFromWavefrontObj(objData);
				
				//GraphicsShape* gfxShape = b3BulletDataExtractor::createGraphicsShapeFromConvexHull(&sUnitSpherePoints[0],MY_UNITSPHERE_POINTS);
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
					printf("Error: no vertices in mesh in b3GpuDynamicsWorld::addRigidBody\n");
					index = -1;
					b3Assert(0);
				}
				
				
			} else
			{
				if (colShape->getShapeType()==COMPOUND_SHAPE_PROXYTYPE)
				{
					
					b3CompoundShape* compound = (b3CompoundShape*) colShape;
					b3AlignedObjectArray<b3GpuChildShape> childShapes;
					
					for (int i=0;i<compound->getNumChildShapes();i++)
					{
						//for now, only support polyhedral child shapes
						b3Assert(compound->getChildShape(i)->isPolyhedral());
						b3GpuChildShape child;
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

					
					
					
					/*printf("Error: unsupported compound type (%d) in b3GpuDynamicsWorld::addRigidBody\n",colShape->getShapeType());
					index = -1;
					b3Assert(0);
					 */
				} else
				{
					if (colShape->getShapeType()==SPHERE_SHAPE_PROXYTYPE)
					{
						m_uniqueShapes.push_back(colShape);
						b3SphereShape* sphere = (b3SphereShape*)colShape;
						
						int gpuShapeIndex = m_gpuPhysics->registerSphereShape(sphere->getRadius());
						m_uniqueShapeMapping.push_back(gpuShapeIndex);
					} else
					{
						if (colShape->getShapeType()==STATIC_PLANE_PROXYTYPE)
						{
							m_uniqueShapes.push_back(colShape);
							b3StaticPlaneShape* plane = (b3StaticPlaneShape*)colShape;
						
							int gpuShapeIndex = m_gpuPhysics->registerPlaneShape(plane->getPlaneNormal(),plane->getPlaneConstant());
							m_uniqueShapeMapping.push_back(gpuShapeIndex);
						} else
						{
							printf("Error: unsupported shape type (%d) in b3GpuDynamicsWorld::addRigidBody\n",colShape->getShapeType());
							index = -1;
							b3Assert(0);
						}
					}
				}
			}
		}
		
	}
	
	return index;
}

void	b3GpuDynamicsWorld::addRigidBody(b3RigidBody* body)
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

void	b3GpuDynamicsWorld::removeCollisionObject(b3CollisionObject* colObj)
{
	b3DynamicsWorld::removeCollisionObject(colObj);
}


