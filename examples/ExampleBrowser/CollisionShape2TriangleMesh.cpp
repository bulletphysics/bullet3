
#include "CollisionShape2TriangleMesh.h"

#include "btBulletCollisionCommon.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"//to create a tesselation of a generic btConvexShape

void CollisionShape2TriangleMesh(btCollisionShape* collisionShape, const btTransform& parentTransform, btAlignedObjectArray<btVector3>& vertexPositions, btAlignedObjectArray<btVector3>& vertexNormals, btAlignedObjectArray<int>& indicesOut)

{
//todo: support all collision shape types
	switch (collisionShape->getShapeType())
	{
		case SOFTBODY_SHAPE_PROXYTYPE:
		{
			//skip the soft body collision shape for now
			break;
		}
		case STATIC_PLANE_PROXYTYPE:
		{
			//draw a box, oriented along the plane normal
			const btStaticPlaneShape* staticPlaneShape = static_cast<const btStaticPlaneShape*>(collisionShape);
			btScalar planeConst = staticPlaneShape->getPlaneConstant();
			const btVector3& planeNormal = staticPlaneShape->getPlaneNormal();
			btVector3 planeOrigin = planeNormal * planeConst;
			btVector3 vec0,vec1;
			btPlaneSpace1(planeNormal,vec0,vec1);
			btScalar vecLen = 100.f;
			btVector3 verts[4];

			verts[0] = planeOrigin + vec0*vecLen + vec1*vecLen;
			verts[1] = planeOrigin - vec0*vecLen + vec1*vecLen;
			verts[2] = planeOrigin - vec0*vecLen - vec1*vecLen;
			verts[3] = planeOrigin + vec0*vecLen - vec1*vecLen;
				
			int startIndex = vertexPositions.size();
			indicesOut.push_back(startIndex+0);
			indicesOut.push_back(startIndex+1);
			indicesOut.push_back(startIndex+2);
			indicesOut.push_back(startIndex+0);
			indicesOut.push_back(startIndex+2);
			indicesOut.push_back(startIndex+3);

			btVector3 triNormal = parentTransform.getBasis()*planeNormal;
				

			for (int i=0;i<4;i++)
			{
				btVector3 pos =parentTransform*verts[i];
				vertexPositions.push_back(pos);
				vertexNormals.push_back(triNormal);
			}
			break;
		}
		case TRIANGLE_MESH_SHAPE_PROXYTYPE:
		{
			

			btBvhTriangleMeshShape* trimesh = (btBvhTriangleMeshShape*) collisionShape;
			btVector3 trimeshScaling = trimesh->getLocalScaling();
			btStridingMeshInterface* meshInterface = trimesh->getMeshInterface();

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
					
				btVector3 triangleVerts[3];
				meshInterface->getLockedReadOnlyVertexIndexBase(&vertexbase,numverts,	type,stride,&indexbase,indexstride,numfaces,indicestype,partId);

				for (int triangleIndex = 0 ; triangleIndex < numfaces;triangleIndex++)
				{
					unsigned int* gfxbase = (unsigned int*)(indexbase+triangleIndex*indexstride);
						
					for (int j=2;j>=0;j--)
					{
							
						int graphicsindex = indicestype==PHY_SHORT?((unsigned short*)gfxbase)[j]:gfxbase[j];
						if (type == PHY_FLOAT)
						{
							float* graphicsbase = (float*)(vertexbase+graphicsindex*stride);
							triangleVerts[j] = btVector3(
															graphicsbase[0]*trimeshScaling.getX(),
															graphicsbase[1]*trimeshScaling.getY(),
															graphicsbase[2]*trimeshScaling.getZ());
						}
						else
						{
							double* graphicsbase = (double*)(vertexbase+graphicsindex*stride);
							triangleVerts[j] = btVector3( btScalar(graphicsbase[0]*trimeshScaling.getX()),
															btScalar(graphicsbase[1]*trimeshScaling.getY()),
															btScalar(graphicsbase[2]*trimeshScaling.getZ()));
						}
					}

					btVector3 triNormal = (triangleVerts[1]-triangleVerts[0]).cross(triangleVerts[2]-triangleVerts[0]);
					btScalar dot = triNormal.dot(triNormal);

					//cull degenerate triangles
					if (dot >= SIMD_EPSILON*SIMD_EPSILON)
					{
						triNormal /= btSqrt(dot);
						for (int v = 0; v < 3; v++)
						{

							btVector3 pos = parentTransform*triangleVerts[v];
							indicesOut.push_back(vertexPositions.size());
							vertexPositions.push_back(pos);
							vertexNormals.push_back(triNormal);
						}
					}
					
				}
			}
			
			break;
		}
		default:
		{
			if (collisionShape->isConvex())
			{
				btConvexShape* convex = (btConvexShape*)collisionShape;
				{
					btShapeHull* hull = new btShapeHull(convex);
					hull->buildHull(0.0);

					{
						//int strideInBytes = 9*sizeof(float);
						//int numVertices = hull->numVertices();
						//int numIndices =hull->numIndices();

						for (int t=0;t<hull->numTriangles();t++)
						{

							btVector3 triNormal;

							int index0 = hull->getIndexPointer()[t*3+0];
							int index1 = hull->getIndexPointer()[t*3+1];
							int index2 = hull->getIndexPointer()[t*3+2];
							btVector3 pos0 =parentTransform*hull->getVertexPointer()[index0];
							btVector3 pos1 =parentTransform*hull->getVertexPointer()[index1];
							btVector3 pos2 =parentTransform*hull->getVertexPointer()[index2];
							triNormal = (pos1-pos0).cross(pos2-pos0);
							triNormal.normalize();

							for (int v=0;v<3;v++)
							{
								int index = hull->getIndexPointer()[t*3+v];
								btVector3 pos =parentTransform*hull->getVertexPointer()[index];
								indicesOut.push_back(vertexPositions.size());
								vertexPositions.push_back(pos);
								vertexNormals.push_back(triNormal);
							}
						}
					}
					delete hull;
				}
			} else
			{
				if (collisionShape->isCompound())
				{
					btCompoundShape* compound = (btCompoundShape*) collisionShape;
					for (int i=0;i<compound->getNumChildShapes();i++)
					{

						btTransform childWorldTrans = parentTransform * compound->getChildTransform(i);
						CollisionShape2TriangleMesh(compound->getChildShape(i),childWorldTrans,vertexPositions,vertexNormals,indicesOut);
					}
                } else if (collisionShape->isConcave())
                {
                    btConcaveShape* concaveShape = (btConcaveShape*) collisionShape;

                    struct MyTriangleCallback : public btTriangleCallback {
                    public:

                        const btTransform& mParentTransform;
                        btAlignedObjectArray<btVector3>& mVertexPositions;
                        btAlignedObjectArray<btVector3>& mVertexNormals;
                        btAlignedObjectArray<int>& mIndicesOut;

                        MyTriangleCallback(const btTransform& parentTransform, btAlignedObjectArray<btVector3>& vertexPositions, btAlignedObjectArray<btVector3>& vertexNormals, btAlignedObjectArray<int>& indicesOut)
                                : mParentTransform(parentTransform), mVertexPositions(vertexPositions), mVertexNormals(vertexNormals), mIndicesOut(indicesOut)
                        {
                        }

                        virtual void processTriangle(btVector3* triangle, int partId, int triangleIndex) {
                            btVector3 triNormal = (triangle[1]-triangle[0]).cross(triangle[2]-triangle[0]);
                            btScalar dot = triNormal.dot(triNormal);

                            //cull degenerate triangles
                            if (dot >= SIMD_EPSILON*SIMD_EPSILON)
                            {
                                triNormal /= btSqrt(dot);
                                for (int v = 0; v < 3; v++)
                                {

                                    btVector3 pos = mParentTransform*triangle[v];
                                    mIndicesOut.push_back(mVertexPositions.size());
                                    mVertexPositions.push_back(pos);
                                    mVertexNormals.push_back(triNormal);
                                }
                            }
                        }
                    };

                    btVector3 aabbMin, aabbMax;

                    concaveShape->getAabb(btTransform::getIdentity(), aabbMin, aabbMax);

                    MyTriangleCallback callback(parentTransform, vertexPositions, vertexNormals, indicesOut);

                    concaveShape->processAllTriangles(&callback, aabbMin, aabbMax);
				} else
				{
					btAssert(0);
				}
					
			}
		}
	};
}


