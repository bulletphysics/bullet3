/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "../CommonInterfaces/CommonExampleInterface.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "../ExampleBrowser/CollisionShape2TriangleMesh.h"
#include "../OpenGLWindow/GLInstanceGraphicsShape.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btHashMap.h"

#include "../TinyRenderer/TinyRenderer.h"
#include "../OpenGLWindow/SimpleCamera.h"

static btVector4 sMyColors[4] =
	{
		btVector4(0.3, 0.3, 1, 1),
		btVector4(0.6, 0.6, 1, 1),
		btVector4(0, 1, 0, 1),
		btVector4(0, 1, 1, 1),
		//btVector4(1,1,0,1),
};

struct TinyRendererTexture
{
	const unsigned char* m_texels;
	int m_width;
	int m_height;
};

struct TinyRendererGUIHelper : public GUIHelperInterface
{
	int m_upAxis;

	btHashMap<btHashInt, TinyRenderObjectData*> m_swRenderObjects;
	btHashMap<btHashInt, int> m_swInstances;
	btHashMap<btHashInt, TinyRendererTexture> m_textures;

	int m_swWidth;
	int m_swHeight;
	TGAImage m_rgbColorBuffer;

	b3AlignedObjectArray<float> m_depthBuffer;

	SimpleCamera m_camera;
	int m_colObjUniqueIndex;

	TinyRendererGUIHelper(int swWidth, int swHeight)
		: m_upAxis(1),
		  m_swWidth(swWidth),
		  m_swHeight(swHeight),
		  m_rgbColorBuffer(swWidth, swHeight, TGAImage::RGB),
		  m_colObjUniqueIndex(0)
	{
		m_depthBuffer.resize(swWidth * swHeight);
	}

	void clearBuffers(TGAColor& clearColor)
	{
		for (int y = 0; y < m_swHeight; ++y)
		{
			for (int x = 0; x < m_swWidth; ++x)
			{
				m_rgbColorBuffer.set(x, y, clearColor);
				m_depthBuffer[x + y * m_swWidth] = -1e30f;
			}
		}
	}

	const TGAImage& getFrameBuffer() const
	{
		return m_rgbColorBuffer;
	}

	virtual ~TinyRendererGUIHelper()
	{
		for (int i = 0; i < m_swRenderObjects.size(); i++)
		{
			TinyRenderObjectData** d = m_swRenderObjects[i];
			if (d && *d)
			{
				delete *d;
			}
		}
	}

	virtual void createRigidBodyGraphicsObject(btRigidBody* body, const btVector3& color) {}

	virtual void createCollisionObjectGraphicsObject(btCollisionObject* obj, const btVector3& color)
	{
		if (obj->getUserIndex() < 0)
		{
			int colIndex = m_colObjUniqueIndex++;
			obj->setUserIndex(colIndex);
			int shapeIndex = obj->getCollisionShape()->getUserIndex();

			if (colIndex >= 0 && shapeIndex >= 0)
			{
				m_swInstances.insert(colIndex, shapeIndex);
			}
		}
	}

	virtual void createCollisionShapeGraphicsObject(btCollisionShape* collisionShape)
	{
		//already has a graphics object?
		if (collisionShape->getUserIndex() >= 0)
			return;

		btAlignedObjectArray<GLInstanceVertex> gfxVertices;

		btAlignedObjectArray<int> indices;
		btTransform startTrans;
		startTrans.setIdentity();

		{
			btAlignedObjectArray<btVector3> vertexPositions;
			btAlignedObjectArray<btVector3> vertexNormals;
			CollisionShape2TriangleMesh(collisionShape, startTrans, vertexPositions, vertexNormals, indices);
			gfxVertices.resize(vertexPositions.size());
			for (int i = 0; i < vertexPositions.size(); i++)
			{
				for (int j = 0; j < 4; j++)
				{
					gfxVertices[i].xyzw[j] = vertexPositions[i][j];
				}
				for (int j = 0; j < 3; j++)
				{
					gfxVertices[i].normal[j] = vertexNormals[i][j];
				}
				for (int j = 0; j < 2; j++)
				{
					gfxVertices[i].uv[j] = 0.5;  //we don't have UV info...
				}
			}
		}

		if (gfxVertices.size() && indices.size())
		{
			int shapeId = registerGraphicsShape(&gfxVertices[0].xyzw[0], gfxVertices.size(), &indices[0], indices.size(),
												1, -1);
			collisionShape->setUserIndex(shapeId);
		}
	}

	virtual void syncPhysicsToGraphics(const btDiscreteDynamicsWorld* rbWorld) {}

	virtual void render(const btDiscreteDynamicsWorld* rbWorld)
	{
		//clear the color buffer
		TGAColor clearColor;
		clearColor.bgra[0] = 255;
		clearColor.bgra[1] = 255;
		clearColor.bgra[2] = 255;
		clearColor.bgra[3] = 255;

		clearBuffers(clearColor);

		ATTRIBUTE_ALIGNED16(btScalar modelMat[16]);
		ATTRIBUTE_ALIGNED16(float viewMat[16]);
		ATTRIBUTE_ALIGNED16(float projMat[16]);

		m_camera.getCameraProjectionMatrix(projMat);
		m_camera.getCameraViewMatrix(viewMat);

		btVector3 lightDirWorld(-5, 200, -40);
		switch (m_upAxis)
		{
			case 1:
				lightDirWorld = btVector3(-50.f, 100, 30);
				break;
			case 2:
				lightDirWorld = btVector3(-50.f, 30, 100);
				break;
			default:
			{
			}
		};

		lightDirWorld.normalize();

		for (int i = 0; i < rbWorld->getNumCollisionObjects(); i++)
		{
			btCollisionObject* colObj = rbWorld->getCollisionObjectArray()[i];
			int colObjIndex = colObj->getUserIndex();
			int shapeIndex = colObj->getCollisionShape()->getUserIndex();
			if (colObjIndex >= 0 && shapeIndex >= 0)
			{
				TinyRenderObjectData* renderObj = 0;

				int* cptr = m_swInstances[colObjIndex];
				if (cptr)
				{
					int c = *cptr;
					TinyRenderObjectData** sptr = m_swRenderObjects[c];
					if (sptr)
					{
						renderObj = *sptr;
						//sync the object transform
						const btTransform& tr = colObj->getWorldTransform();
						tr.getOpenGLMatrix(modelMat);

						for (int i = 0; i < 4; i++)
						{
							for (int j = 0; j < 4; j++)
							{
								renderObj->m_projectionMatrix[i][j] = projMat[i + 4 * j];
								renderObj->m_modelMatrix[i][j] = modelMat[i + 4 * j];
								renderObj->m_viewMatrix[i][j] = viewMat[i + 4 * j];
							}
						}
						renderObj->m_localScaling = colObj->getCollisionShape()->getLocalScaling();
						renderObj->m_lightDirWorld = lightDirWorld;
						renderObj->m_lightAmbientCoeff = 0.6;
						renderObj->m_lightDiffuseCoeff = 0.35;
						renderObj->m_lightSpecularCoeff = 0.05;
						TinyRenderer::renderObject(*renderObj);
					}
				}
			}
		}

		static int counter = 0;
		counter++;
		if ((counter & 7) == 0)
		{
			char filename[1024];
			sprintf(filename, "framebuf%d.tga", counter);
			m_rgbColorBuffer.flip_vertically();
			getFrameBuffer().write_tga_file(filename, true);
		}
		float color[4] = {1, 1, 1, 1};
	}

	virtual void createPhysicsDebugDrawer(btDiscreteDynamicsWorld* rbWorld) {}

	virtual int registerTexture(const unsigned char* texels, int width, int height)
	{
		//do we need to make a copy?
		int textureId = m_textures.size();
		TinyRendererTexture t;
		t.m_texels = texels;
		t.m_width = width;
		t.m_height = height;
		this->m_textures.insert(textureId, t);
		return textureId;
	}

	virtual int registerGraphicsShape(const float* vertices, int numvertices, const int* indices, int numIndices, int primitiveType, int textureId)
	{
		int shapeIndex = m_swRenderObjects.size();

		TinyRenderObjectData* swObj = new TinyRenderObjectData(m_rgbColorBuffer, m_depthBuffer);
		float rgbaColor[4] = {1, 1, 1, 1};

		//if (textureId>=0)
		//{
		//	swObj->registerMeshShape(vertices,numvertices,indices,numIndices,rgbaColor);
		//} else
		{
			swObj->registerMeshShape(vertices, numvertices, indices, numIndices, rgbaColor);
		}
		//swObj->createCube(1,1,1);//MeshShape(vertices,numvertices,indices,numIndices);
		m_swRenderObjects.insert(shapeIndex, swObj);
		return shapeIndex;
	}

	virtual void removeAllGraphicsInstances()
	{
	}

	virtual int registerGraphicsInstance(int shapeIndex, const float* position, const float* quaternion, const float* color, const float* scaling)
	{
		int colIndex = m_colObjUniqueIndex++;

		if (colIndex >= 0 && shapeIndex >= 0)
		{
			TinyRenderObjectData** dPtr = m_swRenderObjects[shapeIndex];
			if (dPtr && *dPtr)
			{
				TinyRenderObjectData* d = *dPtr;
				d->m_localScaling.setValue(scaling[0], scaling[1], scaling[2]);
				m_swInstances.insert(colIndex, shapeIndex);
			}
		}

		return colIndex;
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
		return 0;
	}

	virtual void setUpAxis(int axis)
	{
		m_upAxis = axis;
		m_camera.setCameraUpAxis(axis);
		m_camera.update();
	}
	virtual void resetCamera(float camDist, float pitch, float yaw, float camPosX, float camPosY, float camPosZ)
	{
		m_camera.setCameraDistance(camDist);
		m_camera.setCameraPitch(pitch);
		m_camera.setCameraYaw(yaw);
		m_camera.setCameraTargetPosition(camPosX, camPosY, camPosZ);
		m_camera.setAspectRatio((float)m_swWidth / (float)m_swHeight);
		m_camera.update();
	}

	virtual void copyCameraImageData(const float viewMatrix[16], const float projectionMatrix[16],
									 unsigned char* pixelsRGBA, int rgbaBufferSizeInPixels,
									 float* depthBuffer, int depthBufferSizeInPixels,
									 int* segmentationMaskBuffer, int segmentationMaskBufferSizeInPixels,
									 int startPixelIndex, int destinationWidth, int destinationHeight, int* numPixelsCopied)
	{
		if (numPixelsCopied)
			*numPixelsCopied = 0;
	}

	virtual void autogenerateGraphicsObjects(btDiscreteDynamicsWorld* rbWorld)
	{
		for (int i = 0; i < rbWorld->getNumCollisionObjects(); i++)
		{
			btCollisionObject* colObj = rbWorld->getCollisionObjectArray()[i];
			//btRigidBody* body = btRigidBody::upcast(colObj);
			//does this also work for btMultiBody/btMultiBodyLinkCollider?
			createCollisionShapeGraphicsObject(colObj->getCollisionShape());
			int colorIndex = colObj->getBroadphaseHandle()->getUid() & 3;

			btVector3 color = sMyColors[colorIndex];
			createCollisionObjectGraphicsObject(colObj, color);
		}
	}

	virtual void drawText3D(const char* txt, float posX, float posZY, float posZ, float size)
	{
	}
	virtual int addUserDebugText3D(const char* txt, const double positionXYZ[3], const double textColorRGB[3], double size, double lifeTime)
	{
		return -1;
	}
	virtual int addUserDebugLine(const double debugLineFromXYZ[3], const double debugLineToXYZ[3], const double debugLineColorRGB[3], double lineWidth, double lifeTime)
	{
		return -1;
	}
	virtual void removeUserDebugItem(int debugItemUniqueId)
	{
	}
	virtual void removeAllUserDebugItems()
	{
	}
};

int main(int argc, char* argv[])
{
	TinyRendererGUIHelper noGfx(640, 480);

	CommonExampleOptions options(&noGfx);
	CommonExampleInterface* example = StandaloneExampleCreateFunc(options);

	example->initPhysics();
	example->resetCamera();

	for (int i = 0; i < 1000; i++)
	{
		printf("Simulating step %d\n", i);
		example->stepSimulation(1.f / 60.f);
		example->renderScene();
	}
	example->exitPhysics();

	delete example;

	return 0;
}
