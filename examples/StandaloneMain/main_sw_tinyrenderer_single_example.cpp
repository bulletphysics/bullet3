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

#include "LinearMath/btTransform.h"
#include "LinearMath/btHashMap.h"

#include "../TinyRenderer/TinyRenderer.h"
#include "../OpenGLWindow/SimpleOpenGL3App.h"
#include <stdio.h>
#include "../ExampleBrowser/OpenGLGuiHelper.h"

class SW_And_OpenGLGuiHelper : public OpenGLGuiHelper
{
	btHashMap<btHashInt, TinyRenderObjectData*> m_swRenderObjects;
	btHashMap<btHashInt, int> m_swInstances;

	int m_swWidth;
	int m_swHeight;
	TGAImage m_rgbColorBuffer;

	b3AlignedObjectArray<float> m_depthBuffer;
	int m_textureHandle;
	unsigned char* m_image;
	GLPrimitiveRenderer* m_primRenderer;

public:
	SW_And_OpenGLGuiHelper(CommonGraphicsApp* glApp, bool useOpenGL2, int swWidth, int swHeight, GLPrimitiveRenderer* primRenderer)
		: OpenGLGuiHelper(glApp, useOpenGL2),
		  m_swWidth(swWidth),
		  m_swHeight(swHeight),
		  m_rgbColorBuffer(swWidth, swHeight, TGAImage::RGB),
		  m_primRenderer(primRenderer)
	{
		m_depthBuffer.resize(swWidth * swHeight);
		CommonRenderInterface* render = getRenderInterface();
		m_image = new unsigned char[m_swWidth * m_swHeight * 4];

		m_textureHandle = render->registerTexture(m_image, m_swWidth, m_swHeight);
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

	virtual ~SW_And_OpenGLGuiHelper()
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

	virtual void createCollisionObjectGraphicsObject(btCollisionObject* obj, const btVector3& color)
	{
		OpenGLGuiHelper::createCollisionObjectGraphicsObject(obj, color);
		int colIndex = obj->getUserIndex();
		int shapeIndex = obj->getCollisionShape()->getUserIndex();

		if (colIndex >= 0 && shapeIndex >= 0)
		{
			m_swInstances.insert(colIndex, shapeIndex);
		}
	}

	virtual int registerTexture(const unsigned char* texels, int width, int height)
	{
		return -1;
	}

	virtual int registerGraphicsShape(const float* vertices, int numvertices, const int* indices, int numIndices, int primitiveType, int textureId)
	{
		int shapeIndex = OpenGLGuiHelper::registerGraphicsShape(vertices, numvertices, indices, numIndices, primitiveType, textureId);
		if (shapeIndex >= 0)
		{
			TinyRenderObjectData* swObj = new TinyRenderObjectData(m_rgbColorBuffer, m_depthBuffer);
			float rgbaColor[4] = {1, 1, 1, 1};
			swObj->registerMeshShape(vertices, numvertices, indices, numIndices, rgbaColor);
			//swObj->createCube(1,1,1);//MeshShape(vertices,numvertices,indices,numIndices);
			m_swRenderObjects.insert(shapeIndex, swObj);
		}
		return shapeIndex;
	}

	virtual int registerGraphicsInstance(int shapeIndex, const float* position, const float* quaternion, const float* color, const float* scaling)
	{
		int instanceId = OpenGLGuiHelper::registerGraphicsInstance(shapeIndex, position, quaternion, color, scaling);
		return instanceId;
	}

	virtual void createCollisionShapeGraphicsObject(btCollisionShape* collisionShape)
	{
		OpenGLGuiHelper::createCollisionShapeGraphicsObject(collisionShape);
	}

	virtual void syncPhysicsToGraphics(const btDiscreteDynamicsWorld* rbWorld)
	{
		OpenGLGuiHelper::syncPhysicsToGraphics(rbWorld);
	}

	virtual void render(const btDiscreteDynamicsWorld* rbWorld)
	{
		OpenGLGuiHelper::render(rbWorld);

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

		CommonRenderInterface* render = getRenderInterface();

		render->getActiveCamera()->getCameraProjectionMatrix(projMat);
		render->getActiveCamera()->getCameraViewMatrix(viewMat);

		btVector3 lightDirWorld(-5, 200, -40);
		switch (1)  //app->getUpAxis())
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

		for (int y = 0; y < m_swHeight; ++y)
		{
			unsigned char* pi = m_image + (y)*m_swWidth * 3;
			for (int x = 0; x < m_swWidth; ++x)
			{
				const TGAColor& color = getFrameBuffer().get(x, y);
				pi[0] = color.bgra[2];
				pi[1] = color.bgra[1];
				pi[2] = color.bgra[0];
				pi += 3;
			}
		}
		render->activateTexture(m_textureHandle);
		render->updateTexture(m_textureHandle, m_image);

		static int counter = 0;
		counter++;
		if ((counter & 7) == 0)
		{
			char filename[1024];
			sprintf(filename, "framebuf%d.tga", counter);
			getFrameBuffer().write_tga_file(filename, true);
		}
		float color[4] = {1, 1, 1, 1};
		m_primRenderer->drawTexturedRect(0, 0, m_swWidth, m_swHeight, color, 0, 0, 1, 1, true);
	}

	virtual void autogenerateGraphicsObjects(btDiscreteDynamicsWorld* rbWorld)
	{
		OpenGLGuiHelper::autogenerateGraphicsObjects(rbWorld);
	}
};

int main(int argc, char* argv[])
{
	SimpleOpenGL3App* app = new SimpleOpenGL3App("Standalone Example (Software Renderer, TinyRenderer)", 1024, 768, true);
	int textureWidth = 640;
	int textureHeight = 480;

	SW_And_OpenGLGuiHelper gui(app, false, textureWidth, textureHeight, app->m_primRenderer);

	CommonExampleOptions options(&gui);
	CommonExampleInterface* example = StandaloneExampleCreateFunc(options);

	example->initPhysics();
	example->resetCamera();
	do
	{
		app->m_instancingRenderer->init();
		app->m_instancingRenderer->updateCamera(app->getUpAxis());

		example->stepSimulation(1. / 60.);

		example->renderScene();

		DrawGridData dg;
		dg.upAxis = app->getUpAxis();
		app->drawGrid(dg);
		app->swapBuffer();
	} while (!app->m_window->requestedExit());

	example->exitPhysics();
	delete example;
	delete app;
	return 0;
}
