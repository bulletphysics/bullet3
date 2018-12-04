/* Copyright (C) 2016 Google

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "eglRendererVisualShapeConverter.h"
#include "../Importers/ImportURDFDemo/URDFImporterInterface.h"
#include "btBulletCollisionCommon.h"
#include "../Importers/ImportObjDemo/LoadMeshFromObj.h"
#include "../Importers/ImportSTLDemo/LoadMeshFromSTL.h"
#include "../Importers/ImportColladaDemo/LoadMeshFromCollada.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"  //to create a tesselation of a generic btConvexShape
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "Bullet3Common/b3FileUtils.h"
#include <string>
#include "../Utils/b3ResourcePath.h"
#include "../TinyRenderer/TinyRenderer.h"
#include "../OpenGLWindow/SimpleCamera.h"
#include "../Importers/ImportMeshUtility/b3ImportMeshUtility.h"
#include <iostream>
#include <fstream>
#include "../Importers/ImportURDFDemo/UrdfParser.h"
#include "../SharedMemory/SharedMemoryPublic.h"  //for b3VisualShapeData
#include "../TinyRenderer/model.h"
#include "stb_image/stb_image.h"
#include "LinearMath/btMinMax.h"

#ifdef __APPLE__
#include "OpenGLWindow/MacOpenGLWindow.h"
typedef MacOpenGLWindow DefaultOpenGLWindow;
#else
#ifdef _WIN32
#include "OpenGLWindow/Win32OpenGLWindow.h"
typedef Win32OpenGLWindow DefaultOpenGLWindow;
#else
#ifdef BT_USE_EGL
#include "OpenGLWindow/EGLOpenGLWindow.h"
typedef EGLOpenGLWindow DefaultOpenGLWindow;
#else
#include "OpenGLWindow/X11OpenGLWindow.h"
typedef X11OpenGLWindow DefaultOpenGLWindow;
#endif  //BT_USE_EGL
#endif  // _WIN32
#endif  //__APPLE__

#include "OpenGLWindow/GLInstancingRenderer.h"
#include "OpenGLWindow/GLRenderToTexture.h"

static void printGLString(const char* name, GLenum s)
{
	const char* v = (const char*)glGetString(s);
	printf("%s = %s\n", name, v);
}

using namespace std;

struct MyTexture3
{
	unsigned char* textureData1;
	int m_width;
	int m_height;
	bool m_isCached;
};

struct EGLRendererObjectArray
{
	btAlignedObjectArray<TinyRenderObjectData*> m_renderObjects;
	int m_objectUniqueId;
	int m_linkIndex;
	int m_graphicsInstanceId;
	btTransform m_worldTransform;
	btVector3 m_localScaling;

	EGLRendererObjectArray()
	{
		m_worldTransform.setIdentity();
		m_localScaling.setValue(1, 1, 1);
		m_graphicsInstanceId = -1;
	}
};

#define START_WIDTH 2560
#define START_HEIGHT 2048

struct EGLRendererVisualShapeConverterInternalData
{
	class CommonWindowInterface* m_window;
	class GLInstancingRenderer* m_instancingRenderer;
	btAlignedObjectArray<unsigned char> m_rgbaPixelBuffer1;
	btAlignedObjectArray<float> m_depthBuffer1;

	btAlignedObjectArray<unsigned char> m_segmentationMaskSourceRgbaPixelBuffer;
	btAlignedObjectArray<float> m_segmentationMaskSourceDepthBuffer;

	btAlignedObjectArray<int> m_graphicsIndexToSegmentationMask;
	btHashMap<btHashInt, EGLRendererObjectArray*> m_swRenderInstances;

	btAlignedObjectArray<b3VisualShapeData> m_visualShapes;

	int m_upAxis;
	int m_swWidth;
	int m_swHeight;
	btAlignedObjectArray<unsigned char> m_sourceRgbaPixelBuffer;
	btAlignedObjectArray<float> m_sourceDepthBuffer;

	TGAImage m_rgbColorBuffer;
	b3AlignedObjectArray<MyTexture3> m_textures;
	b3AlignedObjectArray<float> m_depthBuffer;
	b3AlignedObjectArray<float> m_shadowBuffer;
	b3AlignedObjectArray<int> m_segmentationMaskBuffer;
	btVector3 m_lightDirection;
	bool m_hasLightDirection;
	btVector3 m_lightColor;
	bool m_hasLightColor;
	float m_lightDistance;
	bool m_hasLightDistance;
	float m_lightAmbientCoeff;
	bool m_hasLightAmbientCoeff;
	float m_lightDiffuseCoeff;
	bool m_hasLightDiffuseCoeff;
	float m_lightSpecularCoeff;
	bool m_hasLightSpecularCoeff;
	bool m_hasShadow;
	int m_flags;
	SimpleCamera m_camera;

	EGLRendererVisualShapeConverterInternalData()
		: m_upAxis(2),
		  m_swWidth(START_WIDTH),
		  m_swHeight(START_HEIGHT),
		  m_rgbColorBuffer(START_WIDTH, START_HEIGHT, TGAImage::RGB),
		  m_lightDirection(btVector3(-5, 200, -40)),
		  m_hasLightDirection(false),
		  m_lightColor(btVector3(1.0, 1.0, 1.0)),
		  m_hasLightColor(false),
		  m_lightDistance(2.0),
		  m_hasLightDistance(false),
		  m_lightAmbientCoeff(0.6),
		  m_hasLightAmbientCoeff(false),
		  m_lightDiffuseCoeff(0.35),
		  m_hasLightDiffuseCoeff(false),
		  m_lightSpecularCoeff(0.05),
		  m_hasLightSpecularCoeff(false),
		  m_hasShadow(false),
		  m_flags(0)
	{
		m_depthBuffer.resize(m_swWidth * m_swHeight);
		m_shadowBuffer.resize(m_swWidth * m_swHeight);
		m_segmentationMaskBuffer.resize(m_swWidth * m_swHeight, -1);

		// OpenGL window
		bool allowRetina = true;
		m_window = new DefaultOpenGLWindow();
		m_window->setAllowRetina(allowRetina);
		b3gWindowConstructionInfo ci;
		ci.m_title = "Title";
		ci.m_width = m_swWidth;
		ci.m_height = m_swHeight;
		ci.m_renderDevice = 0;

		m_window->createWindow(ci);
		m_window->setWindowTitle(ci.m_title);
		b3Assert(glGetError() == GL_NO_ERROR);
		{
			printGLString("Version", GL_VERSION);
			printGLString("Vendor", GL_VENDOR);
			printGLString("Renderer", GL_RENDERER);
		}
		glClearColor(.7f, .7f, .8f, 1.f);

		m_window->startRendering();

		b3Assert(glGetError() == GL_NO_ERROR);

		glGetError();  //don't remove this call, it is needed for Ubuntu
		b3Assert(glGetError() == GL_NO_ERROR);
		int maxNumObjectCapacity = 128 * 1024;
		int maxShapeCapacityInBytes = 128 * 1024 * 1024;
		m_instancingRenderer = new GLInstancingRenderer(maxNumObjectCapacity, maxShapeCapacityInBytes);
		b3Assert(glGetError() == GL_NO_ERROR);
		m_instancingRenderer->init();
		b3Assert(glGetError() == GL_NO_ERROR);
		m_instancingRenderer->resize(m_swWidth, m_swHeight);
		m_instancingRenderer->InitShaders();
		b3Assert(glGetError() == GL_NO_ERROR);
		m_instancingRenderer->setActiveCamera(&m_camera);
		b3Assert(glGetError() == GL_NO_ERROR);
		m_instancingRenderer->updateCamera();
		b3Assert(glGetError() == GL_NO_ERROR);
		m_instancingRenderer->setLightPosition(m_lightDirection);
		m_window->endRendering();
	}

	virtual ~EGLRendererVisualShapeConverterInternalData()
	{
		delete m_instancingRenderer;
		m_window->closeWindow();
		delete m_window;
	}
};

EGLRendererVisualShapeConverter::EGLRendererVisualShapeConverter()
{
	m_data = new EGLRendererVisualShapeConverterInternalData();

	float dist = 1.5;
	float pitch = -10;
	float yaw = -80;
	float targetPos[3] = {0, 0, 0};
	m_data->m_camera.setCameraUpAxis(m_data->m_upAxis);
	resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
}
EGLRendererVisualShapeConverter::~EGLRendererVisualShapeConverter()
{
	resetAll();
	delete m_data;
}

void EGLRendererVisualShapeConverter::setLightDirection(float x, float y, float z)
{
	m_data->m_lightDirection.setValue(x, y, z);
	m_data->m_hasLightDirection = true;
}

void EGLRendererVisualShapeConverter::setLightColor(float x, float y, float z)
{
	m_data->m_lightColor.setValue(x, y, z);
	m_data->m_hasLightColor = true;
}

void EGLRendererVisualShapeConverter::setLightDistance(float dist)
{
	m_data->m_lightDistance = dist;
	m_data->m_hasLightDistance = true;
}

void EGLRendererVisualShapeConverter::setShadow(bool hasShadow)
{
	m_data->m_hasShadow = hasShadow;
}
void EGLRendererVisualShapeConverter::setFlags(int flags)
{
	m_data->m_flags = flags;
}

void EGLRendererVisualShapeConverter::setLightAmbientCoeff(float ambientCoeff)
{
	m_data->m_lightAmbientCoeff = ambientCoeff;
	m_data->m_hasLightAmbientCoeff = true;
}

void EGLRendererVisualShapeConverter::setLightDiffuseCoeff(float diffuseCoeff)
{
	m_data->m_lightDiffuseCoeff = diffuseCoeff;
	m_data->m_hasLightDiffuseCoeff = true;
}

void EGLRendererVisualShapeConverter::setLightSpecularCoeff(float specularCoeff)
{
	m_data->m_lightSpecularCoeff = specularCoeff;
	m_data->m_hasLightSpecularCoeff = true;
}

///todo: merge into single file with TinyRendererVisualShapeConverter
static void convertURDFToVisualShape2(const UrdfShape* visual, const char* urdfPathPrefix, const btTransform& visualTransform, btAlignedObjectArray<GLInstanceVertex>& verticesOut, btAlignedObjectArray<int>& indicesOut, btAlignedObjectArray<MyTexture3>& texturesOut, b3VisualShapeData& visualShapeOut, struct CommonFileIOInterface* fileIO, int flags)
{
	visualShapeOut.m_visualGeometryType = visual->m_geometry.m_type;
	visualShapeOut.m_dimensions[0] = 0;
	visualShapeOut.m_dimensions[1] = 0;
	visualShapeOut.m_dimensions[2] = 0;
	memset(visualShapeOut.m_meshAssetFileName, 0, sizeof(visualShapeOut.m_meshAssetFileName));
#if 0
	if (visual->m_geometry.m_hasLocalMaterial)
	{
		visualShapeOut.m_rgbaColor[0] = visual->m_geometry.m_localMaterial.m_matColor.m_rgbaColor[0];
		visualShapeOut.m_rgbaColor[1] = visual->m_geometry.m_localMaterial.m_matColor.m_rgbaColor[1];
		visualShapeOut.m_rgbaColor[2] = visual->m_geometry.m_localMaterial.m_matColor.m_rgbaColor[2];
		visualShapeOut.m_rgbaColor[3] = visual->m_geometry.m_localMaterial.m_matColor.m_rgbaColor[3];
	}
#endif

	GLInstanceGraphicsShape* glmesh = 0;

	btConvexShape* convexColShape = 0;

	switch (visual->m_geometry.m_type)
	{
		case URDF_GEOM_CYLINDER:
		case URDF_GEOM_CAPSULE:
		{
			btVector3 p1 = visual->m_geometry.m_capsuleFrom;
			btVector3 p2 = visual->m_geometry.m_capsuleTo;
			btTransform tr;
			tr.setIdentity();
			btScalar rad, len;
			btVector3 center(0, 0, 0);
			btVector3 axis(0, 0, 1);
			btAlignedObjectArray<btVector3> vertices;
			int numSteps = 32;

			if (visual->m_geometry.m_hasFromTo)
			{
				btVector3 v = p2 - p1;
				btVector3 dir = v.normalized();
				tr = visual->m_linkLocalFrame;
				len = v.length();
				rad = visual->m_geometry.m_capsuleRadius;
				btVector3 ax1, ax2;
				btPlaneSpace1(dir, ax1, ax2);

				for (int i = 0; i < numSteps; i++)
				{
					{
						btVector3 vert = p1 + ax1 * rad * btSin(SIMD_2_PI * (float(i) / numSteps)) + ax2 * rad * btCos(SIMD_2_PI * (float(i) / numSteps));
						vertices.push_back(vert);
					}
					{
						btVector3 vert = p2 + ax1 * rad * btSin(SIMD_2_PI * (float(i) / numSteps)) + ax2 * rad * btCos(SIMD_2_PI * (float(i) / numSteps));
						vertices.push_back(vert);
					}
				}
				if (visual->m_geometry.m_type == URDF_GEOM_CAPSULE)
				{
					btVector3 pole1 = p1 - dir * rad;
					btVector3 pole2 = p2 + dir * rad;
					vertices.push_back(pole1);
					vertices.push_back(pole2);
				}
			}
			else
			{
				//assume a capsule along the Z-axis, centered at the origin
				tr = visual->m_linkLocalFrame;
				len = visual->m_geometry.m_capsuleHeight;
				rad = visual->m_geometry.m_capsuleRadius;
				for (int i = 0; i < numSteps; i++)
				{
					btVector3 vert(rad * btSin(SIMD_2_PI * (float(i) / numSteps)), rad * btCos(SIMD_2_PI * (float(i) / numSteps)), len / 2.);
					vertices.push_back(vert);
					vert[2] = -len / 2.;
					vertices.push_back(vert);
				}
				if (visual->m_geometry.m_type == URDF_GEOM_CAPSULE)
				{
					btVector3 pole1(0, 0, +len / 2. + rad);
					btVector3 pole2(0, 0, -len / 2. - rad);
					vertices.push_back(pole1);
					vertices.push_back(pole2);
				}
			}
			visualShapeOut.m_localVisualFrame[0] = tr.getOrigin()[0];
			visualShapeOut.m_localVisualFrame[1] = tr.getOrigin()[1];
			visualShapeOut.m_localVisualFrame[2] = tr.getOrigin()[2];
			visualShapeOut.m_localVisualFrame[3] = tr.getRotation()[0];
			visualShapeOut.m_localVisualFrame[4] = tr.getRotation()[1];
			visualShapeOut.m_localVisualFrame[5] = tr.getRotation()[2];
			visualShapeOut.m_localVisualFrame[6] = tr.getRotation()[3];
			visualShapeOut.m_dimensions[0] = len;
			visualShapeOut.m_dimensions[1] = rad;

			btConvexHullShape* cylZShape = new btConvexHullShape(&vertices[0].x(), vertices.size(), sizeof(btVector3));
			//btCapsuleShape* cylZShape = new btCapsuleShape(rad,len);//btConvexHullShape(&vertices[0].x(), vertices.size(), sizeof(btVector3));

			cylZShape->setMargin(0.001);
			convexColShape = cylZShape;
			break;
		}
		case URDF_GEOM_BOX:
		{
			visualShapeOut.m_dimensions[0] = visual->m_geometry.m_boxSize[0];
			visualShapeOut.m_dimensions[1] = visual->m_geometry.m_boxSize[1];
			visualShapeOut.m_dimensions[2] = visual->m_geometry.m_boxSize[2];

			btVector3 extents = visual->m_geometry.m_boxSize;

			btBoxShape* boxShape = new btBoxShape(extents * 0.5f);
			//btConvexShape* boxShape = new btConeShapeX(extents[2]*0.5,extents[0]*0.5);
			convexColShape = boxShape;
			convexColShape->setMargin(0.001);
			break;
		}
		case URDF_GEOM_SPHERE:
		{
			visualShapeOut.m_dimensions[0] = visual->m_geometry.m_sphereRadius;

			btScalar radius = visual->m_geometry.m_sphereRadius;
			btSphereShape* sphereShape = new btSphereShape(radius);
			convexColShape = sphereShape;
			convexColShape->setMargin(0.001);
			break;
		}
		case URDF_GEOM_MESH:
		{
			strncpy(visualShapeOut.m_meshAssetFileName, visual->m_geometry.m_meshFileName.c_str(), VISUAL_SHAPE_MAX_PATH_LEN);
			visualShapeOut.m_meshAssetFileName[VISUAL_SHAPE_MAX_PATH_LEN - 1] = 0;

			visualShapeOut.m_dimensions[0] = visual->m_geometry.m_meshScale[0];
			visualShapeOut.m_dimensions[1] = visual->m_geometry.m_meshScale[1];
			visualShapeOut.m_dimensions[2] = visual->m_geometry.m_meshScale[2];

			switch (visual->m_geometry.m_meshFileType)
			{
				case UrdfGeometry::FILE_OBJ:
				{
					//glmesh = LoadMeshFromObj(fullPath,visualPathPrefix);
					b3ImportMeshData meshData;
					if (b3ImportMeshUtility::loadAndRegisterMeshFromFileInternal(visual->m_geometry.m_meshFileName, meshData, fileIO))
					{
						if (flags&URDF_USE_MATERIAL_COLORS_FROM_MTL)
						{
							if (meshData.m_flags & B3_IMPORT_MESH_HAS_RGBA_COLOR)
							{
								visualShapeOut.m_rgbaColor[0] = meshData.m_rgbaColor[0];
								visualShapeOut.m_rgbaColor[1] = meshData.m_rgbaColor[1];
								visualShapeOut.m_rgbaColor[2] = meshData.m_rgbaColor[2];
								
								if (flags&URDF_USE_MATERIAL_TRANSPARANCY_FROM_MTL)
								{
									visualShapeOut.m_rgbaColor[3] = meshData.m_rgbaColor[3];
								} else
								{
									visualShapeOut.m_rgbaColor[3] = 1;
								}
							}
						}
						if (meshData.m_textureImage1)
						{
							MyTexture3 texData;
							texData.m_width = meshData.m_textureWidth;
							texData.m_height = meshData.m_textureHeight;
							texData.textureData1 = meshData.m_textureImage1;
							texData.m_isCached = meshData.m_isCached;
							texturesOut.push_back(texData);
						}
						glmesh = meshData.m_gfxShape;
					}
					break;
				}
				case UrdfGeometry::FILE_STL:
					glmesh = LoadMeshFromSTL(visual->m_geometry.m_meshFileName.c_str(), fileIO);
					break;
				case UrdfGeometry::FILE_COLLADA:
				{
					btAlignedObjectArray<GLInstanceGraphicsShape> visualShapes;
					btAlignedObjectArray<ColladaGraphicsInstance> visualShapeInstances;
					btTransform upAxisTrans;
					upAxisTrans.setIdentity();
					float unitMeterScaling = 1;
					int upAxis = 2;

					LoadMeshFromCollada(visual->m_geometry.m_meshFileName.c_str(),
										visualShapes,
										visualShapeInstances,
										upAxisTrans,
										unitMeterScaling,
										upAxis, fileIO);

					glmesh = new GLInstanceGraphicsShape;
					//		int index = 0;
					glmesh->m_indices = new b3AlignedObjectArray<int>();
					glmesh->m_vertices = new b3AlignedObjectArray<GLInstanceVertex>();

					for (int i = 0; i < visualShapeInstances.size(); i++)
					{
						ColladaGraphicsInstance* instance = &visualShapeInstances[i];
						GLInstanceGraphicsShape* gfxShape = &visualShapes[instance->m_shapeIndex];

						b3AlignedObjectArray<GLInstanceVertex> verts;
						verts.resize(gfxShape->m_vertices->size());

						int baseIndex = glmesh->m_vertices->size();

						for (int i = 0; i < gfxShape->m_vertices->size(); i++)
						{
							verts[i].normal[0] = gfxShape->m_vertices->at(i).normal[0];
							verts[i].normal[1] = gfxShape->m_vertices->at(i).normal[1];
							verts[i].normal[2] = gfxShape->m_vertices->at(i).normal[2];
							verts[i].uv[0] = gfxShape->m_vertices->at(i).uv[0];
							verts[i].uv[1] = gfxShape->m_vertices->at(i).uv[1];
							verts[i].xyzw[0] = gfxShape->m_vertices->at(i).xyzw[0];
							verts[i].xyzw[1] = gfxShape->m_vertices->at(i).xyzw[1];
							verts[i].xyzw[2] = gfxShape->m_vertices->at(i).xyzw[2];
							verts[i].xyzw[3] = gfxShape->m_vertices->at(i).xyzw[3];
						}

						int curNumIndices = glmesh->m_indices->size();
						int additionalIndices = gfxShape->m_indices->size();
						glmesh->m_indices->resize(curNumIndices + additionalIndices);
						for (int k = 0; k < additionalIndices; k++)
						{
							glmesh->m_indices->at(curNumIndices + k) = gfxShape->m_indices->at(k) + baseIndex;
						}

						//compensate upAxisTrans and unitMeterScaling here
						btMatrix4x4 upAxisMat;
						upAxisMat.setIdentity();
						//								upAxisMat.setPureRotation(upAxisTrans.getRotation());
						btMatrix4x4 unitMeterScalingMat;
						unitMeterScalingMat.setPureScaling(btVector3(unitMeterScaling, unitMeterScaling, unitMeterScaling));
						btMatrix4x4 worldMat = unitMeterScalingMat * upAxisMat * instance->m_worldTransform;
						//btMatrix4x4 worldMat = instance->m_worldTransform;
						int curNumVertices = glmesh->m_vertices->size();
						int additionalVertices = verts.size();
						glmesh->m_vertices->reserve(curNumVertices + additionalVertices);

						for (int v = 0; v < verts.size(); v++)
						{
							btVector3 pos(verts[v].xyzw[0], verts[v].xyzw[1], verts[v].xyzw[2]);
							pos = worldMat * pos;
							verts[v].xyzw[0] = float(pos[0]);
							verts[v].xyzw[1] = float(pos[1]);
							verts[v].xyzw[2] = float(pos[2]);
							glmesh->m_vertices->push_back(verts[v]);
						}
					}
					glmesh->m_numIndices = glmesh->m_indices->size();
					glmesh->m_numvertices = glmesh->m_vertices->size();
					//glmesh = LoadMeshFromCollada(visual->m_geometry.m_meshFileName.c_str());
					break;
				}

				default:
					// should never get here (findExistingMeshFile returns false if it doesn't recognize extension)
					btAssert(0);
			}

			if (glmesh && glmesh->m_vertices && (glmesh->m_numvertices > 0))
			{
				//apply the geometry scaling
				for (int i = 0; i < glmesh->m_vertices->size(); i++)
				{
					glmesh->m_vertices->at(i).xyzw[0] *= visual->m_geometry.m_meshScale[0];
					glmesh->m_vertices->at(i).xyzw[1] *= visual->m_geometry.m_meshScale[1];
					glmesh->m_vertices->at(i).xyzw[2] *= visual->m_geometry.m_meshScale[2];
				}
			}
			else
			{
				b3Warning("issue extracting mesh from COLLADA/STL file %s\n", visual->m_geometry.m_meshFileName.c_str());
			}
			break;
		}  // case mesh

		case URDF_GEOM_PLANE:
			// TODO: plane in egl renderer
			// TODO: export visualShapeOut for external render
			break;

		default:
		{
			b3Warning("TinyRenderer: unknown visual geometry type %i\n", visual->m_geometry.m_type);
		}
	}

	//if we have a convex, tesselate into localVertices/localIndices
	if ((glmesh == 0) && convexColShape)
	{
		btShapeHull* hull = new btShapeHull(convexColShape);
		hull->buildHull(0.0);
		{
			//	int strideInBytes = 9*sizeof(float);
			int numVertices = hull->numVertices();
			int numIndices = hull->numIndices();

			glmesh = new GLInstanceGraphicsShape;
			//	int index = 0;
			glmesh->m_indices = new b3AlignedObjectArray<int>();
			glmesh->m_vertices = new b3AlignedObjectArray<GLInstanceVertex>();

			for (int i = 0; i < numVertices; i++)
			{
				GLInstanceVertex vtx;
				btVector3 pos = hull->getVertexPointer()[i];
				vtx.xyzw[0] = pos.x();
				vtx.xyzw[1] = pos.y();
				vtx.xyzw[2] = pos.z();
				vtx.xyzw[3] = 1.f;
				btVector3 normal = pos.safeNormalize();
				vtx.normal[0] = normal.x();
				vtx.normal[1] = normal.y();
				vtx.normal[2] = normal.z();
				btScalar u = btAtan2(normal[0], normal[2]) / (2 * SIMD_PI) + 0.5;
				btScalar v = normal[1] * 0.5 + 0.5;
				vtx.uv[0] = u;
				vtx.uv[1] = v;
				glmesh->m_vertices->push_back(vtx);
			}

			btAlignedObjectArray<int> indices;
			for (int i = 0; i < numIndices; i++)
			{
				glmesh->m_indices->push_back(hull->getIndexPointer()[i]);
			}

			glmesh->m_numvertices = glmesh->m_vertices->size();
			glmesh->m_numIndices = glmesh->m_indices->size();
		}
		delete hull;
		delete convexColShape;
		convexColShape = 0;
	}

	if (glmesh && glmesh->m_numIndices > 0 && glmesh->m_numvertices > 0)
	{
		int baseIndex = verticesOut.size();

		for (int i = 0; i < glmesh->m_indices->size(); i++)
		{
			indicesOut.push_back(glmesh->m_indices->at(i) + baseIndex);
		}

		for (int i = 0; i < glmesh->m_vertices->size(); i++)
		{
			GLInstanceVertex& v = glmesh->m_vertices->at(i);
			btVector3 vert(v.xyzw[0], v.xyzw[1], v.xyzw[2]);
			btVector3 vt = visualTransform * vert;
			v.xyzw[0] = vt[0];
			v.xyzw[1] = vt[1];
			v.xyzw[2] = vt[2];
			btVector3 triNormal(v.normal[0], v.normal[1], v.normal[2]);
			triNormal = visualTransform.getBasis() * triNormal;
			v.normal[0] = triNormal[0];
			v.normal[1] = triNormal[1];
			v.normal[2] = triNormal[2];
			verticesOut.push_back(v);
		}
	}
	delete glmesh;
}

static btVector4 sColors[4] =
	{
		btVector4(60. / 256., 186. / 256., 84. / 256., 1),
		btVector4(244. / 256., 194. / 256., 13. / 256., 1),
		btVector4(219. / 256., 50. / 256., 54. / 256., 1),
		btVector4(72. / 256., 133. / 256., 237. / 256., 1),

		//btVector4(1,1,0,1),
};

// If you are getting segfaults in this function it may be ecause you are
// compliling the plugin with differently from pybullet, try complining the
// plugin with distutils too.
void EGLRendererVisualShapeConverter::convertVisualShapes(
	int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame,
	const UrdfLink* linkPtr, const UrdfModel* model,
	int collisionObjectUniqueId, int bodyUniqueId, struct  CommonFileIOInterface* fileIO)
{
	btAssert(linkPtr);  // TODO: remove if (not doing it now, because diff will be 50+ lines)
	if (linkPtr)
	{
		bool useVisual;
		int cnt = 0;

		if (linkPtr->m_visualArray.size() > 0)
		{
			useVisual = true;
			cnt = linkPtr->m_visualArray.size();
		}
		else
		{
			// We have to see something, take collision shape. Useful for MuJoCo xml, where there are no explicit visual shapes.
			useVisual = false;
			cnt = linkPtr->m_collisionArray.size();
		}

		for (int v1 = 0; v1 < cnt; v1++)
		{
			btAlignedObjectArray<MyTexture3> textures;
			btAlignedObjectArray<GLInstanceVertex> vertices;
			btAlignedObjectArray<int> indices;
			btTransform startTrans;
			startTrans.setIdentity();
			//int graphicsIndex = -1;

			const UrdfShape* vis;
			if (useVisual)
			{
				vis = &linkPtr->m_visualArray[v1];
			}
			else
			{
				vis = &linkPtr->m_collisionArray[v1];
			}
			// see note at function header
			btTransform childTrans = vis->m_linkLocalFrame;

			int colorIndex = linkIndex;  //colObj? colObj->getBroadphaseHandle()->getUid() & 3 : 0;
			if (colorIndex < 0)
				colorIndex = 0;
			colorIndex &= 3;
			btVector4 color;
			color = sColors[colorIndex];
			float rgbaColor[4] = {(float)color[0], (float)color[1], (float)color[2], (float)color[3]};
			//if (colObj->getCollisionShape()->getShapeType()==STATIC_PLANE_PROXYTYPE)
			//{
			//	color.setValue(1,1,1,1);
			//}
			if (model)
			{
				if (useVisual)
				{
					btHashString matName(linkPtr->m_visualArray[v1].m_materialName.c_str());
					UrdfMaterial* const* matPtr = model->m_materials[matName];
					if (matPtr)
					{
						for (int i = 0; i < 4; i++)
						{
							rgbaColor[i] = (*matPtr)->m_matColor.m_rgbaColor[i];
						}
						//printf("UrdfMaterial %s, rgba = %f,%f,%f,%f\n",mat->m_name.c_str(),mat->m_rgbaColor[0],mat->m_rgbaColor[1],mat->m_rgbaColor[2],mat->m_rgbaColor[3]);
						//m_data->m_linkColors.insert(linkIndex,mat->m_rgbaColor);
					}
					else
					{
						///programmatic created models may have the color in the visual
						if (vis && vis->m_geometry.m_hasLocalMaterial)
						{
							for (int i = 0; i < 4; i++)
							{
								rgbaColor[i] = vis->m_geometry.m_localMaterial.m_matColor.m_rgbaColor[i];
							}
						}
					}
				}
			}
			else
			{
				if (vis && vis->m_geometry.m_hasLocalMaterial)
				{
					for (int i = 0; i < 4; i++)
					{
						rgbaColor[i] = vis->m_geometry.m_localMaterial.m_matColor.m_rgbaColor[i];
					}
				}
			}

			EGLRendererObjectArray** visualsPtr = m_data->m_swRenderInstances[collisionObjectUniqueId];
			if (visualsPtr == 0)
			{
				m_data->m_swRenderInstances.insert(collisionObjectUniqueId, new EGLRendererObjectArray);
			}
			visualsPtr = m_data->m_swRenderInstances[collisionObjectUniqueId];

			btAssert(visualsPtr);
			EGLRendererObjectArray* visuals = *visualsPtr;
			visuals->m_objectUniqueId = bodyUniqueId;
			visuals->m_linkIndex = linkIndex;

			b3VisualShapeData visualShape;
			visualShape.m_objectUniqueId = bodyUniqueId;
			visualShape.m_linkIndex = linkIndex;
			visualShape.m_localVisualFrame[0] = vis->m_linkLocalFrame.getOrigin()[0];
			visualShape.m_localVisualFrame[1] = vis->m_linkLocalFrame.getOrigin()[1];
			visualShape.m_localVisualFrame[2] = vis->m_linkLocalFrame.getOrigin()[2];
			visualShape.m_localVisualFrame[3] = vis->m_linkLocalFrame.getRotation()[0];
			visualShape.m_localVisualFrame[4] = vis->m_linkLocalFrame.getRotation()[1];
			visualShape.m_localVisualFrame[5] = vis->m_linkLocalFrame.getRotation()[2];
			visualShape.m_localVisualFrame[6] = vis->m_linkLocalFrame.getRotation()[3];
			visualShape.m_rgbaColor[0] = rgbaColor[0];
			visualShape.m_rgbaColor[1] = rgbaColor[1];
			visualShape.m_rgbaColor[2] = rgbaColor[2];
			visualShape.m_rgbaColor[3] = rgbaColor[3];
			{
				B3_PROFILE("convertURDFToVisualShape2");
				convertURDFToVisualShape2(vis, pathPrefix, localInertiaFrame.inverse() * childTrans, vertices, indices, textures, visualShape, fileIO, m_data->m_flags);
			}
			m_data->m_visualShapes.push_back(visualShape);

			if (vertices.size() && indices.size())
			{
				TinyRenderObjectData* tinyObj = new TinyRenderObjectData(m_data->m_rgbColorBuffer, m_data->m_depthBuffer, &m_data->m_shadowBuffer, &m_data->m_segmentationMaskBuffer, bodyUniqueId, linkIndex);
				unsigned char* textureImage1 = 0;
				int textureWidth = 0;
				int textureHeight = 0;
				bool isCached = false;
				int textureIndex = -1;

				if (textures.size())
				{
					textureImage1 = textures[0].textureData1;
					textureWidth = textures[0].m_width;
					textureHeight = textures[0].m_height;
					isCached = textures[0].m_isCached;
					textureIndex = m_data->m_instancingRenderer->registerTexture(textureImage1, textureWidth, textureHeight);
				}

				{
					B3_PROFILE("registerMeshShape");

					tinyObj->registerMeshShape(&vertices[0].xyzw[0], vertices.size(), &indices[0], indices.size(), rgbaColor,
											   textureImage1, textureWidth, textureHeight);
				}
				visuals->m_renderObjects.push_back(tinyObj);

				{
					B3_PROFILE("m_instancingRenderer register");

					// register mesh to m_instancingRenderer too.

					int shapeIndex = m_data->m_instancingRenderer->registerShape(&vertices[0].xyzw[0], vertices.size(), &indices[0], indices.size(), B3_GL_TRIANGLES, textureIndex);
					double scaling[3] = {1, 1, 1};
					visuals->m_graphicsInstanceId = m_data->m_instancingRenderer->registerGraphicsInstance(shapeIndex, &visualShape.m_localVisualFrame[0], &visualShape.m_localVisualFrame[3], &visualShape.m_rgbaColor[0], scaling);

					int segmentationMask = bodyUniqueId + ((linkIndex + 1) << 24);
					{
						int graphicsIndex = visuals->m_graphicsInstanceId;
						if (graphicsIndex >= 0)
						{
							if (m_data->m_graphicsIndexToSegmentationMask.size() < (graphicsIndex + 1))
							{
								m_data->m_graphicsIndexToSegmentationMask.resize(graphicsIndex + 1);
							}
							m_data->m_graphicsIndexToSegmentationMask[graphicsIndex] = segmentationMask;
						}
					}

					m_data->m_instancingRenderer->writeTransforms();
				}
			}
			for (int i = 0; i < textures.size(); i++)
			{
				if (!textures[i].m_isCached)
				{
					free(textures[i].textureData1);
				}
			}
		}
	}
}

int EGLRendererVisualShapeConverter::getNumVisualShapes(int bodyUniqueId)
{
	int start = -1;
	//find first one, then count how many
	for (int i = 0; i < m_data->m_visualShapes.size(); i++)
	{
		if (m_data->m_visualShapes[i].m_objectUniqueId == bodyUniqueId)
		{
			start = i;
			break;
		}
	}
	int count = 0;

	if (start >= 0)
	{
		for (int i = start; i < m_data->m_visualShapes.size(); i++)
		{
			if (m_data->m_visualShapes[i].m_objectUniqueId == bodyUniqueId)
			{
				count++;
			}
			else
			{
				//storage of each visual shape for a given body unique id assumed to be contiguous
				break;
			}
		}
	}
	return count;
}

int EGLRendererVisualShapeConverter::getVisualShapesData(int bodyUniqueId, int shapeIndex, struct b3VisualShapeData* shapeData)
{
	int start = -1;
	//find first one, then count how many
	for (int i = 0; i < m_data->m_visualShapes.size(); i++)
	{
		if (m_data->m_visualShapes[i].m_objectUniqueId == bodyUniqueId)
		{
			start = i;
			break;
		}
	}
	//int count = 0;

	if (start >= 0)
	{
		if (start + shapeIndex < m_data->m_visualShapes.size())
		{
			*shapeData = m_data->m_visualShapes[start + shapeIndex];
			return 1;
		}
	}
	return 0;
}

void EGLRendererVisualShapeConverter::changeRGBAColor(int bodyUniqueId, int linkIndex, int shapeIndex, const double rgbaColor[4])
{
	//int start = -1;
	for (int i = 0; i < m_data->m_visualShapes.size(); i++)
	{
		if (m_data->m_visualShapes[i].m_objectUniqueId == bodyUniqueId && m_data->m_visualShapes[i].m_linkIndex == linkIndex)
		{
			m_data->m_visualShapes[i].m_rgbaColor[0] = rgbaColor[0];
			m_data->m_visualShapes[i].m_rgbaColor[1] = rgbaColor[1];
			m_data->m_visualShapes[i].m_rgbaColor[2] = rgbaColor[2];
			m_data->m_visualShapes[i].m_rgbaColor[3] = rgbaColor[3];
			m_data->m_instancingRenderer->writeSingleInstanceColorToCPU(rgbaColor,i);
		}
	}

	for (int i = 0; i < m_data->m_swRenderInstances.size(); i++)
	{
		EGLRendererObjectArray** ptrptr = m_data->m_swRenderInstances.getAtIndex(i);
		if (ptrptr && *ptrptr)
		{
			float rgba[4] = {(float)rgbaColor[0], (float)rgbaColor[1], (float)rgbaColor[2], (float)rgbaColor[3]};
			EGLRendererObjectArray* visuals = *ptrptr;
			if ((bodyUniqueId == visuals->m_objectUniqueId) && (linkIndex == visuals->m_linkIndex))
			{
				for (int q = 0; q < visuals->m_renderObjects.size(); q++)
				{
					if (shapeIndex < 0 || q == shapeIndex)
					{
						visuals->m_renderObjects[q]->m_model->setColorRGBA(rgba);
					}
				}
			}
		}
	}
}

void EGLRendererVisualShapeConverter::setUpAxis(int axis)
{
	m_data->m_upAxis = axis;
	m_data->m_camera.setCameraUpAxis(axis);
	m_data->m_camera.update();
	m_data->m_instancingRenderer->updateCamera();
}
void EGLRendererVisualShapeConverter::resetCamera(float camDist, float yaw, float pitch, float camPosX, float camPosY, float camPosZ)
{
	m_data->m_camera.setCameraDistance(camDist);
	m_data->m_camera.setCameraPitch(pitch);
	m_data->m_camera.setCameraYaw(yaw);
	m_data->m_camera.setCameraTargetPosition(camPosX, camPosY, camPosZ);
	m_data->m_camera.setAspectRatio((float)m_data->m_swWidth / (float)m_data->m_swHeight);
	m_data->m_camera.update();
}

void EGLRendererVisualShapeConverter::clearBuffers(TGAColor& clearColor)
{
	float farPlane = m_data->m_camera.getCameraFrustumFar();
	for (int y = 0; y < m_data->m_swHeight; ++y)
	{
		for (int x = 0; x < m_data->m_swWidth; ++x)
		{
			m_data->m_rgbColorBuffer.set(x, y, clearColor);
			m_data->m_depthBuffer[x + y * m_data->m_swWidth] = -farPlane;
			m_data->m_shadowBuffer[x + y * m_data->m_swWidth] = -1e30f;
			m_data->m_segmentationMaskBuffer[x + y * m_data->m_swWidth] = -1;
		}
	}
}

void EGLRendererVisualShapeConverter::render()
{
	//mode the the actual render code inside 'copyImageData' since we need to know the width/height
}

void EGLRendererVisualShapeConverter::render(const float viewMat[16], const float projMat[16])
{
	// This code is very similar to that of
	// PhysicsServerCommandProcessor::processRequestCameraImageCommand
	// maybe code from there should be moved.

	// Tiny allows rendering with viewMat, projMat explicitly, but
	// GLInstancingRender calls m_activeCamera, so set this.

	m_data->m_camera.setVRCamera(viewMat, projMat);

	render();

	//cout<<viewMat[4*0 + 0]<<" "<<viewMat[4*0+1]<<" "<<viewMat[4*0+2]<<" "<<viewMat[4*0+3] << endl;
	//cout<<viewMat[4*1 + 0]<<" "<<viewMat[4*1+1]<<" "<<viewMat[4*1+2]<<" "<<viewMat[4*1+3] << endl;
	//cout<<viewMat[4*2 + 0]<<" "<<viewMat[4*2+1]<<" "<<viewMat[4*2+2]<<" "<<viewMat[4*2+3] << endl;
	//cout<<viewMat[4*3 + 0]<<" "<<viewMat[4*3+1]<<" "<<viewMat[4*3+2]<<" "<<viewMat[4*3+3] << endl;
}

void EGLRendererVisualShapeConverter::getWidthAndHeight(int& width, int& height)
{
	width = m_data->m_swWidth;
	height = m_data->m_swHeight;
}

void EGLRendererVisualShapeConverter::setWidthAndHeight(int width, int height)
{
	m_data->m_swWidth = width;
	m_data->m_swHeight = height;

	m_data->m_depthBuffer.resize(m_data->m_swWidth * m_data->m_swHeight);
	m_data->m_shadowBuffer.resize(m_data->m_swWidth * m_data->m_swHeight);
	m_data->m_segmentationMaskBuffer.resize(m_data->m_swWidth * m_data->m_swHeight);
	m_data->m_rgbColorBuffer = TGAImage(width, height, TGAImage::RGB);
}

void EGLRendererVisualShapeConverter::setProjectiveTextureMatrices(const float viewMatrix[16], const float projectionMatrix[16])
{
	m_data->m_instancingRenderer->setProjectiveTextureMatrices(viewMatrix,projectionMatrix);
}

void EGLRendererVisualShapeConverter::setProjectiveTexture(bool useProjectiveTexture)
{
	m_data->m_instancingRenderer->setProjectiveTexture(useProjectiveTexture);
}

//copied from OpenGLGuiHelper.cpp
void EGLRendererVisualShapeConverter::copyCameraImageDataGL(
	unsigned char* pixelsRGBA, int rgbaBufferSizeInPixels,
	float* depthBuffer, int depthBufferSizeInPixels,
	int* segmentationMaskBuffer, int segmentationMaskSizeInPixels,
	int startPixelIndex, int* widthPtr, int* heightPtr, int* numPixelsCopied)
{
	if (numPixelsCopied)
		*numPixelsCopied = 0;

	int destinationWidth = *widthPtr;
	int destinationHeight = *heightPtr;
	int sourceWidth =  btMin(destinationWidth, (int)(m_data->m_window->getWidth() * m_data->m_window->getRetinaScale()));
	int sourceHeight = btMin(destinationHeight, (int)(m_data->m_window->getHeight() * m_data->m_window->getRetinaScale()));

	int numTotalPixels = (*widthPtr) * (*heightPtr);
	int numRemainingPixels = numTotalPixels - startPixelIndex;
	int numBytesPerPixel = 4;  //RGBA
	int numRequestedPixels = btMin(rgbaBufferSizeInPixels, numRemainingPixels);
	if (numRequestedPixels)
	{
		if (startPixelIndex == 0)
		{
			m_data->m_window->endRendering();
			m_data->m_window->startRendering();
			glViewport(0,0, sourceWidth, sourceHeight); 
			B3_PROFILE("m_instancingRenderer render");
			m_data->m_instancingRenderer->writeTransforms();
			if (m_data->m_hasLightDirection)
			{
				m_data->m_instancingRenderer->setLightPosition(m_data->m_lightDirection);
			}
			m_data->m_instancingRenderer->setActiveCamera(&m_data->m_camera);
			m_data->m_instancingRenderer->updateCamera(m_data->m_upAxis);

			m_data->m_instancingRenderer->renderScene();

			int numBytesPerPixel = 4;  //RGBA

			{
				{
					BT_PROFILE("copy pixels");

					//copy the image into our local cache
					m_data->m_sourceRgbaPixelBuffer.resize(sourceWidth * sourceHeight * numBytesPerPixel);
					m_data->m_sourceDepthBuffer.resize(sourceWidth * sourceHeight);
					{
						BT_PROFILE("getScreenPixels");
						int rgbaBufferSizeInPixels = m_data->m_sourceRgbaPixelBuffer.size();
						int depthBufferSizeInPixels = m_data->m_sourceDepthBuffer.size();
						// Copied from SimpleOpenGL3App::getScreenPixels
						b3Assert((sourceWidth * sourceHeight * 4) == rgbaBufferSizeInPixels);
						//glClear(GL_COLOR_BUFFER_BIT);
						//b3Warning("EGL\n");
						if ((sourceWidth * sourceHeight * 4) == rgbaBufferSizeInPixels)  // remove this if
						{
							glReadPixels(0, 0, sourceWidth, sourceHeight, GL_RGBA, GL_UNSIGNED_BYTE, &(m_data->m_sourceRgbaPixelBuffer[0]));
							int glstat;
							glstat = glGetError();
							b3Assert(glstat == GL_NO_ERROR);
						}
						if ((sourceWidth * sourceHeight) == depthBufferSizeInPixels)
						{
							glReadPixels(0, 0, sourceWidth, sourceHeight, GL_DEPTH_COMPONENT, GL_FLOAT, &(m_data->m_sourceDepthBuffer[0]));
							int glstat;
							glstat = glGetError();
							b3Assert(glstat == GL_NO_ERROR);
						}
					}
				}
			}

			m_data->m_rgbaPixelBuffer1.resize((*widthPtr) * (*heightPtr) * numBytesPerPixel);
			m_data->m_depthBuffer1.resize((*widthPtr) * (*heightPtr));
			//rescale and flip
			{
				BT_PROFILE("resize and flip");
				for (int j = 0; j < *heightPtr; j++)
				{
					for (int i = 0; i < *widthPtr; i++)
					{
						int xIndex = int(float(i) * (float(sourceWidth) / float(*widthPtr)));
						int yIndex = int(float(*heightPtr - 1 - j) * (float(sourceHeight) / float(*heightPtr)));
						btClamp(xIndex, 0, sourceWidth);
						btClamp(yIndex, 0, sourceHeight);
						int bytesPerPixel = 4;  //RGBA

						int sourcePixelIndex = (xIndex + yIndex * sourceWidth) * bytesPerPixel;
						int sourceDepthIndex = xIndex + yIndex * sourceWidth;
#define COPY4PIXELS 1
#ifdef COPY4PIXELS
						int* dst = (int*)&m_data->m_rgbaPixelBuffer1[(i + j * (*widthPtr)) * 4 + 0];
						int* src = (int*)&m_data->m_sourceRgbaPixelBuffer[sourcePixelIndex + 0];
						*dst = *src;

#else
						m_data->m_rgbaPixelBuffer1[(i + j * widthPtr) * 4 + 0] = sourceRgbaPixelBuffer[sourcePixelIndex + 0];
						m_data->m_rgbaPixelBuffer1[(i + j * widthPtr) * 4 + 1] = sourceRgbaPixelBuffer[sourcePixelIndex + 1];
						m_data->m_rgbaPixelBuffer1[(i + j * widthPtr) * 4 + 2] = sourceRgbaPixelBuffer[sourcePixelIndex + 2];
						m_data->m_rgbaPixelBuffer1[(i + j * widthPtr) * 4 + 3] = 255;
#endif
						if (depthBuffer)
						{
							m_data->m_depthBuffer1[i + j * (*widthPtr)] = m_data->m_sourceDepthBuffer[sourceDepthIndex];
						}
					}
				}
			}

			if (segmentationMaskBuffer)
			{
				{
					m_data->m_window->startRendering();
					glViewport(0,0, sourceWidth, sourceHeight); 
					BT_PROFILE("renderScene");
					m_data->m_instancingRenderer->renderSceneInternal(B3_SEGMENTATION_MASK_RENDERMODE);
				}

				{
					BT_PROFILE("copy pixels");

					//copy the image into our local cache
					m_data->m_segmentationMaskSourceRgbaPixelBuffer.resize(sourceWidth * sourceHeight * numBytesPerPixel);
					m_data->m_segmentationMaskSourceDepthBuffer.resize(sourceWidth * sourceHeight);
					{
						BT_PROFILE("getScreenPixels");
						{
							glReadPixels(0, 0, sourceWidth, sourceHeight, GL_DEPTH_COMPONENT, GL_FLOAT, &(m_data->m_segmentationMaskSourceDepthBuffer[0]));
							int glstat;
							glstat = glGetError();
							b3Assert(glstat == GL_NO_ERROR);
						}
						{
							glReadPixels(0, 0, sourceWidth, sourceHeight, GL_RGBA, GL_UNSIGNED_BYTE, &(m_data->m_segmentationMaskSourceRgbaPixelBuffer[0]));
							int glstat;
							glstat = glGetError();
							b3Assert(glstat == GL_NO_ERROR);
						}
					}
				}
			}

			m_data->m_segmentationMaskBuffer.resize(destinationWidth * destinationHeight, -1);

			//rescale and flip
			{
				BT_PROFILE("resize and flip");
				for (int j = 0; j < destinationHeight; j++)
				{
					for (int i = 0; i < destinationWidth; i++)
					{
						int xIndex = int(float(i) * (float(sourceWidth) / float(destinationWidth)));
						int yIndex = int(float(destinationHeight - 1 - j) * (float(sourceHeight) / float(destinationHeight)));
						btClamp(xIndex, 0, sourceWidth);
						btClamp(yIndex, 0, sourceHeight);
						int bytesPerPixel = 4;  //RGBA
						int sourcePixelIndex = (xIndex + yIndex * sourceWidth) * bytesPerPixel;
						int sourceDepthIndex = xIndex + yIndex * sourceWidth;

						if (segmentationMaskBuffer)
						{
							float depth = m_data->m_segmentationMaskSourceDepthBuffer[sourceDepthIndex];
							if (depth < 1)
							{
								int segMask = m_data->m_segmentationMaskSourceRgbaPixelBuffer[sourcePixelIndex + 0] + 256 * (m_data->m_segmentationMaskSourceRgbaPixelBuffer[sourcePixelIndex + 1]) + 256 * 256 * (m_data->m_segmentationMaskSourceRgbaPixelBuffer[sourcePixelIndex + 2]);
								m_data->m_segmentationMaskBuffer[i + j * destinationWidth] = segMask;
							}
							else
							{
								m_data->m_segmentationMaskBuffer[i + j * destinationWidth] = -1;
							}
						}
					}
				}
			}
			glViewport(0, 0, m_data->m_window->getWidth() * m_data->m_window->getRetinaScale(), m_data->m_window->getHeight() * m_data->m_window->getRetinaScale());
		}
		if (pixelsRGBA)
		{
			BT_PROFILE("copy rgba pixels");
			for (int i = 0; i < numRequestedPixels * numBytesPerPixel; i++)
			{
				pixelsRGBA[i] = m_data->m_rgbaPixelBuffer1[i + startPixelIndex * numBytesPerPixel];
			}
		}
		if (depthBuffer)
		{
			BT_PROFILE("copy depth buffer pixels");
			for (int i = 0; i < numRequestedPixels; i++)
			{
				depthBuffer[i] = m_data->m_depthBuffer1[i + startPixelIndex];
			}
		}
		if (segmentationMaskBuffer)
		{
			BT_PROFILE("copy segmentation mask buffer pixels");
			for (int i = 0; i < numRequestedPixels; i++)
			{
				int graphicsIndexSegMask = m_data->m_segmentationMaskBuffer[i + startPixelIndex];
				int segMask = -1;
				if (graphicsIndexSegMask >= 0 && graphicsIndexSegMask < m_data->m_graphicsIndexToSegmentationMask.size())
				{
					segMask = m_data->m_graphicsIndexToSegmentationMask[graphicsIndexSegMask];
				}
				if ((m_data->m_flags & ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX) == 0)
				{
					if (segMask >= 0)
					{
						segMask &= ((1 << 24) - 1);
					}
				}
				segmentationMaskBuffer[i] = segMask;
			}
		}

		if (numPixelsCopied)
		{
			*numPixelsCopied = numRequestedPixels;
		}
	}
}

void EGLRendererVisualShapeConverter::copyCameraImageData(unsigned char* pixelsRGBA, int rgbaBufferSizeInPixels,
														  float* depthBuffer, int depthBufferSizeInPixels,
														  int* segmentationMaskBuffer, int segmentationMaskSizeInPixels,
														  int startPixelIndex, int* widthPtr, int* heightPtr, int* numPixelsCopied)
{
	B3_PROFILE("copyCameraImageDataGL");
	copyCameraImageDataGL(pixelsRGBA, rgbaBufferSizeInPixels,
						  depthBuffer, depthBufferSizeInPixels,
						  segmentationMaskBuffer, segmentationMaskSizeInPixels,
						  startPixelIndex, widthPtr, heightPtr, numPixelsCopied);
}

void EGLRendererVisualShapeConverter::removeVisualShape(int collisionObjectUniqueId)
{
	EGLRendererObjectArray** ptrptr = m_data->m_swRenderInstances[collisionObjectUniqueId];
	if (ptrptr && *ptrptr)
	{
		EGLRendererObjectArray* ptr = *ptrptr;
		if (ptr)
		{
			for (int o = 0; o < ptr->m_renderObjects.size(); o++)
			{
				m_data->m_instancingRenderer->removeGraphicsInstance(ptr->m_graphicsInstanceId);
				delete ptr->m_renderObjects[o];
			}
		}
		delete ptr;
		m_data->m_swRenderInstances.remove(collisionObjectUniqueId);
	}
}

void EGLRendererVisualShapeConverter::resetAll()
{
	for (int i = 0; i < m_data->m_swRenderInstances.size(); i++)
	{
		EGLRendererObjectArray** ptrptr = m_data->m_swRenderInstances.getAtIndex(i);
		if (ptrptr && *ptrptr)
		{
			EGLRendererObjectArray* ptr = *ptrptr;
			if (ptr)
			{
				for (int o = 0; o < ptr->m_renderObjects.size(); o++)
				{
					delete ptr->m_renderObjects[o];
				}
			}
			delete ptr;
		}
	}

	for (int i = 0; i < m_data->m_textures.size(); i++)
	{
		if (!m_data->m_textures[i].m_isCached)
		{
			free(m_data->m_textures[i].textureData1);
		}
	}
	m_data->m_textures.clear();
	m_data->m_swRenderInstances.clear();
	m_data->m_visualShapes.clear();
	m_data->m_graphicsIndexToSegmentationMask.clear();
	m_data->m_instancingRenderer->removeAllInstances();
}

void EGLRendererVisualShapeConverter::changeShapeTexture(int objectUniqueId, int jointIndex, int shapeIndex, int textureUniqueId)
{
	btAssert(textureUniqueId < m_data->m_textures.size());
	if (textureUniqueId >= 0 && textureUniqueId < m_data->m_textures.size())
	{
		for (int n = 0; n < m_data->m_swRenderInstances.size(); n++)
		{
			EGLRendererObjectArray** visualArrayPtr = m_data->m_swRenderInstances.getAtIndex(n);
			if (0 == visualArrayPtr)
				continue;  //can this ever happen?
			EGLRendererObjectArray* visualArray = *visualArrayPtr;

			if (visualArray->m_objectUniqueId == objectUniqueId && visualArray->m_linkIndex == jointIndex)
			{
				for (int v = 0; v < visualArray->m_renderObjects.size(); v++)
				{
					TinyRenderObjectData* renderObj = visualArray->m_renderObjects[v];
					if ((shapeIndex < 0) || (shapeIndex == v))
					{
						renderObj->m_model->setDiffuseTextureFromData(m_data->m_textures[textureUniqueId].textureData1, m_data->m_textures[textureUniqueId].m_width, m_data->m_textures[textureUniqueId].m_height);
					}
				}
			}
		}
	}
}

int EGLRendererVisualShapeConverter::registerTexture(unsigned char* texels, int width, int height)
{
	MyTexture3 texData;
	texData.m_width = width;
	texData.m_height = height;
	texData.textureData1 = texels;
	texData.m_isCached = false;
	m_data->m_textures.push_back(texData);
	return m_data->m_textures.size() - 1;
}

int EGLRendererVisualShapeConverter::loadTextureFile(const char* filename, struct CommonFileIOInterface* fileIO)
{
	B3_PROFILE("loadTextureFile");
	int width, height, n;
	unsigned char* image = 0;

	if (fileIO)
	{
		b3AlignedObjectArray<char> buffer;
		buffer.reserve(1024);
		int fileId = fileIO->fileOpen(filename,"rb");
		if (fileId>=0)
		{
			int size = fileIO->getFileSize(fileId);
			if (size>0)
			{
				buffer.resize(size);
				int actual = fileIO->fileRead(fileId,&buffer[0],size);
				if (actual != size)
				{
					b3Warning("image filesize mismatch!\n");
					buffer.resize(0);
				}
			}
			fileIO->fileClose(fileId);
		}
		if (buffer.size())
		{
			image = stbi_load_from_memory((const unsigned char*)&buffer[0], buffer.size(), &width, &height, &n, 3);
		}
	} else
	{
		image = stbi_load(filename, &width, &height, &n, 3);
	}

	if (image && (width >= 0) && (height >= 0))
	{
		return registerTexture(image, width, height);
	}
	return -1;
}

void EGLRendererVisualShapeConverter::syncTransform(int collisionObjectUniqueId, const btTransform& worldTransform, const btVector3& localScaling)
{
	EGLRendererObjectArray** renderObjPtr = m_data->m_swRenderInstances[collisionObjectUniqueId];
	if (renderObjPtr)
	{
		EGLRendererObjectArray* renderObj = *renderObjPtr;
		renderObj->m_worldTransform = worldTransform;
		renderObj->m_localScaling = localScaling;
		if (renderObj->m_graphicsInstanceId >= 0)
		{
			btVector3 pos = worldTransform.getOrigin();
			btQuaternion orn = worldTransform.getRotation();
			m_data->m_instancingRenderer->writeSingleInstanceTransformToCPU(pos, orn, renderObj->m_graphicsInstanceId);
		}
	}
}
