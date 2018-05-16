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


#include "TinyRendererVisualShapeConverter.h"



#include "../Importers/ImportURDFDemo/URDFImporterInterface.h"
#include "btBulletCollisionCommon.h"
#include "../Importers/ImportObjDemo/LoadMeshFromObj.h"
#include "../Importers/ImportSTLDemo/LoadMeshFromSTL.h"
#include "../Importers/ImportColladaDemo/LoadMeshFromCollada.h"
#include "BulletCollision/CollisionShapes/btShapeHull.h"//to create a tesselation of a generic btConvexShape
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
#include "../SharedMemory/SharedMemoryPublic.h"//for b3VisualShapeData
#include "../TinyRenderer/model.h"
#include "stb_image/stb_image.h"

struct MyTexture2
{
	unsigned char* textureData1;
	int m_width;
	int m_height;
	bool m_isCached;
};

struct TinyRendererObjectArray
{
  btAlignedObjectArray<  TinyRenderObjectData*> m_renderObjects;
  int m_objectUniqueId;
  int m_linkIndex;
  btTransform m_worldTransform;
  btVector3 m_localScaling;

  TinyRendererObjectArray()
  {
	  m_worldTransform.setIdentity();
	  m_localScaling.setValue(1,1,1);
  }
};

#define START_WIDTH 640
#define START_HEIGHT 480

struct TinyRendererVisualShapeConverterInternalData
{
	
    btHashMap<btHashInt,TinyRendererObjectArray*> m_swRenderInstances;

	btAlignedObjectArray<b3VisualShapeData> m_visualShapes;
    
   	int m_upAxis;
	int m_swWidth;
	int m_swHeight;
	TGAImage m_rgbColorBuffer;
    b3AlignedObjectArray<MyTexture2> m_textures;
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
	
	
	TinyRendererVisualShapeConverterInternalData()
	:m_upAxis(2),
	m_swWidth(START_WIDTH),
	m_swHeight(START_HEIGHT),
	m_rgbColorBuffer(START_WIDTH,START_HEIGHT,TGAImage::RGB),
	m_lightDirection(btVector3(-5,200,-40)),
	m_hasLightDirection(false),
	m_lightColor(btVector3(1.0,1.0,1.0)),
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
	    m_depthBuffer.resize(m_swWidth*m_swHeight);
        m_shadowBuffer.resize(m_swWidth*m_swHeight);
	    m_segmentationMaskBuffer.resize(m_swWidth*m_swHeight,-1);
	}
	
};



TinyRendererVisualShapeConverter::TinyRendererVisualShapeConverter()
{
	m_data = new TinyRendererVisualShapeConverterInternalData();
	
	float dist = 1.5;
	float pitch = -10;
	float yaw = -80;
	float targetPos[3]={0,0,0};
	m_data->m_camera.setCameraUpAxis(m_data->m_upAxis);
	resetCamera(dist,yaw,pitch,targetPos[0],targetPos[1],targetPos[2]);


}
TinyRendererVisualShapeConverter::~TinyRendererVisualShapeConverter()
{
	resetAll();
	delete m_data;
}
	
void TinyRendererVisualShapeConverter::setLightDirection(float x, float y, float z)
{
	m_data->m_lightDirection.setValue(x, y, z);
	m_data->m_hasLightDirection = true;
}

void TinyRendererVisualShapeConverter::setLightColor(float x, float y, float z)
{
    m_data->m_lightColor.setValue(x, y, z);
    m_data->m_hasLightColor = true;
}

void TinyRendererVisualShapeConverter::setLightDistance(float dist)
{
    m_data->m_lightDistance = dist;
    m_data->m_hasLightDistance = true;
}

void TinyRendererVisualShapeConverter::setShadow(bool hasShadow)
{
    m_data->m_hasShadow = hasShadow;
}
void TinyRendererVisualShapeConverter::setFlags(int flags)
{
	m_data->m_flags = flags;
}


void TinyRendererVisualShapeConverter::setLightAmbientCoeff(float ambientCoeff)
{
    m_data->m_lightAmbientCoeff = ambientCoeff;
    m_data->m_hasLightAmbientCoeff = true;
}

void TinyRendererVisualShapeConverter::setLightDiffuseCoeff(float diffuseCoeff)
{
    m_data->m_lightDiffuseCoeff = diffuseCoeff;
    m_data->m_hasLightDiffuseCoeff = true;
}

void TinyRendererVisualShapeConverter::setLightSpecularCoeff(float specularCoeff)
{
    m_data->m_lightSpecularCoeff = specularCoeff;
    m_data->m_hasLightSpecularCoeff = true;
}

void convertURDFToVisualShape(const UrdfShape* visual, const char* urdfPathPrefix, const btTransform& visualTransform, btAlignedObjectArray<GLInstanceVertex>& verticesOut, btAlignedObjectArray<int>& indicesOut, btAlignedObjectArray<MyTexture2>& texturesOut, b3VisualShapeData& visualShapeOut)
{

	visualShapeOut.m_visualGeometryType = visual->m_geometry.m_type;
	visualShapeOut.m_dimensions[0] = 0;
	visualShapeOut.m_dimensions[1] = 0;
	visualShapeOut.m_dimensions[2] = 0;
	memset(visualShapeOut.m_meshAssetFileName, 0, sizeof(visualShapeOut.m_meshAssetFileName));
	if (visual->m_geometry.m_hasLocalMaterial) {
		visualShapeOut.m_rgbaColor[0] = visual->m_geometry.m_localMaterial.m_matColor.m_rgbaColor[0];
		visualShapeOut.m_rgbaColor[1] = visual->m_geometry.m_localMaterial.m_matColor.m_rgbaColor[1];
		visualShapeOut.m_rgbaColor[2] = visual->m_geometry.m_localMaterial.m_matColor.m_rgbaColor[2];
		visualShapeOut.m_rgbaColor[3] = visual->m_geometry.m_localMaterial.m_matColor.m_rgbaColor[3];
	}
	
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
			btVector3 center(0,0,0);
			btVector3 axis(0,0,1);
			btAlignedObjectArray<btVector3> vertices;
			int numSteps = 32;

			if (visual->m_geometry.m_hasFromTo) 
			{
				btVector3 v      =  p2 - p1;
				btVector3 dir  = v.normalized();
				tr = visual->m_linkLocalFrame;
				len = v.length();
				rad = visual->m_geometry.m_capsuleRadius;
				btVector3 ax1,ax2;
				btPlaneSpace1(dir,ax1,ax2);

				for (int i = 0; i<numSteps; i++)
				{
					{
						btVector3 vert = p1 + ax1*rad*btSin(SIMD_2_PI*(float(i) / numSteps))+ax2*rad*btCos(SIMD_2_PI*(float(i) / numSteps));
						vertices.push_back(vert);
					}
					{
						btVector3 vert = p2 + ax1*rad*btSin(SIMD_2_PI*(float(i) / numSteps))+ax2*rad*btCos(SIMD_2_PI*(float(i) / numSteps));
						vertices.push_back(vert);
					}
				}
				if (visual->m_geometry.m_type==URDF_GEOM_CAPSULE)
				{
					btVector3 pole1 = p1 - dir * rad;
					btVector3 pole2 = p2 + dir * rad;
					vertices.push_back(pole1);
					vertices.push_back(pole2);
				}

			} else {
				//assume a capsule along the Z-axis, centered at the origin
				tr = visual->m_linkLocalFrame;
				len = visual->m_geometry.m_capsuleHeight;
				rad = visual->m_geometry.m_capsuleRadius;
				for (int i = 0; i<numSteps; i++)
				{
					btVector3 vert(rad*btSin(SIMD_2_PI*(float(i) / numSteps)), rad*btCos(SIMD_2_PI*(float(i) / numSteps)), len / 2.);
					vertices.push_back(vert);
					vert[2] = -len / 2.;
					vertices.push_back(vert);
				}
				if (visual->m_geometry.m_type==URDF_GEOM_CAPSULE)
				{
					btVector3 pole1(0, 0, + len / 2. + rad);
					btVector3 pole2(0, 0, - len / 2. - rad);
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
			
			btBoxShape* boxShape = new btBoxShape(extents*0.5f);
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
			visualShapeOut.m_meshAssetFileName[VISUAL_SHAPE_MAX_PATH_LEN-1] = 0;

			visualShapeOut.m_dimensions[0] = visual->m_geometry.m_meshScale[0];
			visualShapeOut.m_dimensions[1] = visual->m_geometry.m_meshScale[1];
			visualShapeOut.m_dimensions[2] = visual->m_geometry.m_meshScale[2];

			switch (visual->m_geometry.m_meshFileType)
			{
			case UrdfGeometry::FILE_OBJ:
				{
					//glmesh = LoadMeshFromObj(fullPath,visualPathPrefix);
					b3ImportMeshData meshData;
					if (b3ImportMeshUtility::loadAndRegisterMeshFromFileInternal(visual->m_geometry.m_meshFileName, meshData))
					{

						if (meshData.m_textureImage1)
						{
							MyTexture2 texData;
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
				glmesh = LoadMeshFromSTL(visual->m_geometry.m_meshFileName.c_str());
				break;
			case UrdfGeometry::FILE_COLLADA:
				{
					btAlignedObjectArray<GLInstanceGraphicsShape> visualShapes;
					btAlignedObjectArray<ColladaGraphicsInstance> visualShapeInstances;
					btTransform upAxisTrans; upAxisTrans.setIdentity();
					float unitMeterScaling = 1;
					int upAxis = 2;

					LoadMeshFromCollada(visual->m_geometry.m_meshFileName.c_str(),
						visualShapes,
						visualShapeInstances,
						upAxisTrans,
						unitMeterScaling,
										upAxis);

					glmesh = new GLInstanceGraphicsShape;
			//		int index = 0;
					glmesh->m_indices = new b3AlignedObjectArray<int>();
					glmesh->m_vertices = new b3AlignedObjectArray<GLInstanceVertex>();

					for (int i = 0; i<visualShapeInstances.size(); i++)
					{
						ColladaGraphicsInstance* instance = &visualShapeInstances[i];
						GLInstanceGraphicsShape* gfxShape = &visualShapes[instance->m_shapeIndex];

						b3AlignedObjectArray<GLInstanceVertex> verts;
						verts.resize(gfxShape->m_vertices->size());

						int baseIndex = glmesh->m_vertices->size();

						for (int i = 0; i<gfxShape->m_vertices->size(); i++)
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
						for (int k = 0; k<additionalIndices; k++)
						{
							glmesh->m_indices->at(curNumIndices + k) = gfxShape->m_indices->at(k) + baseIndex;
						}

						//compensate upAxisTrans and unitMeterScaling here
						btMatrix4x4 upAxisMat;
						upAxisMat.setIdentity();
//								upAxisMat.setPureRotation(upAxisTrans.getRotation());
						btMatrix4x4 unitMeterScalingMat;
						unitMeterScalingMat.setPureScaling(btVector3(unitMeterScaling, unitMeterScaling, unitMeterScaling));
						btMatrix4x4 worldMat = unitMeterScalingMat*upAxisMat*instance->m_worldTransform;
						//btMatrix4x4 worldMat = instance->m_worldTransform;
						int curNumVertices = glmesh->m_vertices->size();
						int additionalVertices = verts.size();
						glmesh->m_vertices->reserve(curNumVertices + additionalVertices);

						for (int v = 0; v<verts.size(); v++)
						{
							btVector3 pos(verts[v].xyzw[0], verts[v].xyzw[1], verts[v].xyzw[2]);
							pos = worldMat*pos;
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

			if (glmesh && glmesh->m_vertices && (glmesh->m_numvertices>0))
			{
				//apply the geometry scaling
				for (int i=0;i<glmesh->m_vertices->size();i++)
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
		} // case mesh

		case URDF_GEOM_PLANE:
			// TODO: plane in tiny renderer
			// TODO: export visualShapeOut for external render
			break;

		default:
		{
			b3Warning("TinyRenderer: unknown visual geometry type %i\n", visual->m_geometry.m_type);
		}
	}

	//if we have a convex, tesselate into localVertices/localIndices
	if ((glmesh==0) && convexColShape)
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
	
	if (glmesh && glmesh->m_numIndices>0 && glmesh->m_numvertices >0)
	{

		int baseIndex = verticesOut.size();



		for (int i = 0; i < glmesh->m_indices->size(); i++)
		{
			indicesOut.push_back(glmesh->m_indices->at(i) + baseIndex);
		}

		for (int i = 0; i < glmesh->m_vertices->size(); i++)
		{
			GLInstanceVertex& v = glmesh->m_vertices->at(i);
			btVector3 vert(v.xyzw[0],v.xyzw[1],v.xyzw[2]);
			btVector3 vt = visualTransform*vert;
			v.xyzw[0] = vt[0];
			v.xyzw[1] = vt[1];
			v.xyzw[2] = vt[2];
			btVector3 triNormal(v.normal[0],v.normal[1],v.normal[2]);
			triNormal = visualTransform.getBasis()*triNormal;
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
	btVector4(60./256.,186./256.,84./256.,1),
	btVector4(244./256.,194./256.,13./256.,1),
	btVector4(219./256.,50./256.,54./256.,1),
	btVector4(72./256.,133./256.,237./256.,1),

	//btVector4(1,1,0,1),
};


void TinyRendererVisualShapeConverter::convertVisualShapes(
	int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame,
	const UrdfLink* linkPtr, const UrdfModel* model,
	int collisionObjectUniqueId, int bodyUniqueId)
{
	btAssert(linkPtr); // TODO: remove if (not doing it now, because diff will be 50+ lines)
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

		for (int v1=0; v1<cnt; v1++)
		{
			btAlignedObjectArray<MyTexture2> textures;
			btAlignedObjectArray<GLInstanceVertex> vertices;
			btAlignedObjectArray<int> indices;
			btTransform startTrans; startTrans.setIdentity();
			//int graphicsIndex = -1;

			const UrdfShape* vis;
			if (useVisual) {
				vis = &linkPtr->m_visualArray[v1];
			} else {
				vis = &linkPtr->m_collisionArray[v1];
			}
			btTransform childTrans = vis->m_linkLocalFrame;

			

			int colorIndex = linkIndex;//colObj? colObj->getBroadphaseHandle()->getUid() & 3 : 0;
			if (colorIndex<0)
				colorIndex=0;
			colorIndex &=3;
			btVector4 color;
			color = sColors[colorIndex];
			float rgbaColor[4] = {(float)color[0],(float)color[1],(float)color[2],(float)color[3]};
			//if (colObj->getCollisionShape()->getShapeType()==STATIC_PLANE_PROXYTYPE)
			//{
			//	color.setValue(1,1,1,1);
			//}
			if (model)
			{
				if (useVisual)
				{
					btHashString matName(linkPtr->m_visualArray[v1].m_materialName.c_str());
					UrdfMaterial*const* matPtr = model->m_materials[matName];
					if (matPtr)
					{
						for (int i = 0; i < 4; i++)
						{
							rgbaColor[i] = (*matPtr)->m_matColor.m_rgbaColor[i];
						}
						//printf("UrdfMaterial %s, rgba = %f,%f,%f,%f\n",mat->m_name.c_str(),mat->m_rgbaColor[0],mat->m_rgbaColor[1],mat->m_rgbaColor[2],mat->m_rgbaColor[3]);
						//m_data->m_linkColors.insert(linkIndex,mat->m_rgbaColor);
					} else
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
			
			TinyRendererObjectArray** visualsPtr = m_data->m_swRenderInstances[collisionObjectUniqueId];
            if (visualsPtr==0)
            {
                m_data->m_swRenderInstances.insert(collisionObjectUniqueId,new TinyRendererObjectArray);
            }
            visualsPtr = m_data->m_swRenderInstances[collisionObjectUniqueId];
			
            btAssert(visualsPtr);
            TinyRendererObjectArray* visuals = *visualsPtr;
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
				B3_PROFILE("convertURDFToVisualShape");
				convertURDFToVisualShape(vis, pathPrefix, localInertiaFrame.inverse()*childTrans, vertices, indices, textures, visualShape);
			}
			m_data->m_visualShapes.push_back(visualShape);

            if (vertices.size() && indices.size())
            {
                TinyRenderObjectData* tinyObj = new TinyRenderObjectData(m_data->m_rgbColorBuffer,m_data->m_depthBuffer, &m_data->m_shadowBuffer, &m_data->m_segmentationMaskBuffer, bodyUniqueId, linkIndex);
				unsigned char* textureImage1=0;
				int textureWidth=0;
				int textureHeight=0;
				bool isCached = false;
				if (textures.size())
				{
					textureImage1 = textures[0].textureData1;
					textureWidth = textures[0].m_width;
					textureHeight = textures[0].m_height;
					isCached = textures[0].m_isCached;
				}
				
				{
					B3_PROFILE("registerMeshShape");

					tinyObj->registerMeshShape(&vertices[0].xyzw[0], vertices.size(), &indices[0], indices.size(), rgbaColor,
						textureImage1, textureWidth, textureHeight);
				}
                visuals->m_renderObjects.push_back(tinyObj);
            }
			for (int i=0;i<textures.size();i++)
			{
				if (!textures[i].m_isCached)
				{
					free(textures[i].textureData1);
				}
			}
		}
	}
}

int TinyRendererVisualShapeConverter::getNumVisualShapes(int bodyUniqueId)
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

int TinyRendererVisualShapeConverter::getVisualShapesData(int bodyUniqueId, int shapeIndex, struct b3VisualShapeData* shapeData)
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



void TinyRendererVisualShapeConverter::changeRGBAColor(int bodyUniqueId, int linkIndex, int shapeIndex, const double rgbaColor[4])
{
    int start = -1;
    for (int i = 0; i < m_data->m_visualShapes.size(); i++)
    {
        if (m_data->m_visualShapes[i].m_objectUniqueId == bodyUniqueId && m_data->m_visualShapes[i].m_linkIndex == linkIndex)
        {
			m_data->m_visualShapes[i].m_rgbaColor[0] = rgbaColor[0];
			m_data->m_visualShapes[i].m_rgbaColor[1] = rgbaColor[1];
			m_data->m_visualShapes[i].m_rgbaColor[2] = rgbaColor[2];
			m_data->m_visualShapes[i].m_rgbaColor[3] = rgbaColor[3];
		}
    }
    
	for (int i=0;i<m_data->m_swRenderInstances.size();i++)
	{
		TinyRendererObjectArray** ptrptr = m_data->m_swRenderInstances.getAtIndex(i);
		if (ptrptr && *ptrptr)
		{
			float rgba[4] = {(float)rgbaColor[0], (float)rgbaColor[1], (float)rgbaColor[2], (float)rgbaColor[3]};
			TinyRendererObjectArray* visuals = *ptrptr;
			if ((bodyUniqueId == visuals->m_objectUniqueId) && (linkIndex == visuals->m_linkIndex))
			{
				for (int q=0;q<visuals->m_renderObjects.size();q++)
				{
					if (shapeIndex<0 || q==shapeIndex)
					{
						visuals->m_renderObjects[q]->m_model->setColorRGBA(rgba);
					}
				}
			}
		}
	}
}



void TinyRendererVisualShapeConverter::setUpAxis(int axis)
{
    m_data->m_upAxis = axis;
    m_data->m_camera.setCameraUpAxis(axis);
    m_data->m_camera.update();
}
void TinyRendererVisualShapeConverter::resetCamera(float camDist, float yaw, float pitch, float camPosX,float camPosY, float camPosZ)
{
    m_data->m_camera.setCameraDistance(camDist);
    m_data->m_camera.setCameraPitch(pitch);
    m_data->m_camera.setCameraYaw(yaw);
    m_data->m_camera.setCameraTargetPosition(camPosX,camPosY,camPosZ);
    m_data->m_camera.setAspectRatio((float)m_data->m_swWidth/(float)m_data->m_swHeight);
    m_data->m_camera.update();

}

void TinyRendererVisualShapeConverter::clearBuffers(TGAColor& clearColor)
{
	float farPlane = m_data->m_camera.getCameraFrustumFar();
    for(int y=0;y<m_data->m_swHeight;++y)
    {
        for(int x=0;x<m_data->m_swWidth;++x)
        {
            m_data->m_rgbColorBuffer.set(x,y,clearColor);
            m_data->m_depthBuffer[x+y*m_data->m_swWidth] = -farPlane;
            m_data->m_shadowBuffer[x+y*m_data->m_swWidth] = -1e30f;
            m_data->m_segmentationMaskBuffer[x+y*m_data->m_swWidth] = -1;
        }
    }
    
}

void TinyRendererVisualShapeConverter::render() 
{

    ATTRIBUTE_ALIGNED16(float viewMat[16]);
    ATTRIBUTE_ALIGNED16(float projMat[16]);

    m_data->m_camera.getCameraProjectionMatrix(projMat);
    m_data->m_camera.getCameraViewMatrix(viewMat);

	render(viewMat,projMat);
}    


void TinyRendererVisualShapeConverter::syncTransform(int collisionObjectUniqueId, const btTransform& worldTransform, const btVector3& localScaling)
{
	TinyRendererObjectArray** renderObjPtr = m_data->m_swRenderInstances[collisionObjectUniqueId];
	if (renderObjPtr)
	{
		TinyRendererObjectArray* renderObj = *renderObjPtr;
		renderObj->m_worldTransform = worldTransform;
		renderObj->m_localScaling = localScaling;
	}
}


void TinyRendererVisualShapeConverter::render(const float viewMat[16], const float projMat[16]) 
{
    //clear the color buffer
    TGAColor clearColor;
    clearColor.bgra[0] = 255;
    clearColor.bgra[1] = 255;
    clearColor.bgra[2] = 255;
    clearColor.bgra[3] = 255;
    
	float near = projMat[14]/(projMat[10]-1);
	float far = projMat[14]/(projMat[10]+1);

	m_data->m_camera.setCameraFrustumNear( near);
	m_data->m_camera.setCameraFrustumFar(far);

	clearBuffers(clearColor);	
    
    ATTRIBUTE_ALIGNED16(btScalar modelMat[16]);
    
    
    btVector3 lightDirWorld(-5,200,-40);
	if (m_data->m_hasLightDirection)
	{
		lightDirWorld = m_data->m_lightDirection;
	}
	else
	{
		switch (m_data->m_upAxis)
		{
		case 1:
			lightDirWorld = btVector3(-50.f, 100, 30);
			break;
		case 2:
			lightDirWorld = btVector3(-50.f, 30, 100);
			break;
		default: {}
		};
	}
    
    lightDirWorld.normalize();
    
    btVector3 lightColor(1.0,1.0,1.0);
    if (m_data->m_hasLightColor)
    {
        lightColor = m_data->m_lightColor;
    }
    
    float lightDistance = 2.0;
    if (m_data->m_hasLightDistance)
    {
        lightDistance = m_data->m_lightDistance;
    }
    
    float lightAmbientCoeff = 0.6;
    if (m_data->m_hasLightAmbientCoeff)
    {
        lightAmbientCoeff = m_data->m_lightAmbientCoeff;
    }
    
    float lightDiffuseCoeff = 0.35;
    if (m_data->m_hasLightDiffuseCoeff)
    {
        lightDiffuseCoeff = m_data->m_lightDiffuseCoeff;
    }
    
    float lightSpecularCoeff = 0.05;
    if (m_data->m_hasLightSpecularCoeff)
    {
        lightSpecularCoeff = m_data->m_lightSpecularCoeff;
    }
    
    if (m_data->m_hasShadow)
    {
        for (int n=0;n<m_data->m_swRenderInstances.size();n++)
        {
            TinyRendererObjectArray** visualArrayPtr = m_data->m_swRenderInstances.getAtIndex(n);
            if (0==visualArrayPtr)
                continue;//can this ever happen?
            TinyRendererObjectArray* visualArray = *visualArrayPtr;
            
            for (int v=0;v<visualArray->m_renderObjects.size();v++)
            {
                
                TinyRenderObjectData* renderObj = visualArray->m_renderObjects[v];
                
                
                //sync the object transform
				const btTransform& tr = visualArray->m_worldTransform;
                tr.getOpenGLMatrix(modelMat);
                
                for (int i=0;i<4;i++)
                {
                    for (int j=0;j<4;j++)
                    {
                        
                        renderObj->m_projectionMatrix[i][j] = projMat[i+4*j];
                        renderObj->m_modelMatrix[i][j] = modelMat[i+4*j];
                        renderObj->m_viewMatrix[i][j] = viewMat[i+4*j];
                    }
                }
				renderObj->m_localScaling = visualArray->m_localScaling;
                renderObj->m_lightDirWorld = lightDirWorld;
                renderObj->m_lightColor = lightColor;
                renderObj->m_lightDistance = lightDistance;
                renderObj->m_lightAmbientCoeff = lightAmbientCoeff;
                renderObj->m_lightDiffuseCoeff = lightDiffuseCoeff;
                renderObj->m_lightSpecularCoeff = lightSpecularCoeff;
                TinyRenderer::renderObjectDepth(*renderObj);
            }
        }
    }

    for (int n=0;n<m_data->m_swRenderInstances.size();n++)
    {
        TinyRendererObjectArray** visualArrayPtr = m_data->m_swRenderInstances.getAtIndex(n);
        if (0==visualArrayPtr)
            continue;//can this ever happen?
        TinyRendererObjectArray* visualArray = *visualArrayPtr;
        
        
        for (int v=0;v<visualArray->m_renderObjects.size();v++)
        {
            
            TinyRenderObjectData* renderObj = visualArray->m_renderObjects[v];
            
            
            //sync the object transform
			const btTransform& tr = visualArray->m_worldTransform;
            tr.getOpenGLMatrix(modelMat);
            
            for (int i=0;i<4;i++)
            {
                for (int j=0;j<4;j++)
                {
                    
                    renderObj->m_projectionMatrix[i][j] = projMat[i+4*j];
                    renderObj->m_modelMatrix[i][j] = modelMat[i+4*j];
                    renderObj->m_viewMatrix[i][j] = viewMat[i+4*j];
                }
            }
			renderObj->m_localScaling = visualArray->m_localScaling;
            renderObj->m_lightDirWorld = lightDirWorld;
            renderObj->m_lightColor = lightColor;
            renderObj->m_lightDistance = lightDistance;
            renderObj->m_lightAmbientCoeff = lightAmbientCoeff;
            renderObj->m_lightDiffuseCoeff = lightDiffuseCoeff;
            renderObj->m_lightSpecularCoeff = lightSpecularCoeff;
            TinyRenderer::renderObject(*renderObj);
        }
    }
	//printf("write tga \n");
	//m_data->m_rgbColorBuffer.write_tga_file("camera.tga");
//	printf("flipped!\n");
	m_data->m_rgbColorBuffer.flip_vertically();

	//flip z-buffer and segmentation Buffer
	{
		int half = m_data->m_swHeight>>1;
		for (int j=0; j<half; j++)
		{
			unsigned long l1 = j*m_data->m_swWidth;
			unsigned long l2 = (m_data->m_swHeight-1-j)*m_data->m_swWidth;
			for (int i=0;i<m_data->m_swWidth;i++)
			{
				btSwap(m_data->m_depthBuffer[l1+i],m_data->m_depthBuffer[l2+i]);
                btSwap(m_data->m_shadowBuffer[l1+i],m_data->m_shadowBuffer[l2+i]);
				btSwap(m_data->m_segmentationMaskBuffer[l1+i],m_data->m_segmentationMaskBuffer[l2+i]);
			}
		}
	}
}

void TinyRendererVisualShapeConverter::getWidthAndHeight(int& width, int& height)
{
    width = m_data->m_swWidth;
    height = m_data->m_swHeight;
}


void TinyRendererVisualShapeConverter::setWidthAndHeight(int width, int height)
{
	m_data->m_swWidth = width;
	m_data->m_swHeight = height;

	m_data->m_depthBuffer.resize(m_data->m_swWidth*m_data->m_swHeight);
    m_data->m_shadowBuffer.resize(m_data->m_swWidth*m_data->m_swHeight);
	m_data->m_segmentationMaskBuffer.resize(m_data->m_swWidth*m_data->m_swHeight);
	m_data->m_rgbColorBuffer = TGAImage(width, height, TGAImage::RGB);
	
		
}

void TinyRendererVisualShapeConverter::copyCameraImageData(unsigned char* pixelsRGBA, int rgbaBufferSizeInPixels, 
                                                            float* depthBuffer, int depthBufferSizeInPixels,
                                                            int* segmentationMaskBuffer, int segmentationMaskSizeInPixels,
                                                            int startPixelIndex, int* widthPtr, int* heightPtr, int* numPixelsCopied)
{
    int w = m_data->m_rgbColorBuffer.get_width();
    int h = m_data->m_rgbColorBuffer.get_height();
    
    if (numPixelsCopied)
        *numPixelsCopied = 0;
    
    if (widthPtr)
        *widthPtr = w;
    
    if (heightPtr)
        *heightPtr = h;
    
    int numTotalPixels = w*h;
    int numRemainingPixels = numTotalPixels - startPixelIndex;
    int numBytesPerPixel = 4;//RGBA
    int numRequestedPixels  = btMin(rgbaBufferSizeInPixels,numRemainingPixels);
    if (numRequestedPixels)
    {
        for (int i=0;i<numRequestedPixels;i++)
        {
			if (depthBuffer)
			{
                float farPlane = m_data->m_camera.getCameraFrustumFar();
                float nearPlane = m_data->m_camera.getCameraFrustumNear();
				
				// TinyRenderer returns clip coordinates, transform to eye coordinates first
				float z_c = -m_data->m_depthBuffer[i+startPixelIndex];
				// float distance = (farPlane - nearPlane) / (farPlane + nearPlane) * (z_c + 2. * farPlane * nearPlane / (farPlane - nearPlane));
				                
                // The depth buffer value is between 0 and 1
                // float a = farPlane / (farPlane - nearPlane);
                // float b = farPlane * nearPlane / (nearPlane - farPlane);
                // depthBuffer[i] = a + b / distance;

				// Simply the above expressions
				depthBuffer[i] = farPlane * (nearPlane + z_c) / (2. * farPlane * nearPlane + farPlane * z_c - nearPlane * z_c);
			}
			if (segmentationMaskBuffer)
            {
				int segMask = m_data->m_segmentationMaskBuffer[i + startPixelIndex];
				if ((m_data->m_flags & ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)==0)
				{
					//if we don't explicitly request link index, clear it out
					//object index are the lower 24bits
					if (segMask >= 0)
					{
						segMask &= ((1 << 24) - 1);
					}
				}
				segmentationMaskBuffer[i] = segMask;
            }
			
            if (pixelsRGBA)
            {
                pixelsRGBA[i*numBytesPerPixel] =   m_data->m_rgbColorBuffer.buffer()[(i+startPixelIndex)*3+0];
                pixelsRGBA[i*numBytesPerPixel+1] = m_data->m_rgbColorBuffer.buffer()[(i+startPixelIndex)*3+1];
                pixelsRGBA[i*numBytesPerPixel+2] = m_data->m_rgbColorBuffer.buffer()[(i+startPixelIndex)*3+2];
                pixelsRGBA[i*numBytesPerPixel+3] = 255;
                
            }
        }
        
        if (numPixelsCopied)
            *numPixelsCopied = numRequestedPixels;
        
    }    
}

void TinyRendererVisualShapeConverter::removeVisualShape(int collisionObjectUniqueId)
{
	TinyRendererObjectArray** ptrptr = m_data->m_swRenderInstances[collisionObjectUniqueId];
	if (ptrptr && *ptrptr)
	{
		TinyRendererObjectArray* ptr = *ptrptr;
		if (ptr)
		{
			for (int o=0;o<ptr->m_renderObjects.size();o++)
			{
				delete ptr->m_renderObjects[o];
			}
		}
		delete ptr;
		m_data->m_swRenderInstances.remove(collisionObjectUniqueId);
	}
}


void TinyRendererVisualShapeConverter::resetAll()
{
	for (int i=0;i<m_data->m_swRenderInstances.size();i++)
	{
		TinyRendererObjectArray** ptrptr = m_data->m_swRenderInstances.getAtIndex(i);
		if (ptrptr && *ptrptr)
		{
			TinyRendererObjectArray* ptr = *ptrptr;
			if (ptr)
			{
				for (int o=0;o<ptr->m_renderObjects.size();o++)
				{
					delete ptr->m_renderObjects[o];
				}
			}
			delete ptr;
		}
	}

	for (int i=0;i<m_data->m_textures.size();i++)
	{
		if (!m_data->m_textures[i].m_isCached)
		{
			free(m_data->m_textures[i].textureData1);
		}
	}
	m_data->m_textures.clear();
	m_data->m_swRenderInstances.clear();
	m_data->m_visualShapes.clear();
}


void TinyRendererVisualShapeConverter::changeShapeTexture(int objectUniqueId, int jointIndex, int shapeIndex, int textureUniqueId)
{
	btAssert(textureUniqueId < m_data->m_textures.size());
	if (textureUniqueId >= 0 && textureUniqueId < m_data->m_textures.size())
	{
		for (int n = 0; n < m_data->m_swRenderInstances.size(); n++)
		{
			TinyRendererObjectArray** visualArrayPtr = m_data->m_swRenderInstances.getAtIndex(n);
			if (0 == visualArrayPtr)
				continue;//can this ever happen?
			TinyRendererObjectArray* visualArray = *visualArrayPtr;

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

int TinyRendererVisualShapeConverter::registerTexture(unsigned char* texels, int width, int height)
{
    MyTexture2 texData;
    texData.m_width = width;
    texData.m_height = height;
    texData.textureData1 = texels;
	texData.m_isCached = false;
    m_data->m_textures.push_back(texData);
    return m_data->m_textures.size()-1;
}

int TinyRendererVisualShapeConverter::loadTextureFile(const char* filename)
{
	B3_PROFILE("loadTextureFile");
    int width,height,n;
    unsigned char* image=0;
    image = stbi_load(filename, &width, &height, &n, 3);
    if (image && (width>=0) && (height>=0))
    {
        return registerTexture(image, width, height);
    }
    return -1;
}
