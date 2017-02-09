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
#include "../ThirdPartyLibs/stb_image/stb_image.h"

enum MyFileType
{
	MY_FILE_STL=1,
	MY_FILE_COLLADA=2,
    MY_FILE_OBJ=3,
};

struct MyTexture2
{
	unsigned char* textureData;
	int m_width;
	int m_height;
};

struct TinyRendererObjectArray
{
  btAlignedObjectArray<  TinyRenderObjectData*> m_renderObjects;
};

#define START_WIDTH 640
#define START_HEIGHT 480

struct TinyRendererVisualShapeConverterInternalData
{
	
    btHashMap<btHashPtr,TinyRendererObjectArray*> m_swRenderInstances;

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
	m_hasShadow(false)
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
	float pitch = -80;
	float yaw = 10;
	float targetPos[3]={0,0,0};
	m_data->m_camera.setCameraUpAxis(m_data->m_upAxis);
	resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);


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

void convertURDFToVisualShape(const UrdfVisual* visual, const char* urdfPathPrefix, const btTransform& visualTransform, btAlignedObjectArray<GLInstanceVertex>& verticesOut, btAlignedObjectArray<int>& indicesOut, btAlignedObjectArray<MyTexture2>& texturesOut, b3VisualShapeData& visualShapeOut)
{

	visualShapeOut.m_visualGeometryType = visual->m_geometry.m_type;
	visualShapeOut.m_dimensions[0] = 0;
	visualShapeOut.m_dimensions[1] = 0;
	visualShapeOut.m_dimensions[2] = 0;
	visualShapeOut.m_meshAssetFileName[0] = 0;
	
	GLInstanceGraphicsShape* glmesh = 0;

	btConvexShape* convexColShape = 0;

	switch (visual->m_geometry.m_type)
	{
		case URDF_GEOM_CYLINDER:
		{
			btAlignedObjectArray<btVector3> vertices;
		
			visualShapeOut.m_dimensions[0] = visual->m_geometry.m_cylinderLength;
			visualShapeOut.m_dimensions[1] = visual->m_geometry.m_cylinderRadius;
			
			//int numVerts = sizeof(barrel_vertices)/(9*sizeof(float));
			int numSteps = 32;
			for (int i = 0; i<numSteps; i++)
			{

				btScalar cylRadius = visual->m_geometry.m_cylinderRadius;
				btScalar cylLength = visual->m_geometry.m_cylinderLength;
				
				btVector3 vert(cylRadius*btSin(SIMD_2_PI*(float(i) / numSteps)), cylRadius*btCos(SIMD_2_PI*(float(i) / numSteps)), cylLength / 2.);
				vertices.push_back(vert);
				vert[2] = -cylLength / 2.;
				vertices.push_back(vert);
			}

			btConvexHullShape* cylZShape = new btConvexHullShape(&vertices[0].x(), vertices.size(), sizeof(btVector3));
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

			break;
		}
		case URDF_GEOM_MESH:
		{
			if (visual->m_name.length())
			{
				//b3Printf("visual->name=%s\n", visual->m_name.c_str());
			}
			if (1)//visual->m_geometry)
			{
				if (visual->m_geometry.m_meshFileName.length())
				{
					const char* filename = visual->m_geometry.m_meshFileName.c_str();
					//b3Printf("mesh->filename=%s\n", filename);
					char fullPath[1024];
					int fileType = 0;
                    
                    char tmpPathPrefix[1024];
                    std::string xml_string;
                    int maxPathLen = 1024;
                    b3FileUtils::extractPath(filename,tmpPathPrefix,maxPathLen);
                   
                    char visualPathPrefix[1024];
                    sprintf(visualPathPrefix,"%s%s",urdfPathPrefix,tmpPathPrefix);
                    
                    
					sprintf(fullPath, "%s%s", urdfPathPrefix, filename);
					b3FileUtils::toLower(fullPath);
					if (strstr(fullPath, ".dae"))
					{
						fileType = MY_FILE_COLLADA;
					}
					if (strstr(fullPath, ".stl"))
					{
						fileType = MY_FILE_STL;
					}
                    if (strstr(fullPath,".obj"))
                    {
                        fileType = MY_FILE_OBJ;
                    }


					sprintf(fullPath, "%s%s", urdfPathPrefix, filename);

					visualShapeOut.m_dimensions[0] = visual->m_geometry.m_meshScale[0];
					visualShapeOut.m_dimensions[1] = visual->m_geometry.m_meshScale[1];
					visualShapeOut.m_dimensions[2] = visual->m_geometry.m_meshScale[2];
                    visualShapeOut.m_localVisualFrame[0] = visual->m_linkLocalFrame.getOrigin()[0];
                    visualShapeOut.m_localVisualFrame[1] = visual->m_linkLocalFrame.getOrigin()[1];
                    visualShapeOut.m_localVisualFrame[2] = visual->m_linkLocalFrame.getOrigin()[2];
                    visualShapeOut.m_localVisualFrame[3] = visual->m_linkLocalFrame.getRotation()[0];
                    visualShapeOut.m_localVisualFrame[4] = visual->m_linkLocalFrame.getRotation()[1];
                    visualShapeOut.m_localVisualFrame[5] = visual->m_linkLocalFrame.getRotation()[2];
                    visualShapeOut.m_localVisualFrame[6] = visual->m_linkLocalFrame.getRotation()[3];
                    
					int sl = strlen(fullPath);
					if (sl < (VISUAL_SHAPE_MAX_PATH_LEN-1))
					{
						memcpy(visualShapeOut.m_meshAssetFileName, fullPath, sl);
						visualShapeOut.m_meshAssetFileName[sl] = 0;
					}
					
					FILE* f = fopen(fullPath, "rb");
					if (f)
					{
						fclose(f);
						


						switch (fileType)
						{
                            case MY_FILE_OBJ:
                            {
                                //glmesh = LoadMeshFromObj(fullPath,visualPathPrefix);
								b3ImportMeshData meshData;
								if (b3ImportMeshUtility::loadAndRegisterMeshFromFileInternal(fullPath, meshData))
								{
									
									if (meshData.m_textureImage)
									{
										MyTexture2 texData;
										texData.m_width = meshData.m_textureWidth;
										texData.m_height = meshData.m_textureHeight;
										texData.textureData = meshData.m_textureImage;
										texturesOut.push_back(texData);
									}
									glmesh = meshData.m_gfxShape;
								}

								
                                break;
                            }
                           
						case MY_FILE_STL:
						{
							glmesh = LoadMeshFromSTL(fullPath);
							break;
						}
						case MY_FILE_COLLADA:
						{

							btAlignedObjectArray<GLInstanceGraphicsShape> visualShapes;
							btAlignedObjectArray<ColladaGraphicsInstance> visualShapeInstances;
							btTransform upAxisTrans; upAxisTrans.setIdentity();
							float unitMeterScaling = 1;
							int upAxis = 2;

							LoadMeshFromCollada(fullPath,
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
							//glmesh = LoadMeshFromCollada(fullPath);

							break;
						}
						default:
						{
                            b3Warning("Error: unsupported file type for Visual mesh: %s\n", fullPath);
                            btAssert(0);
						}
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
							b3Warning("issue extracting mesh from COLLADA/STL file %s\n", fullPath);
						}

					}
					else
					{
						b3Warning("mesh geometry not found %s\n", fullPath);
					}


				}
			}


			break;
		}
		default:
		{
			b3Warning("Error: unknown visual geometry type\n");
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
				pos.normalize();
				vtx.normal[0] = pos.x();
				vtx.normal[1] = pos.y();
				vtx.normal[2] = pos.z();
				vtx.uv[0] = 0.5f;
				vtx.uv[1] = 0.5f;
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



void TinyRendererVisualShapeConverter::convertVisualShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame, const UrdfModel& model, class btCollisionObject* colObj, int bodyUniqueId)
{
    
	
	UrdfLink* const* linkPtr = model.m_links.getAtIndex(linkIndex);
	if (linkPtr)
	{

		const UrdfLink* link = *linkPtr;
	
		for (int v1 = 0; v1 < link->m_visualArray.size();v1++)
		{
			btAlignedObjectArray<MyTexture2> textures;
			btAlignedObjectArray<GLInstanceVertex> vertices;
			btAlignedObjectArray<int> indices;
			btTransform startTrans; startTrans.setIdentity();
			//int graphicsIndex = -1;

			const UrdfVisual& vis = link->m_visualArray[v1];
			btTransform childTrans = vis.m_linkLocalFrame;
			btHashString matName(vis.m_materialName.c_str());
			UrdfMaterial *const * matPtr = model.m_materials[matName];
            
            float rgbaColor[4] = {1,1,1,1};
            
			if (matPtr)
			{
				UrdfMaterial *const  mat = *matPtr;
                for (int i=0;i<4;i++)
                    rgbaColor[i] = mat->m_rgbaColor[i];
				//printf("UrdfMaterial %s, rgba = %f,%f,%f,%f\n",mat->m_name.c_str(),mat->m_rgbaColor[0],mat->m_rgbaColor[1],mat->m_rgbaColor[2],mat->m_rgbaColor[3]);
				//m_data->m_linkColors.insert(linkIndex,mat->m_rgbaColor);
			}
			
			TinyRendererObjectArray** visualsPtr = m_data->m_swRenderInstances[colObj];
            if (visualsPtr==0)
            {
                m_data->m_swRenderInstances.insert(colObj,new TinyRendererObjectArray);
            }
            visualsPtr = m_data->m_swRenderInstances[colObj];
            btAssert(visualsPtr);
            TinyRendererObjectArray* visuals = *visualsPtr;
            
			b3VisualShapeData visualShape;
			visualShape.m_objectUniqueId = bodyUniqueId;
			visualShape.m_linkIndex = linkIndex;
            visualShape.m_localVisualFrame[0] = vis.m_linkLocalFrame.getOrigin()[0];
            visualShape.m_localVisualFrame[1] = vis.m_linkLocalFrame.getOrigin()[1];
			visualShape.m_localVisualFrame[2] = vis.m_linkLocalFrame.getOrigin()[2];
			visualShape.m_localVisualFrame[3] = vis.m_linkLocalFrame.getRotation()[0];
			visualShape.m_localVisualFrame[4] = vis.m_linkLocalFrame.getRotation()[1];
			visualShape.m_localVisualFrame[5] = vis.m_linkLocalFrame.getRotation()[2];
			visualShape.m_localVisualFrame[6] = vis.m_linkLocalFrame.getRotation()[3];
            visualShape.m_rgbaColor[0] = rgbaColor[0];
            visualShape.m_rgbaColor[1] = rgbaColor[1];
            visualShape.m_rgbaColor[2] = rgbaColor[2];
            visualShape.m_rgbaColor[3] = rgbaColor[3];
            
			convertURDFToVisualShape(&vis, pathPrefix, localInertiaFrame.inverse()*childTrans, vertices, indices,textures, visualShape);
			m_data->m_visualShapes.push_back(visualShape);

            if (vertices.size() && indices.size())
            {
                TinyRenderObjectData* tinyObj = new TinyRenderObjectData(m_data->m_rgbColorBuffer,m_data->m_depthBuffer, &m_data->m_shadowBuffer, &m_data->m_segmentationMaskBuffer, bodyUniqueId);
				unsigned char* textureImage=0;
				int textureWidth=0;
				int textureHeight=0;
				if (textures.size())
				{
					textureImage = textures[0].textureData;
					textureWidth = textures[0].m_width;
					textureHeight = textures[0].m_height;
				}
				
                tinyObj->registerMeshShape(&vertices[0].xyzw[0],vertices.size(),&indices[0],indices.size(),rgbaColor,
										   textureImage,textureWidth,textureHeight);
                visuals->m_renderObjects.push_back(tinyObj);
            }
			for (int i=0;i<textures.size();i++)
			{
				free(textures[i].textureData);
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


void TinyRendererVisualShapeConverter::setUpAxis(int axis)
{
    m_data->m_upAxis = axis;
    m_data->m_camera.setCameraUpAxis(axis);
    m_data->m_camera.update();
}
void TinyRendererVisualShapeConverter::resetCamera(float camDist, float pitch, float yaw, float camPosX,float camPosY, float camPosZ)
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
    for(int y=0;y<m_data->m_swHeight;++y)
    {
        for(int x=0;x<m_data->m_swWidth;++x)
        {
            m_data->m_rgbColorBuffer.set(x,y,clearColor);
            m_data->m_depthBuffer[x+y*m_data->m_swWidth] = -1e30f;
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

void TinyRendererVisualShapeConverter::render(const float viewMat[16], const float projMat[16]) 
{
    //clear the color buffer
    TGAColor clearColor;
    clearColor.bgra[0] = 255;
    clearColor.bgra[1] = 255;
    clearColor.bgra[2] = 255;
    clearColor.bgra[3] = 255;
    
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
            
            btHashPtr colObjHash = m_data->m_swRenderInstances.getKeyAtIndex(n);
            
            
            const btCollisionObject* colObj = (btCollisionObject*) colObjHash.getPointer();
            
            for (int v=0;v<visualArray->m_renderObjects.size();v++)
            {
                
                TinyRenderObjectData* renderObj = visualArray->m_renderObjects[v];
                
                
                //sync the object transform
                const btTransform& tr = colObj->getWorldTransform();
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
                renderObj->m_localScaling = colObj->getCollisionShape()->getLocalScaling();
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
        
        btHashPtr colObjHash = m_data->m_swRenderInstances.getKeyAtIndex(n);
        
        
        const btCollisionObject* colObj = (btCollisionObject*) colObjHash.getPointer();
        
        for (int v=0;v<visualArray->m_renderObjects.size();v++)
        {
            
            TinyRenderObjectData* renderObj = visualArray->m_renderObjects[v];
            
            
            //sync the object transform
            const btTransform& tr = colObj->getWorldTransform();
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
            renderObj->m_localScaling = colObj->getCollisionShape()->getLocalScaling();
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
                float distance = -m_data->m_depthBuffer[i+startPixelIndex];
                float farPlane = m_data->m_camera.getCameraFrustumFar();
                float nearPlane = m_data->m_camera.getCameraFrustumNear();
                
                btClamp(distance,nearPlane,farPlane);
                
                // the depth buffer value is between 0 and 1
                float a = farPlane / (farPlane - nearPlane);
                float b = farPlane * nearPlane / (nearPlane - farPlane);
                depthBuffer[i] = a + b / distance;
			}
			if (segmentationMaskBuffer)
            {
                segmentationMaskBuffer[i] = m_data->m_segmentationMaskBuffer[i+startPixelIndex];
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
	
	m_data->m_swRenderInstances.clear();
}

void TinyRendererVisualShapeConverter::activateShapeTexture(int shapeUniqueId, int textureUniqueId)
{
    btAssert(textureUniqueId < m_data->m_textures.size());
    TinyRendererObjectArray** ptrptr = m_data->m_swRenderInstances.getAtIndex(shapeUniqueId);
    if (ptrptr && *ptrptr)
    {
        TinyRendererObjectArray* ptr = *ptrptr;
        ptr->m_renderObjects[0]->m_model->setDiffuseTextureFromData(m_data->m_textures[textureUniqueId].textureData,m_data->m_textures[textureUniqueId].m_width,m_data->m_textures[textureUniqueId].m_height);
    }
}

void TinyRendererVisualShapeConverter::activateShapeTexture(int objectUniqueId, int jointIndex, int shapeIndex, int textureUniqueId)
{
    int start = -1;
    for (int i = 0; i < m_data->m_visualShapes.size(); i++)
    {
        if (m_data->m_visualShapes[i].m_objectUniqueId == objectUniqueId && m_data->m_visualShapes[i].m_linkIndex == jointIndex)
        {
            start = i;
            break;
        }
    }
    
    if (start >= 0)
    {
        if (start + shapeIndex < m_data->m_visualShapes.size())
        {
            activateShapeTexture(start + shapeIndex, textureUniqueId);
        }
    }
}

int TinyRendererVisualShapeConverter::registerTexture(unsigned char* texels, int width, int height)
{
    MyTexture2 texData;
    texData.m_width = width;
    texData.m_height = height;
    texData.textureData = texels;
    m_data->m_textures.push_back(texData);
    return m_data->m_textures.size()-1;
}

int TinyRendererVisualShapeConverter::loadTextureFile(const char* filename)
{
    int width,height,n;
    unsigned char* image=0;
    image = stbi_load(filename, &width, &height, &n, 3);
    if (image && (width>=0) && (height>=0))
    {
        return registerTexture(image, width, height);
    }
    return -1;
}
