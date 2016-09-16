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
    
   	int m_upAxis;
	int m_swWidth;
	int m_swHeight;
	TGAImage m_rgbColorBuffer;
	b3AlignedObjectArray<float> m_depthBuffer;
	b3AlignedObjectArray<int> m_segmentationMaskBuffer;
	
	SimpleCamera m_camera;
	
	TinyRendererVisualShapeConverterInternalData()
	:m_upAxis(2),
	m_swWidth(START_WIDTH),
	m_swHeight(START_HEIGHT),
	m_rgbColorBuffer(START_WIDTH,START_HEIGHT,TGAImage::RGB)
	{
	    m_depthBuffer.resize(m_swWidth*m_swHeight);
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
	delete m_data;
}
	



void convertURDFToVisualShape(const UrdfVisual* visual, const char* urdfPathPrefix, const btTransform& visualTransform, btAlignedObjectArray<GLInstanceVertex>& verticesOut, btAlignedObjectArray<int>& indicesOut, btAlignedObjectArray<MyTexture2>& texturesOut)
{

	
	GLInstanceGraphicsShape* glmesh = 0;

	btConvexShape* convexColShape = 0;

	switch (visual->m_geometry.m_type)
	{
		case URDF_GEOM_CYLINDER:
		{
			btAlignedObjectArray<btVector3> vertices;
		
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
			
			btVector3 extents = visual->m_geometry.m_boxSize;
			
			btBoxShape* boxShape = new btBoxShape(extents*0.5f);
			//btConvexShape* boxShape = new btConeShapeX(extents[2]*0.5,extents[0]*0.5);
			convexColShape = boxShape;
			convexColShape->setMargin(0.001);
			break;
		}
		case URDF_GEOM_SPHERE:
		{
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



void TinyRendererVisualShapeConverter::convertVisualShapes(int linkIndex, const char* pathPrefix, const btTransform& localInertiaFrame, const UrdfModel& model, class btCollisionObject* colObj, int objectIndex)
{
    
	
	UrdfLink* const* linkPtr = model.m_links.getAtIndex(linkIndex);
	if (linkPtr)
	{

		const UrdfLink* link = *linkPtr;
	
		for (int v = 0; v < link->m_visualArray.size();v++)
		{
			btAlignedObjectArray<MyTexture2> textures;
			btAlignedObjectArray<GLInstanceVertex> vertices;
			btAlignedObjectArray<int> indices;
			btTransform startTrans; startTrans.setIdentity();
			int graphicsIndex = -1;

			const UrdfVisual& vis = link->m_visualArray[v];
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
            
			convertURDFToVisualShape(&vis, pathPrefix, localInertiaFrame.inverse()*childTrans, vertices, indices,textures);

            if (vertices.size() && indices.size())
            {
                TinyRenderObjectData* tinyObj = new TinyRenderObjectData(m_data->m_rgbColorBuffer,m_data->m_depthBuffer, &m_data->m_segmentationMaskBuffer, objectIndex);
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
    switch (m_data->m_upAxis)
    {
    case 1:
            lightDirWorld = btVector3(-50.f,100,30);
        break;
    case 2:
            lightDirWorld = btVector3(-50.f,30,100);
            break;
    default:{}
    };
    
    lightDirWorld.normalize();
    
  //  printf("num m_swRenderInstances = %d\n", m_data->m_swRenderInstances.size());
    for (int i=0;i<m_data->m_swRenderInstances.size();i++)
    {
        TinyRendererObjectArray** visualArrayPtr = m_data->m_swRenderInstances.getAtIndex(i);
        if (0==visualArrayPtr)
            continue;//can this ever happen?
        TinyRendererObjectArray* visualArray = *visualArrayPtr;

        btHashPtr colObjHash = m_data->m_swRenderInstances.getKeyAtIndex(i);
        
        
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
                    renderObj->m_localScaling = colObj->getCollisionShape()->getLocalScaling();
                    renderObj->m_lightDirWorld = lightDirWorld;
                }
            }
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
				depthBuffer[i] = m_data->m_depthBuffer[i+startPixelIndex];
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
			delete ptr;
		}
	}
	
	m_data->m_swRenderInstances.clear();
}

