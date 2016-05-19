#include "b3ImportMeshUtility.h"

#include <vector>
#include "../../OpenGLWindow/GLInstancingRenderer.h"
#include"Wavefront/tiny_obj_loader.h"
#include "../../OpenGLWindow/GLInstanceGraphicsShape.h"
#include "btBulletDynamicsCommon.h"
#include "../../OpenGLWindow/SimpleOpenGL3App.h"
#include "../ImportObjDemo/Wavefront2GLInstanceGraphicsShape.h"
#include "../../Utils/b3ResourcePath.h"
#include "Bullet3Common/b3FileUtils.h"
#include "stb_image/stb_image.h"


bool b3ImportMeshUtility::loadAndRegisterMeshFromFileInternal(const std::string& fileName, b3ImportMeshData& meshData)
{

	meshData.m_gfxShape = 0;
	meshData.m_textureImage = 0;
	meshData.m_textureHeight = 0;
	meshData.m_textureWidth = 0;


	char relativeFileName[1024];
    if (b3ResourcePath::findResourcePath(fileName.c_str(), relativeFileName, 1024))
    {
        char pathPrefix[1024];

        b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);
		btVector3 shift(0,0,0);
    	
		std::vector<tinyobj::shape_t> shapes;
		std::string err = tinyobj::LoadObj(shapes, relativeFileName, pathPrefix);
		
		GLInstanceGraphicsShape* gfxShape = btgCreateGraphicsShapeFromWavefrontObj(shapes);
		
		int textureIndex = -1;
		//try to load some texture
		for (int i=0;i<shapes.size();i++)
		{
			const tinyobj::shape_t& shape = shapes[i];
			if (shape.material.diffuse_texname.length()>0)
			{

				int width,height,n;
				const char* filename = shape.material.diffuse_texname.c_str();
				unsigned char* image=0;

				const char* prefix[]={ pathPrefix,"./","./data/","../data/","../../data/","../../../data/","../../../../data/"};
				int numprefix = sizeof(prefix)/sizeof(const char*);
		
				for (int i=0;!image && i<numprefix;i++)
				{
					char relativeFileName[1024];
					sprintf(relativeFileName,"%s%s",prefix[i],filename);
					char  relativeFileName2[1024];
					if (b3ResourcePath::findResourcePath(relativeFileName, relativeFileName2, 1024))
                    {
                        image = stbi_load(relativeFileName, &width, &height, &n, 3);
						meshData.m_textureImage = image;
						if (image)
						{
							meshData.m_textureWidth = width;
							meshData.m_textureHeight = height;
						} else
						{
							meshData.m_textureWidth = 0;
							meshData.m_textureHeight = 0;
						}

                    } else
                    {
                        b3Warning("not found %s\n",relativeFileName);
                    }
				}
			}

		}
		meshData.m_gfxShape = gfxShape;
		return true;
	
	}
    else
    {
            b3Warning("Cannot find %s\n", fileName.c_str());
    }

	return false;
}

int b3ImportMeshUtility::loadAndRegisterMeshFromFile(const std::string& fileName, CommonRenderInterface* renderer)
{
	int shapeId = -1;

	b3ImportMeshData meshData;
	if (b3ImportMeshUtility::loadAndRegisterMeshFromFileInternal(fileName, meshData))
	{
		int textureIndex = 0;

		if (meshData.m_textureImage)
		{
			textureIndex = renderer->registerTexture(meshData.m_textureImage,meshData.m_textureWidth,meshData.m_textureHeight);
		}

		shapeId = renderer->registerShape(&meshData.m_gfxShape->m_vertices->at(0).xyzw[0], 
									meshData.m_gfxShape->m_numvertices, 
									&meshData.m_gfxShape->m_indices->at(0), 
									meshData.m_gfxShape->m_numIndices,
									B3_GL_TRIANGLES,
									textureIndex);
		delete meshData.m_gfxShape;
		delete meshData.m_textureImage;
	}
	return shapeId;
}
