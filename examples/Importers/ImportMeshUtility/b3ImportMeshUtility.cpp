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


int b3ImportMeshUtility::loadAndRegisterMeshFromFile(const std::string& fileName, CommonRenderInterface* renderer)
{
    int shapeId = -1;

    char relativeFileName[1024];
    if (b3ResourcePath::findResourcePath(fileName.c_str(), relativeFileName, 1024))
    {
        char pathPrefix[1024];

        b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);


	
        btVector3 shift(0,0,0);
        
    //	int index=10;
	
	{
		
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
				const unsigned char* image=0;

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
                    } else
                    {
                        b3Warning("not found %s\n",relativeFileName);
                    }
				}
		
				if (image)
				{
					textureIndex = renderer->registerTexture(image,width,height);
					if (textureIndex>=0)
					{
						break;
					}
				}

			}

		}
		
		shapeId = renderer->registerShape(&gfxShape->m_vertices->at(0).xyzw[0], gfxShape->m_numvertices, &gfxShape->m_indices->at(0), gfxShape->m_numIndices,B3_GL_TRIANGLES,textureIndex);
		
	
		

	
	}
	}
    else
    {
            b3Warning("Cannot find %s\n", fileName.c_str());
    }
    return shapeId;
}

