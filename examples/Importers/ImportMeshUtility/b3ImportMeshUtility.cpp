#include "b3ImportMeshUtility.h"

#include <vector>
#include"../../ThirdPartyLibs/Wavefront/tiny_obj_loader.h"
#include "LinearMath/btVector3.h"
#include "../ImportObjDemo/Wavefront2GLInstanceGraphicsShape.h"
#include "../../Utils/b3ResourcePath.h"
#include "Bullet3Common/b3FileUtils.h"
#include "stb_image/stb_image.h"
#include "../ImportObjDemo/LoadMeshFromObj.h"
bool b3ImportMeshUtility::loadAndRegisterMeshFromFileInternal(const std::string& fileName, b3ImportMeshData& meshData)
{
	B3_PROFILE("loadAndRegisterMeshFromFileInternal");
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
		{
			B3_PROFILE("tinyobj::LoadObj");
			std::string err = LoadFromCachedOrFromObj(shapes, relativeFileName, pathPrefix);
			//std::string err = tinyobj::LoadObj(shapes, relativeFileName, pathPrefix);
		}
		
		GLInstanceGraphicsShape* gfxShape = btgCreateGraphicsShapeFromWavefrontObj(shapes);
		
		//int textureIndex = -1;
		//try to load some texture
		for (int i=0; meshData.m_textureImage==0  && i<shapes.size();i++)
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
							b3Warning("Unsupported texture image format [%s]\n",relativeFileName);
							meshData.m_textureWidth = 0;
							meshData.m_textureHeight = 0;
							break;
						}

                    } else
                    {
                        b3Warning("not found [%s]\n",relativeFileName);
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


