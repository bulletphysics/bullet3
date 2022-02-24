#include "b3ImportMeshUtility.h"

#include <vector>
#include "../../ThirdPartyLibs/Wavefront/tiny_obj_loader.h"
#include "LinearMath/btVector3.h"
#include "../ImportObjDemo/Wavefront2GLInstanceGraphicsShape.h"
#include "../../Utils/b3ResourcePath.h"
#include "Bullet3Common/b3FileUtils.h"
#include "stb_image/stb_image.h"
#include "../ImportObjDemo/LoadMeshFromObj.h"
#include "Bullet3Common/b3HashMap.h"
#include "../../CommonInterfaces/CommonFileIOInterface.h"

struct CachedTextureResult
{
	std::string m_textureName;

	int m_width;
	int m_height;
	unsigned char* m_pixels;
	CachedTextureResult()
		: m_width(0),
		  m_height(0),
		  m_pixels(0)
	{
	}
};

static b3HashMap<b3HashString, CachedTextureResult> gCachedTextureResults;
struct CachedTextureManager
{
	CachedTextureManager()
	{
	}
	virtual ~CachedTextureManager()
	{
		for (int i = 0; i < gCachedTextureResults.size(); i++)
		{
			CachedTextureResult* res = gCachedTextureResults.getAtIndex(i);
			if (res)
			{
				free(res->m_pixels);
			}
		}
	}
};
static CachedTextureManager sTexCacheMgr;

bool b3ImportMeshUtility::loadAndRegisterMeshFromFileInternal(const std::string& fileName, b3ImportMeshData& meshData, struct CommonFileIOInterface* fileIO)
{
	B3_PROFILE("loadAndRegisterMeshFromFileInternal");
	meshData.m_gfxShape = 0;
	meshData.m_textureImage1 = 0;
	meshData.m_textureHeight = 0;
	meshData.m_textureWidth = 0;
	meshData.m_flags = 0;
	meshData.m_isCached = false;

	char relativeFileName[1024];
	if (fileIO->findResourcePath(fileName.c_str(), relativeFileName, 1024))
	{
		char pathPrefix[1024];

		b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);
		btVector3 shift(0, 0, 0);

		std::vector<bt_tinyobj::shape_t> shapes;
		bt_tinyobj::attrib_t attribute;
		{
			B3_PROFILE("tinyobj::LoadObj");
			std::string err = LoadFromCachedOrFromObj(attribute, shapes, relativeFileName, pathPrefix, fileIO);
			//std::string err = tinyobj::LoadObj(shapes, relativeFileName, pathPrefix);
		}

		GLInstanceGraphicsShape* gfxShape = btgCreateGraphicsShapeFromWavefrontObj(attribute, shapes);
		{
			B3_PROFILE("Load Texture");
			//int textureIndex = -1;
			//try to load some texture
			for (int i = 0; meshData.m_textureImage1 == 0 && i < shapes.size(); i++)
			{
				const bt_tinyobj::shape_t& shape = shapes[i];
				meshData.m_rgbaColor[0] = shape.material.diffuse[0];
				meshData.m_rgbaColor[1] = shape.material.diffuse[1];
				meshData.m_rgbaColor[2] = shape.material.diffuse[2];
				meshData.m_rgbaColor[3] = shape.material.transparency;
				meshData.m_flags |= B3_IMPORT_MESH_HAS_RGBA_COLOR;

				meshData.m_specularColor[0] = shape.material.specular[0];
				meshData.m_specularColor[1] = shape.material.specular[1];
				meshData.m_specularColor[2] = shape.material.specular[2];
				meshData.m_specularColor[3] = 1;
				meshData.m_flags |= B3_IMPORT_MESH_HAS_SPECULAR_COLOR;
				
				if (shape.material.diffuse_texname.length() > 0)
				{
					int width, height, n;
					const char* filename = shape.material.diffuse_texname.c_str();
					unsigned char* image = 0;

					const char* prefix[] = {pathPrefix, "./", "./data/", "../data/", "../../data/", "../../../data/", "../../../../data/"};
					int numprefix = sizeof(prefix) / sizeof(const char*);

					for (int i = 0; !image && i < numprefix; i++)
					{
						char relativeFileName[1024];
						sprintf(relativeFileName, "%s%s", prefix[i], filename);
						char relativeFileName2[1024];
						if (fileIO->findResourcePath(relativeFileName, relativeFileName2, 1024))
						{
							if (b3IsFileCachingEnabled())
							{
								CachedTextureResult* texture = gCachedTextureResults[relativeFileName];
								if (texture)
								{
									image = texture->m_pixels;
									width = texture->m_width;
									height = texture->m_height;
									meshData.m_textureWidth = width;
									meshData.m_textureHeight = height;
									meshData.m_textureImage1 = image;
									meshData.m_isCached = true;
								}
							}

							if (image == 0)
							{

								b3AlignedObjectArray<char> buffer;
								buffer.reserve(1024);
								int fileId = fileIO->fileOpen(relativeFileName,"rb");
								if (fileId>=0)
								{
									int size = fileIO->getFileSize(fileId);
									if (size>0)
									{
										buffer.resize(size);
										int actual = fileIO->fileRead(fileId,&buffer[0],size);
										if (actual != size)
										{
											b3Warning("STL filesize mismatch!\n");
											buffer.resize(0);
										}
									}
									fileIO->fileClose(fileId);
								}

								if (buffer.size())
								{
									image = stbi_load_from_memory((const unsigned char*)&buffer[0], buffer.size(), &width, &height, &n, 3);
								}
								//image = stbi_load(relativeFileName, &width, &height, &n, 3);

								meshData.m_textureImage1 = image;

								if (image)
								{
									meshData.m_textureWidth = width;
									meshData.m_textureHeight = height;

									if (b3IsFileCachingEnabled())
									{
										CachedTextureResult result;
										result.m_textureName = relativeFileName;
										result.m_width = width;
										result.m_height = height;
										result.m_pixels = image;
										meshData.m_isCached = true;
										gCachedTextureResults.insert(relativeFileName, result);
									}
								}
								else
								{
									b3Warning("Unsupported texture image format [%s]\n", relativeFileName);

									break;
								}
							}
						}
						else
						{
							b3Warning("not found [%s]\n", relativeFileName);
						}
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
