#include "LoadMeshFromObj.h"

#include "../../OpenGLWindow/GLInstanceGraphicsShape.h"
#include <stdio.h>  //fopen
#include "Bullet3Common/b3AlignedObjectArray.h"
#include <string>
#include <vector>
#include "Wavefront2GLInstanceGraphicsShape.h"
#include "Bullet3Common/b3HashMap.h"

struct CachedObjResult
{
	std::string m_msg;
	std::vector<bt_tinyobj::shape_t> m_shapes;
	bt_tinyobj::attrib_t m_attribute;
};

static b3HashMap<b3HashString, CachedObjResult> gCachedObjResults;
static int gEnableFileCaching = 1;

int b3IsFileCachingEnabled()
{
	return gEnableFileCaching;
}
void b3EnableFileCaching(int enable)
{
	gEnableFileCaching = enable;
	if (enable == 0)
	{
		gCachedObjResults.clear();
	}
}

std::string LoadFromCachedOrFromObj(
	bt_tinyobj::attrib_t& attribute,
	std::vector<bt_tinyobj::shape_t>& shapes,  // [output]
	const char* filename,
	const char* mtl_basepath,
	struct CommonFileIOInterface* fileIO)
{
	CachedObjResult* resultPtr = gCachedObjResults[filename];
	if (resultPtr)
	{
		const CachedObjResult& result = *resultPtr;
		shapes = result.m_shapes;
		attribute = result.m_attribute;
		return result.m_msg;
	}

	std::string err = bt_tinyobj::LoadObj(attribute, shapes, filename, mtl_basepath, fileIO);
	CachedObjResult result;
	result.m_msg = err;
	result.m_shapes = shapes;
	result.m_attribute = attribute;
	if (gEnableFileCaching)
	{
		gCachedObjResults.insert(filename, result);
	}
	return err;
}

GLInstanceGraphicsShape* LoadMeshFromObj(const char* relativeFileName, const char* materialPrefixPath, struct CommonFileIOInterface* fileIO)
{
	B3_PROFILE("LoadMeshFromObj");
	std::vector<bt_tinyobj::shape_t> shapes;
	bt_tinyobj::attrib_t attribute;
	{
		B3_PROFILE("bt_tinyobj::LoadObj2");
		std::string err = LoadFromCachedOrFromObj(attribute, shapes, relativeFileName, materialPrefixPath, fileIO);
	}

	{
		B3_PROFILE("btgCreateGraphicsShapeFromWavefrontObj");
		GLInstanceGraphicsShape* gfxShape = btgCreateGraphicsShapeFromWavefrontObj(attribute, shapes);
		return gfxShape;
	}
}
