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
	std::vector<tinyobj::shape_t> m_shapes;
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
	std::vector<tinyobj::shape_t>& shapes,  // [output]
	const char* filename,
	const char* mtl_basepath,
	struct CommonFileIOInterface* fileIO
	)
{
	CachedObjResult* resultPtr = gCachedObjResults[filename];
	if (resultPtr)
	{
		const CachedObjResult& result = *resultPtr;
		shapes = result.m_shapes;
		return result.m_msg;
	}

	std::string err = tinyobj::LoadObj(shapes, filename, mtl_basepath,fileIO);
	CachedObjResult result;
	result.m_msg = err;
	result.m_shapes = shapes;
	if (gEnableFileCaching)
	{
		gCachedObjResults.insert(filename, result);
	}
	return err;
}

GLInstanceGraphicsShape* LoadMeshFromObj(const char* relativeFileName, const char* materialPrefixPath, struct CommonFileIOInterface* fileIO)
{
	B3_PROFILE("LoadMeshFromObj");
	std::vector<tinyobj::shape_t> shapes;
	{
		B3_PROFILE("tinyobj::LoadObj2");
		std::string err = LoadFromCachedOrFromObj(shapes, relativeFileName, materialPrefixPath,fileIO);
	}

	{
		B3_PROFILE("btgCreateGraphicsShapeFromWavefrontObj");
		GLInstanceGraphicsShape* gfxShape = btgCreateGraphicsShapeFromWavefrontObj(shapes);
		return gfxShape;
	}
}
