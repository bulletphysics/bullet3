#include "LoadMeshFromObj.h"
#include"../../ThirdPartyLibs/Wavefront/tiny_obj_loader.h"
#include "../../OpenGLWindow/GLInstanceGraphicsShape.h"
#include <stdio.h> //fopen
#include "Bullet3Common/b3AlignedObjectArray.h"
#include <string>
#include <vector>
#include "Wavefront2GLInstanceGraphicsShape.h"

GLInstanceGraphicsShape* LoadMeshFromObj(const char* relativeFileName, const char* materialPrefixPath)
{
	B3_PROFILE("LoadMeshFromObj");
	std::vector<tinyobj::shape_t> shapes;
	{
		B3_PROFILE("tinyobj::LoadObj2");
		std::string err = tinyobj::LoadObj(shapes, relativeFileName, materialPrefixPath);
	}

	{
		B3_PROFILE("btgCreateGraphicsShapeFromWavefrontObj");
		GLInstanceGraphicsShape* gfxShape = btgCreateGraphicsShapeFromWavefrontObj(shapes);
		return gfxShape;
	}
}
