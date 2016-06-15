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
	std::vector<tinyobj::shape_t> shapes;
	std::string err = tinyobj::LoadObj(shapes, relativeFileName, materialPrefixPath);
		
	GLInstanceGraphicsShape* gfxShape = btgCreateGraphicsShapeFromWavefrontObj(shapes);
	return gfxShape;
}
