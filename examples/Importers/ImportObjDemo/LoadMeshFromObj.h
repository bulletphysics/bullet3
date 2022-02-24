#ifndef LOAD_MESH_FROM_OBJ_H
#define LOAD_MESH_FROM_OBJ_H

struct GLInstanceGraphicsShape;

#include "../../ThirdPartyLibs/Wavefront/tiny_obj_loader.h"

int b3IsFileCachingEnabled();
void b3EnableFileCaching(int enable);

std::string LoadFromCachedOrFromObj(
	bt_tinyobj::attrib_t& attribute,
	std::vector<bt_tinyobj::shape_t>& shapes,  // [output]
	const char* filename,
	const char* mtl_basepath,
	struct CommonFileIOInterface* fileIO);

GLInstanceGraphicsShape* LoadMeshFromObj(const char* relativeFileName, const char* materialPrefixPath,struct CommonFileIOInterface* fileIO);

#endif  //LOAD_MESH_FROM_OBJ_H
