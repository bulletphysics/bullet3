#ifndef B3_IMPORT_MESH_UTILITY_H
#define B3_IMPORT_MESH_UTILITY_H

#include <string>

struct b3ImportMeshData
{
	struct GLInstanceGraphicsShape* m_gfxShape;

	unsigned char* m_textureImage;//in 3 component 8-bit RGB data
	int m_textureWidth;
	int m_textureHeight;
};

class b3ImportMeshUtility
{
public:

static bool loadAndRegisterMeshFromFileInternal(const std::string& fileName, b3ImportMeshData& meshData);

};


#endif //B3_IMPORT_MESH_UTILITY_H

