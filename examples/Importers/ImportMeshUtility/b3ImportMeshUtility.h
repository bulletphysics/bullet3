#ifndef B3_IMPORT_MESH_UTILITY_H
#define B3_IMPORT_MESH_UTILIY_H

#include <string>

class b3ImportMeshUtility
{
public:
static int loadAndRegisterMeshFromFile(const std::string& fileName, class CommonRenderInterface* renderer);

};


#endif //B3_IMPORT_MESH_UTILITY_H

