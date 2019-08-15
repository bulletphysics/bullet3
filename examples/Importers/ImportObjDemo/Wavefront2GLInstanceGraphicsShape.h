#ifndef WAVEFRONT2GRAPHICS_H
#define WAVEFRONT2GRAPHICS_H

#include "../../ThirdPartyLibs/Wavefront/tiny_obj_loader.h"
#include <vector>

struct GLInstanceGraphicsShape* btgCreateGraphicsShapeFromWavefrontObj(const tinyobj::attrib_t& attribute, std::vector<tinyobj::shape_t>& shapes, bool flatShading = false);

#endif  //WAVEFRONT2GRAPHICS_H
