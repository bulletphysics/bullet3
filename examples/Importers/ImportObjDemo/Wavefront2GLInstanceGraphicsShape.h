#ifndef WAVEFRONT2GRAPHICS_H
#define WAVEFRONT2GRAPHICS_H

#include "../../ThirdPartyLibs/Wavefront/tiny_obj_loader.h"
#include <vector>

struct GLInstanceGraphicsShape* btgCreateGraphicsShapeFromWavefrontObj(const bt_tinyobj::attrib_t& attribute, std::vector<bt_tinyobj::shape_t>& shapes, bool flatShading = false);

#endif  //WAVEFRONT2GRAPHICS_H
