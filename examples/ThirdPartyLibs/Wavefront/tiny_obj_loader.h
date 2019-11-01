//
// Copyright 2012-2013, Syoyo Fujita.
//
// Licensed under 2-clause BSD liecense.
//
#ifndef _TINY_OBJ_LOADER_H
#define _TINY_OBJ_LOADER_H

#include <string>
#include <vector>
#include <map>

struct CommonFileIOInterface;

namespace tinyobj
{
struct vertex_index_t
{
	int v_idx, vt_idx, vn_idx;
	vertex_index_t() : v_idx(-1), vt_idx(-1), vn_idx(-1) {}
	explicit vertex_index_t(int idx) : v_idx(idx), vt_idx(idx), vn_idx(idx) {}
	vertex_index_t(int vidx, int vtidx, int vnidx)
		: v_idx(vidx), vt_idx(vtidx), vn_idx(vnidx) {}
};

typedef std::vector<vertex_index_t> face_t;

typedef struct
{
	std::string name;

	float ambient[3];
	float diffuse[3];
	float specular[3];
	float transmittance[3];
	float emission[3];
	float shininess;
	float transparency;  // 1 == opaque; 0 == fully transparent

	std::string ambient_texname;   // map_Ka
	std::string diffuse_texname;   // map_Kd
	std::string specular_texname;  // map_Ks
	std::string normal_texname;
	std::map<std::string, std::string> unknown_parameter;
} material_t;

// Index struct to support different indices for vtx/normal/texcoord.
// -1 means not used.
typedef struct
{
	int vertex_index;
	int normal_index;
	int texcoord_index;
} index_t;

typedef struct
{
	std::vector<index_t> indices;
} mesh_t;

typedef struct
{
	std::string name;
	material_t material;
	mesh_t mesh;
} shape_t;

// Vertex attributes
struct attrib_t
{
	std::vector<float> vertices;   // 'v'(xyz)
	std::vector<float> normals;    // 'vn'
	std::vector<float> texcoords;  // 'vt'(uv)
	attrib_t() {}
};
/// Loads .obj from a file.
/// 'shapes' will be filled with parsed shape data
/// The function returns error string.
/// Returns empty string when loading .obj success.
/// 'mtl_basepath' is optional, and used for base path for .mtl file.
#ifdef USE_STREAM
std::string LoadObj(
	attrib_t& attrib,
	std::vector<shape_t>& shapes,  // [output]
	const char* filename,
	const char* mtl_basepath = NULL);
#else
std::string
LoadObj(
	attrib_t& attrib,
	std::vector<shape_t>& shapes,
	const char* filename,
	const char* mtl_basepath,
	CommonFileIOInterface* fileIO);
#endif

};  // namespace tinyobj

#endif  // _TINY_OBJ_LOADER_H
