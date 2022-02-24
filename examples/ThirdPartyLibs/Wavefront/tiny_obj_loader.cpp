//
// Copyright 2012-2013, Syoyo Fujita.
//
// Licensed under 2-clause BSD liecense.
//

// Erwin Coumans: improved performance, especially in debug mode on Visual Studio (25sec -> 4sec)
//
// version 0.9.5: Parse multiple group name.
//                Add support of specifying the base path to load material file.
// version 0.9.4: Initial suupport of group tag(g)
// version 0.9.3: Fix parsing triple 'x/y/z'
// version 0.9.2: Add more .mtl load support
// version 0.9.1: Add initial .mtl load support
// version 0.9.0: Initial
//

#include <cstdlib>
#include <cstring>
#include <cassert>

#include <string>
#include <vector>
#include <map>
#ifdef USE_STREAM
#include <fstream>
#else
#include "../../CommonInterfaces/CommonFileIOInterface.h"
#endif
#include <sstream>
#include "tiny_obj_loader.h"
#include <stdio.h>

namespace bt_tinyobj
{
#ifdef USE_STREAM
//See http://stackoverflow.com/questions/6089231/getting-std-ifstream-to-handle-lf-cr-and-crlf
std::istream& safeGetline(std::istream& is, std::string& t)
{
	t.clear();

	// The characters in the stream are read one-by-one using a std::streambuf.
	// That is faster than reading them one-by-one using the std::istream.
	// Code that uses streambuf this way must be guarded by a sentry object.
	// The sentry object performs various tasks,
	// such as thread synchronization and updating the stream state.

	std::istream::sentry se(is, true);
	std::streambuf* sb = is.rdbuf();

	for (;;)
	{
		int c = sb->sbumpc();
		switch (c)
		{
			case '\n':
				return is;
			case '\r':
				if (sb->sgetc() == '\n')
					sb->sbumpc();
				return is;
			case EOF:
				// Also handle the case when the last line has no line ending
				if (t.empty())
					is.setstate(std::ios::eofbit);
				return is;
			default:
				t += (char)c;
		}
	}
}
#endif

static inline bool isSpace(const char c)
{
	return (c == ' ') || (c == '\t');
}

static inline bool isNewLine(const char c)
{
	return (c == '\r') || (c == '\n') || (c == '\0');
}

// Make index zero-base, and also support relative index.
static inline bool fixIndex(int idx, int n, int* ret)
{
	if (!ret)
	{
		return false;
	}

	if (idx > 0)
	{
		(*ret) = idx - 1;
		return true;
	}

	if (idx == 0)
	{
		// zero is not allowed according to the spec.
		return false;
	}

	if (idx < 0)
	{
		(*ret) = n + idx;  // negative value = relative
		return true;
	}

	return false;  // never reach here.
}
static inline std::string parseString(const char*& token)
{
	std::string s;
	int b = strspn(token, " \t");
	int e = strcspn(token, " \t\r");
	s = std::string(&token[b], &token[e]);

	token += (e - b);
	return s;
}

static inline float parseFloat(const char*& token)
{
	token += strspn(token, " \t");
	float f = (float)atof(token);
	token += strcspn(token, " \t\r");
	return f;
}

static inline void parseFloat2(
	float& x, float& y,
	const char*& token)
{
	x = parseFloat(token);
	y = parseFloat(token);
}

static inline void parseFloat3(
	float& x, float& y, float& z,
	const char*& token)
{
	x = parseFloat(token);
	y = parseFloat(token);
	z = parseFloat(token);
}

// Parse triples with index offsets: i, i/j/k, i//k, i/j
static bool parseTriple(const char** token, int vsize, int vnsize, int vtsize,
						vertex_index_t* ret)
{
	if (!ret)
	{
		return false;
	}

	vertex_index_t vi(-1);

	if (!fixIndex(atoi((*token)), vsize, &(vi.v_idx)))
	{
		return false;
	}

	(*token) += strcspn((*token), "/ \t\r");
	if ((*token)[0] != '/')
	{
		(*ret) = vi;
		return true;
	}
	(*token)++;

	// i//k
	if ((*token)[0] == '/')
	{
		(*token)++;
		if (!fixIndex(atoi((*token)), vnsize, &(vi.vn_idx)))
		{
			return false;
		}
		(*token) += strcspn((*token), "/ \t\r");
		(*ret) = vi;
		return true;
	}

	// i/j/k or i/j
	if (!fixIndex(atoi((*token)), vtsize, &(vi.vt_idx)))
	{
		return false;
	}

	(*token) += strcspn((*token), "/ \t\r");
	if ((*token)[0] != '/')
	{
		(*ret) = vi;
		return true;
	}

	// i/j/k
	(*token)++;  // skip '/'
	if (!fixIndex(atoi((*token)), vnsize, &(vi.vn_idx)))
	{
		return false;
	}
	(*token) += strcspn((*token), "/ \t\r");

	(*ret) = vi;

	return true;
}

static bool exportFaceGroupToShape(shape_t* shape, const std::vector<face_t>& face_group,
								   const material_t material, const std::string& name,
								   const std::vector<float>& v)
{
	if (face_group.empty())
	{
		return false;
	}

	shape->name = name;
	// Flattened version of vertex data

	// Flatten vertices and indices
	for (size_t i = 0; i < face_group.size(); i++)
	{
		const face_t& face = face_group[i];
		size_t npolys = face.size();

		if (npolys < 3)
		{
			// Face must have 3+ vertices.
			continue;
		}
		vertex_index_t i0 = face[0];
		vertex_index_t i1(-1);
		vertex_index_t i2 = face[1];

		face_t remainingFace = face;  // copy
		size_t guess_vert = 0;
		vertex_index_t ind[3];

		// How many iterations can we do without decreasing the remaining
		// vertices.
		size_t remainingIterations = face.size();
		size_t previousRemainingVertices = remainingFace.size();

		while (remainingFace.size() > 3 && remainingIterations > 0)
		{
			npolys = remainingFace.size();
			if (guess_vert >= npolys)
			{
				guess_vert -= npolys;
			}

			if (previousRemainingVertices != npolys)
			{
				// The number of remaining vertices decreased. Reset counters.
				previousRemainingVertices = npolys;
				remainingIterations = npolys;
			}
			else
			{
				// We didn't consume a vertex on previous iteration, reduce the
				// available iterations.
				remainingIterations--;
			}

			for (size_t k = 0; k < 3; k++)
			{
				ind[k] = remainingFace[(guess_vert + k) % npolys];
				size_t vi = size_t(ind[k].v_idx);
			}
			// this triangle is an ear
			{
				index_t idx0, idx1, idx2;
				idx0.vertex_index = ind[0].v_idx;
				idx0.normal_index = ind[0].vn_idx;
				idx0.texcoord_index = ind[0].vt_idx;
				idx1.vertex_index = ind[1].v_idx;
				idx1.normal_index = ind[1].vn_idx;
				idx1.texcoord_index = ind[1].vt_idx;
				idx2.vertex_index = ind[2].v_idx;
				idx2.normal_index = ind[2].vn_idx;
				idx2.texcoord_index = ind[2].vt_idx;

				shape->mesh.indices.push_back(idx0);
				shape->mesh.indices.push_back(idx1);
				shape->mesh.indices.push_back(idx2);
			}

			// remove v1 from the list
			size_t removed_vert_index = (guess_vert + 1) % npolys;
			while (removed_vert_index + 1 < npolys)
			{
				remainingFace[removed_vert_index] =
					remainingFace[removed_vert_index + 1];
				removed_vert_index += 1;
			}
			remainingFace.pop_back();
		}

		if (remainingFace.size() == 3)
		{
			i0 = remainingFace[0];
			i1 = remainingFace[1];
			i2 = remainingFace[2];
			{
				index_t idx0, idx1, idx2;
				idx0.vertex_index = i0.v_idx;
				idx0.normal_index = i0.vn_idx;
				idx0.texcoord_index = i0.vt_idx;
				idx1.vertex_index = i1.v_idx;
				idx1.normal_index = i1.vn_idx;
				idx1.texcoord_index = i1.vt_idx;
				idx2.vertex_index = i2.v_idx;
				idx2.normal_index = i2.vn_idx;
				idx2.texcoord_index = i2.vt_idx;

				shape->mesh.indices.push_back(idx0);
				shape->mesh.indices.push_back(idx1);
				shape->mesh.indices.push_back(idx2);
			}
		}
	}
	shape->material = material;
	return true;
}

void InitMaterial(material_t& material)
{
	material.name = "";
	material.ambient_texname = "";
	material.diffuse_texname = "";
	material.specular_texname = "";
	for (int i = 0; i < 3; i++)
	{
		material.ambient[i] = 0.f;
		material.diffuse[i] = 0.f;
		material.specular[i] = 0.f;
		material.transmittance[i] = 0.f;
		material.emission[i] = 0.f;
	}
	material.shininess = 1.f;
	material.transparency = 1.f;
}

std::string LoadMtl(
	std::map<std::string, material_t>& material_map,
	const char* filename,
	const char* mtl_basepath,
	CommonFileIOInterface* fileIO)
{
	material_map.clear();
	std::stringstream err;

	std::string filepath;

	if (mtl_basepath)
	{
		filepath = std::string(mtl_basepath) + std::string(filename);
	}
	else
	{
		filepath = std::string(filename);
	}
#ifdef USE_STREAM
	std::ifstream ifs(filepath.c_str());
	if (!ifs)
	{
		err << "Cannot open file [" << filepath << "]" << std::endl;
		return err.str();
	}
#else
	int fileHandle = fileIO->fileOpen(filepath.c_str(), "r");
	if (fileHandle < 0)
	{
		err << "Cannot open file [" << filepath << "]" << std::endl;
		return err.str();
	}
#endif

	material_t material;

	int maxchars = 8192;              // Alloc enough size.
	std::vector<char> buf(maxchars);  // Alloc enough size.
#ifdef USE_STREAM
	while (ifs.peek() != -1)
#else
	char* line = 0;
	do
#endif
	{
		std::string linebuf;
#ifdef USE_STREAM
		safeGetline(ifs, linebuf);
#else
		char tmpBuf[1024];
		line = fileIO->readLine(fileHandle, tmpBuf, 1024);
		if (line)
		{
			linebuf = line;
		}
#endif

		// Trim newline '\r\n' or '\r'
		if (linebuf.size() > 0)
		{
			if (linebuf[linebuf.size() - 1] == '\n') linebuf.erase(linebuf.size() - 1);
		}
		if (linebuf.size() > 0)
		{
			if (linebuf[linebuf.size() - 1] == '\r') linebuf.erase(linebuf.size() - 1);
		}

		// Skip if empty line.
		if (linebuf.empty())
		{
			continue;
		}

		linebuf = linebuf.substr(0, linebuf.find_last_not_of(" \t") + 1);
		// Skip leading space.
		const char* token = linebuf.c_str();
		token += strspn(token, " \t");

		assert(token);
		if (token[0] == '\0') continue;  // empty line

		if (token[0] == '#') continue;  // comment line

		// new mtl
		if ((0 == strncmp(token, "newmtl", 6)) && isSpace((token[6])))
		{
			// flush previous material.
			material_map.insert(std::pair<std::string, material_t>(material.name, material));

			// initial temporary material
			InitMaterial(material);

			// set new mtl name
			char namebuf[4096];
			token += 7;
			sscanf(token, "%s", namebuf);
			material.name = namebuf;
			continue;
		}

		// ambient
		if (token[0] == 'K' && token[1] == 'a' && isSpace((token[2])))
		{
			token += 2;
			float r, g, b;
			parseFloat3(r, g, b, token);
			material.ambient[0] = r;
			material.ambient[1] = g;
			material.ambient[2] = b;
			continue;
		}

		// diffuse
		if (token[0] == 'K' && token[1] == 'd' && isSpace((token[2])))
		{
			token += 2;
			float r, g, b;
			parseFloat3(r, g, b, token);
			material.diffuse[0] = r;
			material.diffuse[1] = g;
			material.diffuse[2] = b;
			continue;
		}

		// specular
		if (token[0] == 'K' && token[1] == 's' && isSpace((token[2])))
		{
			token += 2;
			float r, g, b;
			parseFloat3(r, g, b, token);
			material.specular[0] = r;
			material.specular[1] = g;
			material.specular[2] = b;
			continue;
		}

		// specular
		if (token[0] == 'K' && token[1] == 't' && isSpace((token[2])))
		{
			token += 2;
			float r, g, b;
			parseFloat3(r, g, b, token);
			material.specular[0] = r;
			material.specular[1] = g;
			material.specular[2] = b;
			continue;
		}

		// emission
		if (token[0] == 'K' && token[1] == 'e' && isSpace(token[2]))
		{
			token += 2;
			float r, g, b;
			parseFloat3(r, g, b, token);
			material.emission[0] = r;
			material.emission[1] = g;
			material.emission[2] = b;
			continue;
		}

		// shininess
		if (token[0] == 'N' && token[1] == 's' && isSpace(token[2]))
		{
			token += 2;
			material.shininess = parseFloat(token);
			continue;
		}

		// transparency
		if (token[0] == 'T' && token[1] == 'r' && isSpace(token[2]))
		{
			token += 2;
			material.transparency = parseFloat(token);
			continue;
		}

		// transparency
		if (token[0] == 'd' && isSpace(token[1]))
		{
			token += 1;
			material.transparency = parseFloat(token);
			continue;
		}

		// ambient texture
		if ((0 == strncmp(token, "map_Ka", 6)) && isSpace(token[6]))
		{
			token += 7;
			material.ambient_texname = token;
			continue;
		}

		// diffuse texture
		if ((0 == strncmp(token, "map_Kd", 6)) && isSpace(token[6]))
		{
			token += 7;
			material.diffuse_texname = token;
			continue;
		}

		// specular texture
		if ((0 == strncmp(token, "map_Ks", 6)) && isSpace(token[6]))
		{
			token += 7;
			material.specular_texname = token;
			continue;
		}

		// normal texture
		if ((0 == strncmp(token, "map_Ns", 6)) && isSpace(token[6]))
		{
			token += 7;
			material.normal_texname = token;
			continue;
		}

		// unknown parameter
		const char* _space = strchr(token, ' ');
		if (!_space)
		{
			_space = strchr(token, '\t');
		}
		if (_space)
		{
			int len = _space - token;
			std::string key(token, len);
			std::string value = _space + 1;
			material.unknown_parameter.insert(std::pair<std::string, std::string>(key, value));
		}
	}
#ifndef USE_STREAM
	while (line)
		;
#endif
	// flush last material.
	material_map.insert(std::pair<std::string, material_t>(material.name, material));

	if (fileHandle >= 0)
	{
		fileIO->fileClose(fileHandle);
	}
	return err.str();
}

std::string
LoadObj(
	attrib_t& attrib,
	std::vector<shape_t>& shapes,
	const char* filename,
	const char* mtl_basepath,
	CommonFileIOInterface* fileIO)
{
	attrib.vertices.clear();
	attrib.normals.clear();
	attrib.texcoords.clear();
	shapes.clear();
	std::string tmp = filename;
	if (!mtl_basepath)
	{
		int last_slash = 0;
		for (int c = 0; c < (int)tmp.size(); ++c)
			if (tmp[c] == '/' || tmp[c] == '\\')
				last_slash = c;
		tmp = tmp.substr(0, last_slash);
		mtl_basepath = tmp.c_str();
		//fprintf(stderr, "MTL PATH '%s' orig '%s'\n", mtl_basepath, filename);
	}
	std::stringstream err;
#ifdef USE_STREAM
	std::ifstream ifs(filename);
	if (!ifs)
	{
		err << "Cannot open file [" << filename << "]" << std::endl;
		return err.str();
	}
#else
	int fileHandle = fileIO->fileOpen(filename, "r");
	if (fileHandle < 0)
	{
		err << "Cannot open file [" << filename << "]" << std::endl;
		return err.str();
	}
#endif

	std::vector<float> v;
	std::vector<float> vn;
	std::vector<float> vt;
	std::string name;

	int greatest_v_idx = -1;
	int greatest_vn_idx = -1;
	int greatest_vt_idx = -1;

	std::vector<face_t> faceGroup;
	// material
	std::map<std::string, material_t> material_map;
	material_t material;
	InitMaterial(material);

	int maxchars = 8192;              // Alloc enough size.
	std::vector<char> buf(maxchars);  // Alloc enough size.
	std::string linebuf;
	linebuf.reserve(maxchars);

#ifdef USE_STREAM
	while (ifs.peek() != -1)
#else
	char* line = 0;
	do
#endif
	{
		linebuf.resize(0);
#ifdef USE_STREAM
		safeGetline(ifs, linebuf);
#else
		char tmpBuf[1024];
		line = fileIO->readLine(fileHandle, tmpBuf, 1024);
		if (line)
		{
			linebuf = line;
		}
#endif
		// Trim newline '\r\n' or '\r'
		if (linebuf.size() > 0)
		{
			if (linebuf[linebuf.size() - 1] == '\n') linebuf.erase(linebuf.size() - 1);
		}
		if (linebuf.size() > 0)
		{
			if (linebuf[linebuf.size() - 1] == '\n') linebuf.erase(linebuf.size() - 1);
		}

		// Skip if empty line.
		if (linebuf.empty())
		{
			continue;
		}

		// Skip leading space.
		const char* token = linebuf.c_str();
		token += strspn(token, " \t");

		assert(token);
		if (token[0] == '\0') continue;  // empty line

		if (token[0] == '#') continue;  // comment line

		// vertex
		if (token[0] == 'v' && isSpace((token[1])))
		{
			token += 2;
			float x, y, z;
			parseFloat3(x, y, z, token);
			v.push_back(x);
			v.push_back(y);
			v.push_back(z);
			continue;
		}

		// normal
		if (token[0] == 'v' && token[1] == 'n' && isSpace((token[2])))
		{
			token += 3;
			float x, y, z;
			parseFloat3(x, y, z, token);
			vn.push_back(x);
			vn.push_back(y);
			vn.push_back(z);
			continue;
		}

		// texcoord
		if (token[0] == 'v' && token[1] == 't' && isSpace((token[2])))
		{
			token += 3;
			float x, y;
			parseFloat2(x, y, token);
			vt.push_back(x);
			vt.push_back(y);
			continue;
		}

		// face
		if (token[0] == 'f' && isSpace((token[1])))
		{
			token += 2;
			token += strspn(token, " \t");

			face_t face;

			face.reserve(3);

			while (!isNewLine(token[0]))
			{
				vertex_index_t vi;
				if (!parseTriple(&token, static_cast<int>(v.size() / 3),
								 static_cast<int>(vn.size() / 3),
								 static_cast<int>(vt.size() / 2), &vi))
				{
					err << "Failed parse `f' line(e.g. zero value for face index.";
					return err.str();
				}

				greatest_v_idx = greatest_v_idx > vi.v_idx ? greatest_v_idx : vi.v_idx;
				greatest_vn_idx =
					greatest_vn_idx > vi.vn_idx ? greatest_vn_idx : vi.vn_idx;
				greatest_vt_idx =
					greatest_vt_idx > vi.vt_idx ? greatest_vt_idx : vi.vt_idx;

				face.push_back(vi);
				size_t n = strspn(token, " \t\r");
				token += n;
			}
			faceGroup.push_back(face);
			continue;
		}
		// use mtl
		if ((0 == strncmp(token, "usemtl", 6)) && isSpace((token[6])))
		{
			char namebuf[4096];
			token += 7;
			sscanf(token, "%s", namebuf);

			if (material_map.find(namebuf) != material_map.end())
			{
				material = material_map[namebuf];
			}
			else
			{
				// { error!! material not found }
				InitMaterial(material);
			}
			continue;
		}

		// load mtl
		if ((0 == strncmp(token, "mtllib", 6)) && isSpace((token[6])))
		{
			char namebuf[4096];
			token += 7;
			sscanf(token, "%s", namebuf);

			std::string err_mtl = LoadMtl(material_map, namebuf, mtl_basepath, fileIO);
			if (!err_mtl.empty())
			{
				//face_group.resize(0);  // for safety
				//return err_mtl;
			}
			continue;
		}

		// group name
		if (token[0] == 'g' && isSpace((token[1])))
		{
			// flush previous face group.
			shape_t shape;
			bool ret = exportFaceGroupToShape(&shape, faceGroup, material, name, v);
			if (ret)
			{
				shapes.push_back(shape);
			}

			faceGroup.resize(0);

			std::vector<std::string> names;
			while (!isNewLine(token[0]))
			{
				std::string str = parseString(token);
				names.push_back(str);
				token += strspn(token, " \t\r");  // skip tag
			}

			assert(names.size() > 0);

			// names[0] must be 'g', so skipt 0th element.
			if (names.size() > 1)
			{
				name = names[1];
			}
			else
			{
				name = "";
			}

			continue;
		}

		// object name
		if (token[0] == 'o' && isSpace((token[1])))
		{
			// flush previous face group.
			shape_t shape;
			bool ret = exportFaceGroupToShape(&shape, faceGroup, material, name, v);
			if (ret)
			{
				shapes.push_back(shape);
			}

			faceGroup.resize(0);

			// @todo { multiple object name? }
			char namebuf[4096];
			token += 2;
			sscanf(token, "%s", namebuf);
			name = std::string(namebuf);

			continue;
		}

		// Ignore unknown command.
	}
#ifndef USE_STREAM
	while (line)
		;
#endif

	shape_t shape;
	bool ret = exportFaceGroupToShape(&shape, faceGroup, material, name, v);
	if (ret)
	{
		shapes.push_back(shape);
	}
	faceGroup.resize(0);  // for safety

	attrib.vertices.swap(v);
	attrib.normals.swap(vn);
	attrib.texcoords.swap(vt);

	if (fileHandle >= 0)
	{
		fileIO->fileClose(fileHandle);
	}
	return err.str();
}

};  // namespace bt_tinyobj
