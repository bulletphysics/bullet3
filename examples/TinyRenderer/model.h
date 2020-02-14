#ifndef __MODEL_H__
#define __MODEL_H__
#include <vector>
#include <string>
#include "geometry.h"
#include "tgaimage.h"

namespace TinyRender
{
class Model
{
private:
	std::vector<Vec3f> verts_;
	std::vector<std::vector<Vec3i> > faces_;  // attention, this Vec3i means vertex/uv/normal
	std::vector<Vec3f> norms_;
	std::vector<Vec2f> uv_;
	TGAImage diffusemap_;
	TGAImage normalmap_;
	TGAImage specularmap_;
	Vec4f m_colorRGBA;

	void load_texture(std::string filename, const char* suffix, TGAImage& img);

public:
	Model(const char* filename);
	Model();
	void setColorRGBA(const float rgba[4])
	{
		for (int i = 0; i < 4; i++)
			m_colorRGBA[i] = rgba[i];
	}

	const Vec4f& getColorRGBA() const
	{
		return m_colorRGBA;
	}
	void loadDiffuseTexture(const char* relativeFileName);
	void setDiffuseTextureFromData(unsigned char* textureImage, int textureWidth, int textureHeight);
	void reserveMemory(int numVertices, int numIndices);
	void addVertex(float x, float y, float z, float normalX, float normalY, float normalZ, float u, float v);
	void addTriangle(int vertexposIndex0, int normalIndex0, int uvIndex0,
					 int vertexposIndex1, int normalIndex1, int uvIndex1,
					 int vertexposIndex2, int normalIndex2, int uvIndex2);

	~Model();
	int nverts();
	int nfaces();
	Vec3f normal(int iface, int nthvert);
	Vec3f normal(Vec2f uv);
	Vec3f vert(int i);
	Vec3f vert(int iface, int nthvert);
	Vec2f uv(int iface, int nthvert);
	TGAColor diffuse(Vec2f uv);
	float specular(Vec2f uv);
	std::vector<int> face(int idx);
};
}

#endif  //__MODEL_H__
