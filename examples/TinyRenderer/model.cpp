
#include <iostream>
#include <fstream>
#include <sstream>
#include "model.h"

Model::Model(const char *filename) : verts_(), faces_(), norms_(), uv_(), diffusemap_(), normalmap_(), specularmap_() {
    std::ifstream in;
    in.open (filename, std::ifstream::in);
    if (in.fail()) return;
    std::string line;
    while (!in.eof()) {
        std::getline(in, line);
        std::istringstream iss(line.c_str());
        char trash;
        if (!line.compare(0, 2, "v ")) {
            iss >> trash;
            Vec3f v;
            for (int i=0;i<3;i++) iss >> v[i];
            verts_.push_back(v);
        } else if (!line.compare(0, 3, "vn ")) {
            iss >> trash >> trash;
            Vec3f n;
            for (int i=0;i<3;i++) iss >> n[i];
            norms_.push_back(n);
        } else if (!line.compare(0, 3, "vt ")) {
            iss >> trash >> trash;
            Vec2f uv;
            for (int i=0;i<2;i++) iss >> uv[i];
            uv_.push_back(uv);
        }  else if (!line.compare(0, 2, "f ")) {
            std::vector<Vec3i> f;
            Vec3i tmp;
            iss >> trash;
            while (iss >> tmp[0] >> trash >> tmp[1] >> trash >> tmp[2]) {
                for (int i=0; i<3; i++) tmp[i]--; // in wavefront obj all indices start at 1, not zero
                f.push_back(tmp);
            }
            faces_.push_back(f);
        }
    }
    std::cerr << "# v# " << verts_.size() << " f# "  << faces_.size() << " vt# " << uv_.size() << " vn# " << norms_.size() << std::endl;
    load_texture(filename, "_diffuse.tga", diffusemap_);
    load_texture(filename, "_nm_tangent.tga",      normalmap_);
    load_texture(filename, "_spec.tga",    specularmap_);
}

Model::Model():verts_(), faces_(), norms_(), uv_(), diffusemap_(), normalmap_(), specularmap_()
{
}

void Model::setDiffuseTextureFromData(unsigned char* textureImage,int textureWidth,int textureHeight)
{
	diffusemap_ = TGAImage(textureWidth, textureHeight, TGAImage::RGB);
	for (int i=0;i<textureWidth;i++)
	{
		for (int j=0;j<textureHeight;j++)
		{
			TGAColor color;
            color.bgra[0] = textureImage[(i+j*textureWidth)*3+0];
            color.bgra[1] = textureImage[(i+j*textureWidth)*3+1];
            color.bgra[2] = textureImage[(i+j*textureWidth)*3+2];
			color.bgra[3] = 255;

			color.bytespp = 3;
			diffusemap_.set(i,j,color);
			
		}
	}
	
	diffusemap_.flip_vertically();
}

void Model::loadDiffuseTexture(const char* relativeFileName)
{
    diffusemap_.read_tga_file(relativeFileName);
}

void Model::reserveMemory(int numVertices, int numIndices)
{
	verts_.reserve(numVertices);
	norms_.reserve(numVertices);
	uv_.reserve(numVertices);
	faces_.reserve(numIndices);
}

void Model::addVertex(float x,float y,float z, float normalX, float normalY, float normalZ, float u, float v)
{
    verts_.push_back(Vec3f(x,y,z));
    norms_.push_back(Vec3f(normalX,normalY,normalZ));
    uv_.push_back(Vec2f(u,v));
    
}
void Model::addTriangle(int vertexposIndex0, int normalIndex0, int uvIndex0,
                        int vertexposIndex1, int normalIndex1, int uvIndex1,
                        int vertexposIndex2, int normalIndex2, int uvIndex2)
{
    std::vector<Vec3i> f;
    f.push_back(Vec3i(vertexposIndex0, normalIndex0, uvIndex0));
    f.push_back(Vec3i(vertexposIndex1, normalIndex1, uvIndex1));
    f.push_back(Vec3i(vertexposIndex2, normalIndex2, uvIndex2));
    faces_.push_back(f);
}


Model::~Model() {}

int Model::nverts() {
    return (int)verts_.size();
}

int Model::nfaces() {
    return (int)faces_.size();
}

std::vector<int> Model::face(int idx) {
    std::vector<int> face;
    for (int i=0; i<(int)faces_[idx].size(); i++) face.push_back(faces_[idx][i][0]);
    return face;
}

Vec3f Model::vert(int i) {
    return verts_[i];
}

Vec3f Model::vert(int iface, int nthvert) {
    return verts_[faces_[iface][nthvert][0]];
}

void Model::load_texture(std::string filename, const char *suffix, TGAImage &img) {
    std::string texfile(filename);
    size_t dot = texfile.find_last_of(".");
    if (dot!=std::string::npos) {
        texfile = texfile.substr(0,dot) + std::string(suffix);
        std::cerr << "texture file " << texfile << " loading " << (img.read_tga_file(texfile.c_str()) ? "ok" : "failed") << std::endl;
        img.flip_vertically();
    }
}

TGAColor Model::diffuse(Vec2f uvf) {
    if (diffusemap_.get_width() && diffusemap_.get_height())
    {
        Vec2i uv(uvf[0]*diffusemap_.get_width(), uvf[1]*diffusemap_.get_height());
        return diffusemap_.get(uv[0], uv[1]);
    }
    return TGAColor(255,255,255,255);
}

Vec3f Model::normal(Vec2f uvf) {
    Vec2i uv(uvf[0]*normalmap_.get_width(), uvf[1]*normalmap_.get_height());
    TGAColor c = normalmap_.get(uv[0], uv[1]);
    Vec3f res;
    for (int i=0; i<3; i++)
        res[2-i] = (float)c[i]/255.f*2.f - 1.f;
    return res;
}

Vec2f Model::uv(int iface, int nthvert) {
    return uv_[faces_[iface][nthvert][1]];
}

float Model::specular(Vec2f uvf) {
    Vec2i uv(uvf[0]*specularmap_.get_width(), uvf[1]*specularmap_.get_height());
    return specularmap_.get(uv[0], uv[1])[0]/1.f;
}

Vec3f Model::normal(int iface, int nthvert) {
    int idx = faces_[iface][nthvert][2];
    return norms_[idx].normalize();
}

