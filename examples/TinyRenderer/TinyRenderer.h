#ifndef TINY_RENDERER_H
#define TINY_RENDERER_H

#include "geometry.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "tgaimage.h"

struct TinyRenderObjectData
{
    //Camera
    Matrix m_viewMatrix;
    Matrix m_projectionMatrix;
    Matrix m_viewportMatrix;

    //Model (vertices, indices, textures, shader)
    Matrix m_modelMatrix;
    class Model*  m_model;
    //class IShader* m_shader; todo(erwincoumans) expose the shader, for now we use a default shader
            
    //Output
    int m_width;
    int m_height;
    TGAImage& m_rgbColorBuffer;
    b3AlignedObjectArray<float>& m_depthBuffer;
    
    TinyRenderObjectData(int width, int height,TGAImage& rgbColorBuffer,b3AlignedObjectArray<float>& depthBuffer);
    virtual ~TinyRenderObjectData();
    
    void loadModel(const char* fileName);
    void createCube(float HalfExtentsX,float HalfExtentsY,float HalfExtentsZ);
    void registerMeshShape(const float* vertices, int numVertices,const int* indices, int numIndices);
    
    void* m_userData;
    int m_userIndex;
};


class TinyRenderer
{
    public:
        static void renderObject(TinyRenderObjectData& renderData);
};

#endif // TINY_RENDERER_Hbla
