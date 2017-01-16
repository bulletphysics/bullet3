#include "Wavefront2GLInstanceGraphicsShape.h"

#include "../../OpenGLWindow/GLInstancingRenderer.h"
#include "../../OpenGLWindow/GLInstanceGraphicsShape.h"
#include "btBulletDynamicsCommon.h"
#include "../../OpenGLWindow/SimpleOpenGL3App.h"
#include "Wavefront2GLInstanceGraphicsShape.h"
#include "../../OpenGLWindow/GLInstancingRenderer.h"
#include "../../OpenGLWindow/GLInstanceGraphicsShape.h"

GLInstanceGraphicsShape* btgCreateGraphicsShapeFromWavefrontObj(std::vector<tinyobj::shape_t>& shapes, bool flatShading)
{
	
	b3AlignedObjectArray<GLInstanceVertex>* vertices = new b3AlignedObjectArray<GLInstanceVertex>;
	{
		//		int numVertices = obj->vertexCount;
		//	int numIndices = 0;
		b3AlignedObjectArray<int>* indicesPtr = new b3AlignedObjectArray<int>;
		
		for (int s=0;s<(int)shapes.size();s++)
		{
			tinyobj::shape_t& shape = shapes[s];
			int faceCount = shape.mesh.indices.size();
			
			
			for (int f=0;f<faceCount;f+=3)
			{
				
				//btVector3 normal(face.m_plane[0],face.m_plane[1],face.m_plane[2]);
				if (1)
				{
					btVector3 normal(0,1,0);
					int vtxBaseIndex = vertices->size();
					
					
					if (f<0 && f>=int(shape.mesh.indices.size()))
					{
						continue;
					}
					
					GLInstanceVertex vtx0;
					vtx0.xyzw[0] = shape.mesh.positions[shape.mesh.indices[f]*3+0];
					vtx0.xyzw[1] = shape.mesh.positions[shape.mesh.indices[f]*3+1];
					vtx0.xyzw[2] = shape.mesh.positions[shape.mesh.indices[f]*3+2];
					vtx0.xyzw[3] = 0.f;
					

					if (shape.mesh.texcoords.size() )
					{
						int uv0Index = shape.mesh.indices[f]*2+0;
						int uv1Index = shape.mesh.indices[f]*2+1;
						if (uv0Index>=0 && uv1Index>=0 && (uv0Index < int(shape.mesh.texcoords.size()) && (uv1Index < shape.mesh.texcoords.size())))
						{
							vtx0.uv[0] = shape.mesh.texcoords[uv0Index];
							vtx0.uv[1] = shape.mesh.texcoords[uv1Index];
						} else
						{
							b3Warning("obj texture coordinate out-of-range!");
							vtx0.uv[0] = 0;
							vtx0.uv[1] = 0;
						}
						
					} else
					{
						vtx0.uv[0] = 0.5;
						vtx0.uv[1] = 0.5;
					}
					
					GLInstanceVertex vtx1;
					vtx1.xyzw[0] = shape.mesh.positions[shape.mesh.indices[f+1]*3+0];
					vtx1.xyzw[1] = shape.mesh.positions[shape.mesh.indices[f+1]*3+1];
					vtx1.xyzw[2] = shape.mesh.positions[shape.mesh.indices[f+1]*3+2];
					vtx1.xyzw[3]= 0.f;

					if (shape.mesh.texcoords.size())
					{
						int uv0Index = shape.mesh.indices[f+1]*2+0;
						int uv1Index = shape.mesh.indices[f+1]*2+1;
						if (uv0Index>=0 && uv1Index>=0 && (uv0Index < shape.mesh.texcoords.size()) && (uv1Index < shape.mesh.texcoords.size()))
						{
							vtx1.uv[0] = shape.mesh.texcoords[uv0Index];
							vtx1.uv[1] = shape.mesh.texcoords[uv1Index];
						} else
						{
							b3Warning("obj texture coordinate out-of-range!");
							vtx1.uv[0] = 0;
							vtx1.uv[1] = 0;
						}
					} else
					{
						vtx1.uv[0] = 0.5f;
						vtx1.uv[1] = 0.5f;
					}
					
					GLInstanceVertex vtx2;
					vtx2.xyzw[0] = shape.mesh.positions[shape.mesh.indices[f+2]*3+0];
					vtx2.xyzw[1] = shape.mesh.positions[shape.mesh.indices[f+2]*3+1];
					vtx2.xyzw[2] = shape.mesh.positions[shape.mesh.indices[f+2]*3+2];
					vtx2.xyzw[3] = 0.f;
					if (shape.mesh.texcoords.size())
					{
						int uv0Index = shape.mesh.indices[f+2]*2+0;
						int uv1Index = shape.mesh.indices[f+2]*2+1;
						if (uv0Index>=0 && uv1Index>=0 && (uv0Index < shape.mesh.texcoords.size()) && (uv1Index < shape.mesh.texcoords.size()))
						{
							vtx2.uv[0] = shape.mesh.texcoords[uv0Index];
							vtx2.uv[1] = shape.mesh.texcoords[uv1Index];
						} else
						{
							b3Warning("obj texture coordinate out-of-range!");
							vtx2.uv[0] = 0;
							vtx2.uv[1] = 0;
						}
					} else
					{
						vtx2.uv[0] = 0.5;
						vtx2.uv[1] = 0.5;
					}
					
					
					
					btVector3 v0(vtx0.xyzw[0],vtx0.xyzw[1],vtx0.xyzw[2]);
					btVector3 v1(vtx1.xyzw[0],vtx1.xyzw[1],vtx1.xyzw[2]);
					btVector3 v2(vtx2.xyzw[0],vtx2.xyzw[1],vtx2.xyzw[2]);
					
					unsigned int maxIndex = 0;
					maxIndex = b3Max(maxIndex,shape.mesh.indices[f]*3+0);
					maxIndex = b3Max(maxIndex,shape.mesh.indices[f]*3+1);
					maxIndex = b3Max(maxIndex,shape.mesh.indices[f]*3+2);
					maxIndex = b3Max(maxIndex,shape.mesh.indices[f+1]*3+0);
					maxIndex = b3Max(maxIndex,shape.mesh.indices[f+1]*3+1);
					maxIndex = b3Max(maxIndex,shape.mesh.indices[f+1]*3+2);
					maxIndex = b3Max(maxIndex,shape.mesh.indices[f+2]*3+0);
					maxIndex = b3Max(maxIndex,shape.mesh.indices[f+2]*3+1);
					maxIndex = b3Max(maxIndex,shape.mesh.indices[f+2]*3+2);
					bool hasNormals = (shape.mesh.normals.size() && maxIndex<shape.mesh.normals.size() );
					
					if (flatShading || !hasNormals)
                    {
                        normal = (v1-v0).cross(v2-v0);
                        btScalar len2 = normal.length2();
                        //skip degenerate triangles
                        if (len2 > SIMD_EPSILON)
                        {
                            normal.normalize();
                        } else
                        {
                            normal.setValue(0,0,0);
                        }
                        vtx0.normal[0] = normal[0];
                        vtx0.normal[1] = normal[1];
                        vtx0.normal[2] = normal[2];
                        vtx1.normal[0] = normal[0];
                        vtx1.normal[1] = normal[1];
                        vtx1.normal[2] = normal[2];
                        vtx2.normal[0] = normal[0];
                        vtx2.normal[1] = normal[1];
                        vtx2.normal[2] = normal[2];
                    } else
                    {
                        
                        vtx0.normal[0] = shape.mesh.normals[shape.mesh.indices[f]*3+0];
                        vtx0.normal[1] = shape.mesh.normals[shape.mesh.indices[f]*3+1];
                        vtx0.normal[2] = shape.mesh.normals[shape.mesh.indices[f]*3+2]; //shape.mesh.indices[f+1]*3+0
                        vtx1.normal[0] = shape.mesh.normals[shape.mesh.indices[f+1]*3+0];
                        vtx1.normal[1] = shape.mesh.normals[shape.mesh.indices[f+1]*3+1];
                        vtx1.normal[2] = shape.mesh.normals[shape.mesh.indices[f+1]*3+2];
                        vtx2.normal[0] = shape.mesh.normals[shape.mesh.indices[f+2]*3+0];
                        vtx2.normal[1] = shape.mesh.normals[shape.mesh.indices[f+2]*3+1];
                        vtx2.normal[2] = shape.mesh.normals[shape.mesh.indices[f+2]*3+2];
                        
                        
                    }
                    vertices->push_back(vtx0);
                    vertices->push_back(vtx1);
                    vertices->push_back(vtx2);
                    indicesPtr->push_back(vtxBaseIndex);
                    indicesPtr->push_back(vtxBaseIndex+1);
                    indicesPtr->push_back(vtxBaseIndex+2);
                
                    
				}
			}
		}
		
		
		GLInstanceGraphicsShape* gfxShape = new GLInstanceGraphicsShape;
		gfxShape->m_vertices = vertices;
		gfxShape->m_numvertices = vertices->size();
		gfxShape->m_indices = indicesPtr;
		gfxShape->m_numIndices = indicesPtr->size();
		for (int i=0;i<4;i++)
			gfxShape->m_scaling[i] = 1;//bake the scaling into the vertices
		return gfxShape;
	}
}

