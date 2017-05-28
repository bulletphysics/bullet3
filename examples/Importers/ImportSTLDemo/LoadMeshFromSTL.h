
#ifndef LOAD_MESH_FROM_STL_H
#define LOAD_MESH_FROM_STL_H

#include "../../OpenGLWindow/GLInstanceGraphicsShape.h"
#include <stdio.h> //fopen
#include "Bullet3Common/b3AlignedObjectArray.h"

struct MySTLTriangle
{
	float normal[3];
	float vertex0[3];
	float vertex1[3];
	float vertex2[3];
};

static GLInstanceGraphicsShape* LoadMeshFromSTL(const char* relativeFileName)
{
	GLInstanceGraphicsShape* shape = 0;
	
	FILE* file = fopen(relativeFileName,"rb");
	if (file)
	{
		int size=0;
		if (fseek(file, 0, SEEK_END) || (size = ftell(file)) == EOF || fseek(file, 0, SEEK_SET))
		{
			b3Warning("Error: Cannot access file to determine size of %s\n", relativeFileName);
		} else
		{
			if (size)
			{
				//b3Warning("Open STL file of %d bytes\n",size);
				char* memoryBuffer = new char[size+1];
				int actualBytesRead = fread(memoryBuffer,1,size,file);
				if (actualBytesRead!=size)
				{
					b3Warning("Error reading from file %s",relativeFileName);
				} else
				{
					int numTriangles = *(int*)&memoryBuffer[80];
					
					if (numTriangles)
					{
						{
							//perform a sanity check instead of crashing on invalid triangles/STL files
							int expectedBinaryFileSize = numTriangles* 50 + 84;
							if (expectedBinaryFileSize != size)
							{
								delete[] memoryBuffer;
								return 0;
							}

						}
						shape = new GLInstanceGraphicsShape;
//						b3AlignedObjectArray<GLInstanceVertex>*	m_vertices;
//						int				m_numvertices;
//						b3AlignedObjectArray<int>* 		m_indices;
//						int				m_numIndices;
//						float			m_scaling[4];
						shape->m_scaling[0] = 1;
						shape->m_scaling[1] = 1;
						shape->m_scaling[2] = 1;
						shape->m_scaling[3] = 1;
						int index = 0;
						shape->m_indices = new b3AlignedObjectArray<int>();
						shape->m_vertices = new b3AlignedObjectArray<GLInstanceVertex>();
						for (int i=0;i<numTriangles;i++)
						{
							char* curPtr = &memoryBuffer[84+i*50];
							MySTLTriangle tmp;
							memcpy(&tmp,curPtr,sizeof(MySTLTriangle));
							
							GLInstanceVertex v0,v1,v2;
							v0.uv[0] = v1.uv[0] = v2.uv[0] = 0.5;
							v0.uv[1] = v1.uv[1] = v2.uv[1] = 0.5;
							for (int v=0;v<3;v++)
							{
								v0.xyzw[v] = tmp.vertex0[v];
								v1.xyzw[v] = tmp.vertex1[v];
								v2.xyzw[v] = tmp.vertex2[v];
								v0.normal[v] = v1.normal[v] = v2.normal[v] = tmp.normal[v];
							}
							v0.xyzw[3] = v1.xyzw[3] = v2.xyzw[3] = 0.f;
							
							shape->m_vertices->push_back(v0);
							shape->m_vertices->push_back(v1);
							shape->m_vertices->push_back(v2);
							
							shape->m_indices->push_back(index++);
							shape->m_indices->push_back(index++);
							shape->m_indices->push_back(index++);
							
						}
					}
				}
				
				delete[] memoryBuffer;
			}
		}
		fclose(file);
	}
	if (shape)
	{
		shape->m_numIndices = shape->m_indices->size();
		shape->m_numvertices = shape->m_vertices->size();
	}
	return shape;
}

#endif //LOAD_MESH_FROM_STL_H
