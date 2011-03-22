/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2008 Advanced Micro Devices

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include "gl_win.h" //for OpenGL stuff

#include "bmpLoader.h"
#include <string>
#include <cstring>
#include "LinearMath/btScalar.h"
#include <stdio.h>


struct vertex_struct 
{
	float pos[3];
	float normal[3];
	float texcoord[2];

};

class btVertexBufferDescriptor;

class piece_of_cloth 
{
	public:

	void destroy(void)
	{
		if(created)
		{
			if(cpu_buffer) delete [] cpu_buffer;
		}
	}

	piece_of_cloth()
	{
		created = false;
		cpu_buffer = NULL;
		m_vertexBufferDescriptor = NULL;
#ifdef USE_GPU_COPY
		clothVBO = 0;
#endif
	}

	bool created;

	vertex_struct* cpu_buffer;
	unsigned int* indices;
	btVertexBufferDescriptor *m_vertexBufferDescriptor;

	double x_offset, y_offset, z_offset;

	int width;
	int height;

	GLuint m_texture;
#ifdef USE_GPU_COPY
	
	GLuint clothVBO;

	GLuint getVBO()
	{
		return clothVBO;
	}
#endif //USE_GPU_COPY

	void draw(void)
	{
		glEnable(GL_TEXTURE_2D);
		glBindTexture (GL_TEXTURE_2D, m_texture);

		glEnable(GL_DEPTH_TEST);

		glColor3f(0.0f, 1.0f, 1.0f);
#ifdef USE_GPU_COPY
		int error = 0;
		glBindBuffer(GL_ARRAY_BUFFER, clothVBO);
#ifndef USE_GPU_COPY
		// Upload data to VBO
		// Needed while we're not doing interop
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertex_struct)*width*height, &(cpu_buffer[0]), GL_DYNAMIC_DRAW);
#endif
#endif
		glEnableClientState(GL_VERTEX_ARRAY);
#ifdef USE_GPU_COPY
		glEnableClientState(GL_NORMAL_ARRAY);
#endif
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);

		glBindTexture(GL_TEXTURE_2D, m_texture);
#ifdef USE_GPU_COPY
		error = glGetError();

		// VBO version
		glVertexPointer( 3, GL_FLOAT, sizeof(vertex_struct), (const GLvoid *)0 );
		error = glGetError();
		glNormalPointer( GL_FLOAT, sizeof(vertex_struct), (const GLvoid *)(sizeof(float)*3) );
		error = glGetError();
		glTexCoordPointer( 2, GL_FLOAT, sizeof(vertex_struct), (const GLvoid *)(sizeof(float)*6) );
		error = glGetError();
	

#else
		glVertexPointer( 3, GL_FLOAT, sizeof(vertex_struct), reinterpret_cast< GLvoid* >(&(cpu_buffer[0].pos[0])) );
		//glNormalPointer( 3, sizeof(vertex_struct), reinterpret_cast< GLvoid* >(&(cpu_buffer[0].normal[0])) );
		glTexCoordPointer( 2, GL_FLOAT, sizeof(vertex_struct), reinterpret_cast< GLvoid* >(&(cpu_buffer[0].texcoord[0])) );
#endif

		glDrawElements(GL_TRIANGLES, (height-1  )*(width-1)*3*2, GL_UNSIGNED_INT, indices);
//		glDisableClientState(GL_NORMAL_ARRAY);
		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_TEXTURE_COORD_ARRAY);
		glBindTexture(GL_TEXTURE_2D, 0);
#ifdef	USE_GPU_COPY
		error = glGetError();
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		error = glGetError();
#endif

	}

	void create_texture(std::string filename)
	{
		amd::BitMap texBMP(filename.c_str());

		if ( !texBMP.isLoaded() ) 
		{
		
			//alternative path
			char newPath[1024];
			sprintf(newPath,"Demos/OpenCLClothDemo/%s",filename.c_str());
			texBMP.load(newPath);
			if (!texBMP.isLoaded())
			{
				sprintf(newPath,"../../../../../Demos/OpenCLClothDemo/%s",filename.c_str());
				texBMP.load(newPath);
			}
		}


		if ( texBMP.isLoaded() ) {
			glEnable(GL_TEXTURE_2D);
			glGenTextures(1, &m_texture);

			glBindTexture(GL_TEXTURE_2D, m_texture);

			glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

			glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
    
			glTexImage2D(
				GL_TEXTURE_2D,
				0,
				GL_RGBA8,
				texBMP.getWidth(),
				texBMP.getHeight(),
				0,
				GL_RGBA,
				GL_UNSIGNED_BYTE,
				texBMP.getPixels());

			glBindTexture(GL_TEXTURE_2D, 0);
		}
		else {
			std::cout << "ERROR: could not load bitmap, using placeholder " << std::endl;

				GLubyte*	image=new GLubyte[256*256*3];
				for(int y=0;y<256;++y)
				{
					const int	t=y>>4;
					GLubyte*	pi=image+y*256*3;
					for(int x=0;x<256;++x)
					{
						const int		s=x>>4;
						const GLubyte	b=180;					
						GLubyte			c=b+((s+t&1)&1)*(255-b);
						pi[0]=pi[1]=pi[2]=c;pi+=3;
					}
				}

				glGenTextures(1,(GLuint*)&m_texture);
				glBindTexture(GL_TEXTURE_2D,m_texture);
				glTexEnvf(GL_TEXTURE_ENV,GL_TEXTURE_ENV_MODE,GL_MODULATE);
				glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_LINEAR);
				glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR_MIPMAP_LINEAR);
				glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
				glTexParameterf(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
				gluBuild2DMipmaps(GL_TEXTURE_2D,3,256,256,GL_RGB,GL_UNSIGNED_BYTE,image);
				delete[] image;

			
		}
	}

	void create_buffers(int width_, int height_)
	{	    
		width = width_;
		height = height_;
		
		created = true;

		cpu_buffer = new vertex_struct[width*height];
		memset(cpu_buffer, 0, width*height*sizeof(vertex_struct));


		// Initial test data for rendering
		for(int y = 0; y < height; y++)
		{
			for(int x = 0; x < width; x++)
			{
				double coord = btSin(x/5.0)*0.01;
				//coord = sin(y/);

				cpu_buffer[y*width+x].pos[0]      = (x/((float)(width-1)))*1;
				cpu_buffer[y*width+x].pos[1]      = coord;
				cpu_buffer[y*width+x].pos[2]      = (y/((float)(height-1)))*1; 
				cpu_buffer[y*width+x].normal[0]   = 1;
				cpu_buffer[y*width+x].normal[1]   = 0;
				cpu_buffer[y*width+x].normal[2]   = 0;
				cpu_buffer[y*width+x].texcoord[0] = x/((float)(width-1));
				cpu_buffer[y*width+x].texcoord[1] = y/((float)(height-1));
			}
		}


		// Generate and fill index array for rendering
		indices = new unsigned int[width*3*2+2 + height*width*3*2];

		for(int y = 0; y < height-1; y++)
		{
			for(int x = 0; x < width-1; x++)
			{
				// *3 indices/triangle, *2 triangles/quad
				int baseIndex = (x + y*(width-1))*3*2;
				indices[baseIndex] = x + y*width;
				indices[baseIndex+1] = x+1 + y*width;
				indices[baseIndex+2] = x+width + y*width;


				indices[baseIndex+3] = x + 1 +  y*width;
				indices[baseIndex+4] = x+(width+1) + y*width;
				indices[baseIndex+5] = x+width + y*width;
			}
		}
#ifdef USE_GPU_COPY
		// Construct VBO
		glGenBuffers(1, &clothVBO);
		glBindBuffer(GL_ARRAY_BUFFER, clothVBO);
		// Do initial upload to ensure that the buffer exists on the device
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertex_struct)*width*height, &(cpu_buffer[0]), GL_DYNAMIC_DRAW);
		int error = glGetError();
		glBindBuffer(GL_ARRAY_BUFFER, 0);
#endif
	}
};
