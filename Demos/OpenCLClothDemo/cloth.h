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
#include "LinearMath/btScalar.h"


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
	}

	bool created;

	vertex_struct* cpu_buffer;
	unsigned int* indices;
	btVertexBufferDescriptor *m_vertexBufferDescriptor;

	double x_offset, y_offset, z_offset;

	int width;
	int height;

	GLuint texture;

	void draw(void)
	{
		glEnable(GL_TEXTURE_2D);
		glBindTexture (GL_TEXTURE_2D, texture);

		glEnable(GL_DEPTH_TEST);

		glColor3f(0.0f, 1.0f, 1.0f);

		glEnableClientState(GL_VERTEX_ARRAY);
		//glEnableClientState(GL_NORMAL_ARRAY);
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);

		glBindTexture(GL_TEXTURE_2D, texture);

		glVertexPointer( 3, GL_FLOAT, sizeof(vertex_struct), reinterpret_cast< GLvoid* >(&(cpu_buffer[0].pos[0])) );
		//glNormalPointer( 3, sizeof(vertex_struct), reinterpret_cast< GLvoid* >(&(cpu_buffer[0].normal[0])) );
		glTexCoordPointer( 2, GL_FLOAT, sizeof(vertex_struct), reinterpret_cast< GLvoid* >(&(cpu_buffer[0].texcoord[0])) );

		glDrawElements(GL_TRIANGLES, (height-1  )*(width-1)*3*2, GL_UNSIGNED_INT, indices);
//		glDisableClientState(GL_NORMAL_ARRAY);
		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_TEXTURE_COORD_ARRAY);

		glBindTexture(GL_TEXTURE_2D, 0);
	}

	void create_texture(std::string filename)
	{
		amd::BitMap texBMP(filename.c_str());
		if ( texBMP.isLoaded() ) {
			glEnable(GL_TEXTURE_2D);
			glGenTextures(1, &texture);

			glBindTexture(GL_TEXTURE_2D, texture);

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
			std::cout << "ERROR: could not load bitmap " << "texture.bmp" << std::endl;
			exit(1);
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
	}
};
