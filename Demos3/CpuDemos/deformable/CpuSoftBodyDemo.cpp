#include "CpuSoftBodyDemo.h"
#include "OpenGLWindow/ShapeData.h"
#include "OpenGLWindow/GLInstancingRenderer.h"
#include "Bullet3Common/b3Quaternion.h"
#include "OpenGLWindow/b3gWindowInterface.h"


#include "OpenGLWindow/OpenGLInclude.h"
#include "OpenGLWindow/GLInstanceRendererInternalData.h"

#include "Bullet3Collision/NarrowPhaseCollision/b3Config.h"
#include "CpuSoftBodyDemoInternalData.h"
#include "Bullet3Collision/BroadPhaseCollision/b3DynamicBvhBroadphase.h"
#include "stb_image/stb_image.h"

static b3KeyboardCallback oldCallback = 0;
extern bool gReset;






CpuSoftBodyDemo::CpuSoftBodyDemo()
:m_instancingRenderer(0),
m_window(0)
{
	m_data = new CpuSoftBodyDemoInternalData;
}
CpuSoftBodyDemo::~CpuSoftBodyDemo()
{
	
	delete m_data;
}







static void PairKeyboardCallback(int key, int state)
{
	if (key=='R' && state)
	{
		gReset = true;
	}
	
	//b3DefaultKeyboardCallback(key,state);
	oldCallback(key,state);
}

void	CpuSoftBodyDemo::setupScene(const ConstructionInfo& ci)
{

}

void	CpuSoftBodyDemo::initPhysics(const ConstructionInfo& ci)
{

	GLint err = glGetError();
	b3Assert(err==GL_NO_ERROR);

	if (ci.m_window)
	{
		m_window = ci.m_window;
		oldCallback = ci.m_window->getKeyboardCallback();
		ci.m_window->setKeyboardCallback(PairKeyboardCallback);

	}

	m_instancingRenderer = ci.m_instancingRenderer;

	

	setupScene(ci);

	err = glGetError();
	b3Assert(err==GL_NO_ERROR);



	m_instancingRenderer->writeTransforms();
	
}

void	CpuSoftBodyDemo::exitPhysics()
{

	m_window->setKeyboardCallback(oldCallback);
	
}

	

struct GraphicsVertex
{
	float pos[4];
	float normal[3];
	float texcoord[2];
};

void CpuSoftClothDemo::renderScene()
{
	if (m_data->m_clothShapeIndex>=0 && m_data->m_clothVertices)
	{
		GraphicsVertex* cpu_buffer = (GraphicsVertex*)m_data->m_clothVertices;
		int width = 256;
		int height=256;
		static float shift = 0.f;
		shift+=0.01;
		if (shift>B3_2_PI)
			shift-=B3_2_PI;

		int numVertices = 0;
		// Initial test data for rendering
		for(int y = 0; y < height; y++)
		{
			for(int x = 0; x < width; x++)
			{
				
				double coord = b3Sin(x/5.0+shift)*0.01;
				//coord = sin(y/);

				cpu_buffer[y*width+x].pos[0]      = (x/((float)(width-1)))*1;
				cpu_buffer[y*width+x].pos[1]      = coord;
				cpu_buffer[y*width+x].pos[2]      = (y/((float)(height-1)))*1; 
				cpu_buffer[y*width+x].pos[3]      = 0.f;

				cpu_buffer[y*width+x].normal[0]   = 1;
				cpu_buffer[y*width+x].normal[1]   = 0;
				cpu_buffer[y*width+x].normal[2]   = 0;
				cpu_buffer[y*width+x].texcoord[0] = 1*x/((float)(width-1));
				cpu_buffer[y*width+x].texcoord[1] = (1.f-4*y/((float)(height-1)));
				numVertices++;
			}
		}

		m_instancingRenderer->updateShape(m_data->m_clothShapeIndex,m_data->m_clothVertices);
	}
	m_instancingRenderer->renderScene();

}
void CpuSoftBodyDemo::renderScene()
{
	m_instancingRenderer->renderScene();
}

void CpuSoftBodyDemo::clientMoveAndDisplay()
{
	GLint err = glGetError();
	b3Assert(err==GL_NO_ERROR);
}


CpuSoftClothDemo::CpuSoftClothDemo()
{
}
CpuSoftClothDemo::~CpuSoftClothDemo()
{
}
	

unsigned char* CpuSoftClothDemo::loadImage(const char* fileName, int& width, int& height, int& n)
{
		unsigned char *data = stbi_load(fileName, &width, &height, &n, 0);
		if (data)
		{
			GLubyte*	image=new GLubyte[512*256*4];
			for(int y=0;y<256;++y)
			{
				const int	t=y>>4;
				GLubyte*	pi=image+y*512*3;
				for(int x=0;x<512;++x)
				{
					const int		s=x>>5;
					const GLubyte	b=180;					
					GLubyte			c=b+((s+t&1)&1)*(255-b);
					pi[0]=pi[1]=pi[2]=c;pi+=3;
				}
			}

			{
			
				for (int i=0;i<width;i++)
				{
					for (int j=0;j<height;j++)
					{
						int offsetx = (512-width)/2;
						int offsety = (256-height)/2;

						GLubyte*	pi=image+((j+offsety)*512+i+offsetx)*3;
						const GLubyte*	src=data+(j*width+i)*4;
						pi[0] = src[0];
						pi[1] = src[1];
						pi[2] = src[2];
					}
				}

			
			}
			width = 512;
			height = 256;
			return image;
		}
		return 0;
}

void	CpuSoftClothDemo::setupScene(const ConstructionInfo& ci)
{
	GLint err = glGetError();
	b3Assert(err==GL_NO_ERROR);

	
	int width = 256;
	int height = 256;

	GraphicsVertex* cpu_buffer = new GraphicsVertex[width*height];
	memset(cpu_buffer, 0, width*height*sizeof(GraphicsVertex));


	int numVertices = 0;
	// Initial test data for rendering
	for(int y = 0; y < height; y++)
	{
		for(int x = 0; x < width; x++)
		{
			double coord = b3Sin(x/5.0);//*0.01;
			//coord = sin(y/);

			cpu_buffer[y*width+x].pos[0]      = (x/((float)(width-1)))*1;
			cpu_buffer[y*width+x].pos[1]      = coord;
			cpu_buffer[y*width+x].pos[2]      = (y/((float)(height-1)))*1; 
			cpu_buffer[y*width+x].pos[3]      = 0.f;

			cpu_buffer[y*width+x].normal[0]   = 1;
			cpu_buffer[y*width+x].normal[1]   = 0;
			cpu_buffer[y*width+x].normal[2]   = 0;
			cpu_buffer[y*width+x].texcoord[0] = 1*x/((float)(width-1));
			cpu_buffer[y*width+x].texcoord[1] = (1.f-4*y/((float)(height-1)));
			numVertices++;
		}
	}

	err = glGetError();
	b3Assert(err==GL_NO_ERROR);

	int numIndices = 0;

	// Generate and fill index array for rendering
	 int* indices = new  int[width*3*2+2 + height*width*3*2];

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
			numIndices++;
		}
	}


	int num_barrel_vertices = sizeof(barrel_vertices)/(9*sizeof(float));
	int num_barrel_indices = sizeof(barrel_indices)/sizeof(int);

	int textureIndex  = -1;
	{
		int width,height,n;
		FILE* f = fopen("test.tst","wb");
		fclose(f);
		const char* filename = "data/bullet_logo.png";
		const unsigned char* image=0;
		
		const char* prefix[]={"./","../","../../","../../../","../../../../"};
		int numprefix = sizeof(prefix)/sizeof(const char*);
		
		for (int i=0;!image && i<numprefix;i++)
		{
			char relativeFileName[1024];
			sprintf(relativeFileName,"%s%s",prefix[i],filename);
			image = loadImage(relativeFileName,width,height,n);
		}
		
		b3Assert(image);
		if (image)
		{
			textureIndex = ci.m_instancingRenderer->registerTexture(image,width,height);
		}
	}
//	int shapeIndex = ci.m_instancingRenderer->registerShape(barrel_vertices,num_barrel_vertices,barrel_indices,num_barrel_indices);

	err = glGetError();
	b3Assert(err==GL_NO_ERROR);

	m_data->m_clothVertices = &cpu_buffer[0].pos[0];

	int shapeIndex = ci.m_instancingRenderer->registerShape(&cpu_buffer[0].pos[0],numVertices,indices,numIndices,B3_GL_TRIANGLES,textureIndex);
	m_data->m_clothShapeIndex = shapeIndex;

	err = glGetError();
	b3Assert(err==GL_NO_ERROR);

	
	float pos[4] = {0,0,0,0};
	float orn[4] = {0,0,0,1};
	float color[4] = {1,1,1,1};
	float scaling[4] = {10,10,10,1};

	ci.m_instancingRenderer->registerGraphicsInstance(shapeIndex,pos,orn,color,scaling);
	ci.m_instancingRenderer->setCameraDistance(4);
	ci.m_instancingRenderer->setCameraTargetPosition(pos);

	err = glGetError();
	b3Assert(err==GL_NO_ERROR);

}	