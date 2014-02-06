#include "SimpleOpenGL3App.h"
#include "ShapeData.h"

#ifdef __APPLE__
#include "OpenGLWindow/MacOpenGLWindow.h"
#else

#include "GL/glew.h"
#ifdef _WIN32
#include "OpenGLWindow/Win32OpenGLWindow.h"
#else
//let's cross the fingers it is Linux/X11
#include "OpenGLWindow/X11OpenGLWindow.h"
#endif //_WIN32
#endif//__APPLE__

#include "OpenGLWindow/GLPrimitiveRenderer.h"
#include "OpenGLWindow/GLInstancingRenderer.h"

#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Logging.h"

#include "../btgui/OpenGLTrueTypeFont/fontstash.h"
#include "../btgui/OpenGLWindow/TwFonts.h"
#include "OpenGLTrueTypeFont/opengl_fontstashcallbacks.h"
#include <assert.h>

struct SimpleInternalData
{
	GLuint m_fontTextureId;
	struct sth_stash* m_fontStash;
	OpenGL2RenderCallbacks*		m_renderCallbacks;
	int m_droidRegular;

};

static SimpleOpenGL3App* gApp=0;

void SimpleResizeCallback( float width, float height)
{
	gApp->m_instancingRenderer->resize(width,height);
	gApp->m_primRenderer->setScreenSize(width,height);
	
}

static GLuint BindFont(const CTexFont *_Font)
{
    GLuint TexID = 0;
    glGenTextures(1, &TexID);
    glBindTexture(GL_TEXTURE_2D, TexID);
    glPixelStorei(GL_UNPACK_SWAP_BYTES, GL_FALSE);
    glPixelStorei(GL_UNPACK_LSB_FIRST, GL_FALSE);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
    glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, _Font->m_TexWidth, _Font->m_TexHeight, 0, GL_RED, GL_UNSIGNED_BYTE, _Font->m_TexBytes);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,GL_NEAREST);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);

    return TexID;
}

extern char OpenSansData[];

SimpleOpenGL3App::SimpleOpenGL3App(	const char* title, int width,int height)
{
	gApp = this;
	m_data = new SimpleInternalData;
	m_window = new b3gDefaultOpenGLWindow();
	b3gWindowConstructionInfo ci;
	ci.m_title = title;
	ci.m_width = width;
	ci.m_height = height;
	m_window->createWindow(ci);
	
	m_window->setWindowTitle(title);
	glClearColor(1,1,1,1);
	m_window->startRendering();
#ifndef __APPLE__
	glewInit();
#endif

	m_primRenderer = new GLPrimitiveRenderer(width,height);
	
	m_instancingRenderer = new GLInstancingRenderer(128*1024,4*1024*1024);
	m_instancingRenderer->init();
	m_instancingRenderer->resize(width,height);
	m_instancingRenderer->InitShaders();

	m_window->setMouseMoveCallback(b3DefaultMouseMoveCallback);
	m_window->setMouseButtonCallback(b3DefaultMouseButtonCallback);
	m_window->setKeyboardCallback(b3DefaultKeyboardCallback);
	m_window->setWheelCallback(b3DefaultWheelCallback);
	m_window->setResizeCallback(SimpleResizeCallback);

	TwGenerateDefaultFonts();
	m_data->m_fontTextureId = BindFont(g_DefaultNormalFont);


	{
		GLint err;
	
	int datasize;

	float sx,sy,dx,dy,lh;
	GLuint texture;
	m_data->m_renderCallbacks = new OpenGL2RenderCallbacks(m_primRenderer);
	m_data->m_fontStash = sth_create(512,512,m_data->m_renderCallbacks);//256,256);//,1024);//512,512);
    err = glGetError();
    assert(err==GL_NO_ERROR);

	if (!m_data->m_fontStash)
	{
		b3Warning("Could not create stash");
		//fprintf(stderr, "Could not create stash.\n");
	}
	int droidRegular=0;

	char* data2 = OpenSansData;
	unsigned char* data = (unsigned char*) data2;
	if (!(m_data->m_droidRegular = sth_add_font_from_memory(m_data->m_fontStash, data)))
	{
		b3Warning("error!\n");
	}
    err = glGetError();
    assert(err==GL_NO_ERROR);
	}
}


struct sth_stash* SimpleOpenGL3App::getFontStash()
{
	return m_data->m_fontStash;
}



void SimpleOpenGL3App::drawText( const char* txt, int posX, int posY)
{
		
    
    
        
    //
    //printf("str = %s\n",unicodeText);

    float dx=0;
        
    int measureOnly=0;
        
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	
	if (1)//m_useTrueTypeFont)
	{
		bool measureOnly = false;
			
		float fontSize= 64;//512;//128;
		sth_draw_text(m_data->m_fontStash,
                    m_data->m_droidRegular,fontSize,posX,posY,
					txt,&dx, this->m_instancingRenderer->getScreenWidth(),this->m_instancingRenderer->getScreenHeight(),measureOnly,m_window->getRetinaScale());
		sth_end_draw(m_data->m_fontStash);
		sth_flush_draw(m_data->m_fontStash);
	} else
	{
		//float width = 0.f;
		int pos=0;
		float color[]={0.2f,0.2,0.2f,1.f};
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D,m_data->m_fontTextureId);
		
		//float width = r.x;
		float extraSpacing = 0.;

		int startX = posX;
		int startY = posY;
		

		while (txt[pos])
		{
			int c = txt[pos];
			//r.h = g_DefaultNormalFont->m_CharHeight;
			//r.w = g_DefaultNormalFont->m_CharWidth[c]+extraSpacing;
			int endX = startX+g_DefaultNormalFont->m_CharWidth[c];
			int endY = startY+g_DefaultNormalFont->m_CharHeight;
			//Gwen::Rect rect = r;
			//Translate( rect );

			
			float currentColor[]={0.2f,0.2,0.2f,1.f};

			m_primRenderer->drawTexturedRect(startX, startY, endX, endY, currentColor,g_DefaultNormalFont->m_CharU0[c],g_DefaultNormalFont->m_CharV0[c],g_DefaultNormalFont->m_CharU1[c],g_DefaultNormalFont->m_CharV1[c]);

			//DrawTexturedRect(0,r,g_DefaultNormalFont->m_CharU0[c],g_DefaultNormalFont->m_CharV0[c],g_DefaultNormalFont->m_CharU1[c],g_DefaultNormalFont->m_CharV1[c]);
		//	DrawFilledRect(r);

			startX = endX;
			//startY = endY;
			
			pos++;
				
		}
		glBindTexture(GL_TEXTURE_2D,0);
	}

	glDisable(GL_BLEND);
}
int SimpleOpenGL3App::registerCubeShape()
{
	int strideInBytes = 9*sizeof(float);
	int numVertices = sizeof(cube_vertices)/strideInBytes;
	int numIndices = sizeof(cube_indices)/sizeof(int);
	int shapeId = m_instancingRenderer->registerShape(&cube_vertices[0],numVertices,cube_indices,numIndices);
	return shapeId;
}


int	SimpleOpenGL3App::registerGraphicsSphereShape(float radius, bool usePointSprites, int largeSphereThreshold, int mediumSphereThreshold)
{
	
	int strideInBytes = 9*sizeof(float);
	
	int graphicsShapeIndex = -1;
	
	if (radius>=largeSphereThreshold)
	{
		int numVertices = sizeof(detailed_sphere_vertices)/strideInBytes;
		int numIndices = sizeof(detailed_sphere_indices)/sizeof(int);
		graphicsShapeIndex = m_instancingRenderer->registerShape(&detailed_sphere_vertices[0],numVertices,detailed_sphere_indices,numIndices);
	} else
	{
		
		if (usePointSprites)
		{
			int numVertices = sizeof(point_sphere_vertices)/strideInBytes;
			int numIndices = sizeof(point_sphere_indices)/sizeof(int);
			graphicsShapeIndex = m_instancingRenderer->registerShape(&point_sphere_vertices[0],numVertices,point_sphere_indices,numIndices,B3_GL_POINTS);
		} else
		{
			if (radius>=mediumSphereThreshold)
			{
				int numVertices = sizeof(medium_sphere_vertices)/strideInBytes;
				int numIndices = sizeof(medium_sphere_indices)/sizeof(int);
				graphicsShapeIndex = m_instancingRenderer->registerShape(&medium_sphere_vertices[0],numVertices,medium_sphere_indices,numIndices);
			} else
			{
				int numVertices = sizeof(low_sphere_vertices)/strideInBytes;
				int numIndices = sizeof(low_sphere_indices)/sizeof(int);
				graphicsShapeIndex = m_instancingRenderer->registerShape(&low_sphere_vertices[0],numVertices,low_sphere_indices,numIndices);
			}
		}
	}
	return graphicsShapeIndex;
}

void SimpleOpenGL3App::drawGrid(int gridSize, float yOffset)
{
	
	b3Vector3 gridColor = b3MakeVector3(0.5,0.5,0.5);
	for(int i=-gridSize;i<=gridSize;i++)
	{
		
		GLint err = glGetError();
		b3Assert(err==GL_NO_ERROR);
		
		m_instancingRenderer->drawLine(b3MakeVector3(float(i),yOffset,float(-gridSize)),b3MakeVector3(float(i),yOffset,float(gridSize)),gridColor);
		
		err = glGetError();
		b3Assert(err==GL_NO_ERROR);
		
		m_instancingRenderer->drawLine(b3MakeVector3(float(-gridSize),yOffset,float(i)),b3MakeVector3(float(gridSize),yOffset,float(i)),gridColor);
	}
				
	m_instancingRenderer->drawLine(b3MakeVector3(0,0,0),b3MakeVector3(1,0,0),b3MakeVector3(1,0,0),3);
	m_instancingRenderer->drawLine(b3MakeVector3(0,0,0),b3MakeVector3(0,1,0),b3MakeVector3(0,1,0),3);
	m_instancingRenderer->drawLine(b3MakeVector3(0,0,0),b3MakeVector3(0,0,1),b3MakeVector3(0,0,1),3);

	m_instancingRenderer->drawPoint(b3MakeVector3(1,0,0),b3MakeVector3(1,0,0),6);
	m_instancingRenderer->drawPoint(b3MakeVector3(0,1,0),b3MakeVector3(0,1,0),6);
	m_instancingRenderer->drawPoint(b3MakeVector3(0,0,1),b3MakeVector3(0,0,1),6);
}

SimpleOpenGL3App::~SimpleOpenGL3App()
{
	delete m_primRenderer ;

	m_window->closeWindow();
	delete m_window;
	delete m_data ;
}

void SimpleOpenGL3App::swapBuffer()
{
	m_window->endRendering();
	m_window->startRendering();
}