#ifndef NO_OPENGL3

#include "SimpleOpenGL3App.h"
#include "ShapeData.h"
#ifdef __APPLE__
#include "MacOpenGLWindow.h"
#else



#ifdef _WIN32
#include "Win32OpenGLWindow.h"
#else
//let's cross the fingers it is Linux/X11
#ifdef BT_USE_EGL
#include "EGLOpenGLWindow.h"
#else
#include "X11OpenGLWindow.h"
#endif //BT_USE_EGL
#endif //_WIN32
#endif//__APPLE__
#include <stdio.h>

#include "GLPrimitiveRenderer.h"
#include "GLInstancingRenderer.h"

#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Logging.h"

#include "fontstash.h"
#include "TwFonts.h"
#include "opengl_fontstashcallbacks.h"
#include <assert.h>
#include "GLRenderToTexture.h"
#include "Bullet3Common/b3Quaternion.h"
#include <string.h>//memset
#ifdef _WIN32
    #define popen _popen
    #define pclose _pclose
#endif // _WIN32

struct SimpleInternalData
{
	GLuint m_fontTextureId;
	GLuint m_largeFontTextureId;
	struct sth_stash* m_fontStash;
	struct sth_stash* m_fontStash2;

	RenderCallbacks*		m_renderCallbacks;
	RenderCallbacks*		m_renderCallbacks2;
	
	int m_droidRegular;
	int m_droidRegular2;

	const char* m_frameDumpPngFileName;
	FILE* m_ffmpegFile;
	GLRenderToTexture*  m_renderTexture;
	void* m_userPointer;
	int m_upAxis;//y=1 or z=2 is supported

};

static SimpleOpenGL3App* gApp=0;

static void SimpleResizeCallback( float widthf, float heightf)
{
	int width = (int)widthf;
	int height = (int)heightf;
    if (gApp && gApp->m_instancingRenderer)
        gApp->m_instancingRenderer->resize(width,height);

    if (gApp && gApp->m_primRenderer)
        gApp->m_primRenderer->setScreenSize(width,height);

}

static void SimpleKeyboardCallback(int key, int state)
{
    if (key==B3G_ESCAPE && gApp && gApp->m_window)
    {
        gApp->m_window->setRequestExit();
    } else
    {
        //gApp->defaultKeyboardCallback(key,state);
    }
}

void SimpleMouseButtonCallback( int button, int state, float x, float y)
{
	gApp->defaultMouseButtonCallback(button,state,x,y);
}
void SimpleMouseMoveCallback(  float x, float y)
{
	gApp->defaultMouseMoveCallback(x,y);
}
	
void SimpleWheelCallback( float deltax, float deltay)
{
	gApp->defaultWheelCallback(deltax,deltay);
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
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,GL_LINEAR_MIPMAP_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);

    return TexID;
}




//static unsigned int s_indexData[INDEX_COUNT];
//static GLuint s_indexArrayObject, s_indexBuffer;
//static GLuint s_vertexArrayObject,s_vertexBuffer;


extern unsigned char OpenSansData[];

struct	MyRenderCallbacks : public RenderCallbacks
{
	GLInstancingRenderer* m_instancingRenderer;

	b3AlignedObjectArray<unsigned char> m_rgbaTexture;
	float m_color[4];
	float m_worldPosition[3];
	float m_worldOrientation[4];

	int m_textureIndex;

	MyRenderCallbacks(GLInstancingRenderer* instancingRenderer)
		:m_instancingRenderer(instancingRenderer),
		m_textureIndex(-1)
	{
		for (int i=0;i<4;i++)
		{
			m_color[i] = 1;
			m_worldOrientation[i]=0;
		}
		m_worldPosition[0]=0;
		m_worldPosition[1]=0;
		m_worldPosition[2]=0;

		m_worldOrientation[0] = 0;
		m_worldOrientation[1] = 0;
		m_worldOrientation[2] = 0;
		m_worldOrientation[3] = 1;
	}
	virtual ~MyRenderCallbacks()
	{
		m_rgbaTexture.clear();
	}

	virtual void setWorldPosition(float pos[3])
	{
		for (int i=0;i<3;i++)
		{
			m_worldPosition[i] = pos[i];
		}
	}

	virtual void setWorldOrientation(float orn[4])
	{
		for (int i=0;i<4;i++)
		{
			m_worldOrientation[i] = orn[i];
		}
	}


	virtual void setColorRGBA(float color[4])
	{
		for (int i=0;i<4;i++)
		{
			m_color[i] = color[i];
		}
	}
	virtual void updateTexture(sth_texture* texture, sth_glyph* glyph, int textureWidth, int textureHeight)
	{
		if (glyph)
		{
				m_rgbaTexture.resize(textureWidth*textureHeight*3);
				for (int i=0;i<textureWidth*textureHeight;i++)
				{
					m_rgbaTexture[i*3+0] = texture->m_texels[i];
					m_rgbaTexture[i*3+1] = texture->m_texels[i];
					m_rgbaTexture[i*3+2] = texture->m_texels[i];
				}
				bool flipPixelsY = false;
				m_instancingRenderer->updateTexture(m_textureIndex, &m_rgbaTexture[0], flipPixelsY);
		} else
		{
			if (textureWidth && textureHeight)
			{
				texture->m_texels = (unsigned char*)malloc(textureWidth*textureHeight);
				memset(texture->m_texels,0,textureWidth*textureHeight);
				if (m_textureIndex<0)
				{
					m_rgbaTexture.resize(textureWidth*textureHeight*3);
					bool flipPixelsY = false;
					m_textureIndex = m_instancingRenderer->registerTexture(&m_rgbaTexture[0],textureWidth,textureHeight,flipPixelsY);

					int strideInBytes = 9*sizeof(float);
					int numVertices = sizeof(cube_vertices_textured)/strideInBytes;
					int numIndices = sizeof(cube_indices)/sizeof(int);

					float halfExtentsX=1;
					float halfExtentsY=1;
					float halfExtentsZ=1;
					float textureScaling = 4;

					b3AlignedObjectArray<GfxVertexFormat1> verts;
					verts.resize(numVertices);
					for (int i=0;i<numVertices;i++)
					{
						verts[i].x = halfExtentsX*cube_vertices_textured[i*9];
						verts[i].y = halfExtentsY*cube_vertices_textured[i*9+1];
						verts[i].z = halfExtentsZ*cube_vertices_textured[i*9+2];
						verts[i].w = cube_vertices_textured[i*9+3];
						verts[i].nx = cube_vertices_textured[i*9+4];
						verts[i].ny = cube_vertices_textured[i*9+5];
						verts[i].nz = cube_vertices_textured[i*9+6];
						verts[i].u = cube_vertices_textured[i*9+7]*textureScaling;
						verts[i].v = cube_vertices_textured[i*9+8]*textureScaling;
					}
	
				
					int shapeId = m_instancingRenderer->registerShape(&verts[0].x,numVertices,cube_indices,numIndices,B3_GL_TRIANGLES,m_textureIndex);
					b3Vector3 pos = b3MakeVector3(0, 0, 0);
					b3Quaternion orn(0, 0, 0, 1);
					b3Vector4 color = b3MakeVector4(1, 1, 1,1);
					b3Vector3 scaling = b3MakeVector3 (.1, .1, .1);
					//m_instancingRenderer->registerGraphicsInstance(shapeId, pos, orn, color, scaling);
					m_instancingRenderer->writeTransforms();
				
				} else
				{
					b3Assert(0);
				}
			} else
			{
				delete texture->m_texels;
				texture->m_texels = 0;
				//there is no m_instancingRenderer->freeTexture (yet), all textures are released at reset/deletion of the renderer
			}
		}
		
	}
	virtual void render(sth_texture* texture)
	{
		int index=0;
		
		float width = 1;
		b3AlignedObjectArray< unsigned int> indices;
		indices.resize(texture->nverts);
		for (int i=0;i<indices.size();i++)
		{
			indices[i] = i;
		}

		m_instancingRenderer->drawTexturedTriangleMesh(m_worldPosition,m_worldOrientation, &texture->newverts[0].position.p[0],texture->nverts,&indices[0],indices.size(),m_color,m_textureIndex);
	}
};

SimpleOpenGL3App::SimpleOpenGL3App(	const char* title, int width,int height, bool allowRetina)
{
	gApp = this;

	m_data = new SimpleInternalData;
	m_data->m_frameDumpPngFileName = 0;
	m_data->m_renderTexture = 0;
	m_data->m_ffmpegFile = 0;
	m_data->m_userPointer = 0;
	m_data->m_upAxis = 1;

	m_window = new b3gDefaultOpenGLWindow();
   
	m_window->setAllowRetina(allowRetina);
	
	b3gWindowConstructionInfo ci;
	ci.m_title = title;
	ci.m_width = width;
	ci.m_height = height;
	m_window->createWindow(ci);

	m_window->setWindowTitle(title);

	b3Assert(glGetError() ==GL_NO_ERROR);

	glClearColor(	m_backgroundColorRGB[0],
					m_backgroundColorRGB[1],
					m_backgroundColorRGB[2],
					1.f);
	
	m_window->startRendering();
    width = m_window->getWidth();
    height = m_window->getHeight();
    
	b3Assert(glGetError() ==GL_NO_ERROR);

#ifndef NO_GLEW
#ifndef __APPLE__
#ifndef _WIN32
    //some Linux implementations need the 'glewExperimental' to be true
    glewExperimental = GL_TRUE;
#endif //_WIN32
    
    
    if (glewInit() != GLEW_OK)
        exit(1); // or handle the error in a nicer way
    if (!GLEW_VERSION_2_1)  // check that the machine supports the 2.1 API.
        exit(1); // or handle the error in a nicer way
    
#endif //__APPLE__
#endif //NO_GLEW
    glGetError();//don't remove this call, it is needed for Ubuntu

    b3Assert(glGetError() ==GL_NO_ERROR);

	m_parameterInterface = 0;
    
    b3Assert(glGetError() ==GL_NO_ERROR);

	m_instancingRenderer = new GLInstancingRenderer(128*1024,128*1024*1024);

    m_primRenderer = new GLPrimitiveRenderer(width,height);
    
    m_renderer = m_instancingRenderer ;
    m_window->setResizeCallback(SimpleResizeCallback);
    
    
    m_instancingRenderer->init();
	m_instancingRenderer->resize(width,height);
    m_primRenderer->setScreenSize(width,height);
    b3Assert(glGetError() ==GL_NO_ERROR);

	m_instancingRenderer->InitShaders();

	m_window->setMouseMoveCallback(SimpleMouseMoveCallback);
	m_window->setMouseButtonCallback(SimpleMouseButtonCallback);
    m_window->setKeyboardCallback(SimpleKeyboardCallback);
    m_window->setWheelCallback(SimpleWheelCallback);
	
	TwGenerateDefaultFonts();
	m_data->m_fontTextureId = BindFont(g_DefaultNormalFont);
	m_data->m_largeFontTextureId = BindFont(g_DefaultLargeFont);
	


	{


	m_data->m_renderCallbacks = new OpenGL2RenderCallbacks(m_primRenderer);
	m_data->m_renderCallbacks2 = new MyRenderCallbacks(m_instancingRenderer);
	m_data->m_fontStash2 = sth_create(512,512, m_data->m_renderCallbacks2);
	m_data->m_fontStash = sth_create(512,512,m_data->m_renderCallbacks);//256,256);//,1024);//512,512);

    b3Assert(glGetError() ==GL_NO_ERROR);

	if (!m_data->m_fontStash)
	{
		b3Warning("Could not create stash");
		//fprintf(stderr, "Could not create stash.\n");
	}

	if (!m_data->m_fontStash2)
	{
		b3Warning("Could not create fontStash2");
	}

	unsigned char* data2 = OpenSansData;
	unsigned char* data = (unsigned char*) data2;
	if (!(m_data->m_droidRegular = sth_add_font_from_memory(m_data->m_fontStash, data)))
	{
		b3Warning("error!\n");
	}
	if (!(m_data->m_droidRegular2 = sth_add_font_from_memory(m_data->m_fontStash2, data)))
	{
		b3Warning("error!\n");
	}

	b3Assert(glGetError() ==GL_NO_ERROR);
	}
}


struct sth_stash* SimpleOpenGL3App::getFontStash()
{
	return m_data->m_fontStash;
}

void SimpleOpenGL3App::drawText3D( const char* txt, float position[3], float orientation[4], float color[4], float size, int optionFlag)
{
	B3_PROFILE("SimpleOpenGL3App::drawText3D");
	float viewMat[16];
	float projMat[16];
	CommonCameraInterface* cam = m_instancingRenderer->getActiveCamera();

	cam->getCameraViewMatrix(viewMat);
	cam->getCameraProjectionMatrix(projMat);

	
	float camPos[4];
	cam->getCameraPosition(camPos);
	//b3Vector3 cp= b3MakeVector3(camPos[0],camPos[2],camPos[1]);
	//b3Vector3 p = b3MakeVector3(worldPosX,worldPosY,worldPosZ);
	//float dist = (cp-p).length();
	//float dv = 0;//dist/1000.f;
    //
    //printf("str = %s\n",unicodeText);

    float dx=0;

    //int measureOnly=0;

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	int viewport[4]={0,0,m_instancingRenderer->getScreenWidth(),m_instancingRenderer->getScreenHeight()};

	float posX = position[0];
	float posY = position[1];
	float posZ = position[2];
	float winx,winy, winz;

	
	if (optionFlag&CommonGraphicsApp::eDrawText3D_OrtogonalFaceCamera)
	{
		if (!projectWorldCoordToScreen(position[0], position[1], position[2],viewMat,projMat,viewport,&winx, &winy, &winz))
		{
			return;
		}
		posX = winx;
		posY = m_instancingRenderer->getScreenHeight()/2+(m_instancingRenderer->getScreenHeight()/2)-winy;
		posZ = 0.f;
	} 
	
	if (optionFlag&CommonGraphicsApp::eDrawText3D_TrueType)
	{
		bool measureOnly = false;

		float fontSize= 64;//512;//128;
		
		if (optionFlag&CommonGraphicsApp::eDrawText3D_OrtogonalFaceCamera)
		{
			sth_draw_text(m_data->m_fontStash,
						m_data->m_droidRegular,fontSize,posX,posY,
						txt,&dx, this->m_instancingRenderer->getScreenWidth(),this->m_instancingRenderer->getScreenHeight(),measureOnly,m_window->getRetinaScale(),color);
			sth_end_draw(m_data->m_fontStash);
			sth_flush_draw(m_data->m_fontStash);
		} else
		{
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


			m_data->m_renderCallbacks2->setColorRGBA(color);

			m_data->m_renderCallbacks2->setWorldPosition(position);
			m_data->m_renderCallbacks2->setWorldOrientation(orientation);

			sth_draw_text3D(m_data->m_fontStash2,
						m_data->m_droidRegular2,fontSize,0,0,0,
						txt,&dx,size,color,0);
			sth_end_draw(m_data->m_fontStash2);
			sth_flush_draw(m_data->m_fontStash2);
			glDisable(GL_BLEND);
		}
	} else
	{
		//float width = 0.f;
		int pos=0;
		//float color[]={0.2f,0.2,0.2f,1.f};
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D,m_data->m_largeFontTextureId);

		//float width = r.x;
		//float extraSpacing = 0.;

		float startX = posX;
		float startY = posY+g_DefaultLargeFont->m_CharHeight*size;
		float z = position[2];//2.f*winz-1.f;//*(far
			
		if (optionFlag&CommonGraphicsApp::eDrawText3D_OrtogonalFaceCamera)
		{
			posX = winx;
			posY = m_instancingRenderer->getScreenHeight()/2+(m_instancingRenderer->getScreenHeight()/2)-winy;
			z = 2.f*winz-1.f;
			startY = posY-g_DefaultLargeFont->m_CharHeight*size;
		}

		while (txt[pos])
		{
			int c = txt[pos];
			//r.h = g_DefaultNormalFont->m_CharHeight;
			//r.w = g_DefaultNormalFont->m_CharWidth[c]+extraSpacing;
			float endX = startX+g_DefaultLargeFont->m_CharWidth[c]*size;
			if (optionFlag&CommonGraphicsApp::eDrawText3D_OrtogonalFaceCamera)
			{
				endX = startX+g_DefaultLargeFont->m_CharWidth[c]*size;
			}
			float endY = posY;


			//float currentColor[]={1.f,1.,0.2f,1.f};

		//	m_primRenderer->drawTexturedRect(startX, startY, endX, endY, currentColor,g_DefaultLargeFont->m_CharU0[c],g_DefaultLargeFont->m_CharV0[c],g_DefaultLargeFont->m_CharU1[c],g_DefaultLargeFont->m_CharV1[c]);
			float u0 = g_DefaultLargeFont->m_CharU0[c];
			float u1 = g_DefaultLargeFont->m_CharU1[c];
			float v0 = g_DefaultLargeFont->m_CharV0[c];
			float v1 = g_DefaultLargeFont->m_CharV1[c];
			//float color[4] = {currentColor[0],currentColor[1],currentColor[2],currentColor[3]};
			float x0 = startX;
			float x1 = endX;
			float y0 = startY;
			float y1 = endY;
			int screenWidth = m_instancingRenderer->getScreenWidth();
			int screenHeight = m_instancingRenderer->getScreenHeight();

			 float identity[16]={1,0,0,0,
						0,1,0,0,
						0,0,1,0,
						0,0,0,1};
			 if (optionFlag&CommonGraphicsApp::eDrawText3D_OrtogonalFaceCamera)
			 {
				   PrimVertex vertexData[4] = {
					PrimVertex(PrimVec4(-1.f+2.f*x0/float(screenWidth), 1.f-2.f*y0/float(screenHeight), z, 1.f ), PrimVec4( color[0], color[1], color[2], color[3] ) ,PrimVec2(u0,v0)),
					PrimVertex(PrimVec4(-1.f+2.f*x0/float(screenWidth),  1.f-2.f*y1/float(screenHeight), z, 1.f ), PrimVec4( color[0], color[1], color[2], color[3] ) ,PrimVec2(u0,v1)),
					PrimVertex(PrimVec4( -1.f+2.f*x1/float(screenWidth),  1.f-2.f*y1/float(screenHeight), z, 1.f ), PrimVec4(color[0], color[1], color[2], color[3]) ,PrimVec2(u1,v1)),
					PrimVertex(PrimVec4( -1.f+2.f*x1/float(screenWidth), 1.f-2.f*y0/float(screenHeight), z, 1.f ), PrimVec4( color[0], color[1], color[2], color[3] ) ,PrimVec2(u1,v0))
				};
				m_primRenderer->drawTexturedRect3D(vertexData[0],vertexData[1],vertexData[2],vertexData[3],identity,identity,false);

			 }     else
			 {
				PrimVertex vertexData[4] = {
						PrimVertex(PrimVec4(x0, y0, z, 1.f ), PrimVec4( color[0], color[1], color[2], color[3] ) ,PrimVec2(u0,v0)),
						PrimVertex(PrimVec4(x0,  y1, z, 1.f ), PrimVec4( color[0], color[1], color[2], color[3] ) ,PrimVec2(u0,v1)),
						PrimVertex(PrimVec4( x1,  y1, z, 1.f ), PrimVec4(color[0], color[1], color[2], color[3]) ,PrimVec2(u1,v1)),
						PrimVertex(PrimVec4( x1, y0, z, 1.f ), PrimVec4( color[0], color[1], color[2], color[3] ) ,PrimVec2(u1,v0))
					};

				m_primRenderer->drawTexturedRect3D(vertexData[0],vertexData[1],vertexData[2],vertexData[3],viewMat,projMat,false);
			 }
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

void SimpleOpenGL3App::drawText3D( const char* txt, float worldPosX, float worldPosY, float worldPosZ, float size1)
{
	
	float position[3] = {worldPosX,worldPosY,worldPosZ};
	float orientation[4] = {0,0,0,1};
	float color[4] = {0,0,0,1};
	int optionFlags = CommonGraphicsApp::eDrawText3D_OrtogonalFaceCamera;
	drawText3D(txt,position,orientation,color,size1,optionFlags);

}



void SimpleOpenGL3App::drawText( const char* txt, int posXi, int posYi, float size, float colorRGBA[4])
{

	float posX = (float)posXi;
	float posY = (float) posYi;


    //
    //printf("str = %s\n",unicodeText);

    float dx=0;

    //int measureOnly=0;

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


	if (1)//m_useTrueTypeFont)
	{
		bool measureOnly = false;

		float fontSize= 64*size;//512;//128;
		sth_draw_text(m_data->m_fontStash,
                    m_data->m_droidRegular,fontSize,posX,posY,
					txt,&dx, this->m_instancingRenderer->getScreenWidth(),
					this->m_instancingRenderer->getScreenHeight(),
					measureOnly,
					m_window->getRetinaScale(),colorRGBA);
					
		sth_end_draw(m_data->m_fontStash);
		sth_flush_draw(m_data->m_fontStash);
	} else
	{
		//float width = 0.f;
		int pos=0;
		//float color[]={0.2f,0.2,0.2f,1.f};
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D,m_data->m_largeFontTextureId);

		//float width = r.x;
		//float extraSpacing = 0.;

		float startX = posX;
		float startY = posY;


		while (txt[pos])
		{
			int c = txt[pos];
			//r.h = g_DefaultNormalFont->m_CharHeight;
			//r.w = g_DefaultNormalFont->m_CharWidth[c]+extraSpacing;
			float endX = startX+g_DefaultLargeFont->m_CharWidth[c];
			float endY = startY+g_DefaultLargeFont->m_CharHeight;


			float currentColor[]={0.2f,0.2,0.2f,1.f};

			m_primRenderer->drawTexturedRect(startX, startY, endX, endY, currentColor,g_DefaultLargeFont->m_CharU0[c],g_DefaultLargeFont->m_CharV0[c],g_DefaultLargeFont->m_CharU1[c],g_DefaultLargeFont->m_CharV1[c]);

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


void SimpleOpenGL3App::drawTexturedRect(float x0, float y0, float x1, float y1, float color[4], float u0,float v0, float u1, float v1, int useRGBA)
{
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	m_primRenderer->drawTexturedRect(x0,y0,x1,y1,color,u0,v0,u1,v1,useRGBA);
	glDisable(GL_BLEND);
}




int	SimpleOpenGL3App::registerCubeShape(float halfExtentsX,float halfExtentsY, float halfExtentsZ, int textureIndex, float textureScaling)
{


	int strideInBytes = 9*sizeof(float);
	int numVertices = sizeof(cube_vertices_textured)/strideInBytes;
	int numIndices = sizeof(cube_indices)/sizeof(int);

	b3AlignedObjectArray<GfxVertexFormat1> verts;
	verts.resize(numVertices);
	for (int i=0;i<numVertices;i++)
	{
		verts[i].x = halfExtentsX*cube_vertices_textured[i*9];
		verts[i].y = halfExtentsY*cube_vertices_textured[i*9+1];
		verts[i].z = halfExtentsZ*cube_vertices_textured[i*9+2];
		verts[i].w = cube_vertices_textured[i*9+3];
		verts[i].nx = cube_vertices_textured[i*9+4];
		verts[i].ny = cube_vertices_textured[i*9+5];
		verts[i].nz = cube_vertices_textured[i*9+6];
		verts[i].u = cube_vertices_textured[i*9+7]*textureScaling;
		verts[i].v = cube_vertices_textured[i*9+8]*textureScaling;
	}
	
	int shapeId = m_instancingRenderer->registerShape(&verts[0].x,numVertices,cube_indices,numIndices,B3_GL_TRIANGLES,textureIndex);
	return shapeId;
}

void SimpleOpenGL3App::registerGrid(int cells_x, int cells_z, float color0[4], float color1[4])
{
	b3Vector3 cubeExtents=b3MakeVector3(0.5,0.5,0.5);
	double halfHeight=0.1;
	cubeExtents[m_data->m_upAxis] = halfHeight;
	int cubeId = registerCubeShape(cubeExtents[0],cubeExtents[1],cubeExtents[2]);
	b3Quaternion orn(0,0,0,1);
	b3Vector3 center=b3MakeVector3(0,0,0,1);
	b3Vector3 scaling=b3MakeVector3(1,1,1,1);

	for ( int i = 0; i < cells_x; i++) 
	{
		for (int j = 0; j < cells_z; j++) 
		{
			float* color =0;
			if ((i + j) % 2 == 0) 
			{
				color = (float*)color0;
			} else {
				color = (float*)color1;
			}
			if (this->m_data->m_upAxis==1)
			{
				center =b3MakeVector3((i + 0.5f) - cells_x * 0.5f, -halfHeight, (j + 0.5f) - cells_z * 0.5f);
			} else
			{
				center =b3MakeVector3((i + 0.5f) - cells_x * 0.5f, (j + 0.5f) - cells_z * 0.5f,-halfHeight );
			}
			m_instancingRenderer->registerGraphicsInstance(cubeId,center,orn,color,scaling);
		}
	}
	
}

int	SimpleOpenGL3App::registerGraphicsUnitSphereShape(EnumSphereLevelOfDetail lod, int textureId)
{

	int strideInBytes = 9*sizeof(float);

	int graphicsShapeIndex = -1;

	switch (lod)
	{
		case 		SPHERE_LOD_POINT_SPRITE:
		{
			int numVertices = sizeof(point_sphere_vertices)/strideInBytes;
			int numIndices = sizeof(point_sphere_indices)/sizeof(int);
			graphicsShapeIndex = m_instancingRenderer->registerShape(&point_sphere_vertices[0],numVertices,point_sphere_indices,numIndices,B3_GL_POINTS,textureId);
			break;
		}

		case SPHERE_LOD_LOW:
		{
			int numVertices = sizeof(low_sphere_vertices)/strideInBytes;
			int numIndices = sizeof(low_sphere_indices)/sizeof(int);
			graphicsShapeIndex = m_instancingRenderer->registerShape(&low_sphere_vertices[0],numVertices,low_sphere_indices,numIndices,B3_GL_TRIANGLES,textureId);
			break;
		}
		case SPHERE_LOD_MEDIUM:
		{
			int numVertices = sizeof(medium_sphere_vertices)/strideInBytes;
			int numIndices = sizeof(medium_sphere_indices)/sizeof(int);
			graphicsShapeIndex = m_instancingRenderer->registerShape(&medium_sphere_vertices[0],numVertices,medium_sphere_indices,numIndices,B3_GL_TRIANGLES,textureId);
			break;
		}
		case SPHERE_LOD_HIGH:
		default:
		{
			int numVertices = sizeof(detailed_sphere_vertices)/strideInBytes;
			int numIndices = sizeof(detailed_sphere_indices)/sizeof(int);
			graphicsShapeIndex = m_instancingRenderer->registerShape(&detailed_sphere_vertices[0],numVertices,detailed_sphere_indices,numIndices,B3_GL_TRIANGLES,textureId);
			break;
		}
	};
	return graphicsShapeIndex;
}


void SimpleOpenGL3App::drawGrid(DrawGridData data)
{
    int gridSize = data.gridSize;
    float upOffset = data.upOffset;
    int upAxis = data.upAxis;
    float gridColor[4];
    gridColor[0] = data.gridColor[0];
    gridColor[1] = data.gridColor[1];
    gridColor[2] = data.gridColor[2];
    gridColor[3] = data.gridColor[3];

	int sideAxis=-1;
	int forwardAxis=-1;

	switch (upAxis)
	{
		case 1:
			forwardAxis=2;
			sideAxis=0;
			break;
		case 2:
			forwardAxis=1;
			sideAxis=0;
			break;
		default:
			b3Assert(0);
	};
	//b3Vector3 gridColor = b3MakeVector3(0.5,0.5,0.5);

	b3AlignedObjectArray<unsigned int> indices;
	b3AlignedObjectArray<b3Vector3> vertices;
	int lineIndex=0;
	for(int i=-gridSize;i<=gridSize;i++)
	{
		{
			b3Assert(glGetError() ==GL_NO_ERROR);
			b3Vector3 from = b3MakeVector3(0,0,0);
			from[sideAxis] = float(i);
			from[upAxis] = upOffset;
			from[forwardAxis] = float(-gridSize);
			b3Vector3 to=b3MakeVector3(0,0,0);
			to[sideAxis] = float(i);
			to[upAxis] = upOffset;
			to[forwardAxis] = float(gridSize);
			vertices.push_back(from);
			indices.push_back(lineIndex++);
			vertices.push_back(to);
			indices.push_back(lineIndex++);
			// m_instancingRenderer->drawLine(from,to,gridColor);
		}

		b3Assert(glGetError() ==GL_NO_ERROR);
		{

			b3Assert(glGetError() ==GL_NO_ERROR);
			b3Vector3 from=b3MakeVector3(0,0,0);
			from[sideAxis] = float(-gridSize);
			from[upAxis] = upOffset;
			from[forwardAxis] = float(i);
			b3Vector3 to=b3MakeVector3(0,0,0);
			to[sideAxis] = float(gridSize);
			to[upAxis] = upOffset;
			to[forwardAxis] = float(i);
			vertices.push_back(from);
			indices.push_back(lineIndex++);
			vertices.push_back(to);
			indices.push_back(lineIndex++);
			// m_instancingRenderer->drawLine(from,to,gridColor);
		}

	}


	m_instancingRenderer->drawLines(&vertices[0].x,
			gridColor,
			vertices.size(),sizeof(b3Vector3),&indices[0],indices.size(),1);
	

	m_instancingRenderer->drawLine(b3MakeVector3(0,0,0),b3MakeVector3(1,0,0),b3MakeVector3(1,0,0),3);
	m_instancingRenderer->drawLine(b3MakeVector3(0,0,0),b3MakeVector3(0,1,0),b3MakeVector3(0,1,0),3);
	m_instancingRenderer->drawLine(b3MakeVector3(0,0,0),b3MakeVector3(0,0,1),b3MakeVector3(0,0,1),3);

//	void GLInstancingRenderer::drawPoints(const float* positions, const float color[4], int numPoints, int pointStrideInBytes, float pointDrawSize)

	//we don't use drawPoints because all points would have the same color
//	b3Vector3 points[3] = { b3MakeVector3(1, 0, 0), b3MakeVector3(0, 1, 0), b3MakeVector3(0, 0, 1) };
//	m_instancingRenderer->drawPoints(&points[0].x, b3MakeVector3(1, 0, 0), 3, sizeof(b3Vector3), 6);

	m_instancingRenderer->drawPoint(b3MakeVector3(1,0,0),b3MakeVector3(1,0,0),6);
	m_instancingRenderer->drawPoint(b3MakeVector3(0,1,0),b3MakeVector3(0,1,0),6);
	m_instancingRenderer->drawPoint(b3MakeVector3(0,0,1),b3MakeVector3(0,0,1),6);
}

void SimpleOpenGL3App::setBackgroundColor(float red, float green, float blue)
{
	CommonGraphicsApp::setBackgroundColor(red,green,blue);
	glClearColor(m_backgroundColorRGB[0],m_backgroundColorRGB[1],m_backgroundColorRGB[2],1.f);
}

SimpleOpenGL3App::~SimpleOpenGL3App()
{
	
	delete m_instancingRenderer;
	delete m_primRenderer ;
	sth_delete(m_data->m_fontStash);
	delete m_data->m_renderCallbacks;

	sth_delete(m_data->m_fontStash2);
	delete m_data->m_renderCallbacks2;

	TwDeleteDefaultFonts();
	m_window->closeWindow();
	
	delete m_window;
	delete m_data ;
}

void SimpleOpenGL3App::getScreenPixels(unsigned char* rgbaBuffer, int bufferSizeInBytes, float* depthBuffer, int depthBufferSizeInBytes)
{
    
    int width = (int)m_window->getRetinaScale()*m_instancingRenderer->getScreenWidth();
    int height = (int)m_window->getRetinaScale()*m_instancingRenderer->getScreenHeight();
    if ((width*height*4) == bufferSizeInBytes)
    {
        glReadPixels(0,0,width, height, GL_RGBA, GL_UNSIGNED_BYTE, rgbaBuffer);
		int glstat;
		glstat = glGetError();
		b3Assert(glstat==GL_NO_ERROR);
    }
    if ((width*height*sizeof(float)) == depthBufferSizeInBytes)
    {
        glReadPixels(0,0,width, height, GL_DEPTH_COMPONENT, GL_FLOAT, depthBuffer);
		int glstat;
		glstat = glGetError();
		b3Assert(glstat==GL_NO_ERROR);
    }
    
}

//#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
static void writeTextureToFile(int textureWidth, int textureHeight, const char* fileName, FILE* ffmpegVideo)
{
	int numComponents = 4;
	//glPixelStorei(GL_PACK_ALIGNMENT,1);
	
	b3Assert(glGetError()==GL_NO_ERROR);
	//glReadBuffer(GL_BACK);//COLOR_ATTACHMENT0);
	
	float* orgPixels = (float*)malloc(textureWidth*textureHeight*numComponents*4);
	glReadPixels(0,0,textureWidth, textureHeight, GL_RGBA, GL_FLOAT, orgPixels);
	//it is useful to have the actual float values for debugging purposes

	//convert float->char
	char* pixels = (char*)malloc(textureWidth*textureHeight*numComponents);
	assert(glGetError()==GL_NO_ERROR);

	for (int j=0;j<textureHeight;j++)
	{
		for (int i=0;i<textureWidth;i++)
		{
			pixels[(j*textureWidth+i)*numComponents] = char(orgPixels[(j*textureWidth+i)*numComponents]*255.f);
			pixels[(j*textureWidth+i)*numComponents+1]=char(orgPixels[(j*textureWidth+i)*numComponents+1]*255.f);
			pixels[(j*textureWidth+i)*numComponents+2]=char(orgPixels[(j*textureWidth+i)*numComponents+2]*255.f);
			pixels[(j*textureWidth+i)*numComponents+3]=char(orgPixels[(j*textureWidth+i)*numComponents+3]*255.f);
		}
	}



    if (ffmpegVideo)
    {
        fwrite(pixels, textureWidth*textureHeight*numComponents, 1, ffmpegVideo);
        //fwrite(pixels, 100,1,ffmpegVideo);//textureWidth*textureHeight*numComponents, 1, ffmpegVideo);
    } else
    {
        if (1)
        {
            //swap the pixels
            unsigned char tmp;

            for (int j=0;j<textureHeight/2;j++)
            {
                for (int i=0;i<textureWidth;i++)
                {
                    for (int c=0;c<numComponents;c++)
                    {
                        tmp = pixels[(j*textureWidth+i)*numComponents+c];
                        pixels[(j*textureWidth+i)*numComponents+c]=
                        pixels[((textureHeight-j-1)*textureWidth+i)*numComponents+c];
                        pixels[((textureHeight-j-1)*textureWidth+i)*numComponents+c] = tmp;
                    }
                }
            }
        }
        stbi_write_png(fileName, textureWidth,textureHeight, numComponents, pixels, textureWidth*numComponents);
    }


	free(pixels);
	free(orgPixels);

}


void SimpleOpenGL3App::swapBuffer()
{

	if (m_data->m_frameDumpPngFileName)
    {
        int width = (int)m_window->getRetinaScale()*m_instancingRenderer->getScreenWidth();
        int height = (int) m_window->getRetinaScale()*this->m_instancingRenderer->getScreenHeight();
        writeTextureToFile(width,
                          height,m_data->m_frameDumpPngFileName,
                          m_data->m_ffmpegFile);
        m_data->m_renderTexture->disable();
        if (m_data->m_ffmpegFile==0)
        {
			m_data->m_frameDumpPngFileName = 0;
        }
    }
	m_window->endRendering();
	m_window->startRendering();
}

// see also http://blog.mmacklin.com/2013/06/11/real-time-video-capture-with-ffmpeg/
void SimpleOpenGL3App::dumpFramesToVideo(const char* mp4FileName)
{
	if (mp4FileName)
	{
		int width = (int)m_window->getRetinaScale()*m_instancingRenderer->getScreenWidth();
		int height = (int)m_window->getRetinaScale()*m_instancingRenderer->getScreenHeight();
		char cmd[8192];

#ifdef _WIN32
	sprintf(cmd, "ffmpeg -r 60 -f rawvideo -pix_fmt rgba -s %dx%d -i - "
		"-threads 0 -y -b:v 50000k   -c:v libx264 -preset slow -crf 22 -an   -pix_fmt yuv420p -vf vflip %s", width, height, mp4FileName);

    //sprintf(cmd, "ffmpeg -r 60 -f rawvideo -pix_fmt rgba   -s %dx%d -i - "
    //		"-y -crf 0  -b:v 1500000 -an -vcodec h264 -vf vflip  %s", width, height, mp4FileName);
#else
   
   sprintf(cmd, "ffmpeg -r 60 -f rawvideo -pix_fmt rgba -s %dx%d -i - "
    		"-threads 0 -y -b 50000k   -c:v libx264 -preset slow -crf 22 -an   -pix_fmt yuv420p -vf vflip %s", width, height, mp4FileName);
#endif
    
    //sprintf(cmd,"ffmpeg -r 60 -f rawvideo -pix_fmt rgba -s %dx%d -i - "
     //            "-threads 0 -y -crf 0 -b 50000k -vf vflip %s",width,height,mp4FileName);

    //              sprintf(cmd,"ffmpeg -r 60 -f rawvideo -pix_fmt rgba -s %dx%d -i - "
    //              "-threads 0 -preset fast -y -crf 21 -vf vflip %s",width,height,mp4FileName);

		if (m_data->m_ffmpegFile)
		{
			pclose(m_data->m_ffmpegFile);
		}
		if (mp4FileName)
		{
			m_data->m_ffmpegFile = popen(cmd, "w");

			m_data->m_frameDumpPngFileName = mp4FileName;
		}
	} else
	{
		if (m_data->m_ffmpegFile)
		{
			fflush(m_data->m_ffmpegFile);
			pclose(m_data->m_ffmpegFile);
			m_data->m_frameDumpPngFileName = 0;
		}
		m_data->m_ffmpegFile = 0;
	}
}
void SimpleOpenGL3App::dumpNextFrameToPng(const char* filename)
{

    // open pipe to ffmpeg's stdin in binary write mode

    m_data->m_frameDumpPngFileName = filename;

//you could use m_renderTexture to allow to render at higher resolutions, such as 4k or so
    if (!m_data->m_renderTexture)
    {
            m_data->m_renderTexture = new GLRenderToTexture();
            GLuint renderTextureId;
            glGenTextures(1, &renderTextureId);

            // "Bind" the newly created texture : all future texture functions will modify this texture
            glBindTexture(GL_TEXTURE_2D, renderTextureId);

            // Give an empty image to OpenGL ( the last "0" )
            //glTexImage2D(GL_TEXTURE_2D, 0,GL_RGB, g_OpenGLWidth,g_OpenGLHeight, 0,GL_RGBA, GL_UNSIGNED_BYTE, 0);
            //glTexImage2D(GL_TEXTURE_2D, 0,GL_RGBA32F, g_OpenGLWidth,g_OpenGLHeight, 0,GL_RGBA, GL_FLOAT, 0);
            glTexImage2D(GL_TEXTURE_2D, 0,GL_RGBA32F,
                         m_instancingRenderer->getScreenWidth()*m_window->getRetinaScale(),m_instancingRenderer->getScreenHeight()*m_window->getRetinaScale()
                         , 0,GL_RGBA, GL_FLOAT, 0);

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

            m_data->m_renderTexture->init(m_instancingRenderer->getScreenWidth()*m_window->getRetinaScale(),this->m_instancingRenderer->getScreenHeight()*m_window->getRetinaScale(),renderTextureId, RENDERTEXTURE_COLOR);
    }

    m_data->m_renderTexture->enable();

}

void SimpleOpenGL3App::setUpAxis(int axis)
{
	b3Assert((axis == 1)||(axis==2));//only Y or Z is supported at the moment
	m_data->m_upAxis = axis;
}
int SimpleOpenGL3App::getUpAxis() const
{
	return m_data->m_upAxis;
}
#endif//#ifndef NO_OPENGL3

