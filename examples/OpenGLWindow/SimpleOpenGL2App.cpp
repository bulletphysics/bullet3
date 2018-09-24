#include "SimpleOpenGL2App.h"
#define USE_OPENGL2
#include "OpenGLInclude.h"
#include "ShapeData.h"
#include "Bullet3Common/b3Logging.h"  //b3Assert?
#include "Bullet3Common/b3Scalar.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "../OpenGLWindow/GLPrimitiveRenderer.h"
#include "GLInstanceGraphicsShape.h"
#include "stdlib.h"
#include "TwFonts.h"
#include "SimpleOpenGL2Renderer.h"

#ifdef B3_USE_GLFW
#include "GLFWOpenGLWindow.h"
#else

#ifdef __APPLE__
#include "MacOpenGLWindow.h"
#else

//#include "GL/glew.h"
#ifdef _WIN32
#include "Win32OpenGLWindow.h"
#else
//let's cross the fingers it is Linux/X11
#include "X11OpenGLWindow.h"
#ifdef BT_USE_EGL
#include "EGLOpenGLWindow.h"
#else
#endif  //BT_USE_EGL
#endif  //_WIN32
#endif  //__APPLE__
#endif  //#ifdef B3_USE_GLFW

#include <stdio.h>
#include "../CommonInterfaces/CommonRenderInterface.h"

static SimpleOpenGL2App* gApp2 = 0;

static void Simple2ResizeCallback(float widthf, float heightf)
{
	int width = (int)widthf;
	int height = (int)heightf;
	if (gApp2->m_renderer && gApp2->m_window)
		gApp2->m_renderer->resize(width, height);  //*gApp2->m_window->getRetinaScale(),height*gApp2->m_window->getRetinaScale());
}

static void Simple2KeyboardCallback(int key, int state)
{
	if (key == B3G_ESCAPE && gApp2 && gApp2->m_window)
	{
		gApp2->m_window->setRequestExit();
	}
	else
	{
		//gApp2->defaultKeyboardCallback(key,state);
	}
}

void Simple2MouseButtonCallback(int button, int state, float x, float y)
{
	if (gApp2 && gApp2->m_window)
	{
		gApp2->defaultMouseButtonCallback(button, state, x, y);
	}
}
void Simple2MouseMoveCallback(float x, float y)
{
	if (gApp2 && gApp2->m_window)
	{
		gApp2->defaultMouseMoveCallback(x, y);
	}
}

void Simple2WheelCallback(float deltax, float deltay)
{
	gApp2->defaultWheelCallback(deltax, deltay);
}

struct SimpleOpenGL2AppInternalData
{
	GLuint m_fontTextureId;
	GLuint m_largeFontTextureId;
	int m_upAxis;
	SimpleOpenGL2AppInternalData()
		: m_upAxis(1)
	{
	}
};
static GLuint BindFont2(const CTexFont* _Font)
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
	glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, _Font->m_TexWidth, _Font->m_TexHeight, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, _Font->m_TexBytes);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glBindTexture(GL_TEXTURE_2D, 0);

	return TexID;
}

SimpleOpenGL2App::SimpleOpenGL2App(const char* title, int width, int height)
{
	gApp2 = this;
	m_data = new SimpleOpenGL2AppInternalData;

	m_window = new b3gDefaultOpenGLWindow();
	b3gWindowConstructionInfo ci;
	ci.m_title = title;
	ci.m_openglVersion = 2;
	ci.m_width = width;
	ci.m_height = height;
	m_window->createWindow(ci);

	m_window->setWindowTitle(title);

#ifndef NO_GLEW
#ifndef __APPLE__
#ifndef _WIN32
#ifndef B3_USE_GLFW
	//some Linux implementations need the 'glewExperimental' to be true
#endif  //B3_USE_GLFW
#endif  //_WIN32

#ifndef B3_USE_GLFW
	//gladLoadGLLoader((GLADloadproc) glfwGetProcAddress);

#if 0
	if (glewInit() != GLEW_OK)
    {
        b3Error("glewInit failed");
        exit(1);
    }
    if (!GLEW_VERSION_2_1)  // check that the machine supports the 2.1 API.
    {
        b3Error("GLEW_VERSION_2_1 needs to support 2_1");
        exit(1); // or handle the error in a nicer way
    }
#endif
#endif  //B3_USE_GLFW
#endif  //__APPLE__
#endif  //NO_GLEW

	TwGenerateDefaultFonts();
	m_data->m_fontTextureId = BindFont2(g_DefaultNormalFont);
	m_data->m_largeFontTextureId = BindFont2(g_DefaultLargeFont);

	glGetError();  //don't remove this call, it is needed for Ubuntu
	glClearColor(m_backgroundColorRGB[0],
				 m_backgroundColorRGB[1],
				 m_backgroundColorRGB[2],
				 1.f);

	b3Assert(glGetError() == GL_NO_ERROR);

	//m_primRenderer = new GLPrimitiveRenderer(width,height);
	m_parameterInterface = 0;

	b3Assert(glGetError() == GL_NO_ERROR);

	//m_renderer = new GLInstancingRenderer(128*1024,32*1024*1024);
	//m_renderer->init();
	//m_renderer->resize(width,height);

	b3Assert(glGetError() == GL_NO_ERROR);

	//m_renderer->InitShaders();

	m_window->setMouseMoveCallback(Simple2MouseMoveCallback);
	m_window->setMouseButtonCallback(Simple2MouseButtonCallback);
	m_window->setKeyboardCallback(Simple2KeyboardCallback);
	m_window->setWheelCallback(Simple2WheelCallback);
	m_window->setResizeCallback(Simple2ResizeCallback);

	m_renderer = new SimpleOpenGL2Renderer(width, height);
}

SimpleOpenGL2App::~SimpleOpenGL2App()
{
	gApp2 = 0;
	delete m_data;
}

void SimpleOpenGL2App::setBackgroundColor(float red, float green, float blue)
{
	CommonGraphicsApp::setBackgroundColor(red, green, blue);
	glClearColor(m_backgroundColorRGB[0], m_backgroundColorRGB[1], m_backgroundColorRGB[2], 1.f);
}

void SimpleOpenGL2App::drawGrid(DrawGridData data)
{
	glEnable(GL_COLOR_MATERIAL);
	int gridSize = data.gridSize;
	float upOffset = data.upOffset;
	int upAxis = data.upAxis;
	float gridColor[4];
	gridColor[0] = data.gridColor[0];
	gridColor[1] = data.gridColor[1];
	gridColor[2] = data.gridColor[2];
	gridColor[3] = data.gridColor[3];

	int sideAxis = -1;
	int forwardAxis = -1;

	switch (upAxis)
	{
		case 1:
			forwardAxis = 2;
			sideAxis = 0;
			break;
		case 2:
			forwardAxis = 1;
			sideAxis = 0;
			break;
		default:
			b3Assert(0);
	};
	//b3Vector3 gridColor = b3MakeVector3(0.5,0.5,0.5);

	b3AlignedObjectArray<unsigned int> indices;
	b3AlignedObjectArray<b3Vector3> vertices;
	int lineIndex = 0;
	for (int i = -gridSize; i <= gridSize; i++)
	{
		{
			b3Assert(glGetError() == GL_NO_ERROR);
			b3Vector3 from = b3MakeVector3(0, 0, 0);
			from[sideAxis] = float(i);
			from[upAxis] = upOffset;
			from[forwardAxis] = float(-gridSize);
			b3Vector3 to = b3MakeVector3(0, 0, 0);
			to[sideAxis] = float(i);
			to[upAxis] = upOffset;
			to[forwardAxis] = float(gridSize);
			vertices.push_back(from);
			indices.push_back(lineIndex++);
			vertices.push_back(to);
			indices.push_back(lineIndex++);
			//	m_renderer->drawLine(from,to,gridColor);
		}

		b3Assert(glGetError() == GL_NO_ERROR);
		{
			b3Assert(glGetError() == GL_NO_ERROR);
			b3Vector3 from = b3MakeVector3(0, 0, 0);
			from[sideAxis] = float(-gridSize);
			from[upAxis] = upOffset;
			from[forwardAxis] = float(i);
			b3Vector3 to = b3MakeVector3(0, 0, 0);
			to[sideAxis] = float(gridSize);
			to[upAxis] = upOffset;
			to[forwardAxis] = float(i);
			vertices.push_back(from);
			indices.push_back(lineIndex++);
			vertices.push_back(to);
			indices.push_back(lineIndex++);
			//	m_renderer->drawLine(from,to,gridColor);
		}
	}

	m_renderer->drawLines(&vertices[0].x,
						  gridColor,
						  vertices.size(), sizeof(b3Vector3), &indices[0], indices.size(), 1);

	m_renderer->drawLine(b3MakeVector3(0, 0, 0), b3MakeVector3(1, 0, 0), b3MakeVector3(1, 0, 0), 3);
	m_renderer->drawLine(b3MakeVector3(0, 0, 0), b3MakeVector3(0, 1, 0), b3MakeVector3(0, 1, 0), 3);
	m_renderer->drawLine(b3MakeVector3(0, 0, 0), b3MakeVector3(0, 0, 1), b3MakeVector3(0, 0, 1), 3);

	//	void GLInstancingRenderer::drawPoints(const float* positions, const float color[4], int numPoints, int pointStrideInBytes, float pointDrawSize)

	//we don't use drawPoints because all points would have the same color
	//	b3Vector3 points[3] = { b3MakeVector3(1, 0, 0), b3MakeVector3(0, 1, 0), b3MakeVector3(0, 0, 1) };
	//	m_renderer->drawPoints(&points[0].x, b3MakeVector3(1, 0, 0), 3, sizeof(b3Vector3), 6);
}
void SimpleOpenGL2App::setUpAxis(int axis)
{
	this->m_data->m_upAxis = axis;
}
int SimpleOpenGL2App::getUpAxis() const
{
	return this->m_data->m_upAxis;
}

void SimpleOpenGL2App::swapBuffer()
{
	m_window->endRendering();
	m_window->startRendering();
}

void SimpleOpenGL2App::drawText(const char* txt, int posXi, int posYi, float size, float colorRGBA[4])
{
}

static void restoreOpenGLState()
{
	glPopClientAttrib();
	glPopAttrib();
}

static void saveOpenGLState(int screenWidth, int screenHeight)
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glPushClientAttrib(GL_CLIENT_ALL_ATTRIB_BITS);

	glDisable(GL_TEXTURE_GEN_S);
	glDisable(GL_TEXTURE_GEN_T);
	glDisable(GL_TEXTURE_GEN_R);

	glDisable(GL_LINE_SMOOTH);
	//    glDisable(GL_LINE_STIPPLE);
	glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);
	glEnable(GL_BLEND);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glDisable(GL_TEXTURE_2D);
}

void SimpleOpenGL2App::drawText3D(const char* txt, float position[3], float orientation[4], float color[4], float size, int optionFlag)
{
}

void SimpleOpenGL2App::drawText3D(const char* txt, float worldPosX, float worldPosY, float worldPosZ, float size1)
{
	saveOpenGLState(gApp2->m_renderer->getScreenWidth(), gApp2->m_renderer->getScreenHeight());
	float viewMat[16];
	float projMat[16];
	CommonCameraInterface* cam = gApp2->m_renderer->getActiveCamera();

	cam->getCameraViewMatrix(viewMat);
	cam->getCameraProjectionMatrix(projMat);

	float camPos[4];
	cam->getCameraPosition(camPos);
	//b3Vector3 cp= b3MakeVector3(camPos[0],camPos[2],camPos[1]);
	//	b3Vector3 p = b3MakeVector3(worldPosX,worldPosY,worldPosZ);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);

	glAlphaFunc(GL_GREATER, 1.0f);

	int viewport[4] = {0, 0, gApp2->m_renderer->getScreenWidth(), gApp2->m_renderer->getScreenHeight()};

	float posX = 450.f;
	float posY = 100.f;
	float winx, winy, winz;

	if (!projectWorldCoordToScreen(worldPosX, worldPosY, worldPosZ, viewMat, projMat, viewport, &winx, &winy, &winz))
	{
		return;
	}
	posX = winx;
	posY = gApp2->m_renderer->getScreenHeight() / 2 + (gApp2->m_renderer->getScreenHeight() / 2) - winy;

	{
		//float width = 0.f;
		int pos = 0;
		//float color[]={0.2f,0.2,0.2f,1.f};
		glActiveTexture(GL_TEXTURE0);
		glMatrixMode(GL_TEXTURE);
		glLoadIdentity();
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		glBindTexture(GL_TEXTURE_2D, m_data->m_largeFontTextureId);

		glEnable(GL_TEXTURE_2D);  //BindTexture
		//float width = r.x;
		//float extraSpacing = 0.;

		float startX = posX;
		float startY = posY - g_DefaultLargeFont->m_CharHeight * size1;
		glEnable(GL_COLOR_MATERIAL);

		while (txt[pos])
		{
			int c = txt[pos];
			//r.h = g_DefaultNormalFont->m_CharHeight;
			//r.w = g_DefaultNormalFont->m_CharWidth[c]+extraSpacing;
			float endX = startX + g_DefaultLargeFont->m_CharWidth[c] * size1;
			float endY = posY;

			float currentColor[] = {1.f, 0.2, 0.2f, 1.f};
			float u0 = g_DefaultLargeFont->m_CharU0[c];
			float u1 = g_DefaultLargeFont->m_CharU1[c];
			float v0 = g_DefaultLargeFont->m_CharV0[c];
			float v1 = g_DefaultLargeFont->m_CharV1[c];
			float color[4] = {currentColor[0], currentColor[1], currentColor[2], currentColor[3]};
			float x0 = startX;
			float x1 = endX;
			float y0 = startY;
			float y1 = endY;
			int screenWidth = gApp2->m_renderer->getScreenWidth();
			int screenHeight = gApp2->m_renderer->getScreenHeight();

			float z = 2.f * winz - 1.f;  //*(far
										 /*float identity[16]={1,0,0,0,
						0,1,0,0,
						0,0,1,0,
						0,0,0,1};
*/
			PrimVertex vertexData[4] = {
				PrimVertex(PrimVec4(-1.f + 2.f * x0 / float(screenWidth), 1.f - 2.f * y0 / float(screenHeight), z, 1.f), PrimVec4(color[0], color[1], color[2], color[3]), PrimVec2(u0, v0)),
				PrimVertex(PrimVec4(-1.f + 2.f * x0 / float(screenWidth), 1.f - 2.f * y1 / float(screenHeight), z, 1.f), PrimVec4(color[0], color[1], color[2], color[3]), PrimVec2(u0, v1)),
				PrimVertex(PrimVec4(-1.f + 2.f * x1 / float(screenWidth), 1.f - 2.f * y1 / float(screenHeight), z, 1.f), PrimVec4(color[0], color[1], color[2], color[3]), PrimVec2(u1, v1)),
				PrimVertex(PrimVec4(-1.f + 2.f * x1 / float(screenWidth), 1.f - 2.f * y0 / float(screenHeight), z, 1.f), PrimVec4(color[0], color[1], color[2], color[3]), PrimVec2(u1, v0))};

			glBegin(GL_TRIANGLES);
			//use red colored text for now
			glColor4f(1, 0, 0, 1);

			float scaling = 1;

			glTexCoord2f(vertexData[0].uv.p[0], vertexData[0].uv.p[1]);
			glVertex3d(vertexData[0].position.p[0] * scaling, vertexData[0].position.p[1] * scaling, vertexData[0].position.p[2] * scaling);
			glTexCoord2f(vertexData[1].uv.p[0], vertexData[1].uv.p[1]);
			glVertex3d(vertexData[1].position.p[0] * scaling, vertexData[1].position.p[1] * scaling, vertexData[1].position.p[2] * scaling);
			glTexCoord2f(vertexData[2].uv.p[0], vertexData[2].uv.p[1]);
			glVertex3d(vertexData[2].position.p[0] * scaling, vertexData[2].position.p[1] * scaling, vertexData[2].position.p[2] * scaling);

			glTexCoord2f(vertexData[0].uv.p[0], vertexData[0].uv.p[1]);
			glVertex3d(vertexData[0].position.p[0] * scaling, vertexData[0].position.p[1] * scaling, vertexData[0].position.p[2] * scaling);
			glTexCoord2f(vertexData[2].uv.p[0], vertexData[2].uv.p[1]);
			glVertex3d(vertexData[2].position.p[0] * scaling, vertexData[2].position.p[1] * scaling, vertexData[2].position.p[2] * scaling);
			glTexCoord2f(vertexData[3].uv.p[0], vertexData[3].uv.p[1]);
			glVertex3d(vertexData[3].position.p[0] * scaling, vertexData[3].position.p[1] * scaling, vertexData[3].position.p[2] * scaling);

			glEnd();

			startX = endX;
			pos++;
		}
	}

	glBindTexture(GL_TEXTURE_2D, 0);
	glDisable(GL_BLEND);
	glDisable(GL_TEXTURE_2D);

	restoreOpenGLState();
}

void SimpleOpenGL2App::registerGrid(int cells_x, int cells_z, float color0[4], float color1[4])
{
	b3Vector3 cubeExtents = b3MakeVector3(0.5, 0.5, 0.5);
	double halfHeight = 0.1;
	cubeExtents[m_data->m_upAxis] = halfHeight;
	int cubeId = registerCubeShape(cubeExtents[0], cubeExtents[1], cubeExtents[2]);

	b3Quaternion orn(0, 0, 0, 1);
	b3Vector3 center = b3MakeVector3(0, 0, 0, 1);
	b3Vector3 scaling = b3MakeVector3(1, 1, 1, 1);

	for (int i = 0; i < cells_x; i++)
	{
		for (int j = 0; j < cells_z; j++)
		{
			float* color = 0;
			if ((i + j) % 2 == 0)
			{
				color = (float*)color0;
			}
			else
			{
				color = (float*)color1;
			}
			if (this->m_data->m_upAxis == 1)
			{
				center = b3MakeVector3((i + 0.5f) - cells_x * 0.5f, -halfHeight, (j + 0.5f) - cells_z * 0.5f);
			}
			else
			{
				center = b3MakeVector3((i + 0.5f) - cells_x * 0.5f, (j + 0.5f) - cells_z * 0.5f, -halfHeight);
			}
			m_renderer->registerGraphicsInstance(cubeId, center, orn, color, scaling);
		}
	}
}

int SimpleOpenGL2App::registerGraphicsUnitSphereShape(EnumSphereLevelOfDetail lod, int textureId)
{
	int strideInBytes = 9 * sizeof(float);

	int graphicsShapeIndex = -1;

	switch (lod)
	{
		case SPHERE_LOD_POINT_SPRITE:
		{
			int numVertices = sizeof(point_sphere_vertices) / strideInBytes;
			int numIndices = sizeof(point_sphere_indices) / sizeof(int);
			graphicsShapeIndex = m_renderer->registerShape(&point_sphere_vertices[0], numVertices, point_sphere_indices, numIndices, B3_GL_POINTS, textureId);
			break;
		}

		case SPHERE_LOD_LOW:
		{
			int numVertices = sizeof(low_sphere_vertices) / strideInBytes;
			int numIndices = sizeof(low_sphere_indices) / sizeof(int);
			graphicsShapeIndex = m_renderer->registerShape(&low_sphere_vertices[0], numVertices, low_sphere_indices, numIndices, B3_GL_TRIANGLES, textureId);
			break;
		}
		case SPHERE_LOD_MEDIUM:
		{
			int numVertices = sizeof(medium_sphere_vertices) / strideInBytes;
			int numIndices = sizeof(medium_sphere_indices) / sizeof(int);
			graphicsShapeIndex = m_renderer->registerShape(&medium_sphere_vertices[0], numVertices, medium_sphere_indices, numIndices, B3_GL_TRIANGLES, textureId);
			break;
		}
		case SPHERE_LOD_HIGH:
		default:
		{
			int numVertices = sizeof(detailed_sphere_vertices) / strideInBytes;
			int numIndices = sizeof(detailed_sphere_indices) / sizeof(int);
			graphicsShapeIndex = m_renderer->registerShape(&detailed_sphere_vertices[0], numVertices, detailed_sphere_indices, numIndices, B3_GL_TRIANGLES, textureId);
			break;
		}
	};
	return graphicsShapeIndex;
}

int SimpleOpenGL2App::registerCubeShape(float halfExtentsX, float halfExtentsY, float halfExtentsZ, int textureIndex, float textureScaling)
{
	int strideInBytes = 9 * sizeof(float);
	int numVertices = sizeof(cube_vertices_textured) / strideInBytes;
	int numIndices = sizeof(cube_indices) / sizeof(int);

	b3AlignedObjectArray<GLInstanceVertex> verts;
	verts.resize(numVertices);
	for (int i = 0; i < numVertices; i++)
	{
		verts[i].xyzw[0] = halfExtentsX * cube_vertices_textured[i * 9];
		verts[i].xyzw[1] = halfExtentsY * cube_vertices_textured[i * 9 + 1];
		verts[i].xyzw[2] = halfExtentsZ * cube_vertices_textured[i * 9 + 2];
		verts[i].xyzw[3] = cube_vertices_textured[i * 9 + 3];
		verts[i].normal[0] = cube_vertices_textured[i * 9 + 4];
		verts[i].normal[1] = cube_vertices_textured[i * 9 + 5];
		verts[i].normal[2] = cube_vertices_textured[i * 9 + 6];
		verts[i].uv[0] = cube_vertices_textured[i * 9 + 7] * textureScaling;
		verts[i].uv[1] = cube_vertices_textured[i * 9 + 8] * textureScaling;
	}

	int shapeId = m_renderer->registerShape(&verts[0].xyzw[0], numVertices, cube_indices, numIndices, B3_GL_TRIANGLES, textureIndex);
	return shapeId;
}
