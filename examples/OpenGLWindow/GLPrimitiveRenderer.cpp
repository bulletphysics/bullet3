#ifndef NO_OPENGL3
#include "GLPrimitiveRenderer.h"
#include "GLPrimInternalData.h"
#include "Bullet3Common/b3Scalar.h"
#include "LoadShader.h"

static const char *vertexShader3D =
	"#version 150   \n"
	"\n"
	"uniform mat4 viewMatrix, projMatrix;\n"
	"in vec4 position;\n"
	"in vec4 colour;\n"
	"out vec4 colourV;\n"
	"\n"
	"in vec2 texuv;\n"
	"out vec2 texuvV;\n"
	"\n"
	"\n"
	"void main (void)\n"
	"{\n"
	"    colourV = colour;\n"
	"   gl_Position = projMatrix * viewMatrix * position ;\n"
	"	texuvV=texuv;\n"
	"}\n";

static const char *fragmentShader3D =
	"#version 150\n"
	"\n"
	"uniform vec2 p;\n"
	"in vec4 colourV;\n"
	"out vec4 fragColour;\n"
	"in vec2 texuvV;\n"
	"\n"
	"uniform sampler2D Diffuse;\n"
	"\n"
	"void main(void)\n"
	"{\n"
	"	vec4 texcolor = texture(Diffuse,texuvV);\n"
	"  if (p.x==0.f)\n"
	"  {\n"
	"		texcolor = vec4(1,1,1,texcolor.x);\n"
	"  }\n"
	"   fragColour = colourV*texcolor;\n"
	"}\n";

static unsigned int s_indexData[6] = {0, 1, 2, 0, 2, 3};
#define MAX_VERTICES2 8192
struct PrimInternalData2
{
	PrimInternalData2()
		: m_numVerticesText(0),
		  m_numVerticesRect(0)
	{
	}
	int m_numVerticesText;
	int m_numVerticesRect;
	PrimVertex m_verticesText[MAX_VERTICES2];
	PrimVertex m_verticesRect[MAX_VERTICES2];
};

GLPrimitiveRenderer::GLPrimitiveRenderer(int screenWidth, int screenHeight)
	: m_screenWidth(screenWidth),
	  m_screenHeight(screenHeight)
{
	m_data = new PrimInternalData;
	m_data2 = new PrimInternalData2;

	m_data->m_shaderProg = gltLoadShaderPair(vertexShader3D, fragmentShader3D);

	m_data->m_viewmatUniform = glGetUniformLocation(m_data->m_shaderProg, "viewMatrix");
	if (m_data->m_viewmatUniform < 0)
	{
		b3Assert(0);
	}
	m_data->m_projMatUniform = glGetUniformLocation(m_data->m_shaderProg, "projMatrix");
	if (m_data->m_projMatUniform < 0)
	{
		b3Assert(0);
	}
	m_data->m_positionUniform = glGetUniformLocation(m_data->m_shaderProg, "p");
	if (m_data->m_positionUniform < 0)
	{
		b3Assert(0);
	}
	m_data->m_colourAttribute = glGetAttribLocation(m_data->m_shaderProg, "colour");
	if (m_data->m_colourAttribute < 0)
	{
		b3Assert(0);
	}
	m_data->m_positionAttribute = glGetAttribLocation(m_data->m_shaderProg, "position");
	if (m_data->m_positionAttribute < 0)
	{
		b3Assert(0);
	}
	m_data->m_textureAttribute = glGetAttribLocation(m_data->m_shaderProg, "texuv");
	if (m_data->m_textureAttribute < 0)
	{
		b3Assert(0);
	}

	loadBufferData();
}

void GLPrimitiveRenderer::loadBufferData()
{
	PrimVertex vertexData[4] = {
		PrimVertex(PrimVec4(-1, -1, 0.0, 1.0), PrimVec4(1.0, 0.0, 0.0, 1.0), PrimVec2(0, 0)),
		PrimVertex(PrimVec4(-1, 1, 0.0, 1.0), PrimVec4(0.0, 1.0, 0.0, 1.0), PrimVec2(0, 1)),
		PrimVertex(PrimVec4(1, 1, 0.0, 1.0), PrimVec4(0.0, 0.0, 1.0, 1.0), PrimVec2(1, 1)),
		PrimVertex(PrimVec4(1, -1, 0.0, 1.0), PrimVec4(1.0, 1.0, 1.0, 1.0), PrimVec2(1, 0))};

	glGenVertexArrays(1, &m_data->m_vertexArrayObject);
	glBindVertexArray(m_data->m_vertexArrayObject);

	glGenBuffers(1, &m_data->m_vertexBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, 4 * sizeof(PrimVertex), vertexData, GL_DYNAMIC_DRAW);

	glGenVertexArrays(1, &m_data->m_vertexArrayObject2);
	glBindVertexArray(m_data->m_vertexArrayObject2);
	glGenBuffers(1, &m_data->m_vertexBuffer2);
	glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vertexBuffer2);
	glBufferData(GL_ARRAY_BUFFER, MAX_VERTICES2 * sizeof(PrimVertex), 0, GL_DYNAMIC_DRAW);

	b3Assert(glGetError() == GL_NO_ERROR);

	glGenBuffers(1, &m_data->m_indexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_data->m_indexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, 6 * sizeof(int), s_indexData, GL_STATIC_DRAW);

	unsigned int indexData[MAX_VERTICES2 * 2];
	int count = 0;
	for (int i = 0; i < MAX_VERTICES2; i += 4)
	{
		indexData[count++] = i;
		indexData[count++] = i + 1;
		indexData[count++] = i + 2;

		indexData[count++] = i;
		indexData[count++] = i + 2;
		indexData[count++] = i + 3;
	}
	glGenBuffers(1, &m_data->m_indexBuffer2);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_data->m_indexBuffer2);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, count * sizeof(int), indexData, GL_STATIC_DRAW);

	glEnableVertexAttribArray(m_data->m_positionAttribute);
	glEnableVertexAttribArray(m_data->m_colourAttribute);
	b3Assert(glGetError() == GL_NO_ERROR);

	glEnableVertexAttribArray(m_data->m_textureAttribute);

	glVertexAttribPointer(m_data->m_positionAttribute, 4, GL_FLOAT, GL_FALSE, sizeof(PrimVertex), (const GLvoid *)0);
	glVertexAttribPointer(m_data->m_colourAttribute, 4, GL_FLOAT, GL_FALSE, sizeof(PrimVertex), (const GLvoid *)sizeof(PrimVec4));
	glVertexAttribPointer(m_data->m_textureAttribute, 2, GL_FLOAT, GL_FALSE, sizeof(PrimVertex), (const GLvoid *)(sizeof(PrimVec4) + sizeof(PrimVec4)));
	b3Assert(glGetError() == GL_NO_ERROR);

	glActiveTexture(GL_TEXTURE0);

	GLubyte *image = new GLubyte[256 * 256 * 3];
	for (int y = 0; y < 256; ++y)
	{
		//   const int	t=y>>5;
		GLubyte *pi = image + y * 256 * 3;
		for (int x = 0; x < 256; ++x)
		{
			if (x < y)  //x<2||y<2||x>253||y>253)
			{
				pi[0] = 255;
				pi[1] = 0;
				pi[2] = 0;
			}
			else

			{
				pi[0] = 255;
				pi[1] = 255;
				pi[2] = 255;
			}

			pi += 3;
		}
	}

	glGenTextures(1, (GLuint *)&m_data->m_texturehandle);
	glBindTexture(GL_TEXTURE_2D, m_data->m_texturehandle);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 256, 256, 0, GL_RGB, GL_UNSIGNED_BYTE, image);
	glGenerateMipmap(GL_TEXTURE_2D);

	b3Assert(glGetError() == GL_NO_ERROR);

	delete[] image;
}

GLPrimitiveRenderer::~GLPrimitiveRenderer()
{
	glBindTexture(GL_TEXTURE_2D, 0);
	glUseProgram(0);
	glBindTexture(GL_TEXTURE_2D, 0);
	glDeleteProgram(m_data->m_shaderProg);
	delete m_data;
	delete m_data2;
}

void GLPrimitiveRenderer::drawLine()
{
}

void GLPrimitiveRenderer::drawRect(float x0, float y0, float x1, float y1, float color[4])
{
	b3Assert(glGetError() == GL_NO_ERROR);
	glActiveTexture(GL_TEXTURE0);
	b3Assert(glGetError() == GL_NO_ERROR);

	glBindTexture(GL_TEXTURE_2D, m_data->m_texturehandle);
	b3Assert(glGetError() == GL_NO_ERROR);
	drawTexturedRect(x0, y0, x1, y1, color, 0, 0, 1, 1);
	b3Assert(glGetError() == GL_NO_ERROR);
}

void GLPrimitiveRenderer::drawTexturedRect3D(const PrimVertex &v0, const PrimVertex &v1, const PrimVertex &v2, const PrimVertex &v3, float viewMat[16], float projMat[16], bool useRGBA)
{
	//B3_PROFILE("GLPrimitiveRenderer::drawTexturedRect3D");

	b3Assert(glGetError() == GL_NO_ERROR);

	glUseProgram(m_data->m_shaderProg);

	glUniformMatrix4fv(m_data->m_viewmatUniform, 1, false, viewMat);
	glUniformMatrix4fv(m_data->m_projMatUniform, 1, false, projMat);

	b3Assert(glGetError() == GL_NO_ERROR);

	glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vertexBuffer);
	glBindVertexArray(m_data->m_vertexArrayObject);

	bool useFiltering = false;
	if (useFiltering)
	{
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	}
	else
	{
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	}

	PrimVertex vertexData[4] = {
		v0, v1, v2, v3};

	glBufferSubData(GL_ARRAY_BUFFER, 0, 4 * sizeof(PrimVertex), vertexData);

	b3Assert(glGetError() == GL_NO_ERROR);

	PrimVec2 p(0.f, 0.f);  //?b?0.5f * sinf(timeValue), 0.5f * cosf(timeValue) );
	if (useRGBA)
	{
		p.p[0] = 1.f;
		p.p[1] = 1.f;
	}

	glUniform2fv(m_data->m_positionUniform, 1, (const GLfloat *)&p);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	b3Assert(glGetError() == GL_NO_ERROR);

	glEnableVertexAttribArray(m_data->m_positionAttribute);
	b3Assert(glGetError() == GL_NO_ERROR);

	glEnableVertexAttribArray(m_data->m_colourAttribute);
	b3Assert(glGetError() == GL_NO_ERROR);

	glEnableVertexAttribArray(m_data->m_textureAttribute);

	glVertexAttribPointer(m_data->m_positionAttribute, 4, GL_FLOAT, GL_FALSE, sizeof(PrimVertex), (const GLvoid *)0);
	glVertexAttribPointer(m_data->m_colourAttribute, 4, GL_FLOAT, GL_FALSE, sizeof(PrimVertex), (const GLvoid *)sizeof(PrimVec4));
	glVertexAttribPointer(m_data->m_textureAttribute, 2, GL_FLOAT, GL_FALSE, sizeof(PrimVertex), (const GLvoid *)(sizeof(PrimVec4) + sizeof(PrimVec4)));
	b3Assert(glGetError() == GL_NO_ERROR);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_data->m_indexBuffer);

	//glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
	int indexCount = 6;
	b3Assert(glGetError() == GL_NO_ERROR);

	glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
	b3Assert(glGetError() == GL_NO_ERROR);

	glBindVertexArray(0);
	b3Assert(glGetError() == GL_NO_ERROR);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	b3Assert(glGetError() == GL_NO_ERROR);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	b3Assert(glGetError() == GL_NO_ERROR);

	//glDisableVertexAttribArray(m_data->m_textureAttribute);
	b3Assert(glGetError() == GL_NO_ERROR);

	glUseProgram(0);

	b3Assert(glGetError() == GL_NO_ERROR);
}

void GLPrimitiveRenderer::drawTexturedRect3D2Text(bool useRGBA)
{
	drawTexturedRect3D2(&m_data2->m_verticesText[0], m_data2->m_numVerticesText, useRGBA);
	m_data2->m_numVerticesText = 0;
}

void GLPrimitiveRenderer::drawTexturedRect3D2(PrimVertex *vertices, int numVertices, bool useRGBA)
{
	//B3_PROFILE("drawTexturedRect3D2");
	if (numVertices == 0)
	{
		return;
	}
	//B3_PROFILE("GLPrimitiveRenderer::drawTexturedRect3D");

	b3Assert(glGetError() == GL_NO_ERROR);
	float identity[16] = {1, 0, 0, 0,
						  0, 1, 0, 0,
						  0, 0, 1, 0,
						  0, 0, 0, 1};

	glUseProgram(m_data->m_shaderProg);

	glUniformMatrix4fv(m_data->m_viewmatUniform, 1, false, identity);
	glUniformMatrix4fv(m_data->m_projMatUniform, 1, false, identity);

	b3Assert(glGetError() == GL_NO_ERROR);

	glBindBuffer(GL_ARRAY_BUFFER, m_data->m_vertexBuffer2);
	glBindVertexArray(m_data->m_vertexArrayObject2);

	bool useFiltering = false;
	if (useFiltering)
	{
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	}
	else
	{
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	}

	/*   PrimVertex vertexData[4] = {
		   v0,v1,v2,v3
		};
    */

	glBufferSubData(GL_ARRAY_BUFFER, 0, numVertices * sizeof(PrimVertex), vertices);

	b3Assert(glGetError() == GL_NO_ERROR);

	PrimVec2 p(0.f, 0.f);  //?b?0.5f * sinf(timeValue), 0.5f * cosf(timeValue) );
	if (useRGBA)
	{
		p.p[0] = 1.f;
		p.p[1] = 1.f;
	}

	glUniform2fv(m_data->m_positionUniform, 1, (const GLfloat *)&p);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	b3Assert(glGetError() == GL_NO_ERROR);

	glEnableVertexAttribArray(m_data->m_positionAttribute);
	b3Assert(glGetError() == GL_NO_ERROR);

	glEnableVertexAttribArray(m_data->m_colourAttribute);
	b3Assert(glGetError() == GL_NO_ERROR);

	glEnableVertexAttribArray(m_data->m_textureAttribute);

	glVertexAttribPointer(m_data->m_positionAttribute, 4, GL_FLOAT, GL_FALSE, sizeof(PrimVertex), (const GLvoid *)0);
	glVertexAttribPointer(m_data->m_colourAttribute, 4, GL_FLOAT, GL_FALSE, sizeof(PrimVertex), (const GLvoid *)sizeof(PrimVec4));
	glVertexAttribPointer(m_data->m_textureAttribute, 2, GL_FLOAT, GL_FALSE, sizeof(PrimVertex), (const GLvoid *)(sizeof(PrimVec4) + sizeof(PrimVec4)));
	b3Assert(glGetError() == GL_NO_ERROR);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_data->m_indexBuffer2);

	//glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
	int indexCount = (numVertices / 4) * 6;
	b3Assert(glGetError() == GL_NO_ERROR);

	glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
	b3Assert(glGetError() == GL_NO_ERROR);

	glBindVertexArray(0);
	b3Assert(glGetError() == GL_NO_ERROR);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	b3Assert(glGetError() == GL_NO_ERROR);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	b3Assert(glGetError() == GL_NO_ERROR);

	//glDisableVertexAttribArray(m_data->m_textureAttribute);
	b3Assert(glGetError() == GL_NO_ERROR);

	glUseProgram(0);

	b3Assert(glGetError() == GL_NO_ERROR);
}

void GLPrimitiveRenderer::drawTexturedRect2a(float x0, float y0, float x1, float y1, float color[4], float u0, float v0, float u1, float v1, int useRGBA)
{
	PrimVertex vertexData[4] = {
		PrimVertex(PrimVec4(-1.f + 2.f * x0 / float(m_screenWidth), 1.f - 2.f * y0 / float(m_screenHeight), 0.f, 1.f), PrimVec4(color[0], color[1], color[2], color[3]), PrimVec2(u0, v0)),
		PrimVertex(PrimVec4(-1.f + 2.f * x0 / float(m_screenWidth), 1.f - 2.f * y1 / float(m_screenHeight), 0.f, 1.f), PrimVec4(color[0], color[1], color[2], color[3]), PrimVec2(u0, v1)),
		PrimVertex(PrimVec4(-1.f + 2.f * x1 / float(m_screenWidth), 1.f - 2.f * y1 / float(m_screenHeight), 0.f, 1.f), PrimVec4(color[0], color[1], color[2], color[3]), PrimVec2(u1, v1)),
		PrimVertex(PrimVec4(-1.f + 2.f * x1 / float(m_screenWidth), 1.f - 2.f * y0 / float(m_screenHeight), 0.f, 1.f), PrimVec4(color[0], color[1], color[2], color[3]), PrimVec2(u1, v0))};

	//	int sz = m_data2->m_numVerticesText;

	m_data2->m_verticesRect[m_data2->m_numVerticesRect++] = vertexData[0];
	m_data2->m_verticesRect[m_data2->m_numVerticesRect++] = vertexData[1];
	m_data2->m_verticesRect[m_data2->m_numVerticesRect++] = vertexData[2];
	m_data2->m_verticesRect[m_data2->m_numVerticesRect++] = vertexData[3];

	if (m_data2->m_numVerticesRect >= MAX_VERTICES2)
	{
		flushBatchedRects();
	}
}

void GLPrimitiveRenderer::flushBatchedRects()
{
	if (m_data2->m_numVerticesRect == 0)
		return;

	glActiveTexture(GL_TEXTURE0);
	b3Assert(glGetError() == GL_NO_ERROR);
	glBindTexture(GL_TEXTURE_2D, m_data->m_texturehandle);
	drawTexturedRect3D2(m_data2->m_verticesRect, m_data2->m_numVerticesRect, 0);
	m_data2->m_numVerticesRect = 0;
}
void GLPrimitiveRenderer::drawTexturedRect2(float x0, float y0, float x1, float y1, float color[4], float u0, float v0, float u1, float v1, int useRGBA)
{
	PrimVertex vertexData[4] = {
		PrimVertex(PrimVec4(-1.f + 2.f * x0 / float(m_screenWidth), 1.f - 2.f * y0 / float(m_screenHeight), 0.f, 1.f), PrimVec4(color[0], color[1], color[2], color[3]), PrimVec2(u0, v0)),
		PrimVertex(PrimVec4(-1.f + 2.f * x0 / float(m_screenWidth), 1.f - 2.f * y1 / float(m_screenHeight), 0.f, 1.f), PrimVec4(color[0], color[1], color[2], color[3]), PrimVec2(u0, v1)),
		PrimVertex(PrimVec4(-1.f + 2.f * x1 / float(m_screenWidth), 1.f - 2.f * y1 / float(m_screenHeight), 0.f, 1.f), PrimVec4(color[0], color[1], color[2], color[3]), PrimVec2(u1, v1)),
		PrimVertex(PrimVec4(-1.f + 2.f * x1 / float(m_screenWidth), 1.f - 2.f * y0 / float(m_screenHeight), 0.f, 1.f), PrimVec4(color[0], color[1], color[2], color[3]), PrimVec2(u1, v0))};

	//	int sz = m_data2->m_numVerticesText;

	m_data2->m_verticesText[m_data2->m_numVerticesText++] = vertexData[0];
	m_data2->m_verticesText[m_data2->m_numVerticesText++] = vertexData[1];
	m_data2->m_verticesText[m_data2->m_numVerticesText++] = vertexData[2];
	m_data2->m_verticesText[m_data2->m_numVerticesText++] = vertexData[3];

	if (m_data2->m_numVerticesText >= MAX_VERTICES2)
	{
		drawTexturedRect3D2(m_data2->m_verticesText, m_data2->m_numVerticesText, useRGBA);
		m_data2->m_numVerticesText = 0;
	}
}

void GLPrimitiveRenderer::drawTexturedRect(float x0, float y0, float x1, float y1, float color[4], float u0, float v0, float u1, float v1, int useRGBA)
{
	float identity[16] = {1, 0, 0, 0,
						  0, 1, 0, 0,
						  0, 0, 1, 0,
						  0, 0, 0, 1};
	PrimVertex vertexData[4] = {
		PrimVertex(PrimVec4(-1.f + 2.f * x0 / float(m_screenWidth), 1.f - 2.f * y0 / float(m_screenHeight), 0.f, 1.f), PrimVec4(color[0], color[1], color[2], color[3]), PrimVec2(u0, v0)),
		PrimVertex(PrimVec4(-1.f + 2.f * x0 / float(m_screenWidth), 1.f - 2.f * y1 / float(m_screenHeight), 0.f, 1.f), PrimVec4(color[0], color[1], color[2], color[3]), PrimVec2(u0, v1)),
		PrimVertex(PrimVec4(-1.f + 2.f * x1 / float(m_screenWidth), 1.f - 2.f * y1 / float(m_screenHeight), 0.f, 1.f), PrimVec4(color[0], color[1], color[2], color[3]), PrimVec2(u1, v1)),
		PrimVertex(PrimVec4(-1.f + 2.f * x1 / float(m_screenWidth), 1.f - 2.f * y0 / float(m_screenHeight), 0.f, 1.f), PrimVec4(color[0], color[1], color[2], color[3]), PrimVec2(u1, v0))};

	drawTexturedRect3D(vertexData[0], vertexData[1], vertexData[2], vertexData[3], identity, identity, useRGBA);
}

void GLPrimitiveRenderer::setScreenSize(int width, int height)
{
	m_screenWidth = width;
	m_screenHeight = height;
}
#endif
