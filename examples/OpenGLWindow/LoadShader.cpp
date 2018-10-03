#include "LoadShader.h"
#include "OpenGLInclude.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>


// Load the shader from the header / source text. header can be empty/null
void gltLoadShaderSrc(const char *szShaderHdr, const char *szShaderSrc, GLuint shader)
{
	GLchar *fsStringPtr[2];
	fsStringPtr[0] = szShaderHdr ? (GLchar *)szShaderHdr : (GLchar *)szShaderSrc;
	fsStringPtr[1] = (GLchar *)szShaderSrc;
	glShaderSource(shader, szShaderHdr ? 2 : 1, (const GLchar **)fsStringPtr, NULL);
}


// Load the shader, any of the parameters can be empty / null
GLuint gltLoadShaderTriple(const char *szHeader, const char *szVertexProg, const char *szGeometryProg, const char *szFragmentProg)
{

  assert(glGetError()==GL_NO_ERROR);

	// Temporary Shader objects
	GLuint hVertexShader = 0;
	GLuint hGeometryShader = 0;
	GLuint hFragmentShader = 0;
	GLuint hReturn = 0;
	GLint testVal;

	if(szVertexProg)
	{
		// Create shader object
		hVertexShader = glCreateShader(GL_VERTEX_SHADER);
		gltLoadShaderSrc(szHeader, szVertexProg, hVertexShader);

		// Compile it
		glCompileShader(hVertexShader);
		assert(glGetError()==GL_NO_ERROR);

		glGetShaderiv(hVertexShader, GL_COMPILE_STATUS, &testVal);
		if(testVal == GL_FALSE)
		{
			char temp[256] = "";
			glGetShaderInfoLog( hVertexShader, 256, NULL, temp);
			fprintf( stderr, "Compile failed:\n%s\n", temp);
			assert(0);
			return 0;
			glDeleteShader(hVertexShader);
			return (GLuint)NULL;
		}

		assert(glGetError()==GL_NO_ERROR);
	}

	if(szGeometryProg)
	{
		hGeometryShader = glCreateShader(GL_GEOMETRY_SHADER);
		gltLoadShaderSrc(szHeader, szGeometryProg, hGeometryShader);

		glCompileShader(hGeometryShader);
		assert(glGetError()==GL_NO_ERROR);

		glGetShaderiv(hGeometryShader, GL_COMPILE_STATUS, &testVal);
		if(testVal == GL_FALSE)
		{
			char temp[256] = "";
			glGetShaderInfoLog( hGeometryShader, 256, NULL, temp);
			fprintf( stderr, "Compile failed:\n%s\n", temp);
			assert(0);
			exit(EXIT_FAILURE);
			glDeleteShader(hVertexShader);
			glDeleteShader(hGeometryShader);
			return (GLuint)NULL;
		}

		assert(glGetError()==GL_NO_ERROR);
	}

	if(szFragmentProg)
	{
		hFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
		gltLoadShaderSrc(szHeader, szFragmentProg, hFragmentShader);

		glCompileShader(hFragmentShader);
		assert(glGetError()==GL_NO_ERROR);

		glGetShaderiv(hFragmentShader, GL_COMPILE_STATUS, &testVal);
		if(testVal == GL_FALSE)
		{
			char temp[256] = "";
			glGetShaderInfoLog( hFragmentShader, 256, NULL, temp);
			fprintf( stderr, "Compile failed:\n%s\n", temp);
			assert(0);
			exit(EXIT_FAILURE);
			glDeleteShader(hVertexShader);
			if(szGeometryProg) glDeleteShader(hGeometryShader);
			glDeleteShader(hFragmentShader);
			return (GLuint)NULL;
		}
	}

	// Check for errors


	// Link them - assuming it works...
	hReturn = glCreateProgram();
	if(szVertexProg) glAttachShader(hReturn, hVertexShader);
	if(szGeometryProg) glAttachShader(hReturn, hGeometryShader);
	if(szFragmentProg) glAttachShader(hReturn, hFragmentShader);

	glLinkProgram(hReturn);

	// These are no longer needed
	if(szVertexProg) glDeleteShader(hVertexShader);
	if(szGeometryProg) glDeleteShader(hGeometryShader);
	if(szFragmentProg) glDeleteShader(hFragmentShader);

	// Make sure link worked too
	glGetProgramiv(hReturn, GL_LINK_STATUS, &testVal);
	if(testVal == GL_FALSE)
	{
		GLsizei maxLen = 4096;
		GLchar infoLog[4096];
		GLsizei actualLen;

		glGetProgramInfoLog(	hReturn,
								maxLen,
								 &actualLen,
								 infoLog);

		printf("Warning/Error in GLSL shader:\n");
		printf("%s\n",infoLog);
		glDeleteProgram(hReturn);
		return (GLuint)NULL;
	}

	return hReturn;
}


GLuint gltLoadShaderPair(const char *szVertexProg, const char *szFragmentProg)
{
	gltLoadShaderTriple(0, szVertexProg, 0, szFragmentProg);
}


