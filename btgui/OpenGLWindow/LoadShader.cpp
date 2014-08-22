#include "LoadShader.h"
#include "OpenGLInclude.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>


// Load the shader from the source text
void gltLoadShaderSrc(const char *szShaderSrc, GLuint shader)
{
	GLchar *fsStringPtr[1];

	fsStringPtr[0] = (GLchar *)szShaderSrc;
	glShaderSource(shader, 1, (const GLchar **)fsStringPtr, NULL);
}


GLuint gltLoadShaderPair(const char *szVertexProg, const char *szFragmentProg)
{

  assert(glGetError()==GL_NO_ERROR);

	// Temporary Shader objects
	GLuint hVertexShader;
	GLuint hFragmentShader;
	GLuint hReturn = 0;
	GLint testVal;

	// Create shader objects
	hVertexShader = glCreateShader(GL_VERTEX_SHADER);
	hFragmentShader = glCreateShader(GL_FRAGMENT_SHADER);

	gltLoadShaderSrc(szVertexProg, hVertexShader);
	gltLoadShaderSrc(szFragmentProg, hFragmentShader);

	// Compile them
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
		glDeleteShader(hFragmentShader);
		return (GLuint)NULL;
	}

  assert(glGetError()==GL_NO_ERROR);

    glCompileShader(hFragmentShader);
    assert(glGetError()==GL_NO_ERROR);

    glGetShaderiv(hFragmentShader, GL_COMPILE_STATUS, &testVal);
	if(testVal == GL_FALSE)
	{
        char temp[256] = "";
        glGetShaderInfoLog( hFragmentShader, 256, NULL, temp);
        fprintf( stderr, "Compile failed:\n%s\n", temp);
        assert(0);
        exit(0);
		glDeleteShader(hVertexShader);
		glDeleteShader(hFragmentShader);
		return (GLuint)NULL;
	}

    assert(glGetError()==GL_NO_ERROR);

	// Check for errors




	// Link them - assuming it works...
	hReturn = glCreateProgram();
	glAttachShader(hReturn, hVertexShader);
	glAttachShader(hReturn, hFragmentShader);

	glLinkProgram(hReturn);

	// These are no longer needed
	glDeleteShader(hVertexShader);
	glDeleteShader(hFragmentShader);

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


