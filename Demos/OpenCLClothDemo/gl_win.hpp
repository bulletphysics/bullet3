#ifndef __GL_WIN_HDR__
#define __GL_WIN_HDR__

#ifdef _WIN32//for glut.h
#include <windows.h>
#endif

//think different
#if defined(__APPLE__) && !defined (VMDMESA)
#include <OpenGL/OpenGL.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else


#ifdef _WINDOWS
#include <windows.h>
#include <GL/gl.h>
#include <GL/glu.h>
#else
#include <GL/glut.h>
#endif //_WINDOWS
#endif //APPLE


#include <string>

void goGL(void);
void preInitGL(int argc, char ** argv);

int getVBO( std::string, int size );

#endif //__GL_WIN_HDR__
