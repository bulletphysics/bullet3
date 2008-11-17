
/**********************\
*                      *
*  Determine OS type   *
*                      *
\**********************/

#if defined(__CYGWIN__)
#define GPUP_WIN32     1
#define GPUP_CYGWIN    1    /* Windoze AND Cygwin. */
#elif defined(_WIN32) || defined(__WIN32__) || defined(_MSC_VER)
#define GPUP_WIN32     1
#define GPUP_MSVC      1    /* Windoze AND MSVC. */
#elif defined(__BEOS__)
#define GPUP_BEOS      1
#elif defined( macintosh )
#define GPUP_MACINTOSH 1
#elif defined(__APPLE__)
#define GPUP_MAC_OSX   1
#elif defined(__linux__)
#define GPUP_LINUX     1
#elif defined(__sgi)
#define GPUP_IRIX      1
#elif defined(_AIX)
#define GPUP_AIX       1
#elif defined(SOLARIS) || defined(sun)
#define GPUP_SOLARIS   1
#elif defined(hpux)
#define GPUP_HPUX      1
#elif (defined(__unix__) || defined(unix)) && !defined(USG)
#define GPUP_BSD       1
#endif
#if defined(BORLANDBUILDER)
#define GPUP_BB     1
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
  Add specialised includes/defines...
*/

#ifdef GPUP_WIN32
#include <windows.h>
#include <mmsystem.h>
#include <regstr.h>
#define  GPUP_WGL     1
#endif

#ifdef GPUP_CYGWIN
#include <unistd.h>
#define  GPUP_WGL     1
#endif

#ifdef GPUP_BEOS
#include <be/kernel/image.h>
#define  GPUP_GLX     1
#endif

#ifdef GPUP_MACINTOSH
#include <CodeFragments.h>
#include <unistd.h>
#define  GPUP_AGL     1
#endif

#ifdef GPUP_MAC_OSX
#include <unistd.h>
#define  GPUP_CGL     1
#endif

#if defined(GPUP_LINUX) || defined(GPUP_BSD) || defined(GPUP_IRIX) || defined(GPUP_SOLARIS) || defined(GPUP_AIX)
#include <unistd.h>
#include <dlfcn.h>
#include <fcntl.h>
#define  GPUP_GLX     1
#endif

#if defined(GPUP_BSD)
#include <sys/param.h>
#define  GPUP_GLX     1
#endif

#include <assert.h>
#include <limits.h>
#include <math.h>
#include <float.h>
#include <errno.h>


#if defined(GPUP_MAC_OSX) && !defined (VMDMESA)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#include <OpenGL/glext.h>
#else
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glut.h>


#endif

#ifdef DISABLE_GL_ERROR_CHECKS
inline void showGLerror ( const char * ) {}
#else
inline void showGLerror ( const char *msg )
{
  GLenum err ;

  while ( (err = glGetError()) != GL_NO_ERROR )
    fprintf ( stderr, "%s: OpenGL Error - %s\n", msg, gluErrorString ( err ) )  ;
}
#endif

