//  ---------------------------------------------------------------------------
//
//  @file       MiniGLUT.h
//  @brief      A subset of GLUT definitions needed to compile helper functions
//              implemented in TwEventGLUT.c
//
//  notes:    - Private header
//            - AntTweakBar.dll does not need to link with GLUT, 
//              it just needs some definitions for its helper functions.
//            - This header is provided to avoid the need of having GLUT
//              installed to recompile AntTweakBar.
//            - Do not use this header in your own programs, better use the
//              GLUT.h header from the actual GLUT library SDK :
//              http://opengl.org/resources/libraries/glut
//
//  ---------------------------------------------------------------------------

#if !defined MINI_GLUT_INCLUDED
#define MINI_GLUT_INCLUDED

#if defined(_WIN32) || defined(_WIN64)
#   define WIN32_LEAN_AND_MEAN
#   include <windows.h> // needed by gl.h
#   define GLUT_CALL     __stdcall
#   define GLUT_CALLBACK __cdecl
#   define GLUT_API      __declspec(dllimport)
#else
#   define GLUT_CALL
#   define GLUT_CALLBACK
#   define GLUT_API      extern
#endif

#if defined(_MACOSX)
#   include <OpenGL/gl.h>
#   include <OpenGL/glu.h>
#else
#   include <GL/gl.h>  // must be included after windows.h
#   include <GL/glu.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif


// Mouse buttons
#define GLUT_LEFT_BUTTON    0
#define GLUT_MIDDLE_BUTTON  1
#define GLUT_RIGHT_BUTTON   2

// Mouse button state
#define GLUT_DOWN           0
#define GLUT_UP             1

// glutGetModifiers return mask
#define GLUT_ACTIVE_SHIFT   1
#define GLUT_ACTIVE_CTRL    2
#define GLUT_ACTIVE_ALT     4

// function keys
#define GLUT_KEY_F1         1
#define GLUT_KEY_F2         2
#define GLUT_KEY_F3         3
#define GLUT_KEY_F4         4
#define GLUT_KEY_F5         5
#define GLUT_KEY_F6         6
#define GLUT_KEY_F7         7
#define GLUT_KEY_F8         8
#define GLUT_KEY_F9         9
#define GLUT_KEY_F10        10
#define GLUT_KEY_F11        11
#define GLUT_KEY_F12        12

// directional keys
#define GLUT_KEY_LEFT       100
#define GLUT_KEY_UP         101
#define GLUT_KEY_RIGHT      102
#define GLUT_KEY_DOWN       103
#define GLUT_KEY_PAGE_UP    104
#define GLUT_KEY_PAGE_DOWN  105
#define GLUT_KEY_HOME       106
#define GLUT_KEY_END        107
#define GLUT_KEY_INSERT     108

// display mode bit masks
#define GLUT_RGB            0
#define GLUT_RGBA           GLUT_RGB
#define GLUT_INDEX          1
#define GLUT_SINGLE         0
#define GLUT_DOUBLE         2
#define GLUT_ACCUM          4
#define GLUT_ALPHA          8
#define GLUT_DEPTH          16
#define GLUT_STENCIL        32

// timer
#define GLUT_ELAPSED_TIME   ((GLenum) 700)


// functions subset
GLUT_API void GLUT_CALL glutInit(int *argcp, char **argv);
GLUT_API void GLUT_CALL glutInitDisplayMode(unsigned int mode);
GLUT_API int  GLUT_CALL glutCreateWindow(const char *title);
GLUT_API int  GLUT_CALL glutGetWindow(void);
GLUT_API void GLUT_CALL glutSetWindow(int win);
GLUT_API int  GLUT_CALL glutCreateSubWindow(int win, int x, int y, int width, int height);
GLUT_API int  GLUT_CALL glutGet(GLenum type);
GLUT_API void GLUT_CALL glutSwapBuffers();
GLUT_API void GLUT_CALL glutPostRedisplay();
GLUT_API void GLUT_CALL glutInitWindowPosition(int x, int y);
GLUT_API void GLUT_CALL glutInitWindowSize(int width, int height);
GLUT_API void GLUT_CALL glutPositionWindow(int x, int y);
GLUT_API void GLUT_CALL glutReshapeWindow(int width, int height);
GLUT_API void GLUT_CALL glutMainLoop();
GLUT_API int  GLUT_CALL glutCreateMenu(void (GLUT_CALLBACK *func)(int));
GLUT_API void GLUT_CALL glutDisplayFunc(void (GLUT_CALLBACK *func)(void));
GLUT_API void GLUT_CALL glutReshapeFunc(void (GLUT_CALLBACK *func)(int width, int height));
GLUT_API void GLUT_CALL glutKeyboardFunc(void (GLUT_CALLBACK *func)(unsigned char key, int x, int y));
GLUT_API void GLUT_CALL glutMouseFunc(void (GLUT_CALLBACK *func)(int button, int state, int x, int y));
GLUT_API void GLUT_CALL glutMotionFunc(void (GLUT_CALLBACK *func)(int x, int y));
GLUT_API void GLUT_CALL glutPassiveMotionFunc(void (GLUT_CALLBACK *func)(int x, int y));
GLUT_API void GLUT_CALL glutSpecialFunc(void (GLUT_CALLBACK *func)(int key, int x, int y));
GLUT_API int  GLUT_CALL glutGetModifiers(void);
GLUT_API void GLUT_CALL glutSolidTorus(GLdouble innerRadius, GLdouble outerRadius, GLint sides, GLint rings);
GLUT_API void GLUT_CALL glutSolidCone(GLdouble base, GLdouble height, GLint slices, GLint stacks);
GLUT_API void GLUT_CALL glutSolidTeapot(GLdouble size);

// GLUT exit problem workaround (see glut.h)
#if (defined(_WIN32) || defined(_WIN64)) && !defined(GLUT_DISABLE_ATEXIT_HACK)
    extern void __cdecl exit(int);
    GLUT_API void GLUT_CALL __glutInitWithExit(int *argcp, char **argv, void (__cdecl *exitfunc)(int));
    static void GLUT_CALL glutInit_ATEXIT_HACK(int *argcp, char **argv) { __glutInitWithExit(argcp, argv, exit); }
    #define glutInit glutInit_ATEXIT_HACK
#endif


#ifdef __cplusplus
}
#endif

#endif // !defined MINI_GLUT_INCLUDED

