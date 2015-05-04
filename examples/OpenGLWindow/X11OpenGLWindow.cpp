#include "X11OpenGLWindow.h"
#include "OpenGLInclude.h"

#include<stdio.h>
#include<stdlib.h>
#ifdef GLEW_STATIC
#include "CustomGL/glew.h"
#else
#include <GL/glew.h>
#endif//GLEW_STATIC

#ifdef GLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS
#include "CustomGL/glxew.h"
#else
#include<GL/glx.h>
#endif // GLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS
#include <assert.h>

//#define DYNAMIC_LOAD_X11_FUNCTIONS
#ifdef DYNAMIC_LOAD_X11_FUNCTIONS
#include <dlfcn.h>
#endif //DYNAMIC_LOAD_X11_FUNCTIONS

//#include<X11/X.h>
//#include<X11/Xlib.h>
//#include<GL/gl.h>

//defined in GL/glxew.h
//#include<GL/glu.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <pthread.h>

GLint                   att[] = { GLX_RGBA,
GLX_DEPTH_SIZE, 24,
GLX_RED_SIZE        , 8,
GLX_GREEN_SIZE      , 8,
GLX_BLUE_SIZE       , 8,
GLX_ALPHA_SIZE      , 8,
GLX_STENCIL_SIZE    , 8,
GLX_DOUBLEBUFFER,
None };
/*
 static int att[] =
            {
                GLX_RGBA, GLX_DEPTH_SIZE, 24, GLX_DOUBLEBUFFER, None

              GLX_X_RENDERABLE    , True,
              GLX_DRAWABLE_TYPE   , GLX_WINDOW_BIT,
              GLX_RENDER_TYPE     , GLX_RGBA_BIT,
              GLX_X_VISUAL_TYPE   , GLX_TRUE_COLOR,
              GLX_RED_SIZE        , 8,
              GLX_GREEN_SIZE      , 8,
              GLX_BLUE_SIZE       , 8,
              GLX_ALPHA_SIZE      , 8,
              GLX_DEPTH_SIZE      , 24,
              GLX_STENCIL_SIZE    , 8,
              GLX_DOUBLEBUFFER    , True,
              None
            };
*/
static bool forceOpenGL3 = true;



#ifdef DYNAMIC_LOAD_X11_FUNCTIONS

///our X11 function typedefs

typedef int (*PFNXFREE)(void*);
typedef XErrorHandler (* PFNXSETERRORHANDLER) (XErrorHandler);
typedef int (* PFNXSYNC) (Display* a,Bool b);
typedef Display* (* PFNXOPENDISPLAY) (_Xconst char* a);
typedef Colormap (*PFNXCREATECOLORMAP) (Display* a,Window b,Visual* c,int d);
typedef Window (*PFNXCREATEWINDOW) (Display* a,Window b,int c,int d,unsigned int e,unsigned int f,unsigned int g,int h,unsigned int i,Visual* j,unsigned long k,XSetWindowAttributes* l);
typedef int (*PFNXMAPWINDOW) (Display*, Window);
typedef int (*PFNXSTORENAME) (Display* a,Window b,_Xconst char* c);
typedef int (*PFNXCLOSEDISPLAY) (Display* a);
typedef int (*PFNXDESTROYWINDOW) (Display* a,Window b);
typedef int (*PFNXRAISEWINDOW) (Display* a, Window b);

#if NeedWidePrototypes
	typedef KeySym* (*PFNXGETKEYBOARDMAPPING) (Display*,unsigned int,int,int*);
	typedef KeySym (*PFNXKEYCODETOKEYSYM) (Display* a,unsigned int b,int c);
#else
	typedef KeySym* (*PFNXGETKEYBOARDMAPPING) (Display*,KeyCode,int,int*);
	typedef KeySym (*PFNXKEYCODETOKEYSYM) (Display* a,KeyCode b,int c);
#endif
typedef void	(*PFNXCONVERTCASE) (KeySym /* sym */,KeySym *		/* lower */,KeySym * /* upper */);
typedef int (*PFNXPENDING) (Display* a);
typedef int (*PFNXNEXTEVENT) (Display* a,XEvent* b);
typedef int (*PFNXEVENTSQUEUED) (Display* a,int b);
typedef int (*PFNXPEEKEVENT) (Display* a,XEvent* b);
typedef KeySym (*PFNXLOOKUPKEYSYM) (XKeyEvent* a,int b);
typedef Status (*PFNXGETWINDOWATTRIBUTES) (Display* a,Window b,XWindowAttributes* c);

#define X11_LIBRARY "libX11.so.6"

#define MyXSync m_data->m_x11_XSync
#define MyXGetKeyboardMapping m_data->m_x11_XGetKeyboardMapping
#define MyXSetErrorHandler m_data->m_x11_XSetErrorHandler
#define MyXOpenDisplay m_data->m_x11_XOpenDisplay
#define MyXCreateColormap m_data->m_x11_XCreateColormap
#define MyXCreateWindow m_data->m_x11_XCreateWindow
#define MyXMapWindow m_data->m_x11_XMapWindow
#define MyXStoreName m_data->m_x11_XStoreName
#define MyXDestroyWindow m_data->m_x11_XDestroyWindow
#define MyXRaiseWindow m_data->m_x11_XRaiseWindow
#define MyXCloseDisplay m_data->m_x11_XCloseDisplay
#define MyXKeycodeToKeysym m_data->m_x11_XKeycodeToKeysym
#define MyXConvertCase m_data->m_x11_XConvertCase
#define MyXPending m_data->m_x11_XPending
#define MyXNextEvent m_data->m_x11_XNextEvent
#define MyXEventsQueued m_data->m_x11_XEventsQueued
#define MyXPeekEvent m_data->m_x11_XPeekEvent
#define MyXNextEvent m_data->m_x11_XNextEvent
#define MyXGetWindowAttributes m_data->m_x11_XGetWindowAttributes
#define MyXStoreName m_data->m_x11_XStoreName
#define MyXFree m_data->m_x11_XFree
#define MyXMapWindow m_data->m_x11_XMapWindow
#define MyXStoreName m_data->m_x11_XStoreName
#define MyXLookupKeysym m_data->m_x11_XLookupKeysym

#else
#define MyXSync XSync
#define MyXGetKeyboardMapping XGetKeyboardMapping
#define MyXSetErrorHandler XSetErrorHandler
#define MyXOpenDisplay XOpenDisplay
#define MyXCreateColormap XCreateColormap
#define MyXCreateWindow XCreateWindow
#define MyXMapWindow XMapWindow
#define MyXStoreName XStoreName
#define MyXDestroyWindow XDestroyWindow
#define MyXRaiseWindow XRaiseWindow
#define MyXCloseDisplay XCloseDisplay
#define MyXKeycodeToKeysym XKeycodeToKeysym
#define MyXConvertCase XConvertCase
#define MyXPending XPending
#define MyXNextEvent XNextEvent
#define MyXEventsQueued XEventsQueued
#define MyXPeekEvent XPeekEvent
#define MyXNextEvent XNextEvent
#define MyXGetWindowAttributes XGetWindowAttributes
#define MyXStoreName XStoreName
#define MyXFree XFree
#define MyXMapWindow XMapWindow
#define MyXStoreName XStoreName
#define MyXLookupKeysym XLookupKeysym

#endif//DYNAMIC_LOAD_X11_FUNCTIONS

enum
{
	MY_X11_ALT_KEY = 1,
	MY_X11_SHIFT_KEY = 2,
	MY_X11_CONTROL_KEY = 4
};

struct InternalData2
{
    Display*                m_dpy;
    Window                  m_root;
    XVisualInfo*            m_vi;
    Colormap                m_cmap;
    XSetWindowAttributes    m_swa;
    Window                  m_win;
    GLXContext              m_glc;
    XWindowAttributes       m_gwa;
    XEvent                  m_xev;
    GLXFBConfig             m_bestFbc;
    int			    m_modifierFlags;

#ifdef DYNAMIC_LOAD_X11_FUNCTIONS
	//dynamically load stuff
	void*									m_x11_library;
	PFNXFREE							m_x11_XFree;
	PFNXSETERRORHANDLER		m_x11_XSetErrorHandler;
	PFNXSYNC							m_x11_XSync;
	PFNXOPENDISPLAY				m_x11_XOpenDisplay;
	PFNXCREATECOLORMAP		m_x11_XCreateColormap;
	PFNXCREATEWINDOW			m_x11_XCreateWindow;
	PFNXMAPWINDOW					m_x11_XMapWindow;
	PFNXSTORENAME					m_x11_XStoreName;
	PFNXCLOSEDISPLAY			m_x11_XCloseDisplay;
	PFNXDESTROYWINDOW			m_x11_XDestroyWindow;
	PFNXRAISEWINDOW				m_x11_XRaiseWindow;
	PFNXKEYCODETOKEYSYM		m_x11_XKeycodeToKeysym;
	PFNXGETKEYBOARDMAPPING m_x11_XGetKeyboardMapping;
	PFNXCONVERTCASE				m_x11_XConvertCase;
	PFNXPENDING						m_x11_XPending;
	PFNXNEXTEVENT					m_x11_XNextEvent;
	PFNXEVENTSQUEUED			m_x11_XEventsQueued;
	PFNXPEEKEVENT					m_x11_XPeekEvent;
	PFNXLOOKUPKEYSYM			m_x11_XLookupKeysym;
	PFNXGETWINDOWATTRIBUTES m_x11_XGetWindowAttributes;
#endif //DYNAMIC_LOAD_X11_FUNCTIONS

	b3WheelCallback m_wheelCallback;
	b3MouseMoveCallback	m_mouseMoveCallback;
	b3MouseButtonCallback	m_mouseButtonCallback;
	b3ResizeCallback		m_resizeCallback;
	b3KeyboardCallback	m_keyboardCallback;

	InternalData2()
	:m_dpy(0),
	m_vi(0),
	m_wheelCallback(0),
	m_mouseMoveCallback(0),
	m_mouseButtonCallback(0),
	m_resizeCallback(0),
	m_keyboardCallback(0)
	{
#ifdef DYNAMIC_LOAD_X11_FUNCTIONS
		 m_x11_library = dlopen(X11_LIBRARY, RTLD_LOCAL | RTLD_NOW);
		 if (!m_x11_library)
		{
			printf("Error opening X11 library %s\n", X11_LIBRARY);
			exit(0);
		}

		bool missingFunc = false;

		missingFunc = ((m_x11_XFree = (PFNXFREE)  dlsym(m_x11_library, "XFree"))==NULL) | missingFunc;
		assert(!missingFunc);
		if (missingFunc)		{ printf("Error: missing func XFree in %s, exiting!\n", X11_LIBRARY);	exit(0);}
		missingFunc = ((m_x11_XSetErrorHandler = (PFNXSETERRORHANDLER) dlsym(m_x11_library,"XSetErrorHandler"))==NULL) | missingFunc;
		if (missingFunc)		{ printf("Error: missing func XSetErrorHandler in %s, exiting!\n", X11_LIBRARY);	exit(0);}
		missingFunc = ((m_x11_XSetErrorHandler = (PFNXSETERRORHANDLER) dlsym(m_x11_library,"XSetErrorHandler"))==NULL) | missingFunc;
		if (missingFunc)		{ printf("Error: missing func XSetErrorHandler in %s, exiting!\n", X11_LIBRARY);	exit(0);}
		missingFunc = ((m_x11_XSync = (PFNXSYNC) dlsym(m_x11_library,"XSync"))==NULL) | missingFunc;
		if (missingFunc)		{ printf("Error: missing func XSync in %s, exiting!\n", X11_LIBRARY);	exit(0);}
		missingFunc = ((m_x11_XOpenDisplay = (PFNXOPENDISPLAY) dlsym(m_x11_library,"XOpenDisplay"))==NULL) | missingFunc;
		if (missingFunc)		{ printf("Error: missing func XOpenDisplay in %s, exiting!\n", X11_LIBRARY);	exit(0);}
		missingFunc = ((m_x11_XCreateColormap = (PFNXCREATECOLORMAP) dlsym(m_x11_library,"XCreateColormap"))==NULL) | missingFunc;
		if (missingFunc)		{ printf("Error: missing func XCreateColormap in %s, exiting!\n", X11_LIBRARY);	exit(0);}
		missingFunc = ((m_x11_XCreateWindow = (PFNXCREATEWINDOW) dlsym(m_x11_library,"XCreateWindow"))==NULL) | missingFunc;
		if (missingFunc)		{ printf("Error: missing func XCreateWindow in %s, exiting!\n", X11_LIBRARY);	exit(0);}
		missingFunc = ((m_x11_XMapWindow = (PFNXMAPWINDOW) dlsym(m_x11_library,"XMapWindow"))==NULL) | missingFunc;
		if (missingFunc)		{ printf("Error: missing func XMapWindow in %s, exiting!\n", X11_LIBRARY);	exit(0);}
		missingFunc = ((m_x11_XStoreName = (PFNXSTORENAME) dlsym(m_x11_library,"XStoreName"))==NULL) | missingFunc;
		if (missingFunc)		{ printf("Error: missing func XStoreName in %s, exiting!\n", X11_LIBRARY);	exit(0);}
		missingFunc = ((m_x11_XCloseDisplay = (PFNXCLOSEDISPLAY) dlsym(m_x11_library,"XCloseDisplay"))==NULL) | missingFunc;
		if (missingFunc)		{ printf("Error: missing func XCloseDisplay in %s, exiting!\n", X11_LIBRARY);	exit(0);}
		missingFunc = ((m_x11_XDestroyWindow = (PFNXDESTROYWINDOW) dlsym(m_x11_library,"XDestroyWindow"))==NULL) | missingFunc;
		if (missingFunc)		{ printf("Error: missing func XDestroyWindow in %s, exiting!\n", X11_LIBRARY);	exit(0);}
		missingFunc = ((m_x11_XRaiseWindow = (PFNXRAISEWINDOW) dlsym(m_x11_library,"XRaiseWindow"))==NULL) | missingFunc;
		if (missingFunc)		{ printf("Error: missing func XRaiseWindow in %s, exiting!\n", X11_LIBRARY);	exit(0);}

		missingFunc = ((m_x11_XGetKeyboardMapping = (PFNXGETKEYBOARDMAPPING) dlsym(m_x11_library,"XGetKeyboardMapping"))==NULL) | missingFunc;
		if (missingFunc)		{ printf("Error: missing func XGetKeyboardMapping in %s, exiting!\n", X11_LIBRARY);	exit(0);}
		missingFunc = ((m_x11_XKeycodeToKeysym = (PFNXKEYCODETOKEYSYM) dlsym(m_x11_library,"XKeycodeToKeysym"))==NULL) | missingFunc;
		if (missingFunc)		{ printf("Error: missing func XKeycodeToKeysym in %s, exiting!\n", X11_LIBRARY);	exit(0);}
		missingFunc = ((m_x11_XConvertCase = (PFNXCONVERTCASE) dlsym(m_x11_library,"XConvertCase"))==NULL) | missingFunc;
		if (missingFunc)		{ printf("Error: missing func XConvertCase in %s, exiting!\n", X11_LIBRARY);	exit(0);}
		missingFunc = ((m_x11_XPending = (PFNXPENDING) dlsym(m_x11_library,"XPending"))==NULL) | missingFunc;
		if (missingFunc)		{ printf("Error: missing func XPending in %s, exiting!\n", X11_LIBRARY);	exit(0);}
		missingFunc = ((m_x11_XNextEvent = (PFNXNEXTEVENT) dlsym(m_x11_library,"XNextEvent"))==NULL) | missingFunc;
		if (missingFunc)		{ printf("Error: missing func XNextEvent in %s, exiting!\n", X11_LIBRARY);	exit(0);}
		missingFunc = ((m_x11_XEventsQueued = (PFNXEVENTSQUEUED) dlsym(m_x11_library,"XEventsQueued"))==NULL) | missingFunc;
		if (missingFunc)		{ printf("Error: missing func XEventsQueued in %s, exiting!\n", X11_LIBRARY);	exit(0);}
		missingFunc = ((m_x11_XPeekEvent = (PFNXPEEKEVENT) dlsym(m_x11_library,"XPeekEvent"))==NULL) | missingFunc;
		if (missingFunc)		{ printf("Error: missing func XPeekEvent in %s, exiting!\n", X11_LIBRARY);	exit(0);}
		missingFunc = ((m_x11_XLookupKeysym = (PFNXLOOKUPKEYSYM) dlsym(m_x11_library,"XLookupKeysym"))==NULL) | missingFunc;
		if (missingFunc)		{ printf("Error: missing func XLookupKeysym in %s, exiting!\n", X11_LIBRARY);	exit(0);}
		missingFunc = ((m_x11_XGetWindowAttributes = (PFNXGETWINDOWATTRIBUTES) dlsym(m_x11_library,"XGetWindowAttributes"))==NULL) | missingFunc;
		if (missingFunc)		{ printf("Error: missing func XGetWindowAttributes in %s, exiting!\n", X11_LIBRARY);	exit(0);}

		if (missingFunc)
		{
			printf("Error: a missing func in %s, exiting!\n", X11_LIBRARY);
			exit(0);
		} else
		{
			printf("X11 functions dynamically loaded using dlopen/dlsym OK!\n");
		}
#endif //DYNAMIC_LOAD_X11_FUNCTIONS
	}
};

#define GLX_CONTEXT_MAJOR_VERSION_ARB       0x2091
#define GLX_CONTEXT_MINOR_VERSION_ARB       0x2092
typedef GLXContext (*glXCreateContextAttribsARBProc)(Display*, GLXFBConfig, GLXContext, Bool, const int*);

// Helper to check for extension string presence.  Adapted from:
//   http://www.opengl.org/resources/features/OGLextensions/
static bool isExtensionSupported(const char *extList, const char *extension)
{
  const char *start;
  const char *where, *terminator;

  /* Extension names should not have spaces. */
  where = strchr(extension, ' ');
  if (where || *extension == '\0')
    return false;

  /* It takes a bit of care to be fool-proof about parsing the
     OpenGL extensions string. Don't be fooled by sub-strings,
     etc. */
  for (start=extList;;) {
    where = strstr(start, extension);

    if (!where)
      break;

    terminator = where + strlen(extension);

    if ( where == start || *(where - 1) == ' ' )
      if ( *terminator == ' ' || *terminator == '\0' )
        return true;

    start = terminator;
  }

  return false;
}

static bool ctxErrorOccurred = false;
static int ctxErrorHandler( Display *dpy, XErrorEvent *ev )
{
    ctxErrorOccurred = true;
    return 0;
}




X11OpenGLWindow::X11OpenGLWindow()
:m_OpenGLInitialized(false),
m_requestedExit(false)
{
    m_data = new InternalData2;
}

X11OpenGLWindow::~X11OpenGLWindow()
{
    if (m_OpenGLInitialized)
    {
        disableOpenGL();
    }

    delete m_data;
}



void X11OpenGLWindow::enableOpenGL()
{


    if (forceOpenGL3)
    {
 // Get the default screen's GLX extension list
  const char *glxExts = glXQueryExtensionsString( m_data->m_dpy,
                                                  DefaultScreen( m_data->m_dpy ) );

  // NOTE: It is not necessary to create or make current to a context before
  // calling glXGetProcAddressARB, unless we dynamically load OpenGL/GLX/X11

  glXCreateContextAttribsARBProc glXCreateContextAttribsARB = 0;
  glXCreateContextAttribsARB = (glXCreateContextAttribsARBProc)
           glXGetProcAddressARB( (const GLubyte *) "glXCreateContextAttribsARB" );

  GLXContext ctx = 0;

  // Install an X error handler so the application won't exit if GL 3.0
  // context allocation fails.
  //
  // Note this error handler is global.  All display connections in all threads
  // of a process use the same error handler, so be sure to guard against other
  // threads issuing X commands while this code is running.
  ctxErrorOccurred = false;
  int (*oldHandler)(Display*, XErrorEvent*) =
         MyXSetErrorHandler(&ctxErrorHandler);

  // Check for the GLX_ARB_create_context extension string and the function.
  // If either is not present, use GLX 1.3 context creation method.
  if ( !isExtensionSupported( glxExts, "GLX_ARB_create_context" ) ||
       !glXCreateContextAttribsARB )
  {
    printf( "glXCreateContextAttribsARB() not found"
            " ... using old-style GLX context\n" );
    ctx = glXCreateNewContext( m_data->m_dpy, m_data->m_bestFbc, GLX_RGBA_TYPE, 0, True );
  }

  // If it does, try to get a GL 3.0 context!
  else
  {
	 int context_attribs[] = {
          GLX_CONTEXT_MAJOR_VERSION_ARB ,3,
          GLX_CONTEXT_MINOR_VERSION_ARB, 2,
          GLX_CONTEXT_FLAGS_ARB, GLX_CONTEXT_DEBUG_BIT_ARB,
          GLX_CONTEXT_PROFILE_MASK_ARB, GLX_CONTEXT_CORE_PROFILE_BIT_ARB,None
     };
/*
    int context_attribs[] =
      {
        GLX_CONTEXT_MAJOR_VERSION_ARB, 3,
        GLX_CONTEXT_MINOR_VERSION_ARB, 2,

        //GLX_CONTEXT_FLAGS_ARB        , GLX_CONTEXT_FORWARD_COMPATIBLE_BIT_ARB,
        None
      };
*/
    printf( "Creating context\n" );
    ctx = glXCreateContextAttribsARB( m_data->m_dpy, m_data->m_bestFbc, 0,
                                      True, context_attribs );

    // Sync to ensure any errors generated are processed.
    MyXSync( m_data->m_dpy, False );
    if ( !ctxErrorOccurred && ctx )
      printf( "Created GL 3.0 context\n" );
    else
    {
      // Couldn't create GL 3.0 context.  Fall back to old-style 2.x context.
      // When a context version below 3.0 is requested, implementations will
      // return the newest context version compatible with OpenGL versions less
      // than version 3.0.
      // GLX_CONTEXT_MAJOR_VERSION_ARB = 1
      context_attribs[1] = 1;
      // GLX_CONTEXT_MINOR_VERSION_ARB = 0
      context_attribs[3] = 0;

      ctxErrorOccurred = false;

      printf( "Failed to create GL 3.0 context"
              " ... using old-style GLX context\n" );
      ctx = glXCreateContextAttribsARB( m_data->m_dpy, m_data->m_bestFbc, 0,
                                        True, context_attribs );
    }
  }

  // Sync to ensure any errors generated are processed.
  MyXSync( m_data->m_dpy, False );

  // Restore the original error handler
  MyXSetErrorHandler( oldHandler );

  if ( ctxErrorOccurred || !ctx )
  {
    printf( "Failed to create an OpenGL context\n" );
    exit(1);
  }

  // Verifying that context is a direct context
  if ( ! glXIsDirect ( m_data->m_dpy, ctx ) )
  {
    printf( "Indirect GLX rendering context obtained\n" );
  }
  else
  {
    printf( "Direct GLX rendering context obtained\n" );
  }

  printf( "Making context current\n" );
  glXMakeCurrent( m_data->m_dpy, m_data->m_win, ctx );

    } else
    {
        m_data->m_glc = glXCreateContext(m_data->m_dpy, m_data->m_vi, NULL, GL_TRUE);
        glXMakeCurrent(m_data->m_dpy, m_data->m_win, m_data->m_glc);
    }

#ifdef GLEW_INIT_OPENGL11_FUNCTIONS
{
	GLboolean res = glewOpenGL11Init();
	if (res==0)
		{
			printf("glewOpenGL11Init OK!\n");
		} else
			{
				printf("ERROR: glewOpenGL11Init failed, exiting!\n");
				exit(0);
			}
}

#endif //GLEW_INIT_OPENGL11_FUNCTIONS

    const GLubyte* ven = glGetString(GL_VENDOR);
    printf("GL_VENDOR=%s\n", ven);

    const GLubyte* ren = glGetString(GL_RENDERER);
    printf("GL_RENDERER=%s\n",ren);
    const GLubyte* ver = glGetString(GL_VERSION);
    printf("GL_VERSION=%s\n", ver);
    const GLubyte* sl = glGetString(GL_SHADING_LANGUAGE_VERSION);
    printf("GL_SHADING_LANGUAGE_VERSION=%s\n", sl);

//Access pthreads as a workaround for a bug in Linux/Ubuntu
//See https://bugs.launchpad.net/ubuntu/+source/nvidia-graphics-drivers-319/+bug/1248642

	int i=pthread_getconcurrency();
        printf("pthread_getconcurrency()=%d\n",i);

//    const GLubyte* ext = glGetString(GL_EXTENSIONS);
//    printf("GL_EXTENSIONS=%s\n", ext);
}

void X11OpenGLWindow::disableOpenGL()
{
    glXMakeCurrent(m_data->m_dpy, None, NULL);
 	glXDestroyContext(m_data->m_dpy, m_data->m_glc);
}


void    X11OpenGLWindow::createWindow(const b3gWindowConstructionInfo& ci)
{

    m_data->m_dpy = MyXOpenDisplay(NULL);

    if(m_data->m_dpy == NULL) {
        printf("\n\tcannot connect to X server\n\n");
            exit(0);
     }

    m_data->m_root = DefaultRootWindow(m_data->m_dpy);


#ifdef GLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS
GLboolean res = glewXInit();
if (res==0)
{
	printf("glewXInit OK\n");
} else
{
	printf("glewXInit failed, exit\n");
	exit(0);
}
#endif //GLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS


    if (ci.m_openglVersion < 3)
    {
        forceOpenGL3 = false;
    }

    if (forceOpenGL3)
    {
        int glxMinor, glxMajor;
        if (!glXQueryVersion(m_data->m_dpy,&glxMajor,&glxMinor) || (((glxMajor==1)&&(glxMinor<3)) || (glxMajor<1)))
        {
            printf("Invalid GLX version: major %d, minor %d\n",glxMajor,glxMinor);
            exit(0);
        }

        static int visual_attribs[] =
            {
              GLX_X_RENDERABLE    , True,
              GLX_DRAWABLE_TYPE   , GLX_WINDOW_BIT,
              GLX_RENDER_TYPE     , GLX_RGBA_BIT,
              GLX_X_VISUAL_TYPE   , GLX_TRUE_COLOR,
              GLX_RED_SIZE        , 8,
              GLX_GREEN_SIZE      , 8,
              GLX_BLUE_SIZE       , 8,
              GLX_ALPHA_SIZE      , 8,
              GLX_DEPTH_SIZE      , 24,
              GLX_STENCIL_SIZE    , 8,
              GLX_DOUBLEBUFFER    , True,
              None
            };
            int fbcount;
            GLXFBConfig* fbc = glXChooseFBConfig(m_data->m_dpy, DefaultScreen(m_data->m_dpy), visual_attribs, &fbcount);
            if (!fbc)
            {
                printf( "Failed to retrieve a framebuffer config\n" );
                exit(1);
            }

            int best_fbc = -1, worst_fbc = -1, best_num_samp = -1, worst_num_samp = 999;

            int i;
            for (i=0; i<fbcount; ++i)
            {
	            XVisualInfo *vi = glXGetVisualFromFBConfig( m_data->m_dpy, fbc[i] );
	            if ( vi )
	            {
	              int samp_buf, samples;
	              glXGetFBConfigAttrib( m_data->m_dpy, fbc[i], GLX_SAMPLE_BUFFERS, &samp_buf );
	              glXGetFBConfigAttrib( m_data->m_dpy, fbc[i], GLX_SAMPLES       , &samples  );

	              //printf( "  Matching fbconfig %d, visual ID 0x%2x: SAMPLE_BUFFERS = %d,"
	               //       " SAMPLES = %d\n",
	                //      i, vi -> visualid, samp_buf, samples );

	              if ( best_fbc < 0 || (samp_buf && (samples > best_num_samp)) )
	                best_fbc = i, best_num_samp = samples;
	              if ( worst_fbc < 0 || (!samp_buf || (samples < worst_num_samp)) )
	                worst_fbc = i, worst_num_samp = samples;
	            }
	            MyXFree( vi );
            }

            m_data->m_bestFbc = fbc[ best_fbc ];
            // Be sure to free the FBConfig list allocated by glXChooseFBConfig()
            MyXFree( fbc );

            m_data->m_vi = glXGetVisualFromFBConfig( m_data->m_dpy, m_data->m_bestFbc );


            m_data->m_swa.colormap = m_data->m_cmap = MyXCreateColormap( m_data->m_dpy,
                                                 RootWindow( m_data->m_dpy, m_data->m_vi->screen ),
                                                 m_data->m_vi->visual, AllocNone );
            m_data->m_swa.background_pixmap = None ;
            m_data->m_swa.border_pixel      = 0;
            m_data->m_swa.event_mask        = ExposureMask | KeyReleaseMask | KeyPressMask |ButtonPressMask | ButtonReleaseMask |PointerMotionMask|StructureNotifyMask;
;
            m_data->m_root =  RootWindow( m_data->m_dpy, m_data->m_vi->screen );

            m_data->m_win = MyXCreateWindow( m_data->m_dpy, m_data->m_root,
                                      0, 0, ci.m_width, ci.m_height, 0, m_data->m_vi->depth, InputOutput,
                                      m_data->m_vi->visual,
                                      CWBorderPixel|CWColormap|CWEventMask, &m_data->m_swa );

            //m_data->m_win = m_data->m_x11_XCreateWindow(m_data->m_dpy, m_data->m_root, 0, 0, ci.m_width, ci.m_height, 0, m_data->m_vi->depth, InputOutput, m_data->m_vi->visual, CWColormap | CWEventMask, &m_data->m_swa);


            if (!m_data->m_win)
            {
                printf("Cannot create window\n");
                exit(0);
            }

            MyXMapWindow(m_data->m_dpy, m_data->m_win);
            MyXStoreName(m_data->m_dpy, m_data->m_win, "OpenGL3 Window");


    } else
    {
         m_data->m_vi = glXChooseVisual(m_data->m_dpy, 0, att);

				printf("4\n");

         if(m_data->m_vi == NULL) {
            printf("\n\tno appropriate visual found\n\n");
                exit(0);
         }
         else {
            printf("\n\tvisual %p selected\n", (void *)m_data->m_vi->visualid); /* %p creates hexadecimal output like in glxinfo */
         }


         m_data->m_cmap = MyXCreateColormap(m_data->m_dpy, m_data->m_root, m_data->m_vi->visual, AllocNone);
         m_data->m_swa.colormap = m_data->m_cmap;
         m_data->m_swa.event_mask = ExposureMask | KeyReleaseMask | KeyPressMask |ButtonPressMask | ButtonReleaseMask |PointerMotionMask|StructureNotifyMask;
         m_data->m_win = MyXCreateWindow(m_data->m_dpy, m_data->m_root, 0, 0, ci.m_width, ci.m_height, 0, m_data->m_vi->depth, InputOutput, m_data->m_vi->visual, CWColormap | CWEventMask, &m_data->m_swa);

         MyXMapWindow(m_data->m_dpy, m_data->m_win);

         MyXStoreName(m_data->m_dpy, m_data->m_win, "OpenGL2 Window");


    }

    enableOpenGL();
}

void    X11OpenGLWindow::closeWindow()
{
    disableOpenGL();

    MyXDestroyWindow(m_data->m_dpy, m_data->m_win);
    MyXCloseDisplay(m_data->m_dpy);
}

int X11OpenGLWindow::getAsciiCodeFromVirtualKeycode(int keycode)
{
    int result = 0;

    KeySym key, key_lc, key_uc;

    int keysyms_per_keycode_return;
    KeySym *keysym = MyXGetKeyboardMapping(m_data->m_dpy,
        keycode,
        1,
        &keysyms_per_keycode_return);

    key = keysym[0];


    //key = MyXKeycodeToKeysym( m_data->m_dpy, keycode, 0 );

    switch( key )
    {
        case XK_Escape:       return B3G_ESCAPE;
        case XK_Return:         return B3G_RETURN;

	case XK_Control_L:
	case XK_Control_R:     {
			return B3G_CONTROL;
            }
    case XK_Left: return B3G_LEFT_ARROW;
    case XK_Right: return B3G_RIGHT_ARROW;
    case XK_Up: return B3G_UP_ARROW;
    case XK_Down: return B3G_DOWN_ARROW;

	case XK_Alt_L:
	case XK_Alt_R:
		{
		  return B3G_ALT;
		}
	case XK_Shift_L:
	case XK_Shift_R:	      return B3G_SHIFT;
        case XK_F1:           return B3G_F1;
        case XK_F2:           return B3G_F2;
        case XK_F3:           return B3G_F3;
        case XK_F4:           return B3G_F4;
        case XK_F5:           return B3G_F5;
        case XK_F6:           return B3G_F6;
        case XK_F7:           return B3G_F7;
        case XK_F8:           return B3G_F8;
        case XK_F9:           return B3G_F9;
        case XK_F10:          return B3G_F10;
        case XK_F11:          return B3G_F11;
        case XK_F12:          return B3G_F12;
        case XK_F13:          return B3G_F13;
        case XK_F14:          return B3G_F14;
        case XK_F15:          return B3G_F15;
        default:
	// Make lowercase
            MyXConvertCase( key, &key_lc, &key_uc );
            key = key_lc;
            // Valid ISO 8859-1 character?
            if( (key >=  32 && key <= 126) ||(key >= 160 && key <= 255) )
            {
                return (int) key;
            }
            result = -1;
    }

    MyXFree(keysym);

    return result;
}

bool    X11OpenGLWindow::isModifierKeyPressed(int key)
{
        bool isPressed = false;

        switch (key)
        {
                case B3G_ALT:
                {
                        isPressed = ((m_data->m_modifierFlags & MY_X11_ALT_KEY)!=0);
                        break;
                };
                case B3G_SHIFT:
                {
                        isPressed = ((m_data->m_modifierFlags & MY_X11_SHIFT_KEY)!=0);
                        break;
                };
                case B3G_CONTROL:
                {
                        isPressed = ((m_data->m_modifierFlags & MY_X11_CONTROL_KEY )!=0);
                        break;
                };

                default:
                {
                }
        };
        return isPressed;
}

void X11OpenGLWindow::pumpMessage()
{

    int buttonState = 1;

     // Process all pending events
    while( MyXPending( m_data->m_dpy ) )
    {
        MyXNextEvent(m_data->m_dpy, &m_data->m_xev);
  //      printf("#");
  //      fflush(stdout);
        switch( m_data->m_xev.type )
        {
            case KeyPress:
            {
                    int keycode = getAsciiCodeFromVirtualKeycode(m_data->m_xev.xkey.keycode);
		    switch (keycode)
			{
			case B3G_ALT:
			m_data->m_modifierFlags |= MY_X11_ALT_KEY;
			break;
			case B3G_SHIFT:
			m_data->m_modifierFlags |= MY_X11_SHIFT_KEY;
			break;
			case B3G_CONTROL:
			m_data->m_modifierFlags |= MY_X11_CONTROL_KEY;
			break;
			default:
			{}
			};
		if (m_data->m_keyboardCallback)
                {

                    int state = 1;
                    (*m_data->m_keyboardCallback)(keycode,state);
                //    printf("keycode %d",keycode);
                  //  fflush(stdout);

                }
                break;
            }

            case KeyRelease:
            {
   //           fflush(stdout);
 int keycode = getAsciiCodeFromVirtualKeycode( m_data->m_xev.xkey.keycode);
		  switch (keycode)
                        {
                        case B3G_ALT:
                        m_data->m_modifierFlags &= ~MY_X11_ALT_KEY;
                        break;
                        case B3G_SHIFT:
                        m_data->m_modifierFlags &= ~MY_X11_SHIFT_KEY;
                        break;
                        case B3G_CONTROL:
                        m_data->m_modifierFlags &= ~MY_X11_CONTROL_KEY;
                        break;
                        default:
                        {}
                        };

                if (m_data->m_keyboardCallback)
                {
#if 0
                     unsigned short is_retriggered = 0;
///filter out keyboard repeat
//see http://stackoverflow.com/questions/2100654/ignore-auto-repeat-in-x11-applications
                     if (MyXEventsQueued(m_data->m_dpy, QueuedAfterReading))
                       {
                         XEvent nev;
                         MyXPeekEvent(m_data->m_dpy, &nev);

                         if (nev.type == KeyPress && nev.xkey.time ==  m_data->m_xev.xkey.time &&
                             nev.xkey.keycode ==  m_data->m_xev.xkey.keycode)
                           {
                             fprintf (stdout, "key #%ld was retriggered.\n",
                               (long) MyXLookupKeysym(&nev.xkey, 0));

                             // delete retriggered KeyPress event
                             MyXNextEvent(m_data->m_dpy, & m_data->m_xev);
                             is_retriggered = 1;
                           }
                       }
#endif
                    int state = 0;
                    (*m_data->m_keyboardCallback)(keycode,state);
                    }

                break;
            }

            case ButtonRelease:
                buttonState = 0;
                //continue with ButtonPress code
            case ButtonPress:
            {
//                printf("!");
//                fflush(stdout);

                int button=-1;

                switch (m_data->m_xev.xbutton.button)
                {
                    case Button1:
                    {
                    button=0;
                    break;
                    }
                    case Button2:
                    {
                        button=1;
                        break;
                    }
                    case Button3:
                    {
                        button=2;
                        break;
                    }
                    case Button4:
                    {
                        if (m_data->m_wheelCallback)
                        {
                            (*m_data->m_wheelCallback)(0,10);
                        }
                        break;
                    }
                    case Button5:
                    {
                        if (m_data->m_wheelCallback)
                        {
                            (*m_data->m_wheelCallback)(0,-10);
                        }
                        break;
                    }
                }
                int xpos = m_data->m_xev.xmotion.x;
                int ypos = m_data->m_xev.xmotion.y;

                if (button>=0 && m_data->m_mouseButtonCallback)
                {
//                      printf("xpos = %d, ypos = %d\n",xpos,ypos);

                    (*m_data->m_mouseButtonCallback)(button,buttonState,xpos,ypos);
                }
                break;
            }
            case MotionNotify:
            {
//                printf("!");
//                fflush(0);
                if (m_data->m_mouseMoveCallback)
                {
                    int xpos = m_data->m_xev.xmotion.x;
                    int ypos = m_data->m_xev.xmotion.y;
                    (*m_data->m_mouseMoveCallback)(xpos,ypos);
                }
                break;
            }
            case ConfigureNotify:
            {
  //              printf("@");
  //              fflush(0);
                if (m_data->m_resizeCallback)
                {
                    (*m_data->m_resizeCallback)(m_data->m_xev.xconfigure.width,m_data->m_xev.xconfigure.height);
                }
                break;
            }
            case ClientMessage:
            {
  //              printf("?");
  //              fflush(stdout);
                break;
            }
            case Expose:
            {
                    break;
            }
            case DestroyNotify:
            {
                break;
            }
            default:
            {
                //XRRUpdateConfiguration( &event );
            }
        };
    }
}



void    X11OpenGLWindow::startRendering()
{
	pumpMessage();

    MyXGetWindowAttributes(m_data->m_dpy, m_data->m_win, &m_data->m_gwa);
    glViewport(0, 0, m_data->m_gwa.width, m_data->m_gwa.height);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);	//clear buffers

    //glCullFace(GL_BACK);
    //glFrontFace(GL_CCW);
    glEnable(GL_DEPTH_TEST);
}

void    X11OpenGLWindow::renderAllObjects()
{

}

void    X11OpenGLWindow::endRendering()
{
    glXSwapBuffers(m_data->m_dpy, m_data->m_win);
}

void    X11OpenGLWindow::runMainLoop()
{

}

float   X11OpenGLWindow::getTimeInSeconds()
{
    return 0.f;
}

bool    X11OpenGLWindow::requestedExit() const
{
    return m_requestedExit;
}

void    X11OpenGLWindow::setRequestExit()
{
	m_requestedExit=true;
}

void X11OpenGLWindow::setRenderCallback( b3RenderCallback renderCallback)
{

}

void X11OpenGLWindow::setWindowTitle(const char* title)
{
    MyXStoreName(m_data->m_dpy, m_data->m_win, title);
}


void X11OpenGLWindow::setWheelCallback(b3WheelCallback wheelCallback)
{
	m_data->m_wheelCallback = wheelCallback;
}

void X11OpenGLWindow::setMouseMoveCallback(b3MouseMoveCallback	mouseCallback)
{
	m_data->m_mouseMoveCallback = mouseCallback;
}

void X11OpenGLWindow::setMouseButtonCallback(b3MouseButtonCallback	mouseCallback)
{
	m_data->m_mouseButtonCallback = mouseCallback;
}

void X11OpenGLWindow::setResizeCallback(b3ResizeCallback	resizeCallback)
{
	m_data->m_resizeCallback = resizeCallback;
}

void X11OpenGLWindow::setKeyboardCallback( b3KeyboardCallback	keyboardCallback)
{
	m_data->m_keyboardCallback = keyboardCallback;

}

b3MouseMoveCallback X11OpenGLWindow::getMouseMoveCallback()
{
	return m_data->m_mouseMoveCallback;
}
b3MouseButtonCallback X11OpenGLWindow::getMouseButtonCallback()
{
	return m_data->m_mouseButtonCallback;
}
b3ResizeCallback X11OpenGLWindow::getResizeCallback()
{
	return m_data->m_resizeCallback;
}
b3WheelCallback X11OpenGLWindow::getWheelCallback()
{
	return m_data->m_wheelCallback;
}


b3KeyboardCallback      X11OpenGLWindow::getKeyboardCallback()
{
	return m_data->m_keyboardCallback;
}

#include <stdio.h>

int X11OpenGLWindow::fileOpenDialog(char* filename, int maxNameLength)
{
	int len = 0;
	FILE * output = popen("zenity --file-selection --file-filter=\"*.urdf\" --file-filter=\"*.*\"","r");
	if (output)
	{
		while( fgets(filename, maxNameLength-1, output) != NULL )
		{
			len=strlen(filename);
			if (len>0)
			{
				filename[len-1]=0;
				printf("file open (length=%d) = %s\n", len,filename);
			}	
		}
		pclose(output);
	} else
	{
		printf("Error: fileOpenDialog no popen output, perhaps install zenity?\n");
	}
	MyXRaiseWindow(m_data->m_dpy, m_data->m_win);
	return len;
	
}
