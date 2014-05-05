#include "X11OpenGLWindow.h"
#include "OpenGLInclude.h"

#include<stdio.h>
#include<stdlib.h>
#include<X11/X.h>
#include<X11/Xlib.h>
#include<GL/gl.h>
#include<GL/glx.h>
//#include<GL/glu.h>

#include <stdio.h>

#include <pthread.h>

GLint                   att[] = { GLX_RGBA, GLX_DEPTH_SIZE, 24, GLX_DOUBLEBUFFER, None };


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

	}
};





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

    m_data->m_glc = glXCreateContext(m_data->m_dpy, m_data->m_vi, NULL, GL_TRUE);
    glXMakeCurrent(m_data->m_dpy, m_data->m_win, m_data->m_glc);

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
    m_data->m_dpy = XOpenDisplay(NULL);

    if(m_data->m_dpy == NULL) {
        printf("\n\tcannot connect to X server\n\n");
            exit(0);
     }

     m_data->m_root = DefaultRootWindow(m_data->m_dpy);

     m_data->m_vi = glXChooseVisual(m_data->m_dpy, 0, att);

     if(m_data->m_vi == NULL) {
        printf("\n\tno appropriate visual found\n\n");
            exit(0);
     }
     else {
        printf("\n\tvisual %p selected\n", (void *)m_data->m_vi->visualid); /* %p creates hexadecimal output like in glxinfo */
     }


     m_data->m_cmap = XCreateColormap(m_data->m_dpy, m_data->m_root, m_data->m_vi->visual, AllocNone);

     m_data->m_swa.colormap = m_data->m_cmap;
     m_data->m_swa.event_mask = ExposureMask | KeyReleaseMask | KeyPressMask |ButtonPressMask | ButtonReleaseMask |PointerMotionMask|StructureNotifyMask;

     m_data->m_win = XCreateWindow(m_data->m_dpy, m_data->m_root, 0, 0, ci.m_width, ci.m_height, 0, m_data->m_vi->depth, InputOutput, m_data->m_vi->visual, CWColormap | CWEventMask, &m_data->m_swa);

     XMapWindow(m_data->m_dpy, m_data->m_win);
     XStoreName(m_data->m_dpy, m_data->m_win, "VERY SIMPLE APPLICATION");

    enableOpenGL();
}

void    X11OpenGLWindow::closeWindow()
{
    disableOpenGL();

    XDestroyWindow(m_data->m_dpy, m_data->m_win);
 	XCloseDisplay(m_data->m_dpy);
}

int X11OpenGLWindow::getAsciiCodeFromVirtualKeycode(int keycode)
{
    KeySym key, key_lc, key_uc;

    key = XKeycodeToKeysym( m_data->m_dpy, keycode, 0 );
    switch( key )
    {
        case XK_Escape:       return B3G_ESCAPE;
	
	case XK_Control_L:
	case XK_Control_R:     {
			return B3G_CONTROL;
	}
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
            // Make uppercase
            XConvertCase( key, &key_lc, &key_uc );
            key = key_uc;
            // Valid ISO 8859-1 character?
            if( (key >=  32 && key <= 126) ||(key >= 160 && key <= 255) )
            {
                return (int) key;
            }
            return -1;
    }
    return 0;
}

void X11OpenGLWindow::pumpMessage()
{

    int buttonState = 1;

     // Process all pending events
    while( XPending( m_data->m_dpy ) )
    {
        XNextEvent(m_data->m_dpy, &m_data->m_xev);
  //      printf("#");
  //      fflush(stdout);
        switch( m_data->m_xev.type )
        {
            case KeyPress:
            {
                if (m_data->m_keyboardCallback)
                {
                    int keycode = getAsciiCodeFromVirtualKeycode(m_data->m_xev.xkey.keycode);
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

                if (m_data->m_keyboardCallback)
                {

                     unsigned short is_retriggered = 0;
///filter out keyboard repeat
//see http://stackoverflow.com/questions/2100654/ignore-auto-repeat-in-x11-applications
                     if (XEventsQueued(m_data->m_dpy, QueuedAfterReading))
                       {
                         XEvent nev;
                         XPeekEvent(m_data->m_dpy, &nev);

                         if (nev.type == KeyPress && nev.xkey.time ==  m_data->m_xev.xkey.time &&
                             nev.xkey.keycode ==  m_data->m_xev.xkey.keycode)
                           {
                             fprintf (stdout, "key #%ld was retriggered.\n",
                               (long) XLookupKeysym (&nev.xkey, 0));

                             // delete retriggered KeyPress event
                             XNextEvent (m_data->m_dpy, & m_data->m_xev);
                             is_retriggered = 1;
                           }
                       }
                    int keycode = getAsciiCodeFromVirtualKeycode( m_data->m_xev.xkey.keycode);
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

    XGetWindowAttributes(m_data->m_dpy, m_data->m_win, &m_data->m_gwa);
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
    XStoreName(m_data->m_dpy, m_data->m_win, title);
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

b3KeyboardCallback      X11OpenGLWindow::getKeyboardCallback()
{
	return m_data->m_keyboardCallback;
}

