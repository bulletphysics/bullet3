/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

#ifdef WIN32		// this prevents warnings when dependencies built
#include <windows.h>
#endif
#include <ode/config.h>
#include <GL/gl.h>

#include "resource.h"
#include "internal.h"

//***************************************************************************
// application globals

static HINSTANCE ghInstance = 0;
static int gnCmdShow = 0;
static HACCEL accelerators = 0;
static HWND main_window = 0;

//***************************************************************************
// error and message handling

static void errorBox (char *title, char *msg, va_list ap)
{
  char s[1000];
  vsprintf (s,msg,ap);
  MessageBox (0,s,title,MB_OK | MB_APPLMODAL | MB_ICONEXCLAMATION);
}


static void dsWarning (char *msg, ...)
{
  va_list ap;
  va_start (ap,msg);
  errorBox ("Warning",msg,ap);
}


extern "C" void dsError (char *msg, ...)
{
  va_list ap;
  va_start (ap,msg);
  errorBox ("Error",msg,ap);
  exit (1);
}


extern "C" void dsDebug (char *msg, ...)
{
  va_list ap;
  va_start (ap,msg);
  errorBox ("INTERNAL ERROR",msg,ap);
  // *((char *)0) = 0;	 ... commit SEGVicide ?
  abort();
  exit (1);	  // should never get here, but just in case...
}


extern "C" void dsPrint (char *msg, ...)
{
  va_list ap;
  va_start (ap,msg);
  vprintf (msg,ap);
}

//***************************************************************************
// rendering thread

// globals used to communicate with rendering thread

static volatile int renderer_run = 1;
static volatile int renderer_pause = 0;	  // 0=run, 1=pause
static volatile int renderer_ss = 0;	  // single step command
static volatile int renderer_width = 1;
static volatile int renderer_height = 1;
static dsFunctions *renderer_fn = 0;
static volatile HDC renderer_dc = 0;
static volatile int keybuffer[16];	  // fifo ring buffer for keypresses
static volatile int keybuffer_head = 0;	  // index of next key to put in (modified by GUI)
static volatile int keybuffer_tail = 0;	  // index of next key to take out (modified by renderer)


static void setupRendererGlobals()
{
  renderer_run = 1;
  renderer_pause = 0;
  renderer_ss = 0;
  renderer_width = 1;
  renderer_height = 1;
  renderer_fn = 0;
  renderer_dc = 0;
  keybuffer[16];
  keybuffer_head = 0;
  keybuffer_tail = 0;
}


static DWORD WINAPI renderingThread (LPVOID lpParam)
{
  // create openGL context and make it current
  HGLRC glc = wglCreateContext (renderer_dc);
  if (glc==NULL) dsError ("could not create OpenGL context");
  if (wglMakeCurrent (renderer_dc,glc) != TRUE)
    dsError ("could not make OpenGL context current");

  // test openGL capabilities
  int maxtsize=0;
  glGetIntegerv (GL_MAX_TEXTURE_SIZE,&maxtsize);
  if (maxtsize < 128) dsWarning ("max texture size too small (%dx%d)",
				 maxtsize,maxtsize);

  dsStartGraphics (renderer_width,renderer_height,renderer_fn);
  if (renderer_fn->start) renderer_fn->start();

  while (renderer_run) {
    // need to make local copy of renderer_ss to help prevent races
    int ss = renderer_ss;
    dsDrawFrame (renderer_width,renderer_height,renderer_fn,
		 renderer_pause && !ss);
    if (ss) renderer_ss = 0;

    // read keys out of ring buffer and feed them to the command function
    while (keybuffer_head != keybuffer_tail) {
      if (renderer_fn->command) renderer_fn->command (keybuffer[keybuffer_tail]);
      keybuffer_tail = (keybuffer_tail+1) & 15;
    }

    // swap buffers
    SwapBuffers (renderer_dc);
  }

  if (renderer_fn->stop) renderer_fn->stop();
  dsStopGraphics();

  // delete openGL context
  wglMakeCurrent (NULL,NULL);
  wglDeleteContext (glc);

  return 123;	    // magic value used to test for thread termination
}

//***************************************************************************
// window handling

// callback function for "about" dialog box

static LRESULT CALLBACK AboutDlgProc (HWND hDlg, UINT uMsg, WPARAM wParam,
				      LPARAM lParam)
{
  switch (uMsg) {
  case WM_INITDIALOG:
    return TRUE;
  case WM_COMMAND:
    switch (wParam) {
    case IDOK:
      EndDialog (hDlg, TRUE);
      return TRUE;
    }
    break;
  }
  return FALSE;
}


// callback function for the main window

static LRESULT CALLBACK mainWndProc (HWND hWnd, UINT msg, WPARAM wParam,
				     LPARAM lParam)
{
  static int button=0,lastx=0,lasty=0;
  int ctrl = wParam & MK_CONTROL;

  switch (msg) {
  case WM_LBUTTONDOWN:
  case WM_MBUTTONDOWN:
  case WM_RBUTTONDOWN:
    if (msg==WM_LBUTTONDOWN) button |= 1;
    else if (msg==WM_MBUTTONDOWN) button |= 2;
    else button |= 4;
    lastx = SHORT(LOWORD(lParam));
    lasty = SHORT(HIWORD(lParam));
    SetCapture (hWnd);
    break;

  case WM_LBUTTONUP:
  case WM_MBUTTONUP:
  case WM_RBUTTONUP:
    if (msg==WM_LBUTTONUP) button &= ~1;
    else if (msg==WM_MBUTTONUP) button &= ~2;
    else button &= ~4;
    if (button==0) ReleaseCapture();
    break;

  case WM_MOUSEMOVE: {
    int x = SHORT(LOWORD(lParam));
    int y = SHORT(HIWORD(lParam));
    if (button) dsMotion (button,x-lastx,y-lasty);
    lastx = x;
    lasty = y;
    break;
  }

  case WM_CHAR: {
    if (wParam >= ' ' && wParam <= 126) {
      int nexth = (keybuffer_head+1) & 15;
      if (nexth != keybuffer_tail) {
	keybuffer[keybuffer_head] = wParam;
	keybuffer_head = nexth;
      }
    }
    break;
  }

  case WM_SIZE:
    // lParam will contain the size of the *client* area!
    renderer_width = LOWORD(lParam);
    renderer_height = HIWORD(lParam);
    break;

  case WM_COMMAND:
    switch (wParam & 0xffff) {
    case IDM_ABOUT:
      DialogBox (ghInstance,MAKEINTRESOURCE(IDD_ABOUT),hWnd,
	(DLGPROC) AboutDlgProc);
      break;
    case IDM_PAUSE: {
      renderer_pause ^= 1;
      CheckMenuItem (GetMenu(hWnd),IDM_PAUSE,
		     renderer_pause ? MF_CHECKED : MF_UNCHECKED);
      if (renderer_pause) renderer_ss = 0;
      break;
    }
    case IDM_SINGLE_STEP: {
      renderer_ss = 1;
      break;
    }
    case IDM_PERF_MONITOR: {
      dsWarning ("Performance monitor not yet implemented.");
      break;
    }
    case IDM_TEXTURES: {
      static int tex = 1;
      tex ^= 1;
      CheckMenuItem (GetMenu(hWnd),IDM_TEXTURES,
		     tex ? MF_CHECKED : MF_UNCHECKED);
      dsSetTextures (tex);
      break;
    }
    case IDM_SHADOWS: {
      static int shadows = 1;
      shadows ^= 1;
      CheckMenuItem (GetMenu(hWnd),IDM_SHADOWS,
		     shadows ? MF_CHECKED : MF_UNCHECKED);
      dsSetShadows (shadows);
      break;
    }
    case IDM_SAVE_SETTINGS: {
      dsWarning ("\"Save Settings\" not yet implemented.");
      break;
    }
    case IDM_EXIT:
      PostQuitMessage (0);
      break;
    }
    break;

  case WM_CLOSE:
    PostQuitMessage (0);
    break;

  default:
    return (DefWindowProc (hWnd, msg, wParam, lParam));
  }

  return 0;
}


// this comes from an MSDN example. believe it or not, this is the recommended
// way to get the console window handle.

static HWND GetConsoleHwnd()
{
  // the console window title to a "unique" value, then find the window
  // that has this title.
  char title[1024];
  wsprintf (title,"DrawStuff:%d/%d",GetTickCount(),GetCurrentProcessId());
  SetConsoleTitle (title);
  Sleep(40);			// ensure window title has been updated
  return FindWindow (NULL,title);
}


static void drawStuffStartup()
{
  static int startup_called = 0;
  if (startup_called) return;
  startup_called = 1;
  ghInstance = GetModuleHandleA (NULL);
  gnCmdShow = SW_SHOWNORMAL;		// @@@ fix this later

  // redirect standard I/O to a new console (except on cygwin)
#ifndef CYGWIN
  FreeConsole();
  if (AllocConsole()==0) dsError ("AllocConsole() failed");
  if (freopen ("CONIN$","rt",stdin)==0) dsError ("could not open stdin");
  if (freopen ("CONOUT$","wt",stdout)==0) dsError ("could not open stdout");
  if (freopen ("CONOUT$","wt",stderr)==0) dsError ("could not open stderr");
  BringWindowToTop (GetConsoleHwnd());
  SetConsoleTitle ("DrawStuff Messages");
#endif

  // register the window class
  WNDCLASS wc;
  wc.style = CS_OWNDC | CS_VREDRAW | CS_HREDRAW;
  wc.lpfnWndProc = mainWndProc;
  wc.cbClsExtra = 0;
  wc.cbWndExtra = 0;
  wc.hInstance = ghInstance;
  wc.hIcon = LoadIcon (NULL,IDI_APPLICATION);
  wc.hCursor = LoadCursor (NULL,IDC_ARROW);
  wc.hbrBackground = (HBRUSH) (COLOR_WINDOW+1);
  wc.lpszMenuName = MAKEINTRESOURCE(IDR_MENU1);
  wc.lpszClassName = "SimAppClass";
  if (RegisterClass (&wc)==0) dsError ("could not register window class");

  // load accelerators
  accelerators = LoadAccelerators (ghInstance,
				   MAKEINTRESOURCE(IDR_ACCELERATOR1));
  if (accelerators==NULL) dsError ("could not load accelerators");
}


void dsPlatformSimLoop (int window_width, int window_height,
			dsFunctions *fn, int initial_pause)
{
  drawStuffStartup();
  setupRendererGlobals();
  renderer_pause = initial_pause;

  // create window - but first get window size for desired size of client area.
  // if this adjustment isn't made then the openGL area will be shifted into
  // the nonclient area and determining the frame buffer coordinate from the
  // client area coordinate will be hard.
  RECT winrect;
  winrect.left = 50;
  winrect.top = 80;
  winrect.right = winrect.left + window_width;
  winrect.bottom = winrect.top + window_height;
  DWORD style = WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN | WS_CLIPSIBLINGS;
  AdjustWindowRect (&winrect,style,1);
  char title[100];
  sprintf (title,"Simulation test environment v%d.%02d",
	   DS_VERSION >> 8,DS_VERSION & 0xff);
  main_window = CreateWindow ("SimAppClass",title,style,
    winrect.left,winrect.top,winrect.right-winrect.left,winrect.bottom-winrect.top,
    NULL,NULL,ghInstance,NULL);
  if (main_window==NULL) dsError ("could not create main window");
  ShowWindow (main_window, gnCmdShow);

  HDC dc = GetDC (main_window);			// get DC for this window
  if (dc==NULL) dsError ("could not get window DC");

  // set pixel format for DC

  PIXELFORMATDESCRIPTOR pfd = {
    sizeof(PIXELFORMATDESCRIPTOR),   // size of this pfd
    1,				     // version number
    PFD_DRAW_TO_WINDOW |	     // support window
    PFD_SUPPORT_OPENGL |	     // support OpenGL
    PFD_DOUBLEBUFFER,		     // double buffered
    PFD_TYPE_RGBA,		     // RGBA type
    24, 			     // 24-bit color depth
    0, 0, 0, 0, 0, 0,		     // color bits ignored
    0,				     // no alpha buffer
    0,				     // shift bit ignored
    0,				     // no accumulation buffer
    0, 0, 0, 0, 		     // accum bits ignored
    32, 			     // 32-bit z-buffer
    0,				     // no stencil buffer
    0,				     // no auxiliary buffer
    PFD_MAIN_PLANE,		     // main layer
    0,				     // reserved
    0, 0, 0			     // layer masks ignored
  };
  // get the best available match of pixel format for the device context
  int iPixelFormat = ChoosePixelFormat (dc,&pfd);
  if (iPixelFormat==0)
    dsError ("could not find a good OpenGL pixel format");
  // set the pixel format of the device context
  if (SetPixelFormat (dc,iPixelFormat,&pfd)==FALSE)
    dsError ("could not set DC pixel format for OpenGL");

  // **********
  // start the rendering thread

  // set renderer globals
  renderer_dc = dc;
  renderer_width = window_width;
  renderer_height = window_height;
  renderer_fn = fn;

  DWORD threadId, thirdParam = 0;
  HANDLE hThread;

  hThread = CreateThread(
	NULL,			     // no security attributes
	0,			     // use default stack size
	renderingThread,	     // thread function
	&thirdParam,		     // argument to thread function
	0,			     // use default creation flags
	&threadId);		     // returns the thread identifier

  if (hThread==NULL) dsError ("Could not create rendering thread");

  // **********
  // start GUI message processing

  MSG msg;
  while (GetMessage (&msg,main_window,0,0)) {
    if (!TranslateAccelerator (main_window,accelerators,&msg)) {
      TranslateMessage (&msg);
      DispatchMessage (&msg);
    }
  }

  // terminate rendering thread
  renderer_run = 0;
  DWORD ret = WaitForSingleObject (hThread,2000);
  if (ret==WAIT_TIMEOUT) dsWarning ("Could not kill rendering thread (1)");
  DWORD exitcode=0;
  if (!(GetExitCodeThread (hThread,&exitcode) && exitcode == 123))
    dsWarning ("Could not kill rendering thread (2)");
  CloseHandle (hThread);	     // dont need thread handle anymore

  // destroy window
  DestroyWindow (main_window);
}


extern "C" void dsStop()
{
  // just calling PostQuitMessage() here wont work, as this function is
  // typically called from the rendering thread, not the GUI thread.
  // instead we must post the message to the GUI window explicitly.

  if (main_window) PostMessage (main_window,WM_QUIT,0,0);
}

//***************************************************************************
// windows entry point
//
// NOTE: WinMain is not guaranteed to be called with MinGW, because MinGW
// always calls main if it is defined and most users of this library will
// define their own main. So the startup functionality is kept in
// zDriverStartup(), which is also called when dsSimulationLoop() is called.

extern "C" int main (int argc, char **argv);


int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance,
		   LPSTR lpCmdLine, int nCmdShow)
{
  drawStuffStartup();
  return main (0,0);	// @@@ should really pass cmd line arguments
}
