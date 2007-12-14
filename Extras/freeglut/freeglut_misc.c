/*
 * freeglut_misc.c
 *
 * Functions that didn't fit anywhere else...
 *
 * Copyright (c) 1999-2000 Pawel W. Olszta. All Rights Reserved.
 * Written by Pawel W. Olszta, <olszta@sourceforge.net>
 * Creation date: Thu Dec 9 1999
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * PAWEL W. OLSZTA BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <GL/freeglut.h>
#include "freeglut_internal.h"

/*
 * TODO BEFORE THE STABLE RELEASE:
 *
 *  glutSetColor()     --
 *  glutGetColor()     --
 *  glutCopyColormap() --
 *  glutSetKeyRepeat() -- this is evil and should be removed from API
 */

/* -- INTERFACE FUNCTIONS -------------------------------------------------- */

/*
 * This functions checks if an OpenGL extension is supported or not
 *
 * XXX Wouldn't this be simpler and clearer if we used strtok()?
 */
int FGAPIENTRY glutExtensionSupported( const char* extension )
{
  const char *extensions, *start;
  const int len = strlen( extension );

  /* Make sure there is a current window, and thus a current context available */
  FREEGLUT_EXIT_IF_NOT_INITIALISED ( "glutExtensionSupported" );
  freeglut_return_val_if_fail( fgStructure.CurrentWindow != NULL, 0 );

  if (strchr(extension, ' '))
    return 0;
  start = extensions = (const char *) glGetString(GL_EXTENSIONS);

  /* XXX consider printing a warning to stderr that there's no current
   * rendering context.
   */
  freeglut_return_val_if_fail( extensions != NULL, 0 );

  while (1) {
     const char *p = strstr(extensions, extension);
     if (!p)
        return 0;  /* not found */
     /* b2Assert that the match isn't a super string */
     if ((p == start || p[-1] == ' ') && (p[len] == ' ' || p[len] == 0))
        return 1;
     /* skip the false match and continue */
     extensions = p + len;
  }

  return 0 ;
}

/*
 * This function reports all the OpenGL errors that happened till now
 */
void FGAPIENTRY glutReportErrors( void )
{
    GLenum error;
    FREEGLUT_EXIT_IF_NOT_INITIALISED ( "glutReportErrors" );
    while( ( error = glGetError() ) != GL_NO_ERROR )
        fgWarning( "GL error: %s", gluErrorString( error ) );
}

/*
 * Control the auto-repeat of keystrokes to the current window
 */
void FGAPIENTRY glutIgnoreKeyRepeat( int ignore )
{
    FREEGLUT_EXIT_IF_NOT_INITIALISED ( "glutIgnoreKeyRepeat" );
    FREEGLUT_EXIT_IF_NO_WINDOW ( "glutIgnoreKeyRepeat" );

    fgStructure.CurrentWindow->State.IgnoreKeyRepeat = ignore ? GL_TRUE : GL_FALSE;
}

/*
 * Set global auto-repeat of keystrokes
 *
 * RepeatMode should be either:
 *    GLUT_KEY_REPEAT_OFF
 *    GLUT_KEY_REPEAT_ON
 *    GLUT_KEY_REPEAT_DEFAULT
 */
void FGAPIENTRY glutSetKeyRepeat( int repeatMode )
{
    FREEGLUT_EXIT_IF_NOT_INITIALISED ( "glutSetKeyRepeat" );

    switch( repeatMode )
    {
    case GLUT_KEY_REPEAT_OFF:
    case GLUT_KEY_REPEAT_ON:
     fgState.KeyRepeat = repeatMode;
     break;

    case GLUT_KEY_REPEAT_DEFAULT:
     fgState.KeyRepeat = GLUT_KEY_REPEAT_ON;
     break;

    default:
        fgError ("Invalid glutSetKeyRepeat mode: %d", repeatMode);
        break;
    }
}

/*
 * Forces the joystick callback to be executed
 */
void FGAPIENTRY glutForceJoystickFunc( void )
{
    FREEGLUT_EXIT_IF_NOT_INITIALISED ( "glutForceJoystickFunc" );
#if !TARGET_HOST_WINCE
    freeglut_return_if_fail( fgStructure.CurrentWindow != NULL );
    freeglut_return_if_fail( FETCH_WCB( *( fgStructure.CurrentWindow ), Joystick ) );
    fgJoystickPollWindow( fgStructure.CurrentWindow );
#endif /* !TARGET_HOST_WINCE */
}

/*
 *
 */
void FGAPIENTRY glutSetColor( int nColor, GLfloat red, GLfloat green, GLfloat blue )
{
    FREEGLUT_EXIT_IF_NOT_INITIALISED ( "glutSetColor" );
    /* We really need to do something here. */
}

/*
 *
 */
GLfloat FGAPIENTRY glutGetColor( int color, int component )
{
    FREEGLUT_EXIT_IF_NOT_INITIALISED ( "glutGetColor" );
    /* We really need to do something here. */
    return( 0.0f );
}

/*
 *
 */
void FGAPIENTRY glutCopyColormap( int window )
{
    FREEGLUT_EXIT_IF_NOT_INITIALISED ( "glutCopyColormap" );
    /* We really need to do something here. */
}

/*** END OF FILE ***/
