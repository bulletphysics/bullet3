/*
 * freeglut_init.c
 *
 * Various freeglut initialization functions.
 *
 * Copyright (c) 1999-2000 Pawel W. Olszta. All Rights Reserved.
 * Written by Pawel W. Olszta, <olszta@sourceforge.net>
 * Creation date: Thu Dec 2 1999
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
 *  fgDeinitialize()        -- Win32's OK, X11 needs the OS-specific
 *                             deinitialization done
 *  glutInitDisplayString() -- display mode string parsing
 *
 * Wouldn't it be cool to use gettext() for error messages? I just love
 * bash saying  "nie znaleziono pliku" instead of "file not found" :)
 * Is gettext easily portable?
 */

/* -- GLOBAL VARIABLES ----------------------------------------------------- */

/*
 * A structure pointed by g_pDisplay holds all information
 * regarding the display, screen, root window etc.
 */
SFG_Display fgDisplay;

/*
 * The settings for the current freeglut session
 */
SFG_State fgState = { { -1, -1, GL_FALSE },  /* Position */
                      { 300, 300, GL_TRUE }, /* Size */
                      GLUT_RGBA | GLUT_SINGLE | GLUT_DEPTH,  /* DisplayMode */
                      GL_FALSE,              /* Initialised */
                      GLUT_TRY_DIRECT_CONTEXT,  /* DirectContext */
                      GL_FALSE,              /* ForceIconic */
                      GL_FALSE,              /* UseCurrentContext */
                      GL_FALSE,              /* GLDebugSwitch */
                      GL_FALSE,              /* XSyncSwitch */
                      GLUT_KEY_REPEAT_ON,    /* KeyRepeat */
                      0xffffffff,            /* Modifiers */
                      0,                     /* FPSInterval */
                      0,                     /* SwapCount */
                      0,                     /* SwapTime */
#if TARGET_HOST_WIN32 || TARGET_HOST_WINCE
                      { 0, GL_FALSE },       /* Time */
#else
                      { { 0, 0 }, GL_FALSE },
#endif
                      { NULL, NULL },         /* Timers */
                      { NULL, NULL },         /* FreeTimers */
                      NULL,                   /* IdleCallback */
                      0,                      /* ActiveMenus */
                      NULL,                   /* MenuStateCallback */
                      NULL,                   /* MenuStatusCallback */
                      { 640, 480, GL_TRUE },  /* GameModeSize */
                      16,                     /* GameModeDepth */
                      72,                     /* GameModeRefresh */
                      GLUT_ACTION_EXIT,       /* ActionOnWindowClose */
                      GLUT_EXEC_STATE_INIT,   /* ExecState */
                      NULL,                   /* ProgramName */
                      GL_FALSE                /* JoysticksInitialised */
};


/* -- PRIVATE FUNCTIONS ---------------------------------------------------- */

/*
 * A call to this function should initialize all the display stuff...
 */
static void fghInitialize( const char* displayName )
{
#if TARGET_HOST_UNIX_X11
    fgDisplay.Display = XOpenDisplay( displayName );

    if( fgDisplay.Display == NULL )
        fgError( "failed to open display '%s'", XDisplayName( displayName ) );

    if( !glXQueryExtension( fgDisplay.Display, NULL, NULL ) )
        fgError( "OpenGL GLX extension not supported by display '%s'",
            XDisplayName( displayName ) );

    fgDisplay.Screen = DefaultScreen( fgDisplay.Display );
    fgDisplay.RootWindow = RootWindow(
        fgDisplay.Display,
        fgDisplay.Screen
    );

    fgDisplay.ScreenWidth  = DisplayWidth(
        fgDisplay.Display,
        fgDisplay.Screen
    );
    fgDisplay.ScreenHeight = DisplayHeight(
        fgDisplay.Display,
        fgDisplay.Screen
    );

    fgDisplay.ScreenWidthMM = DisplayWidthMM(
        fgDisplay.Display,
        fgDisplay.Screen
    );
    fgDisplay.ScreenHeightMM = DisplayHeightMM(
        fgDisplay.Display,
        fgDisplay.Screen
    );

    fgDisplay.Connection = ConnectionNumber( fgDisplay.Display );

    /* Create the window deletion atom */
    fgDisplay.DeleteWindow = XInternAtom(
        fgDisplay.Display,
        "WM_DELETE_WINDOW",
        FALSE
    );

#elif TARGET_HOST_WIN32 || TARGET_HOST_WINCE

    WNDCLASS wc;
    ATOM atom;

    /* What we need to do is to initialize the fgDisplay global structure here. */
    fgDisplay.Instance = GetModuleHandle( NULL );

    atom = GetClassInfo( fgDisplay.Instance, _T("FREEGLUT"), &wc );

    if( atom == 0 )
    {
        ZeroMemory( &wc, sizeof(WNDCLASS) );

        /*
         * Each of the windows should have its own device context, and we
         * want redraw events during Vertical and Horizontal Resizes by
         * the user.
         *
         * XXX Old code had "| CS_DBCLCKS" commented out.  Plans for the
         * XXX future?  Dead-end idea?
         */
        wc.lpfnWndProc    = fgWindowProc;
        wc.cbClsExtra     = 0;
        wc.cbWndExtra     = 0;
        wc.hInstance      = fgDisplay.Instance;
        wc.hIcon          = LoadIcon( fgDisplay.Instance, _T("GLUT_ICON") );

#if TARGET_HOST_WIN32
        wc.style          = CS_OWNDC | CS_HREDRAW | CS_VREDRAW;
        if (!wc.hIcon)
          wc.hIcon        = LoadIcon( NULL, IDI_WINLOGO );
#else /* TARGET_HOST_WINCE */
        wc.style          = CS_HREDRAW | CS_VREDRAW;
#endif

        wc.hCursor        = LoadCursor( NULL, IDC_ARROW );
        wc.hbrBackground  = NULL;
        wc.lpszMenuName   = NULL;
        wc.lpszClassName  = _T("FREEGLUT");

        /* Register the window class */
        atom = RegisterClass( &wc );
        FREEGLUT_INTERNAL_ERROR_EXIT ( atom, "Window Class Not Registered", "fghInitialize" );
    }

    /* The screen dimensions can be obtained via GetSystemMetrics() calls */
    fgDisplay.ScreenWidth  = GetSystemMetrics( SM_CXSCREEN );
    fgDisplay.ScreenHeight = GetSystemMetrics( SM_CYSCREEN );

    {
        HWND desktop = GetDesktopWindow( );
        HDC  context = GetDC( desktop );

        fgDisplay.ScreenWidthMM  = GetDeviceCaps( context, HORZSIZE );
        fgDisplay.ScreenHeightMM = GetDeviceCaps( context, VERTSIZE );

        ReleaseDC( desktop, context );
    }

    /* Set the timer granularity to 1 ms */
    timeBeginPeriod ( 1 );

#endif

    fgState.Initialised = GL_TRUE;
}

/*
 * Perform the freeglut deinitialization...
 */
void fgDeinitialize( void )
{
    SFG_Timer *timer;

    if( !fgState.Initialised )
    {
        fgWarning( "fgDeinitialize(): "
                   "no valid initialization has been performed" );
        return;
    }

    /* If there was a menu created, destroy the rendering context */
    if( fgStructure.MenuContext )
    {
        free( fgStructure.MenuContext );
        fgStructure.MenuContext = NULL;
    }

    fgDestroyStructure( );

    while( ( timer = fgState.Timers.First) )
    {
        fgListRemove( &fgState.Timers, &timer->Node );
        free( timer );
    }

    while( ( timer = fgState.FreeTimers.First) )
    {
        fgListRemove( &fgState.FreeTimers, &timer->Node );
        free( timer );
    }

#if !TARGET_HOST_WINCE
    if ( fgState.JoysticksInitialised )
        fgJoystickClose( );
#endif /* !TARGET_HOST_WINCE */
    fgState.JoysticksInitialised = GL_FALSE;

    fgState.Initialised = GL_FALSE;

    fgState.Position.X = -1;
    fgState.Position.Y = -1;
    fgState.Position.Use = GL_FALSE;

    fgState.Size.X = 300;
    fgState.Size.Y = 300;
    fgState.Size.Use = GL_TRUE;

    fgState.DisplayMode = GLUT_RGBA | GLUT_SINGLE | GLUT_DEPTH;

    fgState.DirectContext  = GLUT_TRY_DIRECT_CONTEXT;
    fgState.ForceIconic         = GL_FALSE;
    fgState.UseCurrentContext   = GL_FALSE;
    fgState.GLDebugSwitch       = GL_FALSE;
    fgState.XSyncSwitch         = GL_FALSE;
    fgState.ActionOnWindowClose = GLUT_ACTION_EXIT;
    fgState.ExecState           = GLUT_EXEC_STATE_INIT;

    fgState.KeyRepeat       = GLUT_KEY_REPEAT_ON;
    fgState.Modifiers       = 0xffffffff;

    fgState.GameModeSize.X  = 640;
    fgState.GameModeSize.Y  = 480;
    fgState.GameModeDepth   =  16;
    fgState.GameModeRefresh =  72;

    fgState.Time.Set = GL_FALSE;

    fgListInit( &fgState.Timers );
    fgListInit( &fgState.FreeTimers );

    fgState.IdleCallback = NULL;
    fgState.MenuStateCallback = ( FGCBMenuState )NULL;
    fgState.MenuStatusCallback = ( FGCBMenuStatus )NULL;

    fgState.SwapCount   = 0;
    fgState.SwapTime    = 0;
    fgState.FPSInterval = 0;

    if( fgState.ProgramName )
    {
        free( fgState.ProgramName );
        fgState.ProgramName = NULL;
    }

#if TARGET_HOST_UNIX_X11

    /*
     * Make sure all X-client data we have created will be destroyed on
     * display closing
     */
    XSetCloseDownMode( fgDisplay.Display, DestroyAll );

    /*
     * Close the display connection, destroying all windows we have
     * created so far
     */
    XCloseDisplay( fgDisplay.Display );

#elif TARGET_HOST_WIN32 || TARGET_HOST_WINCE

    /* Reset the timer granularity */
    timeEndPeriod ( 1 );

#endif

    fgState.Initialised = GL_FALSE;
}

/*
 * Everything inside the following #ifndef is copied from the X sources.
 */

#if TARGET_HOST_WIN32 || TARGET_HOST_WINCE

/*

Copyright 1985, 1986, 1987,1998  The Open Group

Permission to use, copy, modify, distribute, and sell this software and its
documentation for any purpose is hereby granted without fee, provided that
the above copyright notice appear in all copies and that both that
copyright notice and this permission notice appear in supporting
documentation.

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE OPEN GROUP BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

Except as contained in this notice, the name of The Open Group shall
not be used in advertising or otherwise to promote the sale, use or
other dealings in this Software without prior written authorization
from The Open Group.

*/

#define NoValue         0x0000
#define XValue          0x0001
#define YValue          0x0002
#define WidthValue      0x0004
#define HeightValue     0x0008
#define AllValues       0x000F
#define XNegative       0x0010
#define YNegative       0x0020

/*
 *    XParseGeometry parses strings of the form
 *   "=<width>x<height>{+-}<xoffset>{+-}<yoffset>", where
 *   width, height, xoffset, and yoffset are unsigned integers.
 *   Example:  "=80x24+300-49"
 *   The equal sign is optional.
 *   It returns a bitmask that indicates which of the four values
 *   were actually found in the string.  For each value found,
 *   the corresponding argument is updated;  for each value
 *   not found, the corresponding argument is left unchanged.
 */

static int
ReadInteger(char *string, char **NextString)
{
    register int Result = 0;
    int Sign = 1;

    if (*string == '+')
        string++;
    else if (*string == '-')
    {
        string++;
        Sign = -1;
    }
    for (; (*string >= '0') && (*string <= '9'); string++)
    {
        Result = (Result * 10) + (*string - '0');
    }
    *NextString = string;
    if (Sign >= 0)
        return Result;
    else
        return -Result;
}

static int XParseGeometry (
    const char *string,
    int *x,
    int *y,
    unsigned int *width,    /* RETURN */
    unsigned int *height)    /* RETURN */
{
    int mask = NoValue;
    register char *strind;
    unsigned int tempWidth = 0, tempHeight = 0;
    int tempX = 0, tempY = 0;
    char *nextCharacter;

    if ( (string == NULL) || (*string == '\0'))
      return mask;
    if (*string == '=')
        string++;  /* ignore possible '=' at beg of geometry spec */

    strind = (char *)string;
    if (*strind != '+' && *strind != '-' && *strind != 'x') {
        tempWidth = ReadInteger(strind, &nextCharacter);
        if (strind == nextCharacter)
            return 0;
        strind = nextCharacter;
        mask |= WidthValue;
    }

    if (*strind == 'x' || *strind == 'X') {
        strind++;
        tempHeight = ReadInteger(strind, &nextCharacter);
        if (strind == nextCharacter)
            return 0;
        strind = nextCharacter;
        mask |= HeightValue;
    }

    if ((*strind == '+') || (*strind == '-')) {
        if (*strind == '-') {
            strind++;
            tempX = -ReadInteger(strind, &nextCharacter);
            if (strind == nextCharacter)
                return 0;
            strind = nextCharacter;
            mask |= XNegative;
        }
        else
        {
            strind++;
            tempX = ReadInteger(strind, &nextCharacter);
            if (strind == nextCharacter)
                return 0;
            strind = nextCharacter;
        }
        mask |= XValue;
        if ((*strind == '+') || (*strind == '-')) {
            if (*strind == '-') {
                strind++;
                tempY = -ReadInteger(strind, &nextCharacter);
                if (strind == nextCharacter)
                    return 0;
                strind = nextCharacter;
                mask |= YNegative;
            }
            else
            {
                strind++;
                tempY = ReadInteger(strind, &nextCharacter);
                if (strind == nextCharacter)
                    return 0;
                strind = nextCharacter;
            }
            mask |= YValue;
        }
    }

    /* If strind isn't at the end of the string the it's an invalid
       geometry specification. */

    if (*strind != '\0') return 0;

    if (mask & XValue)
        *x = tempX;
    if (mask & YValue)
        *y = tempY;
    if (mask & WidthValue)
        *width = tempWidth;
    if (mask & HeightValue)
        *height = tempHeight;
    return mask;
}
#endif

/* -- INTERFACE FUNCTIONS -------------------------------------------------- */

/*
 * Perform initialization. This usually happens on the program startup
 * and restarting after glutMainLoop termination...
 */
void FGAPIENTRY glutInit( int* pargc, char** argv )
{
    char* displayName = NULL;
    char* geometry = NULL;
    int i, j, argc = *pargc;

    if( fgState.Initialised )
        fgError( "illegal glutInit() reinitialization attempt" );

    if (pargc && *pargc && argv && *argv && **argv)
    {
        fgState.ProgramName = strdup (*argv);

        if( !fgState.ProgramName )
            fgError ("Could not allocate space for the program's name.");
    }

    fgCreateStructure( );

    fgElapsedTime( );

    /* b2Assert if GLUT_FPS env var is set */
#if !TARGET_HOST_WINCE
    {
        const char *fps = getenv( "GLUT_FPS" );
        if( fps )
        {
            int interval;
            sscanf( fps, "%d", &interval );

            if( interval <= 0 )
                fgState.FPSInterval = 5000;  /* 5000 millisecond default */
            else
                fgState.FPSInterval = interval;
        }
    }

    displayName = getenv( "DISPLAY");

    for( i = 1; i < argc; i++ )
    {
        if( strcmp( argv[ i ], "-display" ) == 0 )
        {
            if( ++i >= argc )
                fgError( "-display parameter must be followed by display name" );

            displayName = argv[ i ];

            argv[ i - 1 ] = NULL;
            argv[ i     ] = NULL;
            ( *pargc ) -= 2;
        }
        else if( strcmp( argv[ i ], "-geometry" ) == 0 )
        {
            if( ++i >= argc )
                fgError( "-geometry parameter must be followed by window "
                         "geometry settings" );

            geometry = argv[ i ];

            argv[ i - 1 ] = NULL;
            argv[ i     ] = NULL;
            ( *pargc ) -= 2;
        }
        else if( strcmp( argv[ i ], "-direct" ) == 0)
        {
            if( fgState.DirectContext == GLUT_FORCE_INDIRECT_CONTEXT )
                fgError( "parameters ambiguity, -direct and -indirect "
                    "cannot be both specified" );

            fgState.DirectContext = GLUT_FORCE_DIRECT_CONTEXT;
            argv[ i ] = NULL;
            ( *pargc )--;
        }
        else if( strcmp( argv[ i ], "-indirect" ) == 0 )
        {
            if( fgState.DirectContext == GLUT_FORCE_DIRECT_CONTEXT )
                fgError( "parameters ambiguity, -direct and -indirect "
                    "cannot be both specified" );

            fgState.DirectContext = GLUT_FORCE_INDIRECT_CONTEXT;
            argv[ i ] = NULL;
            (*pargc)--;
        }
        else if( strcmp( argv[ i ], "-iconic" ) == 0 )
        {
            fgState.ForceIconic = GL_TRUE;
            argv[ i ] = NULL;
            ( *pargc )--;
        }
        else if( strcmp( argv[ i ], "-gldebug" ) == 0 )
        {
            fgState.GLDebugSwitch = GL_TRUE;
            argv[ i ] = NULL;
            ( *pargc )--;
        }
        else if( strcmp( argv[ i ], "-sync" ) == 0 )
        {
            fgState.XSyncSwitch = GL_TRUE;
            argv[ i ] = NULL;
            ( *pargc )--;
        }
    }

    /* Compact {argv}. */
    for( i = j = 1; i < *pargc; i++, j++ )
    {
        /* Guaranteed to end because there are "*pargc" arguments left */
        while ( argv[ j ] == NULL )
            j++;
        if ( i != j )
            argv[ i ] = argv[ j ];
    }

#endif /* TARGET_HOST_WINCE */

    /*
     * Have the display created now. If there wasn't a "-display"
     * in the program arguments, we will use the DISPLAY environment
     * variable for opening the X display (see code above):
     */
    fghInitialize( displayName );

    /*
     * Geometry parsing deffered until here because we may need the screen
     * size.
     */

    if (geometry )
    {
        unsigned int parsedWidth, parsedHeight;
        int mask = XParseGeometry( geometry,
                                   &fgState.Position.X, &fgState.Position.Y,
                                   &parsedWidth, &parsedHeight );
        /* TODO: Check for overflow? */
        fgState.Size.X = parsedWidth;
        fgState.Size.Y = parsedHeight;

        if( (mask & (WidthValue|HeightValue)) == (WidthValue|HeightValue) )
            fgState.Size.Use = GL_TRUE;

        if( mask & XNegative )
            fgState.Position.X += fgDisplay.ScreenWidth - fgState.Size.X;

        if( mask & YNegative )
            fgState.Position.Y += fgDisplay.ScreenHeight - fgState.Size.Y;

        if( (mask & (XValue|YValue)) == (XValue|YValue) )
            fgState.Position.Use = GL_TRUE;
    }
}

/*
 * Sets the default initial window position for new windows
 */
void FGAPIENTRY glutInitWindowPosition( int x, int y )
{
    fgState.Position.X = x;
    fgState.Position.Y = y;

    if( ( x >= 0 ) && ( y >= 0 ) )
        fgState.Position.Use = GL_TRUE;
    else
        fgState.Position.Use = GL_FALSE;
}

/*
 * Sets the default initial window size for new windows
 */
void FGAPIENTRY glutInitWindowSize( int width, int height )
{
    fgState.Size.X = width;
    fgState.Size.Y = height;

    if( ( width > 0 ) && ( height > 0 ) )
        fgState.Size.Use = GL_TRUE;
    else
        fgState.Size.Use = GL_FALSE;
}

/*
 * Sets the default display mode for all new windows
 */
void FGAPIENTRY glutInitDisplayMode( unsigned int displayMode )
{
    /* We will make use of this value when creating a new OpenGL context... */
    fgState.DisplayMode = displayMode;
}


/* -- INIT DISPLAY STRING PARSING ------------------------------------------ */

static char* Tokens[] =
{
    "alpha", "acca", "acc", "blue", "buffer", "conformant", "depth", "double",
    "green", "index", "num", "red", "rgba", "rgb", "luminance", "stencil",
    "single", "stereo", "samples", "slow", "win32pdf", "win32pfd", "xvisual",
    "xstaticgray", "xgrayscale", "xstaticcolor", "xpseudocolor",
    "xtruecolor", "xdirectcolor",
    "xstaticgrey", "xgreyscale", "xstaticcolour", "xpseudocolour",
    "xtruecolour", "xdirectcolour", "borderless", "aux"
};
#define NUM_TOKENS             (sizeof(Tokens) / sizeof(*Tokens))

void FGAPIENTRY glutInitDisplayString( const char* displayMode )
{
    int glut_state_flag = 0 ;
    /*
     * Unpack a lot of options from a character string.  The options are
     * delimited by blanks or tabs.
     */
    char *token ;
    int len = strlen ( displayMode );
    char *buffer = (char *)malloc ( (len+1) * sizeof(char) );
    memcpy ( buffer, displayMode, len );
    buffer[len] = '\0';

    token = strtok ( buffer, " \t" );
    while ( token )
    {
        /* Process this token */
        int i ;
        for ( i = 0; i < NUM_TOKENS; i++ )
        {
            if ( strcmp ( token, Tokens[i] ) == 0 ) break ;
        }

        switch ( i )
        {
        case 0 :  /* "alpha":  Alpha color buffer precision in bits */
            glut_state_flag |= GLUT_ALPHA ;  /* Somebody fix this for me! */
            break ;

        case 1 :  /* "acca":  Red, green, blue, and alpha accumulation buffer
                     precision in bits */
            break ;

        case 2 :  /* "acc":  Red, green, and blue accumulation buffer precision
                     in bits with zero bits alpha */
            glut_state_flag |= GLUT_ACCUM ;  /* Somebody fix this for me! */
            break ;

        case 3 :  /* "blue":  Blue color buffer precision in bits */
            break ;

        case 4 :  /* "buffer":  Number of bits in the color index color buffer
                   */
            break ;

        case 5 :  /* "conformant":  Boolean indicating if the frame buffer
                     configuration is conformant or not */
            break ;

        case 6 : /* "depth":  Number of bits of precsion in the depth buffer */
            glut_state_flag |= GLUT_DEPTH ;  /* Somebody fix this for me! */
            break ;

        case 7 :  /* "double":  Boolean indicating if the color buffer is
                     double buffered */
            glut_state_flag |= GLUT_DOUBLE ;
            break ;

        case 8 :  /* "green":  Green color buffer precision in bits */
            break ;

        case 9 :  /* "index":  Boolean if the color model is color index or not
                   */
            glut_state_flag |= GLUT_INDEX ;
            break ;

        case 10 :  /* "num":  A special capability  name indicating where the
                      value represents the Nth frame buffer configuration
                      matching the description string */
            break ;

        case 11 :  /* "red":  Red color buffer precision in bits */
            break ;

        case 12 :  /* "rgba":  Number of bits of red, green, blue, and alpha in
                      the RGBA color buffer */
            glut_state_flag |= GLUT_RGBA ;  /* Somebody fix this for me! */
            break ;

        case 13 :  /* "rgb":  Number of bits of red, green, and blue in the
                      RGBA color buffer with zero bits alpha */
            glut_state_flag |= GLUT_RGB ;  /* Somebody fix this for me! */
            break ;

        case 14 :  /* "luminance":  Number of bits of red in the RGBA and zero
                      bits of green, blue (alpha not specified) of color buffer
                      precision */
            glut_state_flag |= GLUT_LUMINANCE ; /* Somebody fix this for me! */
            break ;

        case 15 :  /* "stencil":  Number of bits in the stencil buffer */
            glut_state_flag |= GLUT_STENCIL;  /* Somebody fix this for me! */
            break ;

        case 16 :  /* "single":  Boolean indicate the color buffer is single
                      buffered */
            glut_state_flag |= GLUT_SINGLE ;
            break ;

        case 17 :  /* "stereo":  Boolean indicating the color buffer supports
                      OpenGL-style stereo */
            glut_state_flag |= GLUT_STEREO ;
            break ;

        case 18 :  /* "samples":  Indicates the number of multisamples to use
                      based on GLX's SGIS_multisample extension (for
                      antialiasing) */
            glut_state_flag |= GLUT_MULTISAMPLE ; /*Somebody fix this for me!*/
            break ;

        case 19 :  /* "slow":  Boolean indicating if the frame buffer
                      configuration is slow or not */
            break ;

        case 20 :  /* "win32pdf": (incorrect spelling but was there before */
        case 21 :  /* "win32pfd":  matches the Win32 Pixel Format Descriptor by
                      number */
#if TARGET_HOST_WIN32
#endif
            break ;

        case 22 :  /* "xvisual":  matches the X visual ID by number */
#if TARGET_HOST_UNIX_X11
#endif
            break ;

        case 23 :  /* "xstaticgray": */
        case 29 :  /* "xstaticgrey":  boolean indicating if the frame buffer
                      configuration's X visual is of type StaticGray */
#if TARGET_HOST_UNIX_X11
#endif
            break ;

        case 24 :  /* "xgrayscale": */
        case 30 :  /* "xgreyscale":  boolean indicating if the frame buffer
                      configuration's X visual is of type GrayScale */
#if TARGET_HOST_UNIX_X11
#endif
            break ;

        case 25 :  /* "xstaticcolor": */
        case 31 :  /* "xstaticcolour":  boolean indicating if the frame buffer
                      configuration's X visual is of type StaticColor */
#if TARGET_HOST_UNIX_X11
#endif
            break ;

        case 26 :  /* "xpseudocolor": */
        case 32 :  /* "xpseudocolour":  boolean indicating if the frame buffer
                      configuration's X visual is of type PseudoColor */
#if TARGET_HOST_UNIX_X11
#endif
            break ;

        case 27 :  /* "xtruecolor": */
        case 33 :  /* "xtruecolour":  boolean indicating if the frame buffer
                      configuration's X visual is of type TrueColor */
#if TARGET_HOST_UNIX_X11
#endif
            break ;

        case 28 :  /* "xdirectcolor": */
        case 34 :  /* "xdirectcolour":  boolean indicating if the frame buffer
                      configuration's X visual is of type DirectColor */
#if TARGET_HOST_UNIX_X11
#endif
            break ;

        case 35 :  /* "borderless":  windows should not have borders */
#if TARGET_HOST_UNIX_X11
#endif
            break ;

        case 36 :  /* "aux":  some number of aux buffers */
            glut_state_flag |= GLUT_AUX1;
            break ;

        case 37 :  /* Unrecognized */
            fgWarning ( "WARNING - Display string token not recognized:  %s",
                        token );
            break ;
        }

        token = strtok ( NULL, " \t" );
    }

    free ( buffer );

    /* We will make use of this value when creating a new OpenGL context... */
    fgState.DisplayMode = glut_state_flag;
}

/*** END OF FILE ***/
