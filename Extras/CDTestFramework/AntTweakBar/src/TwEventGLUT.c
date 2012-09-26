//  ---------------------------------------------------------------------------
//
//  @file       TwEventGLUT.c
//  @brief      Helper: 
//              translate and re-send mouse and keyboard events 
//              from GLUT event callbacks to AntTweakBar
//  
//  @author     Philippe Decaudin - http://www.antisphere.com
//  @date       2006/05/10
//  @license    This file is part of the AntTweakBar library.
//              For conditions of distribution and use, see License.txt
//
//  ---------------------------------------------------------------------------


#define GLUT_NO_LIB_PRAGMA          // we do not want to force linkage with glut
#ifdef _MSC_VER
#   pragma warning(disable: 4505)   // glut generates 'unreferenced function' warnings
#   pragma warning(disable: 4100)   // unreferenced parameter
#endif // _MSC_VER

// #include <GL/glut.h>
#include "MiniGLUT.h" // a subset of glut.h needed to compile TwEventGLUT.c
// note: AntTweakBar.dll does not need to link with GLUT, 
// it just needs some definitions for its helper functions.

#include <AntTweakBar.h>


int TW_GLUT_CALL TwEventMouseButtonGLUT(int glutButton, int glutState, int mouseX, int mouseY)
{
    TwMouseAction action = (glutState==GLUT_DOWN) ? TW_MOUSE_PRESSED : TW_MOUSE_RELEASED;

    TwMouseMotion(mouseX, mouseY);
    switch( glutButton )
    {
    case GLUT_LEFT_BUTTON:
        return TwMouseButton(action, TW_MOUSE_LEFT);
    case GLUT_RIGHT_BUTTON:
        return TwMouseButton(action, TW_MOUSE_RIGHT);
    case GLUT_MIDDLE_BUTTON:
        return TwMouseButton(action, TW_MOUSE_MIDDLE);
    default:
        return 0;
    }
}

int TW_GLUT_CALL TwEventMouseMotionGLUT(int mouseX, int mouseY)
{
    return TwMouseMotion(mouseX, mouseY);
}


//  GLUT does not send modifiers state to 'Key' and 'Special' callbacks,
//  and we cannot call glutGetModifiers here because we do not want to link
//  AntTweakBar with glut, so the following function is used to store
//  a pointer to the glutGetModifiers function of the calling application.
//  It must be called at initialisation of the application.

int (TW_CALL *g_GLUTGetModifiers)(void) = NULL;

int TW_CALL TwGLUTModifiersFunc(int (TW_CALL *glutGetModifiersFunc)(void))
{
    g_GLUTGetModifiers = glutGetModifiersFunc;
    return (g_GLUTGetModifiers==NULL) ? 0 : 1;
}


int TW_GLUT_CALL TwEventKeyboardGLUT(unsigned char glutKey, int mouseX, int mouseY)
{
    int kmod = 0;

    if( g_GLUTGetModifiers!=NULL )
    {
        int glutMod = g_GLUTGetModifiers();

        if( glutMod&GLUT_ACTIVE_SHIFT )
            kmod |= TW_KMOD_SHIFT;
        if( glutMod&GLUT_ACTIVE_CTRL )
            kmod |= TW_KMOD_CTRL;
        if( glutMod&GLUT_ACTIVE_ALT )
            kmod |= TW_KMOD_ALT;
    }

    if( (kmod&TW_KMOD_CTRL) && (glutKey>0 && glutKey<27) )  // CTRL special case
        glutKey += 'a'-1;

    return TwKeyPressed((int)glutKey, kmod);
}


int TW_GLUT_CALL TwEventSpecialGLUT(int glutKey, int mouseX, int mouseY)
{
    int k = 0, kmod = 0;

    if( g_GLUTGetModifiers!=NULL )
    {
        int glutMod = g_GLUTGetModifiers();

        if( glutMod&GLUT_ACTIVE_SHIFT )
            kmod |= TW_KMOD_SHIFT;
        if( glutMod&GLUT_ACTIVE_CTRL )
            kmod |= TW_KMOD_CTRL;
        if( glutMod&GLUT_ACTIVE_ALT )
            kmod |= TW_KMOD_ALT;
    }

    if( glutKey>=GLUT_KEY_F1 && glutKey<=GLUT_KEY_F12 )
        k = TW_KEY_F1 + (glutKey-GLUT_KEY_F1);
    else
    {
        switch( glutKey )
        {
        case GLUT_KEY_LEFT:
            k = TW_KEY_LEFT;
            break;
        case GLUT_KEY_UP:
            k = TW_KEY_UP;
            break;
        case GLUT_KEY_RIGHT:
            k = TW_KEY_RIGHT;
            break;
        case GLUT_KEY_DOWN:
            k = TW_KEY_DOWN;
            break;
        case GLUT_KEY_PAGE_UP:
            k = TW_KEY_PAGE_UP;
            break;
        case GLUT_KEY_PAGE_DOWN:
            k = TW_KEY_PAGE_DOWN;
            break;
        case GLUT_KEY_HOME:
            k = TW_KEY_HOME;
            break;
        case GLUT_KEY_END:
            k = TW_KEY_END;
            break;
        case GLUT_KEY_INSERT:
            k = TW_KEY_INSERT;
            break;
        }
    }

    if( k>0 && k<TW_KEY_LAST )
        return TwKeyPressed(k, kmod);
    else
        return 0;
}


