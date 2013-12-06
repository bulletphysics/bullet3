//  ---------------------------------------------------------------------------
//
//  @file       TwEventGLFW.c
//  @brief      Helper: 
//              translate and re-send mouse and keyboard events 
//              from GLFW event callbacks to AntTweakBar
//  
//  @author     Philippe Decaudin - http://www.antisphere.com
//  @license    This file is part of the AntTweakBar library.
//              For conditions of distribution and use, see License.txt
//
//  ---------------------------------------------------------------------------

// #include <GL/glfw.h>
#include "MiniGLFW.h" // a subset of GLFW.h needed to compile TwEventGLFW.c
// note: AntTweakBar.dll does not need to link with GLFW, 
// it just needs some definitions for its helper functions.

#include <AntTweakBar.h>


int TW_CALL TwEventMouseButtonGLFW(int glfwButton, int glfwAction)
{
    int handled = 0;
    TwMouseAction action = (glfwAction==GLFW_PRESS) ? TW_MOUSE_PRESSED : TW_MOUSE_RELEASED;

    if( glfwButton==GLFW_MOUSE_BUTTON_LEFT )
        handled = TwMouseButton(action, TW_MOUSE_LEFT);
    else if( glfwButton==GLFW_MOUSE_BUTTON_RIGHT )
        handled = TwMouseButton(action, TW_MOUSE_RIGHT);
    else if( glfwButton==GLFW_MOUSE_BUTTON_MIDDLE )
        handled = TwMouseButton(action, TW_MOUSE_MIDDLE);

    return handled;
}


int g_KMod = 0;


int TW_CALL TwEventKeyGLFW(int glfwKey, int glfwAction)
{
    int handled = 0;

    // Register of modifiers state
    if( glfwAction==GLFW_PRESS )
    {
        switch( glfwKey )
        {
        case GLFW_KEY_LSHIFT:
        case GLFW_KEY_RSHIFT:
            g_KMod |= TW_KMOD_SHIFT;
            break;
        case GLFW_KEY_LCTRL:
        case GLFW_KEY_RCTRL:
            g_KMod |= TW_KMOD_CTRL;
            break;
        case GLFW_KEY_LALT:
        case GLFW_KEY_RALT:
            g_KMod |= TW_KMOD_ALT;
            break;
        }
    }
    else
    {
        switch( glfwKey )
        {
        case GLFW_KEY_LSHIFT:
        case GLFW_KEY_RSHIFT:
            g_KMod &= ~TW_KMOD_SHIFT;
            break;
        case GLFW_KEY_LCTRL:
        case GLFW_KEY_RCTRL:
            g_KMod &= ~TW_KMOD_CTRL;
            break;
        case GLFW_KEY_LALT:
        case GLFW_KEY_RALT:
            g_KMod &= ~TW_KMOD_ALT;
            break;
        }
    }

    // Process key pressed
    if( glfwAction==GLFW_PRESS )
    {
        int mod = g_KMod;
        int testkp = ((mod&TW_KMOD_CTRL) || (mod&TW_KMOD_ALT)) ? 1 : 0;

        if( (mod&TW_KMOD_CTRL) && glfwKey>0 && glfwKey<GLFW_KEY_SPECIAL )   // CTRL cases
            handled = TwKeyPressed(glfwKey, mod);
        else if( glfwKey>=GLFW_KEY_SPECIAL )
        {
            int k = 0;

            if( glfwKey>=GLFW_KEY_F1 && glfwKey<=GLFW_KEY_F15 )
                k = TW_KEY_F1 + (glfwKey-GLFW_KEY_F1);
            else if( testkp && glfwKey>=GLFW_KEY_KP_0 && glfwKey<=GLFW_KEY_KP_9 )
                k = '0' + (glfwKey-GLFW_KEY_KP_0);
            else
            {
                switch( glfwKey )
                {
                case GLFW_KEY_ESC:
                    k = TW_KEY_ESCAPE;
                    break;
                case GLFW_KEY_UP:
                    k = TW_KEY_UP;
                    break;
                case GLFW_KEY_DOWN:
                    k = TW_KEY_DOWN;
                    break;
                case GLFW_KEY_LEFT:
                    k = TW_KEY_LEFT;
                    break;
                case GLFW_KEY_RIGHT:
                    k = TW_KEY_RIGHT;
                    break;
                case GLFW_KEY_TAB:
                    k = TW_KEY_TAB;
                    break;
                case GLFW_KEY_ENTER:
                    k = TW_KEY_RETURN;
                    break;
                case GLFW_KEY_BACKSPACE:
                    k = TW_KEY_BACKSPACE;
                    break;
                case GLFW_KEY_INSERT:
                    k = TW_KEY_INSERT;
                    break;
                case GLFW_KEY_DEL:
                    k = TW_KEY_DELETE;
                    break;
                case GLFW_KEY_PAGEUP:
                    k = TW_KEY_PAGE_UP;
                    break;
                case GLFW_KEY_PAGEDOWN:
                    k = TW_KEY_PAGE_DOWN;
                    break;
                case GLFW_KEY_HOME:
                    k = TW_KEY_HOME;
                    break;
                case GLFW_KEY_END:
                    k = TW_KEY_END;
                    break;
                case GLFW_KEY_KP_ENTER:
                    k = TW_KEY_RETURN;
                    break;
                case GLFW_KEY_KP_DIVIDE:
                    if( testkp )
                        k = '/';
                    break;
                case GLFW_KEY_KP_MULTIPLY:
                    if( testkp )
                        k = '*';
                    break;
                case GLFW_KEY_KP_SUBTRACT:
                    if( testkp )
                        k = '-';
                    break;
                case GLFW_KEY_KP_ADD:
                    if( testkp )
                        k = '+';
                    break;
                case GLFW_KEY_KP_DECIMAL:
                    if( testkp )
                        k = '.';
                    break;
                case GLFW_KEY_KP_EQUAL:
                    if( testkp )
                        k = '=';
                    break;
                }
            }

            if( k>0 )
                handled = TwKeyPressed(k, mod);
        }
    }

    return handled;
}


int TW_CALL TwEventCharGLFW(int glfwChar, int glfwAction)
{
    if( glfwAction==GLFW_PRESS && (glfwChar & 0xff00)==0 )
        return TwKeyPressed(glfwChar, g_KMod);

    return 0;
}

// functions with __cdecl calling convension
TW_API int TW_CDECL_CALL TwEventMouseButtonGLFWcdecl(int glfwButton, int glfwAction)
{
    return TwEventMouseButtonGLFW(glfwButton, glfwAction);
}
TW_API int TW_CDECL_CALL TwEventKeyGLFWcdecl(int glfwKey, int glfwAction)
{
    return TwEventKeyGLFW(glfwKey, glfwAction);
}
TW_API int TW_CDECL_CALL TwEventCharGLFWcdecl(int glfwChar, int glfwAction)
{
    return TwEventCharGLFW(glfwChar, glfwAction);
}
TW_API int TW_CDECL_CALL TwEventMousePosGLFWcdecl(int mouseX, int mouseY)
{
    return TwMouseMotion(mouseX, mouseY);
}
TW_API int TW_CDECL_CALL TwEventMouseWheelGLFWcdecl(int wheelPos)
{
    return TwMouseWheel(wheelPos);
}
