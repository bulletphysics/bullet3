//  ---------------------------------------------------------------------------
//
//  @file       TwEventX11.c
//  @brief      Helper: 
//              translate and forward mouse and keyboard events 
//              from X11 to AntTweakBar
//  
//  @contrib    Greg Popovitch
//  @license    This file is part of the AntTweakBar library.
//              For conditions of distribution and use, see License.txt
//
//  ---------------------------------------------------------------------------

#include <X11/Xlib.h>
#include <X11/keysym.h>
#include <X11/Xutil.h>
#include <AntTweakBar.h>

static int s_KMod = 0;
const int buff_sz = 80;

// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
static int _XKeyRelease(XEvent *event)
{
    KeySym keysym;
    char buffer[buff_sz];
        
    XLookupString((XKeyEvent *)event, buffer, buff_sz, &keysym, 0);

    switch (keysym)
    {
    case XK_Control_L:
    case XK_Control_R: s_KMod  &= ~TW_KMOD_CTRL;  break;

    case XK_Shift_L:
    case XK_Shift_R:   s_KMod &= ~TW_KMOD_SHIFT;  break;

    case XK_Alt_L:
    case XK_Alt_R:     s_KMod &= ~TW_KMOD_ALT;    break;
    }
    return 0;
}

// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
static int _XKeyPress(XEvent *event)
{
    int modifiers = 0;  // modifiers sent to AntTweakBar
    int k = 0;          // key sent to AntTweakBar
    KeySym keysym;
    char buffer[buff_sz];
        
    int num_char = XLookupString((XKeyEvent *)event, buffer, buff_sz, &keysym, 0);

    if (event->xkey.state & ControlMask)
        modifiers |= TW_KMOD_CTRL;
    if (event->xkey.state & ShiftMask)
        modifiers |= TW_KMOD_SHIFT;
    if (event->xkey.state & Mod1Mask)
        modifiers |= TW_KMOD_ALT;

    switch (keysym)
    {
    case XK_Control_L:
    case XK_Control_R: s_KMod |= TW_KMOD_CTRL;  break;

    case XK_Shift_L:
    case XK_Shift_R:   s_KMod |= TW_KMOD_SHIFT; break;

    case XK_Alt_L:
    case XK_Alt_R:     s_KMod |= TW_KMOD_ALT;   break;

    case XK_Escape:    k = TW_KEY_ESCAPE;    break;
    case XK_Help:      k = TW_KEY_F1;        break;
    case XK_F1:        k = TW_KEY_F1;        break;
    case XK_F2:        k = TW_KEY_F2;        break;
    case XK_F3:        k = TW_KEY_F3;        break;
    case XK_F4:        k = TW_KEY_F4;        break;
    case XK_F5:        k = TW_KEY_F5;        break;
    case XK_F6:        k = TW_KEY_F6;        break;
    case XK_F7:        k = TW_KEY_F7;        break;
    case XK_F8:        k = TW_KEY_F8;        break;
    case XK_F9:        k = TW_KEY_F9;        break;
    case XK_F10:       k = TW_KEY_F10;       break;
    case XK_F11:       k = TW_KEY_F11;       break;
    case XK_F12:       k = TW_KEY_F12;       break;
    case XK_Up:        k = TW_KEY_UP;        break;
    case XK_Down:      k = TW_KEY_DOWN;      break;
    case XK_Right:     k = TW_KEY_RIGHT;     break;
    case XK_Left:      k = TW_KEY_LEFT;      break;
    case XK_Return:    k = TW_KEY_RETURN;    break;
    case XK_Insert:    k = TW_KEY_INSERT;    break;
    case XK_Delete:    k = TW_KEY_DELETE;    break;
    case XK_BackSpace: k = TW_KEY_BACKSPACE; break;
    case XK_Home:      k = TW_KEY_HOME;      break;
    case XK_Tab:       k = TW_KEY_TAB;       break;
    case XK_End:       k = TW_KEY_END;       break;

#ifdef XK_Enter
    case XK_Enter:     k = TW_KEY_RETURN;    break;
#endif

#ifdef XK_KP_Home
    case XK_KP_Home:   k = TW_KEY_HOME;      break;
    case XK_KP_End:    k = TW_KEY_END;       break;
    case XK_KP_Delete: k = TW_KEY_DELETE;    break;
#endif

#ifdef XK_KP_Up
    case XK_KP_Up:     k = TW_KEY_UP;        break;
    case XK_KP_Down:   k = TW_KEY_DOWN;      break;
    case XK_KP_Right:  k = TW_KEY_RIGHT;     break;
    case XK_KP_Left:   k = TW_KEY_LEFT;      break;
#endif

#ifdef XK_KP_Page_Up
    case XK_KP_Page_Up:   k = TW_KEY_PAGE_UP;    break;
    case XK_KP_Page_Down: k = TW_KEY_PAGE_DOWN;  break;
#endif

#ifdef XK_KP_Tab
    case XK_KP_Tab:    k = TW_KEY_TAB;       break;
#endif

    default:
        if (0)
        {
            // should we do that, or rely on the buffer (see code below)
            if (keysym > 12 && keysym < 127) 
                k = keysym;
        }
        break;
    }  
        
    if (k == 0 && num_char)
    {
        int i, handled = 0;
        for (i=0; i<num_char; ++i)
            if (TwKeyPressed(buffer[i], modifiers))
                handled = 1;
        return handled;
    }

    // if we have a valid key, send to AntTweakBar
    // -------------------------------------------
    return (k > 0) ? TwKeyPressed(k, modifiers) : 0;
}

// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
static int _XButtonEvent(XEvent *event)
{
    TwMouseAction action = (event->type == ButtonPress) ? TW_MOUSE_PRESSED : TW_MOUSE_RELEASED;
    XButtonEvent *xbe = (XButtonEvent *)event;
    return TwMouseButton(action, xbe->button);
}
    
// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
static int _XConfigureEvent(XEvent *event)
{
    XConfigureEvent *xce = (XConfigureEvent *)event;
    TwWindowSize(xce->width, xce->height);
    return 0;
}

// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
static int _XMotionEvent(XEvent *event)
{
    XMotionEvent *xme = (XMotionEvent *)event;
    return TwMouseMotion(xme->x, xme->y);
}

// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
TW_API int TW_CDECL_CALL TwEventX11(void *xevent)
{
    XEvent *event = (XEvent *)xevent;
    
    switch (event->type)
    {
    case KeyPress:    
        return _XKeyPress(xevent);

    case KeyRelease:  
        return 0; // _XKeyRelease(xevent);

    case ButtonPress: 
    case ButtonRelease: 
        return _XButtonEvent(xevent);

    case MotionNotify:
        return _XMotionEvent(xevent);

    case ConfigureNotify:
        return _XConfigureEvent(xevent);

    default:  
        break;
    }
    return 0;
}
        
