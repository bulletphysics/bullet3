//  ---------------------------------------------------------------------------
//
//  @file       AntTweakBar.h
//
//  @brief      AntTweakBar is a light and intuitive graphical user interface 
//              that can be readily integrated into OpenGL and DirectX 
//              applications in order to interactively tweak them.
//
//  @author     Philippe Decaudin - http://www.antisphere.com
//  @date       2005/09/20
//
//  @doc        http://www.antisphere.com/Wiki/tools:anttweakbar
//
//  @license    This file is part of the AntTweakBar library.
//              Copyright © 2005, 2006 Philippe Decaudin.
//              AntTweakBar is a free software released under the zlib license.
//              For conditions of distribution and use, see License.txt
//
//  note:       TAB=4
//
//  ---------------------------------------------------------------------------


#if !defined TW_INCLUDED
#define TW_INCLUDED

#include <stddef.h>

#define TW_VERSION  104     // Version Mmm : M=Major mm=minor (e.g., 102 is version 1.02)


#ifdef  __cplusplus
    extern "C" {
#endif  // __cplusplus


//  ---------------------------------------------------------------------------
//  OS specific definitions
//  ---------------------------------------------------------------------------
 

#ifdef _WIN32
#   define TW_CALL          __stdcall
#   define TW_EXPORT_API    __declspec(dllexport)
#   define TW_IMPORT_API    __declspec(dllimport)
#else
#   define TW_CALL
#   define TW_EXPORT_API
#   define TW_IMPORT_API
#endif


#if defined TW_EXPORTS
#   define TW_API   TW_EXPORT_API
#elif defined TW_STATIC
#   define TW_API
#   if defined(_MSC_VER) && !defined(TW_NO_LIB_PRAGMA)
#       pragma comment(lib, "AntTweakBarStatic")
#   endif
#else
#   define TW_API   TW_IMPORT_API
#   if defined(_MSC_VER) && !defined(TW_NO_LIB_PRAGMA)
#       pragma comment(lib, "AntTweakBar")
#   endif
#endif


//  ---------------------------------------------------------------------------
//  Bar functions and definitions
//  ---------------------------------------------------------------------------


typedef struct CTwBar TwBar;    // structure CTwBar is not exposed.

TW_API TwBar *      TW_CALL TwNewBar(const char *name);
TW_API int          TW_CALL TwDeleteBar(TwBar *bar);
TW_API int          TW_CALL TwDeleteAllBars();
TW_API int          TW_CALL TwSetTopBar(const TwBar *bar);
TW_API TwBar *      TW_CALL TwGetTopBar();
TW_API const char * TW_CALL TwGetBarName(TwBar *bar);


//  ---------------------------------------------------------------------------
//  Var functions and definitions
//  ---------------------------------------------------------------------------


typedef enum ETwType
{
    TW_TYPE_UNDEF   = 0,
#ifdef __cplusplus
    TW_TYPE_BOOLCPP = 1,
#endif // __cplusplus
    TW_TYPE_BOOL8   = 2,
    TW_TYPE_BOOL16,
    TW_TYPE_BOOL32,
    TW_TYPE_CHAR,
    TW_TYPE_INT8,
    TW_TYPE_UINT8,
    TW_TYPE_INT16,
    TW_TYPE_UINT16,
    TW_TYPE_INT32,
    TW_TYPE_UINT32,
    TW_TYPE_FLOAT,
    TW_TYPE_DOUBLE,
    TW_TYPE_COLOR32,        // 32 bits color. Order is RGBA if API is OpenGL, and inversed if API is DirectX (can be modified by defining 'order=...')
    TW_TYPE_COLOR3F,        // 3 floats color. Order is RGB.
    TW_TYPE_COLOR4F,        // 4 floats color. Order is RGBA.
} TwType;
typedef void (TW_CALL * TwSetVarCallback)(const void *value, void *clientData);
typedef void (TW_CALL * TwGetVarCallback)(void *value, void *clientData);
typedef void (TW_CALL * TwButtonCallback)(void *clientData);

TW_API int      TW_CALL TwAddVarRW(TwBar *bar, const char *name, TwType type, void *var, const char *def);
TW_API int      TW_CALL TwAddVarRO(TwBar *bar, const char *name, TwType type, const void *var, const char *def);
TW_API int      TW_CALL TwAddVarCB(TwBar *bar, const char *name, TwType type, TwSetVarCallback setCallback, TwGetVarCallback getCallback, void *clientData, const char *def);
TW_API int      TW_CALL TwAddButton(TwBar *bar, const char *name, TwButtonCallback callback, void *clientData, const char *def);
TW_API int      TW_CALL TwRemoveVar(TwBar *bar, const char *name);
TW_API int      TW_CALL TwRemoveAllVars(TwBar *bar);

typedef struct CTwEnumVal
{
    int           Value;
    const char *  Label;
} TwEnumVal;
typedef struct CTwStructMember
{
    const char *  Name;
    TwType        Type;
    size_t        Offset;
    const char *  DefString;
} TwStructMember;
typedef void (TW_CALL * TwSummaryCallback)(char *summaryString, size_t summaryMaxLength, const void *value, void *clientData);

TW_API int      TW_CALL TwDefine(const char *def);
TW_API TwType   TW_CALL TwDefineEnum(const char *name, const TwEnumVal *enumValues, unsigned int nbValues);
TW_API TwType   TW_CALL TwDefineStruct(const char *name, const TwStructMember *structMembers, unsigned int nbMembers, size_t structSize, TwSummaryCallback summaryCallback, void *summaryClientData);


//  ---------------------------------------------------------------------------
//  Managment functions and definitions
//  ---------------------------------------------------------------------------


typedef enum ETwGraphAPI
{
    TW_OPENGL           = 1,
    TW_DIRECT3D9        = 2,
} TwGraphAPI;

TW_API int      TW_CALL TwInit(TwGraphAPI graphAPI, void *device);
TW_API int      TW_CALL TwTerminate();

TW_API int      TW_CALL TwDraw();
TW_API int      TW_CALL TwWindowSize(int width, int height);

typedef enum ETwKeyModifier
{
    TW_KMOD_NONE        = 0x0000,   // same codes as SDL keysym.mod
    TW_KMOD_SHIFT       = 0x0003,
    TW_KMOD_CTRL        = 0x00c0,
    TW_KMOD_ALT         = 0x0100,
    TW_KMOD_META        = 0x0c00
} TwKeyModifier;
typedef enum EKeySpecial
{
    TW_KEY_BACKSPACE    = '\b',
    TW_KEY_TAB          = '\t',
    TW_KEY_CLEAR        = 0x0c,
    TW_KEY_RETURN       = '\r',
    TW_KEY_PAUSE        = 0x13,
    TW_KEY_ESCAPE       = 0x1b,
    TW_KEY_SPACE        = ' ',
    TW_KEY_DELETE       = 0x7f,
    TW_KEY_UP           = 273,      // same codes and order as SDL keysym.sym
    TW_KEY_DOWN,
    TW_KEY_RIGHT,
    TW_KEY_LEFT,
    TW_KEY_INSERT,
    TW_KEY_HOME,
    TW_KEY_END,
    TW_KEY_PAGE_UP,
    TW_KEY_PAGE_DOWN,
    TW_KEY_F1,
    TW_KEY_F2,
    TW_KEY_F3,
    TW_KEY_F4,
    TW_KEY_F5,
    TW_KEY_F6,
    TW_KEY_F7,
    TW_KEY_F8,
    TW_KEY_F9,
    TW_KEY_F10,
    TW_KEY_F11,
    TW_KEY_F12,
    TW_KEY_F13,
    TW_KEY_F14,
    TW_KEY_F15,
    TW_KEY_LAST
} TwKeySpecial;

TW_API int      TW_CALL TwKeyPressed(int key, int modifiers);

typedef enum ETwMouseAction
{
    TW_MOUSE_RELEASED,
    TW_MOUSE_PRESSED,   
} TwMouseAction;
typedef enum ETwMouseButtonID
{
    TW_MOUSE_LEFT       = 1,    // same code as SDL_BUTTON_LEFT
    TW_MOUSE_MIDDLE     = 2,    // same code as SDL_BUTTON_MIDDLE
    TW_MOUSE_RIGHT      = 3     // same code as SDL_BUTTON_RIGHT
} TwMouseButtonID;

TW_API int      TW_CALL TwMouseButton(TwMouseAction action, TwMouseButtonID button);
TW_API int      TW_CALL TwMouseMotion(int mouseX, int mouseY);
TW_API int      TW_CALL TwMouseWheel(int pos);

TW_API const char * TW_CALL TwGetLastError();
typedef void (TW_CALL * TwErrorHandler)(const char *errorMessage);
TW_API void     TW_CALL TwHandleErrors(TwErrorHandler errorHandler, int breakOnError);


//  ---------------------------------------------------------------------------
//  Helper functions to translate events from some common window management
//  frameworks to AntTweakBar.
//  They call TwKeyPressed, TwMouse* and TwWindowSize for you (implemented in
//  files TwEventWin32.c TwEventSDL.c TwEventGLFW.c TwEventGLUT.c)
//  ---------------------------------------------------------------------------

//  For Win32 message proc
#ifndef _W64    // Microsoft specific (detection of 64 bits portability problems)
#   define _W64
#endif  // _W64
TW_API int      TW_CALL TwEventWin32(void *wnd, unsigned int msg, unsigned int _W64 wParam, int _W64 lParam);

//  For libSDL event loop
TW_API int      TW_CALL TwEventSDL(const void *sdlEvent);
 
//  For GLFW event callbacks
TW_API int      TW_CALL TwEventMouseButtonGLFW(int glfwButton, int glfwAction);
TW_API int      TW_CALL TwEventKeyGLFW(int glfwKey, int glfwAction);
TW_API int      TW_CALL TwEventCharGLFW(int glfwChar, int glfwAction);
#define TwEventMousePosGLFW     TwMouseMotion
#define TwEventMouseWheelGLFW   TwMouseWheel

//  For GLUT event callbacks (calling convention for GLUT callback is cdecl for Win32)
#ifdef _WIN32
#   define TW_GLUT_CALL __cdecl
#else
#   define TW_GLUT_CALL
#endif
TW_API int TW_GLUT_CALL TwEventMouseButtonGLUT(int glutButton, int glutState, int mouseX, int mouseY);
TW_API int TW_GLUT_CALL TwEventMouseMotionGLUT(int mouseX, int mouseY);
TW_API int TW_GLUT_CALL TwEventKeyboardGLUT(unsigned char glutKey, int mouseX, int mouseY);
TW_API int TW_GLUT_CALL TwEventSpecialGLUT(int glutKey, int mouseX, int mouseY);
TW_API int      TW_CALL TwGLUTModifiersFunc(int (TW_CALL *glutGetModifiersFunc)(void));
typedef void (TW_GLUT_CALL *GLUTmousebuttonfun)(int glutButton, int glutState, int mouseX, int mouseY);
typedef void (TW_GLUT_CALL *GLUTmousemotionfun)(int mouseX, int mouseY);
typedef void (TW_GLUT_CALL *GLUTkeyboardfun)(unsigned char glutKey, int mouseX, int mouseY);
typedef void (TW_GLUT_CALL *GLUTspecialfun)(int glutKey, int mouseX, int mouseY);

 
//  ---------------------------------------------------------------------------
//  Make sure the types have the right sizes
//  ---------------------------------------------------------------------------


#define TW_COMPILE_TIME_ASSERT(name, x)     typedef int TW_DUMMY_ ## name[(x) * 2 - 1]

TW_COMPILE_TIME_ASSERT(CHAR,    sizeof(char)    == 1);
TW_COMPILE_TIME_ASSERT(SHORT,   sizeof(short)   == 2);
TW_COMPILE_TIME_ASSERT(INT,     sizeof(int)     == 4);
TW_COMPILE_TIME_ASSERT(FLOAT,   sizeof(float)   == 4);
TW_COMPILE_TIME_ASSERT(DOUBLE,  sizeof(double)  == 8);
//TW_COMPILE_TIME_ASSERT(PTR,     sizeof(void*)   == 4);


//  ---------------------------------------------------------------------------


#ifdef  __cplusplus
    }   // extern "C"
#endif  // __cplusplus


#endif  // !defined TW_INCLUDED
