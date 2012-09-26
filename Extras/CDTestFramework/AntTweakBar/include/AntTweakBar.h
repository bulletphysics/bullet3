// ----------------------------------------------------------------------------
//
//  @file       AntTweakBar.h
//
//  @brief      AntTweakBar is a light and intuitive graphical user interface 
//              that can be readily integrated into OpenGL and DirectX 
//              applications in order to interactively tweak parameters.
//
//  @author     Philippe Decaudin - http://www.antisphere.com
//
//  @doc        http://www.antisphere.com/Wiki/tools:anttweakbar
//
//  @license    This file is part of the AntTweakBar library.
//              AntTweakBar is a free software released under the zlib license.
//              For conditions of distribution and use, see License.txt
//
// ----------------------------------------------------------------------------


#if !defined TW_INCLUDED
#define TW_INCLUDED

#define TW_STATIC 1					//no dynamic link library (DLL) hell
#define TW_NO_LIB_PRAGMA 1  //no magic linking nonsense, thank you

#include <stddef.h>

#define TW_VERSION  115 // Version Mmm : M=Major mm=minor (e.g., 102 is version 1.02)


#ifdef  __cplusplus
#   if defined(_MSC_VER)
#       pragma warning(push)
#       pragma warning(disable: 4995 4530)
#       include <string>
#       pragma warning(pop)
#   else
#       include <string>
#   endif
    extern "C" {
#endif  // __cplusplus


// ----------------------------------------------------------------------------
//  OS specific definitions
// ----------------------------------------------------------------------------

#if (defined(_WIN32) || defined(_WIN64)) && !defined(TW_STATIC)
#   define TW_CALL          __stdcall
#   define TW_CDECL_CALL    __cdecl
#   define TW_EXPORT_API    __declspec(dllexport)
#   define TW_IMPORT_API    __declspec(dllimport)
#else
#   define TW_CALL
#   define TW_CDECL_CALL
#   define TW_EXPORT_API
#   define TW_IMPORT_API
#endif

#if defined TW_EXPORTS
#   define TW_API TW_EXPORT_API
#elif defined TW_STATIC
#   define TW_API
#   if defined(_MSC_VER) && !defined(TW_NO_LIB_PRAGMA)
#       ifdef _WIN64
#           pragma comment(lib, "AntTweakBarStatic64")
#       else
#           pragma comment(lib, "AntTweakBarStatic")
#       endif
#   endif
#else
#   define TW_API TW_IMPORT_API
#   if defined(_MSC_VER) && !defined(TW_NO_LIB_PRAGMA)
#       ifdef _WIN64
#           pragma comment(lib, "AntTweakBar64")
#       else
#           pragma comment(lib, "AntTweakBar")
#       endif
#   endif
#endif


// ----------------------------------------------------------------------------
//  Bar functions and definitions
// ----------------------------------------------------------------------------

typedef struct CTwBar TwBar; // structure CTwBar is not exposed.

TW_API TwBar *      TW_CALL TwNewBar(const char *barName);
TW_API int          TW_CALL TwDeleteBar(TwBar *bar);
TW_API int          TW_CALL TwDeleteAllBars();
TW_API int          TW_CALL TwSetTopBar(const TwBar *bar);
TW_API TwBar *      TW_CALL TwGetTopBar();
TW_API int          TW_CALL TwSetBottomBar(const TwBar *bar);
TW_API TwBar *      TW_CALL TwGetBottomBar();
TW_API const char * TW_CALL TwGetBarName(TwBar *bar);
TW_API int          TW_CALL TwGetBarCount();
TW_API TwBar *      TW_CALL TwGetBarByIndex(int barIndex);
TW_API TwBar *      TW_CALL TwGetBarByName(const char *barName);
TW_API int          TW_CALL TwRefreshBar(TwBar *bar);

// ----------------------------------------------------------------------------
//  Var functions and definitions
// ----------------------------------------------------------------------------

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
    TW_TYPE_COLOR32,    // 32 bits color. Order is RGBA if API is OpenGL or Direct3D10, and inversed if API is Direct3D9 (can be modified by defining 'colorOrder=...', see doc)
    TW_TYPE_COLOR3F,    // 3 floats color. Order is RGB.
    TW_TYPE_COLOR4F,    // 4 floats color. Order is RGBA.
    TW_TYPE_CDSTRING,   // Null-terminated C Dynamic String (pointer to an array of char dynamically allocated with malloc/realloc/strdup)
#ifdef __cplusplus
# if defined(_MSC_VER) && (_MSC_VER > 1500)
    TW_TYPE_STDSTRING = (0x2ffe0000+sizeof(std::string)),  // VS2010 or higher C++ STL string (std::string)
# else
    TW_TYPE_STDSTRING = (0x2fff0000+sizeof(std::string)),  // C++ STL string (std::string)
# endif
#endif // __cplusplus
    TW_TYPE_QUAT4F = TW_TYPE_CDSTRING+2, // 4 floats encoding a quaternion {qx,qy,qz,qs}
    TW_TYPE_QUAT4D,     // 4 doubles encoding a quaternion {qx,qy,qz,qs}
    TW_TYPE_DIR3F,      // direction vector represented by 3 floats
    TW_TYPE_DIR3D       // direction vector represented by 3 doubles
} TwType;
#define TW_TYPE_CSSTRING(n) ((TwType)(0x30000000+((n)&0xfffffff))) // Null-terminated C Static String of size n (defined as char[n], with n<2^28)

typedef void (TW_CALL * TwSetVarCallback)(const void *value, void *clientData);
typedef void (TW_CALL * TwGetVarCallback)(void *value, void *clientData);
typedef void (TW_CALL * TwButtonCallback)(void *clientData);

TW_API int      TW_CALL TwAddVarRW(TwBar *bar, const char *name, TwType type, void *var, const char *def);
TW_API int      TW_CALL TwAddVarRO(TwBar *bar, const char *name, TwType type, const void *var, const char *def);
TW_API int      TW_CALL TwAddVarCB(TwBar *bar, const char *name, TwType type, TwSetVarCallback setCallback, TwGetVarCallback getCallback, void *clientData, const char *def);
TW_API int      TW_CALL TwAddButton(TwBar *bar, const char *name, TwButtonCallback callback, void *clientData, const char *def);
TW_API int      TW_CALL TwAddSeparator(TwBar *bar, const char *name, const char *def);
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
TW_API TwType   TW_CALL TwDefineEnumFromString(const char *name, const char *enumString);
TW_API TwType   TW_CALL TwDefineStruct(const char *name, const TwStructMember *structMembers, unsigned int nbMembers, size_t structSize, TwSummaryCallback summaryCallback, void *summaryClientData);

typedef void (TW_CALL * TwCopyCDStringToClient)(char **destinationClientStringPtr, const char *sourceString);
TW_API void     TW_CALL TwCopyCDStringToClientFunc(TwCopyCDStringToClient copyCDStringFunc);
TW_API void     TW_CALL TwCopyCDStringToLibrary(char **destinationLibraryStringPtr, const char *sourceClientString);
#ifdef __cplusplus
typedef void (TW_CALL * TwCopyStdStringToClient)(std::string& destinationClientString, const std::string& sourceString);
TW_API void     TW_CALL TwCopyStdStringToClientFunc(TwCopyStdStringToClient copyStdStringToClientFunc);
TW_API void     TW_CALL TwCopyStdStringToLibrary(std::string& destinationLibraryString, const std::string& sourceClientString);
#endif // __cplusplus

typedef enum ETwParamValueType
{
    TW_PARAM_INT32,
    TW_PARAM_FLOAT,
    TW_PARAM_DOUBLE,
    TW_PARAM_CSTRING // Null-terminated array of char (ie, c-string)
} TwParamValueType;
TW_API int      TW_CALL TwGetParam(TwBar *bar, const char *varName, const char *paramName, TwParamValueType paramValueType, unsigned int outValueMaxCount, void *outValues);
TW_API int      TW_CALL TwSetParam(TwBar *bar, const char *varName, const char *paramName, TwParamValueType paramValueType, unsigned int inValueCount, const void *inValues);


// ----------------------------------------------------------------------------
//  Management functions and definitions
// ----------------------------------------------------------------------------

typedef enum ETwGraphAPI
{
    TW_OPENGL           = 1,
    TW_DIRECT3D9        = 2,
    TW_DIRECT3D10       = 3,
    TW_DIRECT3D11       = 4,
    TW_OPENGL_CORE      = 5
} TwGraphAPI;

TW_API int      TW_CALL TwInit(TwGraphAPI graphAPI, void *device);
TW_API int      TW_CALL TwTerminate();

TW_API int      TW_CALL TwDraw();
TW_API int      TW_CALL TwWindowSize(int width, int height);

TW_API int      TW_CALL TwSetCurrentWindow(int windowID); // multi-windows support
TW_API int      TW_CALL TwGetCurrentWindow();
TW_API int      TW_CALL TwWindowExists(int windowID);

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
    TW_KEY_UP           = 273,      // same codes and order as SDL 1.2 keysym.sym
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
TW_API int      TW_CALL TwKeyTest(int key, int modifiers);

typedef enum ETwMouseAction
{
    TW_MOUSE_RELEASED,
    TW_MOUSE_PRESSED  
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
TW_API void     TW_CALL TwHandleErrors(TwErrorHandler errorHandler);


// ----------------------------------------------------------------------------
//  Helper functions to translate events from some common window management
//  frameworks to AntTweakBar.
//  They call TwKeyPressed, TwMouse* and TwWindowSize for you (implemented in
//  files TwEventWin.c TwEventSDL*.c TwEventGLFW.c TwEventGLUT.c)
// ----------------------------------------------------------------------------

// For Windows message proc
#ifndef _W64    // Microsoft specific (detection of 64 bits portability issues)
#   define _W64
#endif  // _W64
#ifdef _WIN64
    TW_API int  TW_CALL TwEventWin(void *wnd, unsigned int msg, unsigned __int64 _W64 wParam, __int64 _W64 lParam);
#else
    TW_API int  TW_CALL TwEventWin(void *wnd, unsigned int msg, unsigned int _W64 wParam, int _W64 lParam);
#endif
#define TwEventWin32    TwEventWin // For compatibility with AntTweakBar versions prior to 1.11

// For libSDL event loop
TW_API int      TW_CALL TwEventSDL(const void *sdlEvent, unsigned char sdlMajorVersion, unsigned char sdlMinorVersion);

// For GLFW event callbacks
// You should define GLFW_CDECL before including AntTweakBar.h if your version of GLFW uses cdecl calling convensions
#ifdef GLFW_CDECL
    TW_API int TW_CDECL_CALL TwEventMouseButtonGLFWcdecl(int glfwButton, int glfwAction);
    TW_API int TW_CDECL_CALL TwEventKeyGLFWcdecl(int glfwKey, int glfwAction);
    TW_API int TW_CDECL_CALL TwEventCharGLFWcdecl(int glfwChar, int glfwAction);
    TW_API int TW_CDECL_CALL TwEventMousePosGLFWcdecl(int mouseX, int mouseY);
    TW_API int TW_CDECL_CALL TwEventMouseWheelGLFWcdecl(int wheelPos);
#   define TwEventMouseButtonGLFW TwEventMouseButtonGLFWcdecl
#   define TwEventKeyGLFW         TwEventKeyGLFWcdecl
#   define TwEventCharGLFW        TwEventCharGLFWcdecl
#   define TwEventMousePosGLFW    TwEventMousePosGLFWcdecl
#   define TwEventMouseWheelGLFW  TwEventMouseWheelGLFWcdecl
#else
    TW_API int  TW_CALL TwEventMouseButtonGLFW(int glfwButton, int glfwAction);
    TW_API int  TW_CALL TwEventKeyGLFW(int glfwKey, int glfwAction);
    TW_API int  TW_CALL TwEventCharGLFW(int glfwChar, int glfwAction);
#   define TwEventMousePosGLFW     TwMouseMotion
#   define TwEventMouseWheelGLFW   TwMouseWheel
#endif

// For GLUT event callbacks (Windows calling convention for GLUT callbacks is cdecl)
#if defined(_WIN32) || defined(_WIN64)
#   define TW_GLUT_CALL TW_CDECL_CALL
#else
#   define TW_GLUT_CALL
#endif
TW_API int TW_GLUT_CALL TwEventMouseButtonGLUT(int glutButton, int glutState, int mouseX, int mouseY);
TW_API int TW_GLUT_CALL TwEventMouseMotionGLUT(int mouseX, int mouseY);
TW_API int TW_GLUT_CALL TwEventKeyboardGLUT(unsigned char glutKey, int mouseX, int mouseY);
TW_API int TW_GLUT_CALL TwEventSpecialGLUT(int glutKey, int mouseX, int mouseY);
TW_API int TW_CALL      TwGLUTModifiersFunc(int (TW_CALL *glutGetModifiersFunc)(void));
typedef void (TW_GLUT_CALL *GLUTmousebuttonfun)(int glutButton, int glutState, int mouseX, int mouseY);
typedef void (TW_GLUT_CALL *GLUTmousemotionfun)(int mouseX, int mouseY);
typedef void (TW_GLUT_CALL *GLUTkeyboardfun)(unsigned char glutKey, int mouseX, int mouseY);
typedef void (TW_GLUT_CALL *GLUTspecialfun)(int glutKey, int mouseX, int mouseY);

// For SFML event loop
TW_API int      TW_CALL TwEventSFML(const void *sfmlEvent, unsigned char sfmlMajorVersion, unsigned char sfmlMinorVersion);

// For X11 event loop
#if defined(_UNIX)
    TW_API int TW_CDECL_CALL TwEventX11(void *xevent);
#endif

// ----------------------------------------------------------------------------
//  Make sure the types have the right sizes
// ----------------------------------------------------------------------------

#define TW_COMPILE_TIME_ASSERT(name, x) typedef int TW_DUMMY_ ## name[(x) * 2 - 1]

TW_COMPILE_TIME_ASSERT(TW_CHAR,    sizeof(char)    == 1);
TW_COMPILE_TIME_ASSERT(TW_SHORT,   sizeof(short)   == 2);
TW_COMPILE_TIME_ASSERT(TW_INT,     sizeof(int)     == 4);
TW_COMPILE_TIME_ASSERT(TW_FLOAT,   sizeof(float)   == 4);
TW_COMPILE_TIME_ASSERT(TW_DOUBLE,  sizeof(double)  == 8);

// Check pointer size on Windows
#if !defined(_WIN64) && defined(_WIN32)
    // If the following assert failed, the platform is not 32-bit and _WIN64 is not defined.
    // When targetting 64-bit Windows platform, _WIN64 must be defined.
    TW_COMPILE_TIME_ASSERT(TW_PTR32, sizeof(void*) == 4);
#elif defined(_WIN64)
    // If the following assert failed, _WIN64 is defined but the targeted platform is not 64-bit.
    TW_COMPILE_TIME_ASSERT(TW_PTR64, sizeof(void*) == 8);
#endif

//  ---------------------------------------------------------------------------


#ifdef  __cplusplus
    }   // extern "C"
#endif  // __cplusplus


#endif  // !defined TW_INCLUDED
