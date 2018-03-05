#ifndef __glad_egl_h_
#define __glad_egl_h_

#ifdef __egl_h_
    #error EGL header already included (API: egl), remove previous include!
#endif
#define __egl_h_


#define GLAD_EGL

#if defined(_WIN32) && !defined(APIENTRY) && !defined(__CYGWIN__) && !defined(__SCITECH_SNAP__)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif
#include <windows.h>
#endif
#ifndef APIENTRY
#define APIENTRY
#endif
#ifndef APIENTRYP
#define APIENTRYP APIENTRY *
#endif
#ifdef __cplusplus
extern "C" {
#endif

#ifndef GLAPI
# if defined(GLAD_GLAPI_EXPORT)
#  if defined(_WIN32) || defined(__CYGWIN__)
#   if defined(GLAD_GLAPI_EXPORT_BUILD)
#    if defined(__GNUC__)
#     define GLAPI __attribute__ ((dllexport)) extern
#    else
#     define GLAPI __declspec(dllexport) extern
#    endif
#   else
#    if defined(__GNUC__)
#     define GLAPI __attribute__ ((dllimport)) extern
#    else
#     define GLAPI __declspec(dllimport) extern
#    endif
#   endif
#  elif defined(__GNUC__) && defined(GLAD_GLAPI_EXPORT_BUILD)
#   define GLAPI __attribute__ ((visibility ("default"))) extern
#  else
#   define GLAPI extern
#  endif
# else
#  define GLAPI extern
# endif
#endif

#define EGL_VERSION_1_0 1
GLAPI int GLAD_EGL_VERSION_1_0;
#define EGL_VERSION_1_1 1
GLAPI int GLAD_EGL_VERSION_1_1;
#define EGL_VERSION_1_2 1
GLAPI int GLAD_EGL_VERSION_1_2;
#define EGL_VERSION_1_3 1
GLAPI int GLAD_EGL_VERSION_1_3;
#define EGL_VERSION_1_4 1
GLAPI int GLAD_EGL_VERSION_1_4;
#define EGL_VERSION_1_5 1
GLAPI int GLAD_EGL_VERSION_1_5;
#define EGL_EXT_device_base 1
GLAPI int GLAD_EGL_EXT_device_base;
#define EGL_EXT_device_enumeration 1
GLAPI int GLAD_EGL_EXT_device_enumeration;
#define EGL_EXT_device_query 1
GLAPI int GLAD_EGL_EXT_device_query;
#define EGL_EXT_platform_base 1
GLAPI int GLAD_EGL_EXT_platform_base;
#define EGL_EXT_platform_device 1
GLAPI int GLAD_EGL_EXT_platform_device;
#define EGL_NV_cuda_event 1
GLAPI int GLAD_EGL_NV_cuda_event;
#define EGL_NV_device_cuda 1
GLAPI int GLAD_EGL_NV_device_cuda;

#include <KHR/khrplatform.h>
#include <EGL/eglplatform.h>
struct AHardwareBuffer;
typedef unsigned int EGLBoolean;
typedef unsigned int EGLenum;
typedef intptr_t EGLAttribKHR;
typedef intptr_t EGLAttrib;
typedef void *EGLClientBuffer;
typedef void *EGLConfig;
typedef void *EGLContext;
typedef void *EGLDeviceEXT;
typedef void *EGLDisplay;
typedef void *EGLImage;
typedef void *EGLImageKHR;
typedef void *EGLLabelKHR;
typedef void *EGLObjectKHR;
typedef void *EGLOutputLayerEXT;
typedef void *EGLOutputPortEXT;
typedef void *EGLStreamKHR;
typedef void *EGLSurface;
typedef void *EGLSync;
typedef void *EGLSyncKHR;
typedef void *EGLSyncNV;
typedef void (*__eglMustCastToProperFunctionPointerType)(void);
typedef khronos_utime_nanoseconds_t EGLTimeKHR;
typedef khronos_utime_nanoseconds_t EGLTime;
typedef khronos_utime_nanoseconds_t EGLTimeNV;
typedef khronos_utime_nanoseconds_t EGLuint64NV;
typedef khronos_uint64_t EGLuint64KHR;
typedef khronos_stime_nanoseconds_t EGLnsecsANDROID;
typedef int EGLNativeFileDescriptorKHR;
typedef khronos_ssize_t EGLsizeiANDROID;
typedef void (*EGLSetBlobFuncANDROID) (const void *key, EGLsizeiANDROID keySize, const void *value, EGLsizeiANDROID valueSize);
typedef EGLsizeiANDROID (*EGLGetBlobFuncANDROID) (const void *key, EGLsizeiANDROID keySize, void *value, EGLsizeiANDROID valueSize);
struct EGLClientPixmapHI {
    void  *pData;
    EGLint iWidth;
    EGLint iHeight;
    EGLint iStride;
};
typedef void (APIENTRY *EGLDEBUGPROCKHR)(EGLenum error,const char *command,EGLint messageType,EGLLabelKHR threadLabel,EGLLabelKHR objectLabel,const char* message);

#define EGL_FALSE 0
#define EGL_BACK_BUFFER 0x3084
#define EGL_GL_COLORSPACE_LINEAR 0x308A
#define EGL_GL_TEXTURE_CUBE_MAP_NEGATIVE_X 0x30B4
#define EGL_TEXTURE_RGB 0x305D
#define EGL_CONTEXT_OPENGL_RESET_NOTIFICATION_STRATEGY 0x31BD
#define EGL_VG_ALPHA_FORMAT_PRE 0x308C
#define EGL_LEVEL 0x3029
#define EGL_COLORSPACE_LINEAR 0x308A
#define EGL_DONT_CARE EGL_CAST(EGLint,-1)
#define EGL_OPENVG_IMAGE 0x3096
#define EGL_ALPHA_FORMAT_PRE 0x308C
#define EGL_SLOW_CONFIG 0x3050
#define EGL_GL_TEXTURE_CUBE_MAP_POSITIVE_Y 0x30B5
#define EGL_GREEN_SIZE 0x3023
#define EGL_CONTEXT_MINOR_VERSION 0x30FB
#define EGL_CUDA_DEVICE_NV 0x323A
#define EGL_BAD_CONFIG 0x3005
#define EGL_RENDERABLE_TYPE 0x3040
#define EGL_CONDITION_SATISFIED 0x30F6
#define EGL_OPENGL_ES_BIT 0x0001
#define EGL_VERTICAL_RESOLUTION 0x3091
#define EGL_DEFAULT_DISPLAY EGL_CAST(EGLNativeDisplayType,0)
#define EGL_BAD_PARAMETER 0x300C
#define EGL_DEPTH_SIZE 0x3025
#define EGL_CONTEXT_MAJOR_VERSION 0x3098
#define EGL_HEIGHT 0x3056
#define EGL_OPENGL_ES3_BIT 0x00000040
#define EGL_BAD_ALLOC 0x3003
#define EGL_BAD_DISPLAY 0x3008
#define EGL_CLIENT_APIS 0x308D
#define EGL_VG_ALPHA_FORMAT_NONPRE 0x308B
#define EGL_CONTEXT_OPENGL_PROFILE_MASK 0x30FD
#define EGL_MAX_PBUFFER_HEIGHT 0x302A
#define EGL_SURFACE_TYPE 0x3033
#define EGL_VENDOR 0x3053
#define EGL_GL_COLORSPACE 0x309D
#define EGL_BAD_SURFACE 0x300D
#define EGL_GL_RENDERBUFFER 0x30B9
#define EGL_SAMPLE_BUFFERS 0x3032
#define EGL_VG_ALPHA_FORMAT_PRE_BIT 0x0040
#define EGL_VG_ALPHA_FORMAT 0x3088
#define EGL_PIXMAP_BIT 0x0002
#define EGL_NATIVE_RENDERABLE 0x302D
#define EGL_TEXTURE_RGBA 0x305E
#define EGL_NATIVE_VISUAL_ID 0x302E
#define EGL_VERSION 0x3054
#define EGL_TRANSPARENT_RED_VALUE 0x3037
#define EGL_GL_TEXTURE_CUBE_MAP_NEGATIVE_Y 0x30B6
#define EGL_BLUE_SIZE 0x3022
#define EGL_SWAP_BEHAVIOR_PRESERVED_BIT 0x0400
#define EGL_PIXEL_ASPECT_RATIO 0x3092
#define EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT 0x00000001
#define EGL_STENCIL_SIZE 0x3026
#define EGL_CONFORMANT 0x3042
#define EGL_EXTENSIONS 0x3055
#define EGL_SYNC_CL_EVENT_COMPLETE 0x30FF
#define EGL_BAD_ATTRIBUTE 0x3004
#define EGL_NO_DISPLAY EGL_CAST(EGLDisplay,0)
#define EGL_TEXTURE_TARGET 0x3081
#define EGL_IMAGE_PRESERVED 0x30D2
#define EGL_COLOR_BUFFER_TYPE 0x303F
#define EGL_VG_COLORSPACE_sRGB 0x3089
#define EGL_GL_TEXTURE_3D 0x30B2
#define EGL_MULTISAMPLE_RESOLVE 0x3099
#define EGL_CL_EVENT_HANDLE 0x309C
#define EGL_GL_TEXTURE_2D 0x30B1
#define EGL_CORE_NATIVE_ENGINE 0x305B
#define EGL_GL_TEXTURE_CUBE_MAP_NEGATIVE_Z 0x30B8
#define EGL_UNKNOWN EGL_CAST(EGLint,-1)
#define EGL_TRANSPARENT_TYPE 0x3034
#define EGL_LUMINANCE_SIZE 0x303D
#define EGL_LARGEST_PBUFFER 0x3058
#define EGL_VG_COLORSPACE 0x3087
#define EGL_GL_COLORSPACE_SRGB 0x3089
#define EGL_LUMINANCE_BUFFER 0x308F
#define EGL_FOREVER 0xFFFFFFFFFFFFFFFF
#define EGL_NO_SYNC EGL_CAST(EGLSync,0)
#define EGL_WIDTH 0x3057
#define EGL_ALPHA_FORMAT_NONPRE 0x308B
#define EGL_BUFFER_SIZE 0x3020
#define EGL_NO_CONTEXT EGL_CAST(EGLContext,0)
#define EGL_MIPMAP_LEVEL 0x3083
#define EGL_NO_TEXTURE 0x305C
#define EGL_MULTISAMPLE_RESOLVE_BOX_BIT 0x0200
#define EGL_SINGLE_BUFFER 0x3085
#define EGL_NONE 0x3038
#define EGL_NOT_INITIALIZED 0x3001
#define EGL_VG_COLORSPACE_LINEAR 0x308A
#define EGL_GL_TEXTURE_CUBE_MAP_POSITIVE_Z 0x30B7
#define EGL_OPENVG_API 0x30A1
#define EGL_DRAW 0x3059
#define EGL_OPENGL_BIT 0x0008
#define EGL_PBUFFER_BIT 0x0001
#define EGL_ALPHA_SIZE 0x3021
#define EGL_TRANSPARENT_RGB 0x3052
#define EGL_TRANSPARENT_GREEN_VALUE 0x3036
#define EGL_SWAP_BEHAVIOR 0x3093
#define EGL_SIGNALED 0x30F2
#define EGL_CONFIG_ID 0x3028
#define EGL_SYNC_CUDA_EVENT_COMPLETE_NV 0x323D
#define EGL_GL_TEXTURE_LEVEL 0x30BC
#define EGL_BAD_ACCESS 0x3002
#define EGL_MAX_PBUFFER_PIXELS 0x302B
#define EGL_DEVICE_EXT 0x322C
#define EGL_GL_TEXTURE_CUBE_MAP_POSITIVE_X 0x30B3
#define EGL_ALPHA_FORMAT 0x3088
#define EGL_TRUE 1
#define EGL_MIPMAP_TEXTURE 0x3082
#define EGL_LOSE_CONTEXT_ON_RESET 0x31BF
#define EGL_GL_TEXTURE_ZOFFSET 0x30BD
#define EGL_MIN_SWAP_INTERVAL 0x303B
#define EGL_BAD_CURRENT_SURFACE 0x3007
#define EGL_RED_SIZE 0x3024
#define EGL_BAD_DEVICE_EXT 0x322B
#define EGL_SYNC_FLUSH_COMMANDS_BIT 0x0001
#define EGL_PLATFORM_DEVICE_EXT 0x313F
#define EGL_READ 0x305A
#define EGL_OPENGL_ES_API 0x30A0
#define EGL_OPENGL_API 0x30A2
#define EGL_SYNC_TYPE 0x30F7
#define EGL_CONTEXT_OPENGL_COMPATIBILITY_PROFILE_BIT 0x00000002
#define EGL_SYNC_PRIOR_COMMANDS_COMPLETE 0x30F0
#define EGL_MAX_SWAP_INTERVAL 0x303C
#define EGL_RGB_BUFFER 0x308E
#define EGL_OPENGL_ES2_BIT 0x0004
#define EGL_BAD_NATIVE_WINDOW 0x300B
#define EGL_CONTEXT_LOST 0x300E
#define EGL_TIMEOUT_EXPIRED 0x30F5
#define EGL_TEXTURE_2D 0x305F
#define EGL_RENDER_BUFFER 0x3086
#define EGL_BAD_NATIVE_PIXMAP 0x300A
#define EGL_MATCH_NATIVE_PIXMAP 0x3041
#define EGL_NO_IMAGE EGL_CAST(EGLImage,0)
#define EGL_SUCCESS 0x3000
#define EGL_OPENVG_BIT 0x0002
#define EGL_BIND_TO_TEXTURE_RGB 0x3039
#define EGL_CONTEXT_OPENGL_ROBUST_ACCESS 0x31B2
#define EGL_CONTEXT_OPENGL_DEBUG 0x31B0
#define EGL_BAD_CONTEXT 0x3006
#define EGL_VG_COLORSPACE_LINEAR_BIT 0x0020
#define EGL_CONTEXT_OPENGL_FORWARD_COMPATIBLE 0x31B1
#define EGL_ALPHA_MASK_SIZE 0x303E
#define EGL_CONFIG_CAVEAT 0x3027
#define EGL_BAD_MATCH 0x3009
#define EGL_SYNC_CL_EVENT 0x30FE
#define EGL_CONTEXT_CLIENT_TYPE 0x3097
#define EGL_CONTEXT_CLIENT_VERSION 0x3098
#define EGL_CUDA_EVENT_HANDLE_NV 0x323B
#define EGL_BIND_TO_TEXTURE_RGBA 0x303A
#define EGL_COLORSPACE 0x3087
#define EGL_TEXTURE_FORMAT 0x3080
#define EGL_MULTISAMPLE_RESOLVE_BOX 0x309B
#define EGL_SYNC_STATUS 0x30F1
#define EGL_UNSIGNALED 0x30F3
#define EGL_SYNC_CUDA_EVENT_NV 0x323C
#define EGL_NO_SURFACE EGL_CAST(EGLSurface,0)
#define EGL_TRANSPARENT_BLUE_VALUE 0x3035
#define EGL_WINDOW_BIT 0x0004
#define EGL_NATIVE_VISUAL_TYPE 0x302F
#define EGL_SAMPLES 0x3031
#define EGL_SYNC_CONDITION 0x30F8
#define EGL_NON_CONFORMANT_CONFIG 0x3051
#define EGL_HORIZONTAL_RESOLUTION 0x3090
#define EGL_MULTISAMPLE_RESOLVE_DEFAULT 0x309A
#define EGL_NO_DEVICE_EXT EGL_CAST(EGLDeviceEXT,0)
#define EGL_SYNC_FENCE 0x30F9
#define EGL_DISPLAY_SCALING 10000
#define EGL_BUFFER_DESTROYED 0x3095
#define EGL_MAX_PBUFFER_WIDTH 0x302C
#define EGL_BUFFER_PRESERVED 0x3094
#define EGL_COLORSPACE_sRGB 0x3089
#define EGL_NO_RESET_NOTIFICATION 0x31BE

typedef EGLBoolean (APIENTRYP PFNEGLWAITNATIVEPROC)(EGLint engine);
typedef EGLSurface (APIENTRYP PFNEGLCREATEWINDOWSURFACEPROC)(EGLDisplay dpy, EGLConfig config, EGLNativeWindowType win, const EGLint *attrib_list);
typedef const char * (APIENTRYP PFNEGLQUERYSTRINGPROC)(EGLDisplay dpy, EGLint name);
typedef EGLBoolean (APIENTRYP PFNEGLINITIALIZEPROC)(EGLDisplay dpy, EGLint *major, EGLint *minor);
typedef EGLBoolean (APIENTRYP PFNEGLQUERYCONTEXTPROC)(EGLDisplay dpy, EGLContext ctx, EGLint attribute, EGLint *value);
typedef EGLSurface (APIENTRYP PFNEGLGETCURRENTSURFACEPROC)(EGLint readdraw);
typedef EGLBoolean (APIENTRYP PFNEGLSWAPINTERVALPROC)(EGLDisplay dpy, EGLint interval);
typedef EGLSync (APIENTRYP PFNEGLCREATESYNCPROC)(EGLDisplay dpy, EGLenum type, const EGLAttrib *attrib_list);
typedef EGLBoolean (APIENTRYP PFNEGLCHOOSECONFIGPROC)(EGLDisplay dpy, const EGLint *attrib_list, EGLConfig *configs, EGLint config_size, EGLint *num_config);
typedef EGLSurface (APIENTRYP PFNEGLCREATEPIXMAPSURFACEPROC)(EGLDisplay dpy, EGLConfig config, EGLNativePixmapType pixmap, const EGLint *attrib_list);
typedef EGLBoolean (APIENTRYP PFNEGLGETCONFIGATTRIBPROC)(EGLDisplay dpy, EGLConfig config, EGLint attribute, EGLint *value);
typedef EGLBoolean (APIENTRYP PFNEGLQUERYDEVICEATTRIBEXTPROC)(EGLDeviceEXT device, EGLint attribute, EGLAttrib *value);
typedef EGLSurface (APIENTRYP PFNEGLCREATEPBUFFERSURFACEPROC)(EGLDisplay dpy, EGLConfig config, const EGLint *attrib_list);
typedef EGLSurface (APIENTRYP PFNEGLCREATEPBUFFERFROMCLIENTBUFFERPROC)(EGLDisplay dpy, EGLenum buftype, EGLClientBuffer buffer, EGLConfig config, const EGLint *attrib_list);
typedef EGLBoolean (APIENTRYP PFNEGLMAKECURRENTPROC)(EGLDisplay dpy, EGLSurface draw, EGLSurface read, EGLContext ctx);
typedef EGLDisplay (APIENTRYP PFNEGLGETPLATFORMDISPLAYPROC)(EGLenum platform, void *native_display, const EGLAttrib *attrib_list);
typedef EGLBoolean (APIENTRYP PFNEGLRELEASETEXIMAGEPROC)(EGLDisplay dpy, EGLSurface surface, EGLint buffer);
typedef EGLint (APIENTRYP PFNEGLGETERRORPROC)();
typedef EGLBoolean (APIENTRYP PFNEGLWAITSYNCPROC)(EGLDisplay dpy, EGLSync sync, EGLint flags);
typedef EGLBoolean (APIENTRYP PFNEGLWAITGLPROC)();
typedef __eglMustCastToProperFunctionPointerType (APIENTRYP PFNEGLGETPROCADDRESSPROC)(const char *procname);
typedef EGLContext (APIENTRYP PFNEGLCREATECONTEXTPROC)(EGLDisplay dpy, EGLConfig config, EGLContext share_context, const EGLint *attrib_list);
typedef EGLBoolean (APIENTRYP PFNEGLDESTROYCONTEXTPROC)(EGLDisplay dpy, EGLContext ctx);
typedef EGLBoolean (APIENTRYP PFNEGLBINDAPIPROC)(EGLenum api);
typedef EGLContext (APIENTRYP PFNEGLGETCURRENTCONTEXTPROC)();
typedef EGLBoolean (APIENTRYP PFNEGLGETCONFIGSPROC)(EGLDisplay dpy, EGLConfig *configs, EGLint config_size, EGLint *num_config);
typedef const char * (APIENTRYP PFNEGLQUERYDEVICESTRINGEXTPROC)(EGLDeviceEXT device, EGLint name);
typedef EGLSurface (APIENTRYP PFNEGLCREATEPLATFORMPIXMAPSURFACEPROC)(EGLDisplay dpy, EGLConfig config, void *native_pixmap, const EGLAttrib *attrib_list);
typedef EGLBoolean (APIENTRYP PFNEGLGETSYNCATTRIBPROC)(EGLDisplay dpy, EGLSync sync, EGLint attribute, EGLAttrib *value);
typedef EGLDisplay (APIENTRYP PFNEGLGETPLATFORMDISPLAYEXTPROC)(EGLenum platform, void *native_display, const EGLint *attrib_list);
typedef EGLSurface (APIENTRYP PFNEGLCREATEPLATFORMWINDOWSURFACEEXTPROC)(EGLDisplay dpy, EGLConfig config, void *native_window, const EGLint *attrib_list);
typedef EGLBoolean (APIENTRYP PFNEGLBINDTEXIMAGEPROC)(EGLDisplay dpy, EGLSurface surface, EGLint buffer);
typedef EGLSurface (APIENTRYP PFNEGLCREATEPLATFORMWINDOWSURFACEPROC)(EGLDisplay dpy, EGLConfig config, void *native_window, const EGLAttrib *attrib_list);
typedef EGLBoolean (APIENTRYP PFNEGLCOPYBUFFERSPROC)(EGLDisplay dpy, EGLSurface surface, EGLNativePixmapType target);
typedef EGLSurface (APIENTRYP PFNEGLCREATEPLATFORMPIXMAPSURFACEEXTPROC)(EGLDisplay dpy, EGLConfig config, void *native_pixmap, const EGLint *attrib_list);
typedef EGLDisplay (APIENTRYP PFNEGLGETDISPLAYPROC)(EGLNativeDisplayType display_id);
typedef EGLImage (APIENTRYP PFNEGLCREATEIMAGEPROC)(EGLDisplay dpy, EGLContext ctx, EGLenum target, EGLClientBuffer buffer, const EGLAttrib *attrib_list);
typedef EGLBoolean (APIENTRYP PFNEGLWAITCLIENTPROC)();
typedef EGLDisplay (APIENTRYP PFNEGLGETCURRENTDISPLAYPROC)();
typedef EGLBoolean (APIENTRYP PFNEGLTERMINATEPROC)(EGLDisplay dpy);
typedef EGLBoolean (APIENTRYP PFNEGLDESTROYIMAGEPROC)(EGLDisplay dpy, EGLImage image);
typedef EGLBoolean (APIENTRYP PFNEGLQUERYDISPLAYATTRIBEXTPROC)(EGLDisplay dpy, EGLint attribute, EGLAttrib *value);
typedef EGLBoolean (APIENTRYP PFNEGLQUERYSURFACEPROC)(EGLDisplay dpy, EGLSurface surface, EGLint attribute, EGLint *value);
typedef EGLBoolean (APIENTRYP PFNEGLSWAPBUFFERSPROC)(EGLDisplay dpy, EGLSurface surface);
typedef EGLint (APIENTRYP PFNEGLCLIENTWAITSYNCPROC)(EGLDisplay dpy, EGLSync sync, EGLint flags, EGLTime timeout);
typedef EGLBoolean (APIENTRYP PFNEGLRELEASETHREADPROC)();
typedef EGLBoolean (APIENTRYP PFNEGLDESTROYSURFACEPROC)(EGLDisplay dpy, EGLSurface surface);
typedef EGLBoolean (APIENTRYP PFNEGLQUERYDEVICESEXTPROC)(EGLint max_devices, EGLDeviceEXT *devices, EGLint *num_devices);
typedef EGLBoolean (APIENTRYP PFNEGLSURFACEATTRIBPROC)(EGLDisplay dpy, EGLSurface surface, EGLint attribute, EGLint value);
typedef EGLenum (APIENTRYP PFNEGLQUERYAPIPROC)();
typedef EGLBoolean (APIENTRYP PFNEGLDESTROYSYNCPROC)(EGLDisplay dpy, EGLSync sync);

GLAPI PFNEGLWAITNATIVEPROC glad_eglWaitNative;
#define eglWaitNative glad_eglWaitNative
GLAPI PFNEGLCREATEWINDOWSURFACEPROC glad_eglCreateWindowSurface;
#define eglCreateWindowSurface glad_eglCreateWindowSurface
GLAPI PFNEGLQUERYSTRINGPROC glad_eglQueryString;
#define eglQueryString glad_eglQueryString
GLAPI PFNEGLINITIALIZEPROC glad_eglInitialize;
#define eglInitialize glad_eglInitialize
GLAPI PFNEGLQUERYCONTEXTPROC glad_eglQueryContext;
#define eglQueryContext glad_eglQueryContext
GLAPI PFNEGLGETCURRENTSURFACEPROC glad_eglGetCurrentSurface;
#define eglGetCurrentSurface glad_eglGetCurrentSurface
GLAPI PFNEGLSWAPINTERVALPROC glad_eglSwapInterval;
#define eglSwapInterval glad_eglSwapInterval
GLAPI PFNEGLCREATESYNCPROC glad_eglCreateSync;
#define eglCreateSync glad_eglCreateSync
GLAPI PFNEGLCHOOSECONFIGPROC glad_eglChooseConfig;
#define eglChooseConfig glad_eglChooseConfig
GLAPI PFNEGLCREATEPIXMAPSURFACEPROC glad_eglCreatePixmapSurface;
#define eglCreatePixmapSurface glad_eglCreatePixmapSurface
GLAPI PFNEGLGETCONFIGATTRIBPROC glad_eglGetConfigAttrib;
#define eglGetConfigAttrib glad_eglGetConfigAttrib
GLAPI PFNEGLQUERYDEVICEATTRIBEXTPROC glad_eglQueryDeviceAttribEXT;
#define eglQueryDeviceAttribEXT glad_eglQueryDeviceAttribEXT
GLAPI PFNEGLCREATEPBUFFERSURFACEPROC glad_eglCreatePbufferSurface;
#define eglCreatePbufferSurface glad_eglCreatePbufferSurface
GLAPI PFNEGLCREATEPBUFFERFROMCLIENTBUFFERPROC glad_eglCreatePbufferFromClientBuffer;
#define eglCreatePbufferFromClientBuffer glad_eglCreatePbufferFromClientBuffer
GLAPI PFNEGLMAKECURRENTPROC glad_eglMakeCurrent;
#define eglMakeCurrent glad_eglMakeCurrent
GLAPI PFNEGLGETPLATFORMDISPLAYPROC glad_eglGetPlatformDisplay;
#define eglGetPlatformDisplay glad_eglGetPlatformDisplay
GLAPI PFNEGLRELEASETEXIMAGEPROC glad_eglReleaseTexImage;
#define eglReleaseTexImage glad_eglReleaseTexImage
GLAPI PFNEGLGETERRORPROC glad_eglGetError;
#define eglGetError glad_eglGetError
GLAPI PFNEGLWAITSYNCPROC glad_eglWaitSync;
#define eglWaitSync glad_eglWaitSync
GLAPI PFNEGLWAITGLPROC glad_eglWaitGL;
#define eglWaitGL glad_eglWaitGL
GLAPI PFNEGLGETPROCADDRESSPROC glad_eglGetProcAddress;
#define eglGetProcAddress glad_eglGetProcAddress
GLAPI PFNEGLCREATECONTEXTPROC glad_eglCreateContext;
#define eglCreateContext glad_eglCreateContext
GLAPI PFNEGLDESTROYCONTEXTPROC glad_eglDestroyContext;
#define eglDestroyContext glad_eglDestroyContext
GLAPI PFNEGLBINDAPIPROC glad_eglBindAPI;
#define eglBindAPI glad_eglBindAPI
GLAPI PFNEGLGETCURRENTCONTEXTPROC glad_eglGetCurrentContext;
#define eglGetCurrentContext glad_eglGetCurrentContext
GLAPI PFNEGLGETCONFIGSPROC glad_eglGetConfigs;
#define eglGetConfigs glad_eglGetConfigs
GLAPI PFNEGLQUERYDEVICESTRINGEXTPROC glad_eglQueryDeviceStringEXT;
#define eglQueryDeviceStringEXT glad_eglQueryDeviceStringEXT
GLAPI PFNEGLCREATEPLATFORMPIXMAPSURFACEPROC glad_eglCreatePlatformPixmapSurface;
#define eglCreatePlatformPixmapSurface glad_eglCreatePlatformPixmapSurface
GLAPI PFNEGLGETSYNCATTRIBPROC glad_eglGetSyncAttrib;
#define eglGetSyncAttrib glad_eglGetSyncAttrib
GLAPI PFNEGLGETPLATFORMDISPLAYEXTPROC glad_eglGetPlatformDisplayEXT;
#define eglGetPlatformDisplayEXT glad_eglGetPlatformDisplayEXT
GLAPI PFNEGLCREATEPLATFORMWINDOWSURFACEEXTPROC glad_eglCreatePlatformWindowSurfaceEXT;
#define eglCreatePlatformWindowSurfaceEXT glad_eglCreatePlatformWindowSurfaceEXT
GLAPI PFNEGLBINDTEXIMAGEPROC glad_eglBindTexImage;
#define eglBindTexImage glad_eglBindTexImage
GLAPI PFNEGLCREATEPLATFORMWINDOWSURFACEPROC glad_eglCreatePlatformWindowSurface;
#define eglCreatePlatformWindowSurface glad_eglCreatePlatformWindowSurface
GLAPI PFNEGLCOPYBUFFERSPROC glad_eglCopyBuffers;
#define eglCopyBuffers glad_eglCopyBuffers
GLAPI PFNEGLCREATEPLATFORMPIXMAPSURFACEEXTPROC glad_eglCreatePlatformPixmapSurfaceEXT;
#define eglCreatePlatformPixmapSurfaceEXT glad_eglCreatePlatformPixmapSurfaceEXT
GLAPI PFNEGLGETDISPLAYPROC glad_eglGetDisplay;
#define eglGetDisplay glad_eglGetDisplay
GLAPI PFNEGLCREATEIMAGEPROC glad_eglCreateImage;
#define eglCreateImage glad_eglCreateImage
GLAPI PFNEGLWAITCLIENTPROC glad_eglWaitClient;
#define eglWaitClient glad_eglWaitClient
GLAPI PFNEGLGETCURRENTDISPLAYPROC glad_eglGetCurrentDisplay;
#define eglGetCurrentDisplay glad_eglGetCurrentDisplay
GLAPI PFNEGLTERMINATEPROC glad_eglTerminate;
#define eglTerminate glad_eglTerminate
GLAPI PFNEGLDESTROYIMAGEPROC glad_eglDestroyImage;
#define eglDestroyImage glad_eglDestroyImage
GLAPI PFNEGLQUERYDISPLAYATTRIBEXTPROC glad_eglQueryDisplayAttribEXT;
#define eglQueryDisplayAttribEXT glad_eglQueryDisplayAttribEXT
GLAPI PFNEGLQUERYSURFACEPROC glad_eglQuerySurface;
#define eglQuerySurface glad_eglQuerySurface
GLAPI PFNEGLSWAPBUFFERSPROC glad_eglSwapBuffers;
#define eglSwapBuffers glad_eglSwapBuffers
GLAPI PFNEGLCLIENTWAITSYNCPROC glad_eglClientWaitSync;
#define eglClientWaitSync glad_eglClientWaitSync
GLAPI PFNEGLRELEASETHREADPROC glad_eglReleaseThread;
#define eglReleaseThread glad_eglReleaseThread
GLAPI PFNEGLDESTROYSURFACEPROC glad_eglDestroySurface;
#define eglDestroySurface glad_eglDestroySurface
GLAPI PFNEGLQUERYDEVICESEXTPROC glad_eglQueryDevicesEXT;
#define eglQueryDevicesEXT glad_eglQueryDevicesEXT
GLAPI PFNEGLSURFACEATTRIBPROC glad_eglSurfaceAttrib;
#define eglSurfaceAttrib glad_eglSurfaceAttrib
GLAPI PFNEGLQUERYAPIPROC glad_eglQueryAPI;
#define eglQueryAPI glad_eglQueryAPI
GLAPI PFNEGLDESTROYSYNCPROC glad_eglDestroySync;
#define eglDestroySync glad_eglDestroySync


typedef void* (* GLADloadproc)(const char *name, void* userptr);
typedef void* (* GLADsimpleloadproc)(const char *name);
GLAPI int gladLoadEGL(EGLDisplay display, GLADloadproc load, void* userptr);
GLAPI int gladLoadEGLSimple(EGLDisplay display, GLADsimpleloadproc load);


#ifdef GLAD_EGL

GLAPI int gladLoadEGLInternalLoader(EGLDisplay display);

GLAPI void gladUnloadEGLInternalLoader(void);

#endif
#ifdef __cplusplus
}
#endif
#endif