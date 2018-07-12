#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <glad/glx.h>
#ifdef DYNAMIC_LOAD_X11_FUNCTIONS
#include <dlfcn.h>
#endif

typedef Display* (* PFNXOPENDISPLAY) (_Xconst char* a);
typedef Screen* (* PFNXDEFAULTSCREENOFDISPLAY) (Display*);
typedef int (* PFNXSCREENNUMBEROFSCREEN) (Screen*);
typedef struct
{
    void* library;
    PFNXOPENDISPLAY XOpenDisplay;
    PFNXDEFAULTSCREENOFDISPLAY XDefaultScreenOfDisplay;
    PFNXSCREENNUMBEROFSCREEN XScreenNumberOfScreen;
} X11Struct;
#ifdef DYNAMIC_LOAD_X11_FUNCTIONS
void initX11Struct(X11Struct *x11) {
    const char *X11_LIBRARY = "libX11.so.6";
    x11->library = dlopen(X11_LIBRARY, RTLD_LOCAL | RTLD_NOW);
    if (!x11->library) { fprintf(stderr, "Error opening X11 library %s\n", X11_LIBRARY); exit(EXIT_FAILURE); }
    int missingFunc = 0;
    missingFunc = ((x11->XOpenDisplay = (PFNXOPENDISPLAY) dlsym(x11->library,"XOpenDisplay"))==NULL) | missingFunc;
    if (missingFunc) { fprintf(stderr, "Error: missing func XOpenDisplay in %s, exiting!\n", X11_LIBRARY); exit(EXIT_FAILURE);}
    missingFunc = ((x11->XDefaultScreenOfDisplay = (PFNXDEFAULTSCREENOFDISPLAY) dlsym(x11->library,"XDefaultScreenOfDisplay"))==NULL) | missingFunc;
    if (missingFunc) { fprintf(stderr, "Error: missing func XScreenNumberOfScreen in %s, exiting!\n", X11_LIBRARY); exit(EXIT_FAILURE);}
    missingFunc = ((x11->XScreenNumberOfScreen = (PFNXSCREENNUMBEROFSCREEN) dlsym(x11->library,"XScreenNumberOfScreen"))==NULL) | missingFunc;
    if (missingFunc) { fprintf(stderr, "Error: missing func XScreenNumberOfScreen in %s, exiting!\n", X11_LIBRARY); exit(EXIT_FAILURE);}
    if (!missingFunc) { printf("X11 functions dynamically loaded using dlopen/dlsym OK!\n");}
}
#else
void initX11Struct(X11Struct *x11) {
    x11->XOpenDisplay = XOpenDisplay;
    x11->XDefaultScreenOfDisplay = XDefaultScreenOfDisplay;
    x11->XScreenNumberOfScreen = XScreenNumberOfScreen;
}
#endif  // DYNAMIC_LOAD_X11_FUNCTIONS

#ifndef GLAD_IMPL_UTIL_C_
#define GLAD_IMPL_UTIL_C_

#if _MSC_VER >= 1400
#define GLAD_IMPL_UTIL_STRNCPY(dest, source, len) strncpy_s(dest, len, source, len-1);
#else
#define GLAD_IMPL_UTIL_STRNCPY(dest, source, len) strncpy(dest, source, len);
#endif

#ifdef _MSC_VER
#define GLAD_IMPL_UTIL_SSCANF sscanf_s
#else
#define GLAD_IMPL_UTIL_SSCANF sscanf
#endif

#endif /* GLAD_IMPL_UTIL_C_ */




int GLAD_GLX_VERSION_1_0;
int GLAD_GLX_VERSION_1_1;
int GLAD_GLX_VERSION_1_2;
int GLAD_GLX_VERSION_1_3;
int GLAD_GLX_VERSION_1_4;
int GLAD_GLX_MESA_copy_sub_buffer;
int GLAD_GLX_EXT_create_context_es_profile;
int GLAD_GLX_SGIX_pbuffer;
int GLAD_GLX_SGI_make_current_read;
int GLAD_GLX_OML_sync_control;
int GLAD_GLX_SGIX_hyperpipe;
int GLAD_GLX_INTEL_swap_event;
int GLAD_GLX_EXT_swap_control;
int GLAD_GLX_NV_robustness_video_memory_purge;
int GLAD_GLX_MESA_pixmap_colormap;
int GLAD_GLX_ARB_fbconfig_float;
int GLAD_GLX_EXT_fbconfig_packed_float;
int GLAD_GLX_OML_swap_method;
int GLAD_GLX_NV_video_capture;
int GLAD_GLX_ARB_robustness_application_isolation;
int GLAD_GLX_ARB_create_context_robustness;
int GLAD_GLX_EXT_visual_rating;
int GLAD_GLX_NV_swap_group;
int GLAD_GLX_EXT_texture_from_pixmap;
int GLAD_GLX_SUN_get_transparent_index;
int GLAD_GLX_MESA_release_buffers;
int GLAD_GLX_NV_delay_before_swap;
int GLAD_GLX_EXT_buffer_age;
int GLAD_GLX_MESA_agp_offset;
int GLAD_GLX_EXT_visual_info;
int GLAD_GLX_SGI_swap_control;
int GLAD_GLX_EXT_import_context;
int GLAD_GLX_SGI_video_sync;
int GLAD_GLX_3DFX_multisample;
int GLAD_GLX_ARB_multisample;
int GLAD_GLX_EXT_framebuffer_sRGB;
int GLAD_GLX_SGI_cushion;
int GLAD_GLX_ARB_robustness_share_group_isolation;
int GLAD_GLX_SGIX_fbconfig;
int GLAD_GLX_NV_copy_buffer;
int GLAD_GLX_SGIX_visual_select_group;
int GLAD_GLX_EXT_swap_control_tear;
int GLAD_GLX_ARB_create_context;
int GLAD_GLX_AMD_gpu_association;
int GLAD_GLX_MESA_query_renderer;
int GLAD_GLX_EXT_create_context_es2_profile;
int GLAD_GLX_MESA_swap_control;
int GLAD_GLX_SGIX_video_resize;
int GLAD_GLX_ARB_context_flush_control;
int GLAD_GLX_NV_video_out;
int GLAD_GLX_EXT_no_config_context;
int GLAD_GLX_SGIS_blended_overlay;
int GLAD_GLX_EXT_stereo_tree;
int GLAD_GLX_ARB_create_context_no_error;
int GLAD_GLX_EXT_libglvnd;
int GLAD_GLX_ARB_create_context_profile;
int GLAD_GLX_NV_float_buffer;
int GLAD_GLX_MESA_set_3dfx_mode;
int GLAD_GLX_ARB_framebuffer_sRGB;
int GLAD_GLX_ARB_get_proc_address;
int GLAD_GLX_SGIS_shared_multisample;
int GLAD_GLX_NV_copy_image;
int GLAD_GLX_NV_present_video;
int GLAD_GLX_SGIX_swap_barrier;
int GLAD_GLX_SGIS_multisample;
int GLAD_GLX_SGIX_swap_group;
int GLAD_GLX_ARB_vertex_buffer_object;
int GLAD_GLX_NV_multisample_coverage;



PFNGLXCUSHIONSGIPROC glad_glXCushionSGI;
PFNGLXDESTROYGLXPBUFFERSGIXPROC glad_glXDestroyGLXPbufferSGIX;
PFNGLXSENDPBUFFERTOVIDEONVPROC glad_glXSendPbufferToVideoNV;
PFNGLXGETPROCADDRESSARBPROC glad_glXGetProcAddressARB;
PFNGLXCREATECONTEXTATTRIBSARBPROC glad_glXCreateContextAttribsARB;
PFNGLXJOINSWAPGROUPSGIXPROC glad_glXJoinSwapGroupSGIX;
PFNGLXSWAPINTERVALEXTPROC glad_glXSwapIntervalEXT;
PFNGLXQUERYHYPERPIPECONFIGSGIXPROC glad_glXQueryHyperpipeConfigSGIX;
PFNGLXGETGPUINFOAMDPROC glad_glXGetGPUInfoAMD;
PFNGLXGETTRANSPARENTINDEXSUNPROC glad_glXGetTransparentIndexSUN;
PFNGLXQUERYHYPERPIPENETWORKSGIXPROC glad_glXQueryHyperpipeNetworkSGIX;
PFNGLXQUERYGLXPBUFFERSGIXPROC glad_glXQueryGLXPbufferSGIX;
PFNGLXCREATEGLXPIXMAPWITHCONFIGSGIXPROC glad_glXCreateGLXPixmapWithConfigSGIX;
PFNGLXCOPYCONTEXTPROC glad_glXCopyContext;
PFNGLXCREATEGLXPBUFFERSGIXPROC glad_glXCreateGLXPbufferSGIX;
PFNGLXGETGPUIDSAMDPROC glad_glXGetGPUIDsAMD;
PFNGLXDELETEASSOCIATEDCONTEXTAMDPROC glad_glXDeleteAssociatedContextAMD;
PFNGLXGETVIDEODEVICENVPROC glad_glXGetVideoDeviceNV;
PFNGLXWAITGLPROC glad_glXWaitGL;
PFNGLXGETVIDEOSYNCSGIPROC glad_glXGetVideoSyncSGI;
PFNGLXDESTROYHYPERPIPECONFIGSGIXPROC glad_glXDestroyHyperpipeConfigSGIX;
PFNGLXHYPERPIPECONFIGSGIXPROC glad_glXHyperpipeConfigSGIX;
PFNGLXSWAPBUFFERSMSCOMLPROC glad_glXSwapBuffersMscOML;
PFNGLXCHOOSEFBCONFIGSGIXPROC glad_glXChooseFBConfigSGIX;
PFNGLXWAITVIDEOSYNCSGIPROC glad_glXWaitVideoSyncSGI;
PFNGLXWAITFORSBCOMLPROC glad_glXWaitForSbcOML;
PFNGLXGETPROCADDRESSPROC glad_glXGetProcAddress;
PFNGLXGETSWAPINTERVALMESAPROC glad_glXGetSwapIntervalMESA;
PFNGLXCHANNELRECTSGIXPROC glad_glXChannelRectSGIX;
PFNGLXWAITXPROC glad_glXWaitX;
PFNGLXQUERYSERVERSTRINGPROC glad_glXQueryServerString;
PFNGLXCREATEGLXPIXMAPPROC glad_glXCreateGLXPixmap;
PFNGLXBINDSWAPBARRIERNVPROC glad_glXBindSwapBarrierNV;
PFNGLXQUERYRENDERERSTRINGMESAPROC glad_glXQueryRendererStringMESA;
PFNGLXRELEASEVIDEOCAPTUREDEVICENVPROC glad_glXReleaseVideoCaptureDeviceNV;
PFNGLXWAITFORMSCOMLPROC glad_glXWaitForMscOML;
PFNGLXCOPYSUBBUFFERMESAPROC glad_glXCopySubBufferMESA;
PFNGLXGETCURRENTCONTEXTPROC glad_glXGetCurrentContext;
PFNGLXSELECTEVENTPROC glad_glXSelectEvent;
PFNGLXSWAPINTERVALSGIPROC glad_glXSwapIntervalSGI;
PFNGLXQUERYSWAPGROUPNVPROC glad_glXQuerySwapGroupNV;
PFNGLXMAKECURRENTREADSGIPROC glad_glXMakeCurrentReadSGI;
PFNGLXQUERYDRAWABLEPROC glad_glXQueryDrawable;
PFNGLXCOPYBUFFERSUBDATANVPROC glad_glXCopyBufferSubDataNV;
PFNGLXQUERYCONTEXTPROC glad_glXQueryContext;
PFNGLXGETCONFIGPROC glad_glXGetConfig;
PFNGLXSET3DFXMODEMESAPROC glad_glXSet3DfxModeMESA;
PFNGLXGETVIDEOINFONVPROC glad_glXGetVideoInfoNV;
PFNGLXISDIRECTPROC glad_glXIsDirect;
PFNGLXGETFBCONFIGATTRIBPROC glad_glXGetFBConfigAttrib;
PFNGLXGETVISUALFROMFBCONFIGPROC glad_glXGetVisualFromFBConfig;
PFNGLXBINDTEXIMAGEEXTPROC glad_glXBindTexImageEXT;
PFNGLXBLITCONTEXTFRAMEBUFFERAMDPROC glad_glXBlitContextFramebufferAMD;
PFNGLXQUERYVERSIONPROC glad_glXQueryVersion;
PFNGLXSELECTEVENTSGIXPROC glad_glXSelectEventSGIX;
PFNGLXGETMSCRATEOMLPROC glad_glXGetMscRateOML;
PFNGLXQUERYRENDERERINTEGERMESAPROC glad_glXQueryRendererIntegerMESA;
PFNGLXCREATEWINDOWPROC glad_glXCreateWindow;
PFNGLXCOPYIMAGESUBDATANVPROC glad_glXCopyImageSubDataNV;
PFNGLXSWAPINTERVALMESAPROC glad_glXSwapIntervalMESA;
PFNGLXQUERYHYPERPIPEATTRIBSGIXPROC glad_glXQueryHyperpipeAttribSGIX;
PFNGLXBINDVIDEOCAPTUREDEVICENVPROC glad_glXBindVideoCaptureDeviceNV;
PFNGLXCREATECONTEXTWITHCONFIGSGIXPROC glad_glXCreateContextWithConfigSGIX;
PFNGLXENUMERATEVIDEODEVICESNVPROC glad_glXEnumerateVideoDevicesNV;
PFNGLXHYPERPIPEATTRIBSGIXPROC glad_glXHyperpipeAttribSGIX;
PFNGLXDESTROYPBUFFERPROC glad_glXDestroyPbuffer;
PFNGLXNAMEDCOPYBUFFERSUBDATANVPROC glad_glXNamedCopyBufferSubDataNV;
PFNGLXCREATEPBUFFERPROC glad_glXCreatePbuffer;
PFNGLXQUERYCURRENTRENDERERSTRINGMESAPROC glad_glXQueryCurrentRendererStringMESA;
PFNGLXQUERYHYPERPIPEBESTATTRIBSGIXPROC glad_glXQueryHyperpipeBestAttribSGIX;
PFNGLXGETFBCONFIGATTRIBSGIXPROC glad_glXGetFBConfigAttribSGIX;
PFNGLXQUERYEXTENSIONSSTRINGPROC glad_glXQueryExtensionsString;
PFNGLXBINDVIDEODEVICENVPROC glad_glXBindVideoDeviceNV;
PFNGLXCREATEASSOCIATEDCONTEXTAMDPROC glad_glXCreateAssociatedContextAMD;
PFNGLXQUERYCONTEXTINFOEXTPROC glad_glXQueryContextInfoEXT;
PFNGLXGETSELECTEDEVENTPROC glad_glXGetSelectedEvent;
PFNGLXGETCURRENTDISPLAYPROC glad_glXGetCurrentDisplay;
PFNGLXDESTROYPIXMAPPROC glad_glXDestroyPixmap;
PFNGLXGETSELECTEDEVENTSGIXPROC glad_glXGetSelectedEventSGIX;
PFNGLXRELEASETEXIMAGEEXTPROC glad_glXReleaseTexImageEXT;
PFNGLXUSEXFONTPROC glad_glXUseXFont;
PFNGLXGETSYNCVALUESOMLPROC glad_glXGetSyncValuesOML;
PFNGLXJOINSWAPGROUPNVPROC glad_glXJoinSwapGroupNV;
PFNGLXSWAPBUFFERSPROC glad_glXSwapBuffers;
PFNGLXDELAYBEFORESWAPNVPROC glad_glXDelayBeforeSwapNV;
PFNGLXBINDHYPERPIPESGIXPROC glad_glXBindHyperpipeSGIX;
PFNGLXLOCKVIDEOCAPTUREDEVICENVPROC glad_glXLockVideoCaptureDeviceNV;
PFNGLXGETCURRENTDISPLAYEXTPROC glad_glXGetCurrentDisplayEXT;
PFNGLXGETFBCONFIGSPROC glad_glXGetFBConfigs;
PFNGLXGETCURRENTREADDRAWABLEPROC glad_glXGetCurrentReadDrawable;
PFNGLXRELEASEVIDEOIMAGENVPROC glad_glXReleaseVideoImageNV;
PFNGLXQUERYEXTENSIONPROC glad_glXQueryExtension;
PFNGLXMAKEASSOCIATEDCONTEXTCURRENTAMDPROC glad_glXMakeAssociatedContextCurrentAMD;
PFNGLXCHOOSEVISUALPROC glad_glXChooseVisual;
PFNGLXDESTROYCONTEXTPROC glad_glXDestroyContext;
PFNGLXGETCLIENTSTRINGPROC glad_glXGetClientString;
PFNGLXDESTROYGLXPIXMAPPROC glad_glXDestroyGLXPixmap;
PFNGLXRESETFRAMECOUNTNVPROC glad_glXResetFrameCountNV;
PFNGLXRELEASEVIDEODEVICENVPROC glad_glXReleaseVideoDeviceNV;
PFNGLXGETFBCONFIGFROMVISUALSGIXPROC glad_glXGetFBConfigFromVisualSGIX;
PFNGLXCREATENEWCONTEXTPROC glad_glXCreateNewContext;
PFNGLXMAKECONTEXTCURRENTPROC glad_glXMakeContextCurrent;
PFNGLXQUERYMAXSWAPGROUPSNVPROC glad_glXQueryMaxSwapGroupsNV;
PFNGLXGETVISUALFROMFBCONFIGSGIXPROC glad_glXGetVisualFromFBConfigSGIX;
PFNGLXGETCURRENTDRAWABLEPROC glad_glXGetCurrentDrawable;
PFNGLXGETCURRENTREADDRAWABLESGIPROC glad_glXGetCurrentReadDrawableSGI;
PFNGLXQUERYFRAMECOUNTNVPROC glad_glXQueryFrameCountNV;
PFNGLXGETCONTEXTGPUIDAMDPROC glad_glXGetContextGPUIDAMD;
PFNGLXBINDSWAPBARRIERSGIXPROC glad_glXBindSwapBarrierSGIX;
PFNGLXQUERYMAXSWAPBARRIERSSGIXPROC glad_glXQueryMaxSwapBarriersSGIX;
PFNGLXCREATEGLXPIXMAPMESAPROC glad_glXCreateGLXPixmapMESA;
PFNGLXMAKECURRENTPROC glad_glXMakeCurrent;
PFNGLXDESTROYWINDOWPROC glad_glXDestroyWindow;
PFNGLXBINDVIDEOIMAGENVPROC glad_glXBindVideoImageNV;
PFNGLXQUERYVIDEOCAPTUREDEVICENVPROC glad_glXQueryVideoCaptureDeviceNV;
PFNGLXQUERYCHANNELDELTASSGIXPROC glad_glXQueryChannelDeltasSGIX;
PFNGLXENUMERATEVIDEOCAPTUREDEVICESNVPROC glad_glXEnumerateVideoCaptureDevicesNV;
PFNGLXCREATEASSOCIATEDCONTEXTATTRIBSAMDPROC glad_glXCreateAssociatedContextAttribsAMD;
PFNGLXGETCURRENTASSOCIATEDCONTEXTAMDPROC glad_glXGetCurrentAssociatedContextAMD;
PFNGLXQUERYCHANNELRECTSGIXPROC glad_glXQueryChannelRectSGIX;
PFNGLXCREATECONTEXTPROC glad_glXCreateContext;
PFNGLXFREECONTEXTEXTPROC glad_glXFreeContextEXT;
PFNGLXGETCONTEXTIDEXTPROC glad_glXGetContextIDEXT;
PFNGLXQUERYCURRENTRENDERERINTEGERMESAPROC glad_glXQueryCurrentRendererIntegerMESA;
PFNGLXCHOOSEFBCONFIGPROC glad_glXChooseFBConfig;
PFNGLXBINDCHANNELTOWINDOWSGIXPROC glad_glXBindChannelToWindowSGIX;
PFNGLXCHANNELRECTSYNCSGIXPROC glad_glXChannelRectSyncSGIX;
PFNGLXIMPORTCONTEXTEXTPROC glad_glXImportContextEXT;
PFNGLXRELEASEBUFFERSMESAPROC glad_glXReleaseBuffersMESA;
PFNGLXCREATEPIXMAPPROC glad_glXCreatePixmap;
PFNGLXGETAGPOFFSETMESAPROC glad_glXGetAGPOffsetMESA;


static void load_GLX_VERSION_1_0( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_VERSION_1_0) return;
    glXWaitGL = (PFNGLXWAITGLPROC) load("glXWaitGL", userptr);
    glXWaitX = (PFNGLXWAITXPROC) load("glXWaitX", userptr);
    glXGetConfig = (PFNGLXGETCONFIGPROC) load("glXGetConfig", userptr);
    glXQueryVersion = (PFNGLXQUERYVERSIONPROC) load("glXQueryVersion", userptr);
    glXCreateContext = (PFNGLXCREATECONTEXTPROC) load("glXCreateContext", userptr);
    glXQueryExtension = (PFNGLXQUERYEXTENSIONPROC) load("glXQueryExtension", userptr);
    glXGetCurrentContext = (PFNGLXGETCURRENTCONTEXTPROC) load("glXGetCurrentContext", userptr);
    glXMakeCurrent = (PFNGLXMAKECURRENTPROC) load("glXMakeCurrent", userptr);
    glXCopyContext = (PFNGLXCOPYCONTEXTPROC) load("glXCopyContext", userptr);
    glXIsDirect = (PFNGLXISDIRECTPROC) load("glXIsDirect", userptr);
    glXCreateGLXPixmap = (PFNGLXCREATEGLXPIXMAPPROC) load("glXCreateGLXPixmap", userptr);
    glXGetCurrentDrawable = (PFNGLXGETCURRENTDRAWABLEPROC) load("glXGetCurrentDrawable", userptr);
    glXUseXFont = (PFNGLXUSEXFONTPROC) load("glXUseXFont", userptr);
    glXChooseVisual = (PFNGLXCHOOSEVISUALPROC) load("glXChooseVisual", userptr);
    glXDestroyContext = (PFNGLXDESTROYCONTEXTPROC) load("glXDestroyContext", userptr);
    glXDestroyGLXPixmap = (PFNGLXDESTROYGLXPIXMAPPROC) load("glXDestroyGLXPixmap", userptr);
    glXSwapBuffers = (PFNGLXSWAPBUFFERSPROC) load("glXSwapBuffers", userptr);
}
static void load_GLX_VERSION_1_1( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_VERSION_1_1) return;
    glXQueryServerString = (PFNGLXQUERYSERVERSTRINGPROC) load("glXQueryServerString", userptr);
    glXGetClientString = (PFNGLXGETCLIENTSTRINGPROC) load("glXGetClientString", userptr);
    glXQueryExtensionsString = (PFNGLXQUERYEXTENSIONSSTRINGPROC) load("glXQueryExtensionsString", userptr);
}
static void load_GLX_VERSION_1_2( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_VERSION_1_2) return;
    glXGetCurrentDisplay = (PFNGLXGETCURRENTDISPLAYPROC) load("glXGetCurrentDisplay", userptr);
}
static void load_GLX_VERSION_1_3( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_VERSION_1_3) return;
    glXQueryContext = (PFNGLXQUERYCONTEXTPROC) load("glXQueryContext", userptr);
    glXGetFBConfigs = (PFNGLXGETFBCONFIGSPROC) load("glXGetFBConfigs", userptr);
    glXDestroyPixmap = (PFNGLXDESTROYPIXMAPPROC) load("glXDestroyPixmap", userptr);
    glXCreateNewContext = (PFNGLXCREATENEWCONTEXTPROC) load("glXCreateNewContext", userptr);
    glXGetCurrentReadDrawable = (PFNGLXGETCURRENTREADDRAWABLEPROC) load("glXGetCurrentReadDrawable", userptr);
    glXMakeContextCurrent = (PFNGLXMAKECONTEXTCURRENTPROC) load("glXMakeContextCurrent", userptr);
    glXDestroyWindow = (PFNGLXDESTROYWINDOWPROC) load("glXDestroyWindow", userptr);
    glXChooseFBConfig = (PFNGLXCHOOSEFBCONFIGPROC) load("glXChooseFBConfig", userptr);
    glXCreatePixmap = (PFNGLXCREATEPIXMAPPROC) load("glXCreatePixmap", userptr);
    glXSelectEvent = (PFNGLXSELECTEVENTPROC) load("glXSelectEvent", userptr);
    glXGetFBConfigAttrib = (PFNGLXGETFBCONFIGATTRIBPROC) load("glXGetFBConfigAttrib", userptr);
    glXDestroyPbuffer = (PFNGLXDESTROYPBUFFERPROC) load("glXDestroyPbuffer", userptr);
    glXCreatePbuffer = (PFNGLXCREATEPBUFFERPROC) load("glXCreatePbuffer", userptr);
    glXCreateWindow = (PFNGLXCREATEWINDOWPROC) load("glXCreateWindow", userptr);
    glXGetSelectedEvent = (PFNGLXGETSELECTEDEVENTPROC) load("glXGetSelectedEvent", userptr);
    glXQueryDrawable = (PFNGLXQUERYDRAWABLEPROC) load("glXQueryDrawable", userptr);
    glXGetVisualFromFBConfig = (PFNGLXGETVISUALFROMFBCONFIGPROC) load("glXGetVisualFromFBConfig", userptr);
}
static void load_GLX_VERSION_1_4( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_VERSION_1_4) return;
    glXGetProcAddress = (PFNGLXGETPROCADDRESSPROC) load("glXGetProcAddress", userptr);
}
static void load_GLX_MESA_copy_sub_buffer( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_MESA_copy_sub_buffer) return;
    glXCopySubBufferMESA = (PFNGLXCOPYSUBBUFFERMESAPROC) load("glXCopySubBufferMESA", userptr);
}
static void load_GLX_SGIX_pbuffer( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_SGIX_pbuffer) return;
    glXDestroyGLXPbufferSGIX = (PFNGLXDESTROYGLXPBUFFERSGIXPROC) load("glXDestroyGLXPbufferSGIX", userptr);
    glXQueryGLXPbufferSGIX = (PFNGLXQUERYGLXPBUFFERSGIXPROC) load("glXQueryGLXPbufferSGIX", userptr);
    glXCreateGLXPbufferSGIX = (PFNGLXCREATEGLXPBUFFERSGIXPROC) load("glXCreateGLXPbufferSGIX", userptr);
    glXGetSelectedEventSGIX = (PFNGLXGETSELECTEDEVENTSGIXPROC) load("glXGetSelectedEventSGIX", userptr);
    glXSelectEventSGIX = (PFNGLXSELECTEVENTSGIXPROC) load("glXSelectEventSGIX", userptr);
}
static void load_GLX_SGI_make_current_read( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_SGI_make_current_read) return;
    glXGetCurrentReadDrawableSGI = (PFNGLXGETCURRENTREADDRAWABLESGIPROC) load("glXGetCurrentReadDrawableSGI", userptr);
    glXMakeCurrentReadSGI = (PFNGLXMAKECURRENTREADSGIPROC) load("glXMakeCurrentReadSGI", userptr);
}
static void load_GLX_OML_sync_control( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_OML_sync_control) return;
    glXWaitForMscOML = (PFNGLXWAITFORMSCOMLPROC) load("glXWaitForMscOML", userptr);
    glXGetSyncValuesOML = (PFNGLXGETSYNCVALUESOMLPROC) load("glXGetSyncValuesOML", userptr);
    glXWaitForSbcOML = (PFNGLXWAITFORSBCOMLPROC) load("glXWaitForSbcOML", userptr);
    glXSwapBuffersMscOML = (PFNGLXSWAPBUFFERSMSCOMLPROC) load("glXSwapBuffersMscOML", userptr);
    glXGetMscRateOML = (PFNGLXGETMSCRATEOMLPROC) load("glXGetMscRateOML", userptr);
}
static void load_GLX_SGIX_hyperpipe( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_SGIX_hyperpipe) return;
    glXQueryHyperpipeBestAttribSGIX = (PFNGLXQUERYHYPERPIPEBESTATTRIBSGIXPROC) load("glXQueryHyperpipeBestAttribSGIX", userptr);
    glXQueryHyperpipeAttribSGIX = (PFNGLXQUERYHYPERPIPEATTRIBSGIXPROC) load("glXQueryHyperpipeAttribSGIX", userptr);
    glXDestroyHyperpipeConfigSGIX = (PFNGLXDESTROYHYPERPIPECONFIGSGIXPROC) load("glXDestroyHyperpipeConfigSGIX", userptr);
    glXHyperpipeConfigSGIX = (PFNGLXHYPERPIPECONFIGSGIXPROC) load("glXHyperpipeConfigSGIX", userptr);
    glXHyperpipeAttribSGIX = (PFNGLXHYPERPIPEATTRIBSGIXPROC) load("glXHyperpipeAttribSGIX", userptr);
    glXQueryHyperpipeConfigSGIX = (PFNGLXQUERYHYPERPIPECONFIGSGIXPROC) load("glXQueryHyperpipeConfigSGIX", userptr);
    glXQueryHyperpipeNetworkSGIX = (PFNGLXQUERYHYPERPIPENETWORKSGIXPROC) load("glXQueryHyperpipeNetworkSGIX", userptr);
    glXBindHyperpipeSGIX = (PFNGLXBINDHYPERPIPESGIXPROC) load("glXBindHyperpipeSGIX", userptr);
}
static void load_GLX_EXT_swap_control( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_EXT_swap_control) return;
    glXSwapIntervalEXT = (PFNGLXSWAPINTERVALEXTPROC) load("glXSwapIntervalEXT", userptr);
}
static void load_GLX_MESA_pixmap_colormap( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_MESA_pixmap_colormap) return;
    glXCreateGLXPixmapMESA = (PFNGLXCREATEGLXPIXMAPMESAPROC) load("glXCreateGLXPixmapMESA", userptr);
}
static void load_GLX_NV_video_capture( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_NV_video_capture) return;
    glXQueryVideoCaptureDeviceNV = (PFNGLXQUERYVIDEOCAPTUREDEVICENVPROC) load("glXQueryVideoCaptureDeviceNV", userptr);
    glXEnumerateVideoCaptureDevicesNV = (PFNGLXENUMERATEVIDEOCAPTUREDEVICESNVPROC) load("glXEnumerateVideoCaptureDevicesNV", userptr);
    glXBindVideoCaptureDeviceNV = (PFNGLXBINDVIDEOCAPTUREDEVICENVPROC) load("glXBindVideoCaptureDeviceNV", userptr);
    glXLockVideoCaptureDeviceNV = (PFNGLXLOCKVIDEOCAPTUREDEVICENVPROC) load("glXLockVideoCaptureDeviceNV", userptr);
    glXReleaseVideoCaptureDeviceNV = (PFNGLXRELEASEVIDEOCAPTUREDEVICENVPROC) load("glXReleaseVideoCaptureDeviceNV", userptr);
}
static void load_GLX_NV_swap_group( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_NV_swap_group) return;
    glXQueryFrameCountNV = (PFNGLXQUERYFRAMECOUNTNVPROC) load("glXQueryFrameCountNV", userptr);
    glXQueryMaxSwapGroupsNV = (PFNGLXQUERYMAXSWAPGROUPSNVPROC) load("glXQueryMaxSwapGroupsNV", userptr);
    glXResetFrameCountNV = (PFNGLXRESETFRAMECOUNTNVPROC) load("glXResetFrameCountNV", userptr);
    glXJoinSwapGroupNV = (PFNGLXJOINSWAPGROUPNVPROC) load("glXJoinSwapGroupNV", userptr);
    glXBindSwapBarrierNV = (PFNGLXBINDSWAPBARRIERNVPROC) load("glXBindSwapBarrierNV", userptr);
    glXQuerySwapGroupNV = (PFNGLXQUERYSWAPGROUPNVPROC) load("glXQuerySwapGroupNV", userptr);
}
static void load_GLX_EXT_texture_from_pixmap( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_EXT_texture_from_pixmap) return;
    glXReleaseTexImageEXT = (PFNGLXRELEASETEXIMAGEEXTPROC) load("glXReleaseTexImageEXT", userptr);
    glXBindTexImageEXT = (PFNGLXBINDTEXIMAGEEXTPROC) load("glXBindTexImageEXT", userptr);
}
static void load_GLX_SUN_get_transparent_index( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_SUN_get_transparent_index) return;
    glXGetTransparentIndexSUN = (PFNGLXGETTRANSPARENTINDEXSUNPROC) load("glXGetTransparentIndexSUN", userptr);
}
static void load_GLX_MESA_release_buffers( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_MESA_release_buffers) return;
    glXReleaseBuffersMESA = (PFNGLXRELEASEBUFFERSMESAPROC) load("glXReleaseBuffersMESA", userptr);
}
static void load_GLX_NV_delay_before_swap( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_NV_delay_before_swap) return;
    glXDelayBeforeSwapNV = (PFNGLXDELAYBEFORESWAPNVPROC) load("glXDelayBeforeSwapNV", userptr);
}
static void load_GLX_MESA_agp_offset( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_MESA_agp_offset) return;
    glXGetAGPOffsetMESA = (PFNGLXGETAGPOFFSETMESAPROC) load("glXGetAGPOffsetMESA", userptr);
}
static void load_GLX_SGI_swap_control( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_SGI_swap_control) return;
    glXSwapIntervalSGI = (PFNGLXSWAPINTERVALSGIPROC) load("glXSwapIntervalSGI", userptr);
}
static void load_GLX_EXT_import_context( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_EXT_import_context) return;
    glXGetCurrentDisplayEXT = (PFNGLXGETCURRENTDISPLAYEXTPROC) load("glXGetCurrentDisplayEXT", userptr);
    glXImportContextEXT = (PFNGLXIMPORTCONTEXTEXTPROC) load("glXImportContextEXT", userptr);
    glXFreeContextEXT = (PFNGLXFREECONTEXTEXTPROC) load("glXFreeContextEXT", userptr);
    glXGetContextIDEXT = (PFNGLXGETCONTEXTIDEXTPROC) load("glXGetContextIDEXT", userptr);
    glXQueryContextInfoEXT = (PFNGLXQUERYCONTEXTINFOEXTPROC) load("glXQueryContextInfoEXT", userptr);
}
static void load_GLX_SGI_video_sync( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_SGI_video_sync) return;
    glXGetVideoSyncSGI = (PFNGLXGETVIDEOSYNCSGIPROC) load("glXGetVideoSyncSGI", userptr);
    glXWaitVideoSyncSGI = (PFNGLXWAITVIDEOSYNCSGIPROC) load("glXWaitVideoSyncSGI", userptr);
}
static void load_GLX_SGI_cushion( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_SGI_cushion) return;
    glXCushionSGI = (PFNGLXCUSHIONSGIPROC) load("glXCushionSGI", userptr);
}
static void load_GLX_SGIX_fbconfig( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_SGIX_fbconfig) return;
    glXGetFBConfigAttribSGIX = (PFNGLXGETFBCONFIGATTRIBSGIXPROC) load("glXGetFBConfigAttribSGIX", userptr);
    glXCreateGLXPixmapWithConfigSGIX = (PFNGLXCREATEGLXPIXMAPWITHCONFIGSGIXPROC) load("glXCreateGLXPixmapWithConfigSGIX", userptr);
    glXCreateContextWithConfigSGIX = (PFNGLXCREATECONTEXTWITHCONFIGSGIXPROC) load("glXCreateContextWithConfigSGIX", userptr);
    glXGetVisualFromFBConfigSGIX = (PFNGLXGETVISUALFROMFBCONFIGSGIXPROC) load("glXGetVisualFromFBConfigSGIX", userptr);
    glXChooseFBConfigSGIX = (PFNGLXCHOOSEFBCONFIGSGIXPROC) load("glXChooseFBConfigSGIX", userptr);
    glXGetFBConfigFromVisualSGIX = (PFNGLXGETFBCONFIGFROMVISUALSGIXPROC) load("glXGetFBConfigFromVisualSGIX", userptr);
}
static void load_GLX_NV_copy_buffer( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_NV_copy_buffer) return;
    glXNamedCopyBufferSubDataNV = (PFNGLXNAMEDCOPYBUFFERSUBDATANVPROC) load("glXNamedCopyBufferSubDataNV", userptr);
    glXCopyBufferSubDataNV = (PFNGLXCOPYBUFFERSUBDATANVPROC) load("glXCopyBufferSubDataNV", userptr);
}
static void load_GLX_ARB_create_context( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_ARB_create_context) return;
    glXCreateContextAttribsARB = (PFNGLXCREATECONTEXTATTRIBSARBPROC) load("glXCreateContextAttribsARB", userptr);
}
static void load_GLX_AMD_gpu_association( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_AMD_gpu_association) return;
    glXCreateAssociatedContextAttribsAMD = (PFNGLXCREATEASSOCIATEDCONTEXTATTRIBSAMDPROC) load("glXCreateAssociatedContextAttribsAMD", userptr);
    glXBlitContextFramebufferAMD = (PFNGLXBLITCONTEXTFRAMEBUFFERAMDPROC) load("glXBlitContextFramebufferAMD", userptr);
    glXGetCurrentAssociatedContextAMD = (PFNGLXGETCURRENTASSOCIATEDCONTEXTAMDPROC) load("glXGetCurrentAssociatedContextAMD", userptr);
    glXGetContextGPUIDAMD = (PFNGLXGETCONTEXTGPUIDAMDPROC) load("glXGetContextGPUIDAMD", userptr);
    glXMakeAssociatedContextCurrentAMD = (PFNGLXMAKEASSOCIATEDCONTEXTCURRENTAMDPROC) load("glXMakeAssociatedContextCurrentAMD", userptr);
    glXGetGPUIDsAMD = (PFNGLXGETGPUIDSAMDPROC) load("glXGetGPUIDsAMD", userptr);
    glXCreateAssociatedContextAMD = (PFNGLXCREATEASSOCIATEDCONTEXTAMDPROC) load("glXCreateAssociatedContextAMD", userptr);
    glXDeleteAssociatedContextAMD = (PFNGLXDELETEASSOCIATEDCONTEXTAMDPROC) load("glXDeleteAssociatedContextAMD", userptr);
    glXGetGPUInfoAMD = (PFNGLXGETGPUINFOAMDPROC) load("glXGetGPUInfoAMD", userptr);
}
static void load_GLX_MESA_query_renderer( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_MESA_query_renderer) return;
    glXQueryCurrentRendererStringMESA = (PFNGLXQUERYCURRENTRENDERERSTRINGMESAPROC) load("glXQueryCurrentRendererStringMESA", userptr);
    glXQueryCurrentRendererIntegerMESA = (PFNGLXQUERYCURRENTRENDERERINTEGERMESAPROC) load("glXQueryCurrentRendererIntegerMESA", userptr);
    glXQueryRendererStringMESA = (PFNGLXQUERYRENDERERSTRINGMESAPROC) load("glXQueryRendererStringMESA", userptr);
    glXQueryRendererIntegerMESA = (PFNGLXQUERYRENDERERINTEGERMESAPROC) load("glXQueryRendererIntegerMESA", userptr);
}
static void load_GLX_MESA_swap_control( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_MESA_swap_control) return;
    glXGetSwapIntervalMESA = (PFNGLXGETSWAPINTERVALMESAPROC) load("glXGetSwapIntervalMESA", userptr);
    glXSwapIntervalMESA = (PFNGLXSWAPINTERVALMESAPROC) load("glXSwapIntervalMESA", userptr);
}
static void load_GLX_SGIX_video_resize( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_SGIX_video_resize) return;
    glXBindChannelToWindowSGIX = (PFNGLXBINDCHANNELTOWINDOWSGIXPROC) load("glXBindChannelToWindowSGIX", userptr);
    glXChannelRectSyncSGIX = (PFNGLXCHANNELRECTSYNCSGIXPROC) load("glXChannelRectSyncSGIX", userptr);
    glXQueryChannelDeltasSGIX = (PFNGLXQUERYCHANNELDELTASSGIXPROC) load("glXQueryChannelDeltasSGIX", userptr);
    glXQueryChannelRectSGIX = (PFNGLXQUERYCHANNELRECTSGIXPROC) load("glXQueryChannelRectSGIX", userptr);
    glXChannelRectSGIX = (PFNGLXCHANNELRECTSGIXPROC) load("glXChannelRectSGIX", userptr);
}
static void load_GLX_NV_video_out( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_NV_video_out) return;
    glXGetVideoInfoNV = (PFNGLXGETVIDEOINFONVPROC) load("glXGetVideoInfoNV", userptr);
    glXSendPbufferToVideoNV = (PFNGLXSENDPBUFFERTOVIDEONVPROC) load("glXSendPbufferToVideoNV", userptr);
    glXReleaseVideoImageNV = (PFNGLXRELEASEVIDEOIMAGENVPROC) load("glXReleaseVideoImageNV", userptr);
    glXBindVideoImageNV = (PFNGLXBINDVIDEOIMAGENVPROC) load("glXBindVideoImageNV", userptr);
    glXReleaseVideoDeviceNV = (PFNGLXRELEASEVIDEODEVICENVPROC) load("glXReleaseVideoDeviceNV", userptr);
    glXGetVideoDeviceNV = (PFNGLXGETVIDEODEVICENVPROC) load("glXGetVideoDeviceNV", userptr);
}
static void load_GLX_MESA_set_3dfx_mode( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_MESA_set_3dfx_mode) return;
    glXSet3DfxModeMESA = (PFNGLXSET3DFXMODEMESAPROC) load("glXSet3DfxModeMESA", userptr);
}
static void load_GLX_ARB_get_proc_address( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_ARB_get_proc_address) return;
    glXGetProcAddressARB = (PFNGLXGETPROCADDRESSARBPROC) load("glXGetProcAddressARB", userptr);
}
static void load_GLX_NV_copy_image( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_NV_copy_image) return;
    glXCopyImageSubDataNV = (PFNGLXCOPYIMAGESUBDATANVPROC) load("glXCopyImageSubDataNV", userptr);
}
static void load_GLX_NV_present_video( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_NV_present_video) return;
    glXBindVideoDeviceNV = (PFNGLXBINDVIDEODEVICENVPROC) load("glXBindVideoDeviceNV", userptr);
    glXEnumerateVideoDevicesNV = (PFNGLXENUMERATEVIDEODEVICESNVPROC) load("glXEnumerateVideoDevicesNV", userptr);
}
static void load_GLX_SGIX_swap_barrier( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_SGIX_swap_barrier) return;
    glXBindSwapBarrierSGIX = (PFNGLXBINDSWAPBARRIERSGIXPROC) load("glXBindSwapBarrierSGIX", userptr);
    glXQueryMaxSwapBarriersSGIX = (PFNGLXQUERYMAXSWAPBARRIERSSGIXPROC) load("glXQueryMaxSwapBarriersSGIX", userptr);
}
static void load_GLX_SGIX_swap_group( GLADuserptrloadfunc load, void* userptr) {
    if(!GLAD_GLX_SGIX_swap_group) return;
    glXJoinSwapGroupSGIX = (PFNGLXJOINSWAPGROUPSGIXPROC) load("glXJoinSwapGroupSGIX", userptr);
}



static int has_ext(Display *display, int screen, const char *ext) {
    const char *terminator;
    const char *loc;
    const char *extensions;

    if(!GLAD_GLX_VERSION_1_1)
        return 0;

    extensions = glXQueryExtensionsString(display, screen);

    if(extensions == NULL || ext == NULL)
        return 0;

    while(1) {
        loc = strstr(extensions, ext);
        if(loc == NULL)
            break;

        terminator = loc + strlen(ext);
        if((loc == extensions || *(loc - 1) == ' ') &&
            (*terminator == ' ' || *terminator == '\0'))
        {
            return 1;
        }
        extensions = terminator;
    }

    return 0;
}

static GLADapiproc glad_glx_get_proc_from_userptr(const char* name, void *userptr) {
    return (GLAD_GNUC_EXTENSION (GLADapiproc (*)(const char *name)) userptr)(name);
}

static int find_extensionsGLX(Display *display, int screen) {
    GLAD_GLX_MESA_copy_sub_buffer = has_ext(display, screen, "GLX_MESA_copy_sub_buffer");
    GLAD_GLX_EXT_create_context_es_profile = has_ext(display, screen, "GLX_EXT_create_context_es_profile");
    GLAD_GLX_SGIX_pbuffer = has_ext(display, screen, "GLX_SGIX_pbuffer");
    GLAD_GLX_SGI_make_current_read = has_ext(display, screen, "GLX_SGI_make_current_read");
    GLAD_GLX_OML_sync_control = has_ext(display, screen, "GLX_OML_sync_control");
    GLAD_GLX_SGIX_hyperpipe = has_ext(display, screen, "GLX_SGIX_hyperpipe");
    GLAD_GLX_INTEL_swap_event = has_ext(display, screen, "GLX_INTEL_swap_event");
    GLAD_GLX_EXT_swap_control = has_ext(display, screen, "GLX_EXT_swap_control");
    GLAD_GLX_NV_robustness_video_memory_purge = has_ext(display, screen, "GLX_NV_robustness_video_memory_purge");
    GLAD_GLX_MESA_pixmap_colormap = has_ext(display, screen, "GLX_MESA_pixmap_colormap");
    GLAD_GLX_ARB_fbconfig_float = has_ext(display, screen, "GLX_ARB_fbconfig_float");
    GLAD_GLX_EXT_fbconfig_packed_float = has_ext(display, screen, "GLX_EXT_fbconfig_packed_float");
    GLAD_GLX_OML_swap_method = has_ext(display, screen, "GLX_OML_swap_method");
    GLAD_GLX_NV_video_capture = has_ext(display, screen, "GLX_NV_video_capture");
    GLAD_GLX_ARB_robustness_application_isolation = has_ext(display, screen, "GLX_ARB_robustness_application_isolation");
    GLAD_GLX_ARB_create_context_robustness = has_ext(display, screen, "GLX_ARB_create_context_robustness");
    GLAD_GLX_EXT_visual_rating = has_ext(display, screen, "GLX_EXT_visual_rating");
    GLAD_GLX_NV_swap_group = has_ext(display, screen, "GLX_NV_swap_group");
    GLAD_GLX_EXT_texture_from_pixmap = has_ext(display, screen, "GLX_EXT_texture_from_pixmap");
    GLAD_GLX_SUN_get_transparent_index = has_ext(display, screen, "GLX_SUN_get_transparent_index");
    GLAD_GLX_MESA_release_buffers = has_ext(display, screen, "GLX_MESA_release_buffers");
    GLAD_GLX_NV_delay_before_swap = has_ext(display, screen, "GLX_NV_delay_before_swap");
    GLAD_GLX_EXT_buffer_age = has_ext(display, screen, "GLX_EXT_buffer_age");
    GLAD_GLX_MESA_agp_offset = has_ext(display, screen, "GLX_MESA_agp_offset");
    GLAD_GLX_EXT_visual_info = has_ext(display, screen, "GLX_EXT_visual_info");
    GLAD_GLX_SGI_swap_control = has_ext(display, screen, "GLX_SGI_swap_control");
    GLAD_GLX_EXT_import_context = has_ext(display, screen, "GLX_EXT_import_context");
    GLAD_GLX_SGI_video_sync = has_ext(display, screen, "GLX_SGI_video_sync");
    GLAD_GLX_3DFX_multisample = has_ext(display, screen, "GLX_3DFX_multisample");
    GLAD_GLX_ARB_multisample = has_ext(display, screen, "GLX_ARB_multisample");
    GLAD_GLX_EXT_framebuffer_sRGB = has_ext(display, screen, "GLX_EXT_framebuffer_sRGB");
    GLAD_GLX_SGI_cushion = has_ext(display, screen, "GLX_SGI_cushion");
    GLAD_GLX_ARB_robustness_share_group_isolation = has_ext(display, screen, "GLX_ARB_robustness_share_group_isolation");
    GLAD_GLX_SGIX_fbconfig = has_ext(display, screen, "GLX_SGIX_fbconfig");
    GLAD_GLX_NV_copy_buffer = has_ext(display, screen, "GLX_NV_copy_buffer");
    GLAD_GLX_SGIX_visual_select_group = has_ext(display, screen, "GLX_SGIX_visual_select_group");
    GLAD_GLX_EXT_swap_control_tear = has_ext(display, screen, "GLX_EXT_swap_control_tear");
    GLAD_GLX_ARB_create_context = has_ext(display, screen, "GLX_ARB_create_context");
    GLAD_GLX_AMD_gpu_association = has_ext(display, screen, "GLX_AMD_gpu_association");
    GLAD_GLX_MESA_query_renderer = has_ext(display, screen, "GLX_MESA_query_renderer");
    GLAD_GLX_EXT_create_context_es2_profile = has_ext(display, screen, "GLX_EXT_create_context_es2_profile");
    GLAD_GLX_MESA_swap_control = has_ext(display, screen, "GLX_MESA_swap_control");
    GLAD_GLX_SGIX_video_resize = has_ext(display, screen, "GLX_SGIX_video_resize");
    GLAD_GLX_ARB_context_flush_control = has_ext(display, screen, "GLX_ARB_context_flush_control");
    GLAD_GLX_NV_video_out = has_ext(display, screen, "GLX_NV_video_out");
    GLAD_GLX_EXT_no_config_context = has_ext(display, screen, "GLX_EXT_no_config_context");
    GLAD_GLX_SGIS_blended_overlay = has_ext(display, screen, "GLX_SGIS_blended_overlay");
    GLAD_GLX_EXT_stereo_tree = has_ext(display, screen, "GLX_EXT_stereo_tree");
    GLAD_GLX_ARB_create_context_no_error = has_ext(display, screen, "GLX_ARB_create_context_no_error");
    GLAD_GLX_EXT_libglvnd = has_ext(display, screen, "GLX_EXT_libglvnd");
    GLAD_GLX_ARB_create_context_profile = has_ext(display, screen, "GLX_ARB_create_context_profile");
    GLAD_GLX_NV_float_buffer = has_ext(display, screen, "GLX_NV_float_buffer");
    GLAD_GLX_MESA_set_3dfx_mode = has_ext(display, screen, "GLX_MESA_set_3dfx_mode");
    GLAD_GLX_ARB_framebuffer_sRGB = has_ext(display, screen, "GLX_ARB_framebuffer_sRGB");
    GLAD_GLX_ARB_get_proc_address = has_ext(display, screen, "GLX_ARB_get_proc_address");
    GLAD_GLX_SGIS_shared_multisample = has_ext(display, screen, "GLX_SGIS_shared_multisample");
    GLAD_GLX_NV_copy_image = has_ext(display, screen, "GLX_NV_copy_image");
    GLAD_GLX_NV_present_video = has_ext(display, screen, "GLX_NV_present_video");
    GLAD_GLX_SGIX_swap_barrier = has_ext(display, screen, "GLX_SGIX_swap_barrier");
    GLAD_GLX_SGIS_multisample = has_ext(display, screen, "GLX_SGIS_multisample");
    GLAD_GLX_SGIX_swap_group = has_ext(display, screen, "GLX_SGIX_swap_group");
    GLAD_GLX_ARB_vertex_buffer_object = has_ext(display, screen, "GLX_ARB_vertex_buffer_object");
    GLAD_GLX_NV_multisample_coverage = has_ext(display, screen, "GLX_NV_multisample_coverage");
    return 1;
}

static int find_coreGLX(Display **display, int *screen) {
    X11Struct x11;
    initX11Struct(&x11);
    int major = 0, minor = 0;
    if(*display == NULL) {
        *display = x11.XOpenDisplay(0);
        if (*display == NULL) {
            return 0;
        }
        *screen = x11.XScreenNumberOfScreen(x11.XDefaultScreenOfDisplay(*display));
    }
    glXQueryVersion(*display, &major, &minor);
    GLAD_GLX_VERSION_1_0 = (major == 1 && minor >= 0) || major > 1;
    GLAD_GLX_VERSION_1_1 = (major == 1 && minor >= 1) || major > 1;
    GLAD_GLX_VERSION_1_2 = (major == 1 && minor >= 2) || major > 1;
    GLAD_GLX_VERSION_1_3 = (major == 1 && minor >= 3) || major > 1;
    GLAD_GLX_VERSION_1_4 = (major == 1 && minor >= 4) || major > 1;
    return GLAD_MAKE_VERSION(major, minor);
}

int gladLoadGLXUserPtr(Display *display, int screen, GLADuserptrloadfunc load, void *userptr) {
    int version;
    glXQueryVersion = (PFNGLXQUERYVERSIONPROC) load("glXQueryVersion", userptr);
    if(glXQueryVersion == NULL) return 0;
    version = find_coreGLX(&display, &screen);

    load_GLX_VERSION_1_0(load, userptr);
    load_GLX_VERSION_1_1(load, userptr);
    load_GLX_VERSION_1_2(load, userptr);
    load_GLX_VERSION_1_3(load, userptr);
    load_GLX_VERSION_1_4(load, userptr);

    if (!find_extensionsGLX(display, screen)) return 0;
    load_GLX_MESA_copy_sub_buffer(load, userptr);
    load_GLX_SGIX_pbuffer(load, userptr);
    load_GLX_SGI_make_current_read(load, userptr);
    load_GLX_OML_sync_control(load, userptr);
    load_GLX_SGIX_hyperpipe(load, userptr);
    load_GLX_EXT_swap_control(load, userptr);
    load_GLX_MESA_pixmap_colormap(load, userptr);
    load_GLX_NV_video_capture(load, userptr);
    load_GLX_NV_swap_group(load, userptr);
    load_GLX_EXT_texture_from_pixmap(load, userptr);
    load_GLX_SUN_get_transparent_index(load, userptr);
    load_GLX_MESA_release_buffers(load, userptr);
    load_GLX_NV_delay_before_swap(load, userptr);
    load_GLX_MESA_agp_offset(load, userptr);
    load_GLX_SGI_swap_control(load, userptr);
    load_GLX_EXT_import_context(load, userptr);
    load_GLX_SGI_video_sync(load, userptr);
    load_GLX_SGI_cushion(load, userptr);
    load_GLX_SGIX_fbconfig(load, userptr);
    load_GLX_NV_copy_buffer(load, userptr);
    load_GLX_ARB_create_context(load, userptr);
    load_GLX_AMD_gpu_association(load, userptr);
    load_GLX_MESA_query_renderer(load, userptr);
    load_GLX_MESA_swap_control(load, userptr);
    load_GLX_SGIX_video_resize(load, userptr);
    load_GLX_NV_video_out(load, userptr);
    load_GLX_MESA_set_3dfx_mode(load, userptr);
    load_GLX_ARB_get_proc_address(load, userptr);
    load_GLX_NV_copy_image(load, userptr);
    load_GLX_NV_present_video(load, userptr);
    load_GLX_SGIX_swap_barrier(load, userptr);
    load_GLX_SGIX_swap_group(load, userptr);

    return version;
}

int gladLoadGLX(Display *display, int screen, GLADloadfunc load) {
    return gladLoadGLXUserPtr(display, screen, glad_glx_get_proc_from_userptr, GLAD_GNUC_EXTENSION (void*) load);
}


#ifdef GLAD_GLX

#ifndef GLAD_LOADER_LIBRARY_C_
#define GLAD_LOADER_LIBRARY_C_

#include <stddef.h>
#include <stdlib.h>

#if GLAD_PLATFORM_WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif


static void* glad_get_dlopen_handle(const char *lib_names[], int length) {
    void *handle = NULL;
    int i;

    for (i = 0; i < length; ++i) {
#if GLAD_PLATFORM_WIN32
  #if GLAD_PLATFORM_UWP
        size_t buffer_size = (strlen(lib_names[i]) + 1) * sizeof(WCHAR);
        LPWSTR buffer = (LPWSTR) malloc(buffer_size);
        if (buffer != NULL) {
            int ret = MultiByteToWideChar(CP_ACP, 0, lib_names[i], -1, buffer, buffer_size);
            if (ret != 0) {
                handle = (void*) LoadPackagedLibrary(buffer, 0);
            }
            free((void*) buffer);
        }
  #else
        handle = (void*) LoadLibraryA(lib_names[i]);
  #endif
#else
        handle = dlopen(lib_names[i], RTLD_LAZY | RTLD_LOCAL);
#endif
        if (handle != NULL) {
            return handle;
        }
    }

    return NULL;
}

static void glad_close_dlopen_handle(void* handle) {
    if (handle != NULL) {
#if GLAD_PLATFORM_WIN32
        FreeLibrary((HMODULE) handle);
#else
        dlclose(handle);
#endif
    }
}

static GLADapiproc glad_dlsym_handle(void* handle, const char *name) {
    if (handle == NULL) {
        return NULL;
    }

#if GLAD_PLATFORM_WIN32
    return (GLADapiproc) GetProcAddress((HMODULE) handle, name);
#else
    return GLAD_GNUC_EXTENSION (GLADapiproc) dlsym(handle, name);
#endif
}

#endif /* GLAD_LOADER_LIBRARY_C_ */

typedef void* (GLAD_API_PTR *GLADglxprocaddrfunc)(const char*);

static GLADapiproc glad_glx_get_proc(const char *name, void *userptr) {
    return GLAD_GNUC_EXTENSION ((GLADapiproc (*)(const char *name)) userptr)(name);
}

static void* _glx_handle;

int gladLoaderLoadGLX(Display *display, int screen) {
    static const char *NAMES[] = {
#if defined __CYGWIN__
        "libGL-1.so",
#endif
        "libGL.so.1",
        "libGL.so"
    };

    int version = 0;
    int did_load = 0;
    GLADglxprocaddrfunc loader;

    if (_glx_handle == NULL) {
        _glx_handle = glad_get_dlopen_handle(NAMES, sizeof(NAMES) / sizeof(NAMES[0]));
        did_load = _glx_handle != NULL;
    }

    if (_glx_handle != NULL) {
        loader = (GLADglxprocaddrfunc) glad_dlsym_handle(_glx_handle, "glXGetProcAddressARB");
        if (loader != NULL) {
            version = gladLoadGLXUserPtr(display, screen, glad_glx_get_proc, GLAD_GNUC_EXTENSION (void*) loader);
        }

        if (!version && did_load) {
            glad_close_dlopen_handle(_glx_handle);
            _glx_handle = NULL;
        }
    }

    return version;
}

void gladLoaderUnloadGLX() {
    if (_glx_handle != NULL) {
        glad_close_dlopen_handle(_glx_handle);
        _glx_handle = NULL;
    }
}

#endif /* GLAD_GLX */
