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



int GLAD_GLX_VERSION_1_0;
int GLAD_GLX_VERSION_1_1;
int GLAD_GLX_VERSION_1_2;
int GLAD_GLX_VERSION_1_3;
int GLAD_GLX_VERSION_1_4;
int GLAD_GLX_3DFX_multisample;
int GLAD_GLX_AMD_gpu_association;
int GLAD_GLX_ARB_context_flush_control;
int GLAD_GLX_ARB_create_context;
int GLAD_GLX_ARB_create_context_no_error;
int GLAD_GLX_ARB_create_context_profile;
int GLAD_GLX_ARB_create_context_robustness;
int GLAD_GLX_ARB_fbconfig_float;
int GLAD_GLX_ARB_framebuffer_sRGB;
int GLAD_GLX_ARB_get_proc_address;
int GLAD_GLX_ARB_multisample;
int GLAD_GLX_ARB_robustness_application_isolation;
int GLAD_GLX_ARB_robustness_share_group_isolation;
int GLAD_GLX_ARB_vertex_buffer_object;
int GLAD_GLX_EXT_buffer_age;
int GLAD_GLX_EXT_create_context_es2_profile;
int GLAD_GLX_EXT_create_context_es_profile;
int GLAD_GLX_EXT_fbconfig_packed_float;
int GLAD_GLX_EXT_framebuffer_sRGB;
int GLAD_GLX_EXT_import_context;
int GLAD_GLX_EXT_libglvnd;
int GLAD_GLX_EXT_no_config_context;
int GLAD_GLX_EXT_stereo_tree;
int GLAD_GLX_EXT_swap_control;
int GLAD_GLX_EXT_swap_control_tear;
int GLAD_GLX_EXT_texture_from_pixmap;
int GLAD_GLX_EXT_visual_info;
int GLAD_GLX_EXT_visual_rating;
int GLAD_GLX_INTEL_swap_event;
int GLAD_GLX_MESA_agp_offset;
int GLAD_GLX_MESA_copy_sub_buffer;
int GLAD_GLX_MESA_pixmap_colormap;
int GLAD_GLX_MESA_query_renderer;
int GLAD_GLX_MESA_release_buffers;
int GLAD_GLX_MESA_set_3dfx_mode;
int GLAD_GLX_MESA_swap_control;
int GLAD_GLX_NV_copy_buffer;
int GLAD_GLX_NV_copy_image;
int GLAD_GLX_NV_delay_before_swap;
int GLAD_GLX_NV_float_buffer;
int GLAD_GLX_NV_multisample_coverage;
int GLAD_GLX_NV_present_video;
int GLAD_GLX_NV_robustness_video_memory_purge;
int GLAD_GLX_NV_swap_group;
int GLAD_GLX_NV_video_capture;
int GLAD_GLX_NV_video_out;
int GLAD_GLX_OML_swap_method;
int GLAD_GLX_OML_sync_control;
int GLAD_GLX_SGIS_blended_overlay;
int GLAD_GLX_SGIS_multisample;
int GLAD_GLX_SGIS_shared_multisample;
int GLAD_GLX_SGIX_fbconfig;
int GLAD_GLX_SGIX_hyperpipe;
int GLAD_GLX_SGIX_pbuffer;
int GLAD_GLX_SGIX_swap_barrier;
int GLAD_GLX_SGIX_swap_group;
int GLAD_GLX_SGIX_video_resize;
int GLAD_GLX_SGIX_visual_select_group;
int GLAD_GLX_SGI_cushion;
int GLAD_GLX_SGI_make_current_read;
int GLAD_GLX_SGI_swap_control;
int GLAD_GLX_SGI_video_sync;
int GLAD_GLX_SUN_get_transparent_index;


PFNGLXCHOOSEFBCONFIGSGIXPROC glad_glXChooseFBConfigSGIX;
PFNGLXCREATECONTEXTWITHCONFIGSGIXPROC glad_glXCreateContextWithConfigSGIX;
PFNGLXSELECTEVENTSGIXPROC glad_glXSelectEventSGIX;
PFNGLXDELAYBEFORESWAPNVPROC glad_glXDelayBeforeSwapNV;
PFNGLXWAITFORMSCOMLPROC glad_glXWaitForMscOML;
PFNGLXQUERYGLXPBUFFERSGIXPROC glad_glXQueryGLXPbufferSGIX;
PFNGLXGETPROCADDRESSPROC glad_glXGetProcAddress;
PFNGLXCHANNELRECTSGIXPROC glad_glXChannelRectSGIX;
PFNGLXCHOOSEVISUALPROC glad_glXChooseVisual;
PFNGLXCREATEPIXMAPPROC glad_glXCreatePixmap;
PFNGLXGETPROCADDRESSARBPROC glad_glXGetProcAddressARB;
PFNGLXQUERYHYPERPIPEATTRIBSGIXPROC glad_glXQueryHyperpipeAttribSGIX;
PFNGLXCREATEGLXPIXMAPMESAPROC glad_glXCreateGLXPixmapMESA;
PFNGLXISDIRECTPROC glad_glXIsDirect;
PFNGLXGETCURRENTREADDRAWABLESGIPROC glad_glXGetCurrentReadDrawableSGI;
PFNGLXGETCURRENTASSOCIATEDCONTEXTAMDPROC glad_glXGetCurrentAssociatedContextAMD;
PFNGLXCOPYIMAGESUBDATANVPROC glad_glXCopyImageSubDataNV;
PFNGLXCREATEGLXPIXMAPWITHCONFIGSGIXPROC glad_glXCreateGLXPixmapWithConfigSGIX;
PFNGLXGETFBCONFIGSPROC glad_glXGetFBConfigs;
PFNGLXHYPERPIPEATTRIBSGIXPROC glad_glXHyperpipeAttribSGIX;
PFNGLXCOPYBUFFERSUBDATANVPROC glad_glXCopyBufferSubDataNV;
PFNGLXQUERYVIDEOCAPTUREDEVICENVPROC glad_glXQueryVideoCaptureDeviceNV;
PFNGLXJOINSWAPGROUPSGIXPROC glad_glXJoinSwapGroupSGIX;
PFNGLXGETCURRENTCONTEXTPROC glad_glXGetCurrentContext;
PFNGLXGETVISUALFROMFBCONFIGPROC glad_glXGetVisualFromFBConfig;
PFNGLXGETCURRENTDISPLAYPROC glad_glXGetCurrentDisplay;
PFNGLXQUERYHYPERPIPECONFIGSGIXPROC glad_glXQueryHyperpipeConfigSGIX;
PFNGLXQUERYHYPERPIPEBESTATTRIBSGIXPROC glad_glXQueryHyperpipeBestAttribSGIX;
PFNGLXGETFBCONFIGFROMVISUALSGIXPROC glad_glXGetFBConfigFromVisualSGIX;
PFNGLXGETCLIENTSTRINGPROC glad_glXGetClientString;
PFNGLXSELECTEVENTPROC glad_glXSelectEvent;
PFNGLXGETVIDEODEVICENVPROC glad_glXGetVideoDeviceNV;
PFNGLXQUERYMAXSWAPBARRIERSSGIXPROC glad_glXQueryMaxSwapBarriersSGIX;
PFNGLXCREATENEWCONTEXTPROC glad_glXCreateNewContext;
PFNGLXQUERYCONTEXTPROC glad_glXQueryContext;
PFNGLXRELEASEVIDEOCAPTUREDEVICENVPROC glad_glXReleaseVideoCaptureDeviceNV;
PFNGLXDESTROYPBUFFERPROC glad_glXDestroyPbuffer;
PFNGLXQUERYCONTEXTINFOEXTPROC glad_glXQueryContextInfoEXT;
PFNGLXBLITCONTEXTFRAMEBUFFERAMDPROC glad_glXBlitContextFramebufferAMD;
PFNGLXBINDCHANNELTOWINDOWSGIXPROC glad_glXBindChannelToWindowSGIX;
PFNGLXSWAPBUFFERSPROC glad_glXSwapBuffers;
PFNGLXQUERYRENDERERSTRINGMESAPROC glad_glXQueryRendererStringMESA;
PFNGLXGETAGPOFFSETMESAPROC glad_glXGetAGPOffsetMESA;
PFNGLXQUERYCURRENTRENDERERSTRINGMESAPROC glad_glXQueryCurrentRendererStringMESA;
PFNGLXDESTROYGLXPBUFFERSGIXPROC glad_glXDestroyGLXPbufferSGIX;
PFNGLXGETSYNCVALUESOMLPROC glad_glXGetSyncValuesOML;
PFNGLXHYPERPIPECONFIGSGIXPROC glad_glXHyperpipeConfigSGIX;
PFNGLXBINDVIDEOIMAGENVPROC glad_glXBindVideoImageNV;
PFNGLXCREATEWINDOWPROC glad_glXCreateWindow;
PFNGLXCREATEGLXPBUFFERSGIXPROC glad_glXCreateGLXPbufferSGIX;
PFNGLXGETCONTEXTIDEXTPROC glad_glXGetContextIDEXT;
PFNGLXGETSWAPINTERVALMESAPROC glad_glXGetSwapIntervalMESA;
PFNGLXMAKECURRENTPROC glad_glXMakeCurrent;
PFNGLXRESETFRAMECOUNTNVPROC glad_glXResetFrameCountNV;
PFNGLXGETCONTEXTGPUIDAMDPROC glad_glXGetContextGPUIDAMD;
PFNGLXQUERYDRAWABLEPROC glad_glXQueryDrawable;
PFNGLXBINDVIDEODEVICENVPROC glad_glXBindVideoDeviceNV;
PFNGLXCREATEGLXPIXMAPPROC glad_glXCreateGLXPixmap;
PFNGLXJOINSWAPGROUPNVPROC glad_glXJoinSwapGroupNV;
PFNGLXGETVIDEOSYNCSGIPROC glad_glXGetVideoSyncSGI;
PFNGLXCUSHIONSGIPROC glad_glXCushionSGI;
PFNGLXRELEASETEXIMAGEEXTPROC glad_glXReleaseTexImageEXT;
PFNGLXQUERYCHANNELDELTASSGIXPROC glad_glXQueryChannelDeltasSGIX;
PFNGLXQUERYFRAMECOUNTNVPROC glad_glXQueryFrameCountNV;
PFNGLXQUERYVERSIONPROC glad_glXQueryVersion;
PFNGLXWAITXPROC glad_glXWaitX;
PFNGLXGETCONFIGPROC glad_glXGetConfig;
PFNGLXSWAPINTERVALSGIPROC glad_glXSwapIntervalSGI;
PFNGLXRELEASEVIDEODEVICENVPROC glad_glXReleaseVideoDeviceNV;
PFNGLXCREATEPBUFFERPROC glad_glXCreatePbuffer;
PFNGLXQUERYSWAPGROUPNVPROC glad_glXQuerySwapGroupNV;
PFNGLXGETCURRENTREADDRAWABLEPROC glad_glXGetCurrentReadDrawable;
PFNGLXMAKECONTEXTCURRENTPROC glad_glXMakeContextCurrent;
PFNGLXBINDTEXIMAGEEXTPROC glad_glXBindTexImageEXT;
PFNGLXGETSELECTEDEVENTSGIXPROC glad_glXGetSelectedEventSGIX;
PFNGLXGETVISUALFROMFBCONFIGSGIXPROC glad_glXGetVisualFromFBConfigSGIX;
PFNGLXMAKEASSOCIATEDCONTEXTCURRENTAMDPROC glad_glXMakeAssociatedContextCurrentAMD;
PFNGLXWAITVIDEOSYNCSGIPROC glad_glXWaitVideoSyncSGI;
PFNGLXGETGPUIDSAMDPROC glad_glXGetGPUIDsAMD;
PFNGLXGETTRANSPARENTINDEXSUNPROC glad_glXGetTransparentIndexSUN;
PFNGLXBINDHYPERPIPESGIXPROC glad_glXBindHyperpipeSGIX;
PFNGLXNAMEDCOPYBUFFERSUBDATANVPROC glad_glXNamedCopyBufferSubDataNV;
PFNGLXUSEXFONTPROC glad_glXUseXFont;
PFNGLXGETVIDEOINFONVPROC glad_glXGetVideoInfoNV;
PFNGLXCHOOSEFBCONFIGPROC glad_glXChooseFBConfig;
PFNGLXQUERYMAXSWAPGROUPSNVPROC glad_glXQueryMaxSwapGroupsNV;
PFNGLXGETMSCRATEOMLPROC glad_glXGetMscRateOML;
PFNGLXLOCKVIDEOCAPTUREDEVICENVPROC glad_glXLockVideoCaptureDeviceNV;
PFNGLXCOPYCONTEXTPROC glad_glXCopyContext;
PFNGLXWAITFORSBCOMLPROC glad_glXWaitForSbcOML;
PFNGLXBINDVIDEOCAPTUREDEVICENVPROC glad_glXBindVideoCaptureDeviceNV;
PFNGLXIMPORTCONTEXTEXTPROC glad_glXImportContextEXT;
PFNGLXSET3DFXMODEMESAPROC glad_glXSet3DfxModeMESA;
PFNGLXDESTROYPIXMAPPROC glad_glXDestroyPixmap;
PFNGLXQUERYEXTENSIONPROC glad_glXQueryExtension;
PFNGLXQUERYHYPERPIPENETWORKSGIXPROC glad_glXQueryHyperpipeNetworkSGIX;
PFNGLXDESTROYGLXPIXMAPPROC glad_glXDestroyGLXPixmap;
PFNGLXENUMERATEVIDEOCAPTUREDEVICESNVPROC glad_glXEnumerateVideoCaptureDevicesNV;
PFNGLXDESTROYHYPERPIPECONFIGSGIXPROC glad_glXDestroyHyperpipeConfigSGIX;
PFNGLXDESTROYWINDOWPROC glad_glXDestroyWindow;
PFNGLXRELEASEVIDEOIMAGENVPROC glad_glXReleaseVideoImageNV;
PFNGLXQUERYCHANNELRECTSGIXPROC glad_glXQueryChannelRectSGIX;
PFNGLXCREATEASSOCIATEDCONTEXTAMDPROC glad_glXCreateAssociatedContextAMD;
PFNGLXENUMERATEVIDEODEVICESNVPROC glad_glXEnumerateVideoDevicesNV;
PFNGLXGETCURRENTDRAWABLEPROC glad_glXGetCurrentDrawable;
PFNGLXGETSELECTEDEVENTPROC glad_glXGetSelectedEvent;
PFNGLXSWAPINTERVALMESAPROC glad_glXSwapIntervalMESA;
PFNGLXQUERYCURRENTRENDERERINTEGERMESAPROC glad_glXQueryCurrentRendererIntegerMESA;
PFNGLXCHANNELRECTSYNCSGIXPROC glad_glXChannelRectSyncSGIX;
PFNGLXSWAPBUFFERSMSCOMLPROC glad_glXSwapBuffersMscOML;
PFNGLXQUERYEXTENSIONSSTRINGPROC glad_glXQueryExtensionsString;
PFNGLXCREATEASSOCIATEDCONTEXTATTRIBSAMDPROC glad_glXCreateAssociatedContextAttribsAMD;
PFNGLXSENDPBUFFERTOVIDEONVPROC glad_glXSendPbufferToVideoNV;
PFNGLXCREATECONTEXTPROC glad_glXCreateContext;
PFNGLXGETCURRENTDISPLAYEXTPROC glad_glXGetCurrentDisplayEXT;
PFNGLXDELETEASSOCIATEDCONTEXTAMDPROC glad_glXDeleteAssociatedContextAMD;
PFNGLXDESTROYCONTEXTPROC glad_glXDestroyContext;
PFNGLXQUERYSERVERSTRINGPROC glad_glXQueryServerString;
PFNGLXWAITGLPROC glad_glXWaitGL;
PFNGLXCREATECONTEXTATTRIBSARBPROC glad_glXCreateContextAttribsARB;
PFNGLXBINDSWAPBARRIERSGIXPROC glad_glXBindSwapBarrierSGIX;
PFNGLXFREECONTEXTEXTPROC glad_glXFreeContextEXT;
PFNGLXGETFBCONFIGATTRIBPROC glad_glXGetFBConfigAttrib;
PFNGLXCOPYSUBBUFFERMESAPROC glad_glXCopySubBufferMESA;
PFNGLXGETGPUINFOAMDPROC glad_glXGetGPUInfoAMD;
PFNGLXRELEASEBUFFERSMESAPROC glad_glXReleaseBuffersMESA;
PFNGLXSWAPINTERVALEXTPROC glad_glXSwapIntervalEXT;
PFNGLXMAKECURRENTREADSGIPROC glad_glXMakeCurrentReadSGI;
PFNGLXQUERYRENDERERINTEGERMESAPROC glad_glXQueryRendererIntegerMESA;
PFNGLXBINDSWAPBARRIERNVPROC glad_glXBindSwapBarrierNV;
PFNGLXGETFBCONFIGATTRIBSGIXPROC glad_glXGetFBConfigAttribSGIX;

static void load_GLX_VERSION_1_0(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_VERSION_1_0) return;
    glad_glXQueryVersion = (PFNGLXQUERYVERSIONPROC)load("glXQueryVersion", userptr);
    glad_glXIsDirect = (PFNGLXISDIRECTPROC)load("glXIsDirect", userptr);
    glad_glXChooseVisual = (PFNGLXCHOOSEVISUALPROC)load("glXChooseVisual", userptr);
    glad_glXGetConfig = (PFNGLXGETCONFIGPROC)load("glXGetConfig", userptr);
    glad_glXUseXFont = (PFNGLXUSEXFONTPROC)load("glXUseXFont", userptr);
    glad_glXCreateGLXPixmap = (PFNGLXCREATEGLXPIXMAPPROC)load("glXCreateGLXPixmap", userptr);
    glad_glXCopyContext = (PFNGLXCOPYCONTEXTPROC)load("glXCopyContext", userptr);
    glad_glXWaitGL = (PFNGLXWAITGLPROC)load("glXWaitGL", userptr);
    glad_glXQueryExtension = (PFNGLXQUERYEXTENSIONPROC)load("glXQueryExtension", userptr);
    glad_glXWaitX = (PFNGLXWAITXPROC)load("glXWaitX", userptr);
    glad_glXGetCurrentContext = (PFNGLXGETCURRENTCONTEXTPROC)load("glXGetCurrentContext", userptr);
    glad_glXMakeCurrent = (PFNGLXMAKECURRENTPROC)load("glXMakeCurrent", userptr);
    glad_glXSwapBuffers = (PFNGLXSWAPBUFFERSPROC)load("glXSwapBuffers", userptr);
    glad_glXDestroyContext = (PFNGLXDESTROYCONTEXTPROC)load("glXDestroyContext", userptr);
    glad_glXGetCurrentDrawable = (PFNGLXGETCURRENTDRAWABLEPROC)load("glXGetCurrentDrawable", userptr);
    glad_glXCreateContext = (PFNGLXCREATECONTEXTPROC)load("glXCreateContext", userptr);
    glad_glXDestroyGLXPixmap = (PFNGLXDESTROYGLXPIXMAPPROC)load("glXDestroyGLXPixmap", userptr);
}
static void load_GLX_VERSION_1_1(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_VERSION_1_1) return;
    glad_glXQueryServerString = (PFNGLXQUERYSERVERSTRINGPROC)load("glXQueryServerString", userptr);
    glad_glXGetClientString = (PFNGLXGETCLIENTSTRINGPROC)load("glXGetClientString", userptr);
    glad_glXQueryExtensionsString = (PFNGLXQUERYEXTENSIONSSTRINGPROC)load("glXQueryExtensionsString", userptr);
}
static void load_GLX_VERSION_1_2(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_VERSION_1_2) return;
    glad_glXGetCurrentDisplay = (PFNGLXGETCURRENTDISPLAYPROC)load("glXGetCurrentDisplay", userptr);
}
static void load_GLX_VERSION_1_3(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_VERSION_1_3) return;
    glad_glXChooseFBConfig = (PFNGLXCHOOSEFBCONFIGPROC)load("glXChooseFBConfig", userptr);
    glad_glXSelectEvent = (PFNGLXSELECTEVENTPROC)load("glXSelectEvent", userptr);
    glad_glXCreatePixmap = (PFNGLXCREATEPIXMAPPROC)load("glXCreatePixmap", userptr);
    glad_glXCreateNewContext = (PFNGLXCREATENEWCONTEXTPROC)load("glXCreateNewContext", userptr);
    glad_glXQueryContext = (PFNGLXQUERYCONTEXTPROC)load("glXQueryContext", userptr);
    glad_glXDestroyPixmap = (PFNGLXDESTROYPIXMAPPROC)load("glXDestroyPixmap", userptr);
    glad_glXDestroyWindow = (PFNGLXDESTROYWINDOWPROC)load("glXDestroyWindow", userptr);
    glad_glXCreatePbuffer = (PFNGLXCREATEPBUFFERPROC)load("glXCreatePbuffer", userptr);
    glad_glXDestroyPbuffer = (PFNGLXDESTROYPBUFFERPROC)load("glXDestroyPbuffer", userptr);
    glad_glXCreateWindow = (PFNGLXCREATEWINDOWPROC)load("glXCreateWindow", userptr);
    glad_glXGetVisualFromFBConfig = (PFNGLXGETVISUALFROMFBCONFIGPROC)load("glXGetVisualFromFBConfig", userptr);
    glad_glXGetCurrentReadDrawable = (PFNGLXGETCURRENTREADDRAWABLEPROC)load("glXGetCurrentReadDrawable", userptr);
    glad_glXMakeContextCurrent = (PFNGLXMAKECONTEXTCURRENTPROC)load("glXMakeContextCurrent", userptr);
    glad_glXGetFBConfigAttrib = (PFNGLXGETFBCONFIGATTRIBPROC)load("glXGetFBConfigAttrib", userptr);
    glad_glXQueryDrawable = (PFNGLXQUERYDRAWABLEPROC)load("glXQueryDrawable", userptr);
    glad_glXGetFBConfigs = (PFNGLXGETFBCONFIGSPROC)load("glXGetFBConfigs", userptr);
    glad_glXGetSelectedEvent = (PFNGLXGETSELECTEDEVENTPROC)load("glXGetSelectedEvent", userptr);
}
static void load_GLX_VERSION_1_4(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_VERSION_1_4) return;
    glad_glXGetProcAddress = (PFNGLXGETPROCADDRESSPROC)load("glXGetProcAddress", userptr);
}
static void load_GLX_3DFX_multisample(GLADloadproc load, void* userptr) {
}
static void load_GLX_AMD_gpu_association(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_AMD_gpu_association) return;
    glad_glXMakeAssociatedContextCurrentAMD = (PFNGLXMAKEASSOCIATEDCONTEXTCURRENTAMDPROC)load("glXMakeAssociatedContextCurrentAMD", userptr);
    glad_glXGetGPUInfoAMD = (PFNGLXGETGPUINFOAMDPROC)load("glXGetGPUInfoAMD", userptr);
    glad_glXDeleteAssociatedContextAMD = (PFNGLXDELETEASSOCIATEDCONTEXTAMDPROC)load("glXDeleteAssociatedContextAMD", userptr);
    glad_glXGetGPUIDsAMD = (PFNGLXGETGPUIDSAMDPROC)load("glXGetGPUIDsAMD", userptr);
    glad_glXGetCurrentAssociatedContextAMD = (PFNGLXGETCURRENTASSOCIATEDCONTEXTAMDPROC)load("glXGetCurrentAssociatedContextAMD", userptr);
    glad_glXCreateAssociatedContextAMD = (PFNGLXCREATEASSOCIATEDCONTEXTAMDPROC)load("glXCreateAssociatedContextAMD", userptr);
    glad_glXBlitContextFramebufferAMD = (PFNGLXBLITCONTEXTFRAMEBUFFERAMDPROC)load("glXBlitContextFramebufferAMD", userptr);
    glad_glXCreateAssociatedContextAttribsAMD = (PFNGLXCREATEASSOCIATEDCONTEXTATTRIBSAMDPROC)load("glXCreateAssociatedContextAttribsAMD", userptr);
    glad_glXGetContextGPUIDAMD = (PFNGLXGETCONTEXTGPUIDAMDPROC)load("glXGetContextGPUIDAMD", userptr);
}
static void load_GLX_ARB_context_flush_control(GLADloadproc load, void* userptr) {
}
static void load_GLX_ARB_create_context(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_ARB_create_context) return;
    glad_glXCreateContextAttribsARB = (PFNGLXCREATECONTEXTATTRIBSARBPROC)load("glXCreateContextAttribsARB", userptr);
}
static void load_GLX_ARB_create_context_no_error(GLADloadproc load, void* userptr) {
}
static void load_GLX_ARB_create_context_profile(GLADloadproc load, void* userptr) {
}
static void load_GLX_ARB_create_context_robustness(GLADloadproc load, void* userptr) {
}
static void load_GLX_ARB_fbconfig_float(GLADloadproc load, void* userptr) {
}
static void load_GLX_ARB_framebuffer_sRGB(GLADloadproc load, void* userptr) {
}
static void load_GLX_ARB_get_proc_address(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_ARB_get_proc_address) return;
    glad_glXGetProcAddressARB = (PFNGLXGETPROCADDRESSARBPROC)load("glXGetProcAddressARB", userptr);
}
static void load_GLX_ARB_multisample(GLADloadproc load, void* userptr) {
}
static void load_GLX_ARB_robustness_application_isolation(GLADloadproc load, void* userptr) {
}
static void load_GLX_ARB_robustness_share_group_isolation(GLADloadproc load, void* userptr) {
}
static void load_GLX_ARB_vertex_buffer_object(GLADloadproc load, void* userptr) {
}
static void load_GLX_EXT_buffer_age(GLADloadproc load, void* userptr) {
}
static void load_GLX_EXT_create_context_es2_profile(GLADloadproc load, void* userptr) {
}
static void load_GLX_EXT_create_context_es_profile(GLADloadproc load, void* userptr) {
}
static void load_GLX_EXT_fbconfig_packed_float(GLADloadproc load, void* userptr) {
}
static void load_GLX_EXT_framebuffer_sRGB(GLADloadproc load, void* userptr) {
}
static void load_GLX_EXT_import_context(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_EXT_import_context) return;
    glad_glXQueryContextInfoEXT = (PFNGLXQUERYCONTEXTINFOEXTPROC)load("glXQueryContextInfoEXT", userptr);
    glad_glXGetCurrentDisplayEXT = (PFNGLXGETCURRENTDISPLAYEXTPROC)load("glXGetCurrentDisplayEXT", userptr);
    glad_glXGetContextIDEXT = (PFNGLXGETCONTEXTIDEXTPROC)load("glXGetContextIDEXT", userptr);
    glad_glXFreeContextEXT = (PFNGLXFREECONTEXTEXTPROC)load("glXFreeContextEXT", userptr);
    glad_glXImportContextEXT = (PFNGLXIMPORTCONTEXTEXTPROC)load("glXImportContextEXT", userptr);
}
static void load_GLX_EXT_libglvnd(GLADloadproc load, void* userptr) {
}
static void load_GLX_EXT_no_config_context(GLADloadproc load, void* userptr) {
}
static void load_GLX_EXT_stereo_tree(GLADloadproc load, void* userptr) {
}
static void load_GLX_EXT_swap_control(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_EXT_swap_control) return;
    glad_glXSwapIntervalEXT = (PFNGLXSWAPINTERVALEXTPROC)load("glXSwapIntervalEXT", userptr);
}
static void load_GLX_EXT_swap_control_tear(GLADloadproc load, void* userptr) {
}
static void load_GLX_EXT_texture_from_pixmap(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_EXT_texture_from_pixmap) return;
    glad_glXReleaseTexImageEXT = (PFNGLXRELEASETEXIMAGEEXTPROC)load("glXReleaseTexImageEXT", userptr);
    glad_glXBindTexImageEXT = (PFNGLXBINDTEXIMAGEEXTPROC)load("glXBindTexImageEXT", userptr);
}
static void load_GLX_EXT_visual_info(GLADloadproc load, void* userptr) {
}
static void load_GLX_EXT_visual_rating(GLADloadproc load, void* userptr) {
}
static void load_GLX_INTEL_swap_event(GLADloadproc load, void* userptr) {
}
static void load_GLX_MESA_agp_offset(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_MESA_agp_offset) return;
    glad_glXGetAGPOffsetMESA = (PFNGLXGETAGPOFFSETMESAPROC)load("glXGetAGPOffsetMESA", userptr);
}
static void load_GLX_MESA_copy_sub_buffer(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_MESA_copy_sub_buffer) return;
    glad_glXCopySubBufferMESA = (PFNGLXCOPYSUBBUFFERMESAPROC)load("glXCopySubBufferMESA", userptr);
}
static void load_GLX_MESA_pixmap_colormap(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_MESA_pixmap_colormap) return;
    glad_glXCreateGLXPixmapMESA = (PFNGLXCREATEGLXPIXMAPMESAPROC)load("glXCreateGLXPixmapMESA", userptr);
}
static void load_GLX_MESA_query_renderer(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_MESA_query_renderer) return;
    glad_glXQueryCurrentRendererStringMESA = (PFNGLXQUERYCURRENTRENDERERSTRINGMESAPROC)load("glXQueryCurrentRendererStringMESA", userptr);
    glad_glXQueryRendererIntegerMESA = (PFNGLXQUERYRENDERERINTEGERMESAPROC)load("glXQueryRendererIntegerMESA", userptr);
    glad_glXQueryRendererStringMESA = (PFNGLXQUERYRENDERERSTRINGMESAPROC)load("glXQueryRendererStringMESA", userptr);
    glad_glXQueryCurrentRendererIntegerMESA = (PFNGLXQUERYCURRENTRENDERERINTEGERMESAPROC)load("glXQueryCurrentRendererIntegerMESA", userptr);
}
static void load_GLX_MESA_release_buffers(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_MESA_release_buffers) return;
    glad_glXReleaseBuffersMESA = (PFNGLXRELEASEBUFFERSMESAPROC)load("glXReleaseBuffersMESA", userptr);
}
static void load_GLX_MESA_set_3dfx_mode(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_MESA_set_3dfx_mode) return;
    glad_glXSet3DfxModeMESA = (PFNGLXSET3DFXMODEMESAPROC)load("glXSet3DfxModeMESA", userptr);
}
static void load_GLX_MESA_swap_control(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_MESA_swap_control) return;
    glad_glXGetSwapIntervalMESA = (PFNGLXGETSWAPINTERVALMESAPROC)load("glXGetSwapIntervalMESA", userptr);
    glad_glXSwapIntervalMESA = (PFNGLXSWAPINTERVALMESAPROC)load("glXSwapIntervalMESA", userptr);
}
static void load_GLX_NV_copy_buffer(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_NV_copy_buffer) return;
    glad_glXNamedCopyBufferSubDataNV = (PFNGLXNAMEDCOPYBUFFERSUBDATANVPROC)load("glXNamedCopyBufferSubDataNV", userptr);
    glad_glXCopyBufferSubDataNV = (PFNGLXCOPYBUFFERSUBDATANVPROC)load("glXCopyBufferSubDataNV", userptr);
}
static void load_GLX_NV_copy_image(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_NV_copy_image) return;
    glad_glXCopyImageSubDataNV = (PFNGLXCOPYIMAGESUBDATANVPROC)load("glXCopyImageSubDataNV", userptr);
}
static void load_GLX_NV_delay_before_swap(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_NV_delay_before_swap) return;
    glad_glXDelayBeforeSwapNV = (PFNGLXDELAYBEFORESWAPNVPROC)load("glXDelayBeforeSwapNV", userptr);
}
static void load_GLX_NV_float_buffer(GLADloadproc load, void* userptr) {
}
static void load_GLX_NV_multisample_coverage(GLADloadproc load, void* userptr) {
}
static void load_GLX_NV_present_video(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_NV_present_video) return;
    glad_glXEnumerateVideoDevicesNV = (PFNGLXENUMERATEVIDEODEVICESNVPROC)load("glXEnumerateVideoDevicesNV", userptr);
    glad_glXBindVideoDeviceNV = (PFNGLXBINDVIDEODEVICENVPROC)load("glXBindVideoDeviceNV", userptr);
}
static void load_GLX_NV_robustness_video_memory_purge(GLADloadproc load, void* userptr) {
}
static void load_GLX_NV_swap_group(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_NV_swap_group) return;
    glad_glXQueryMaxSwapGroupsNV = (PFNGLXQUERYMAXSWAPGROUPSNVPROC)load("glXQueryMaxSwapGroupsNV", userptr);
    glad_glXQuerySwapGroupNV = (PFNGLXQUERYSWAPGROUPNVPROC)load("glXQuerySwapGroupNV", userptr);
    glad_glXJoinSwapGroupNV = (PFNGLXJOINSWAPGROUPNVPROC)load("glXJoinSwapGroupNV", userptr);
    glad_glXResetFrameCountNV = (PFNGLXRESETFRAMECOUNTNVPROC)load("glXResetFrameCountNV", userptr);
    glad_glXBindSwapBarrierNV = (PFNGLXBINDSWAPBARRIERNVPROC)load("glXBindSwapBarrierNV", userptr);
    glad_glXQueryFrameCountNV = (PFNGLXQUERYFRAMECOUNTNVPROC)load("glXQueryFrameCountNV", userptr);
}
static void load_GLX_NV_video_capture(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_NV_video_capture) return;
    glad_glXLockVideoCaptureDeviceNV = (PFNGLXLOCKVIDEOCAPTUREDEVICENVPROC)load("glXLockVideoCaptureDeviceNV", userptr);
    glad_glXBindVideoCaptureDeviceNV = (PFNGLXBINDVIDEOCAPTUREDEVICENVPROC)load("glXBindVideoCaptureDeviceNV", userptr);
    glad_glXQueryVideoCaptureDeviceNV = (PFNGLXQUERYVIDEOCAPTUREDEVICENVPROC)load("glXQueryVideoCaptureDeviceNV", userptr);
    glad_glXReleaseVideoCaptureDeviceNV = (PFNGLXRELEASEVIDEOCAPTUREDEVICENVPROC)load("glXReleaseVideoCaptureDeviceNV", userptr);
    glad_glXEnumerateVideoCaptureDevicesNV = (PFNGLXENUMERATEVIDEOCAPTUREDEVICESNVPROC)load("glXEnumerateVideoCaptureDevicesNV", userptr);
}
static void load_GLX_NV_video_out(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_NV_video_out) return;
    glad_glXGetVideoDeviceNV = (PFNGLXGETVIDEODEVICENVPROC)load("glXGetVideoDeviceNV", userptr);
    glad_glXReleaseVideoImageNV = (PFNGLXRELEASEVIDEOIMAGENVPROC)load("glXReleaseVideoImageNV", userptr);
    glad_glXReleaseVideoDeviceNV = (PFNGLXRELEASEVIDEODEVICENVPROC)load("glXReleaseVideoDeviceNV", userptr);
    glad_glXBindVideoImageNV = (PFNGLXBINDVIDEOIMAGENVPROC)load("glXBindVideoImageNV", userptr);
    glad_glXSendPbufferToVideoNV = (PFNGLXSENDPBUFFERTOVIDEONVPROC)load("glXSendPbufferToVideoNV", userptr);
    glad_glXGetVideoInfoNV = (PFNGLXGETVIDEOINFONVPROC)load("glXGetVideoInfoNV", userptr);
}
static void load_GLX_OML_swap_method(GLADloadproc load, void* userptr) {
}
static void load_GLX_OML_sync_control(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_OML_sync_control) return;
    glad_glXWaitForMscOML = (PFNGLXWAITFORMSCOMLPROC)load("glXWaitForMscOML", userptr);
    glad_glXSwapBuffersMscOML = (PFNGLXSWAPBUFFERSMSCOMLPROC)load("glXSwapBuffersMscOML", userptr);
    glad_glXGetMscRateOML = (PFNGLXGETMSCRATEOMLPROC)load("glXGetMscRateOML", userptr);
    glad_glXWaitForSbcOML = (PFNGLXWAITFORSBCOMLPROC)load("glXWaitForSbcOML", userptr);
    glad_glXGetSyncValuesOML = (PFNGLXGETSYNCVALUESOMLPROC)load("glXGetSyncValuesOML", userptr);
}
static void load_GLX_SGIS_blended_overlay(GLADloadproc load, void* userptr) {
}
static void load_GLX_SGIS_multisample(GLADloadproc load, void* userptr) {
}
static void load_GLX_SGIS_shared_multisample(GLADloadproc load, void* userptr) {
}
static void load_GLX_SGIX_fbconfig(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_SGIX_fbconfig) return;
    glad_glXChooseFBConfigSGIX = (PFNGLXCHOOSEFBCONFIGSGIXPROC)load("glXChooseFBConfigSGIX", userptr);
    glad_glXGetVisualFromFBConfigSGIX = (PFNGLXGETVISUALFROMFBCONFIGSGIXPROC)load("glXGetVisualFromFBConfigSGIX", userptr);
    glad_glXCreateContextWithConfigSGIX = (PFNGLXCREATECONTEXTWITHCONFIGSGIXPROC)load("glXCreateContextWithConfigSGIX", userptr);
    glad_glXCreateGLXPixmapWithConfigSGIX = (PFNGLXCREATEGLXPIXMAPWITHCONFIGSGIXPROC)load("glXCreateGLXPixmapWithConfigSGIX", userptr);
    glad_glXGetFBConfigFromVisualSGIX = (PFNGLXGETFBCONFIGFROMVISUALSGIXPROC)load("glXGetFBConfigFromVisualSGIX", userptr);
    glad_glXGetFBConfigAttribSGIX = (PFNGLXGETFBCONFIGATTRIBSGIXPROC)load("glXGetFBConfigAttribSGIX", userptr);
}
static void load_GLX_SGIX_hyperpipe(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_SGIX_hyperpipe) return;
    glad_glXHyperpipeAttribSGIX = (PFNGLXHYPERPIPEATTRIBSGIXPROC)load("glXHyperpipeAttribSGIX", userptr);
    glad_glXDestroyHyperpipeConfigSGIX = (PFNGLXDESTROYHYPERPIPECONFIGSGIXPROC)load("glXDestroyHyperpipeConfigSGIX", userptr);
    glad_glXHyperpipeConfigSGIX = (PFNGLXHYPERPIPECONFIGSGIXPROC)load("glXHyperpipeConfigSGIX", userptr);
    glad_glXQueryHyperpipeAttribSGIX = (PFNGLXQUERYHYPERPIPEATTRIBSGIXPROC)load("glXQueryHyperpipeAttribSGIX", userptr);
    glad_glXBindHyperpipeSGIX = (PFNGLXBINDHYPERPIPESGIXPROC)load("glXBindHyperpipeSGIX", userptr);
    glad_glXQueryHyperpipeNetworkSGIX = (PFNGLXQUERYHYPERPIPENETWORKSGIXPROC)load("glXQueryHyperpipeNetworkSGIX", userptr);
    glad_glXQueryHyperpipeConfigSGIX = (PFNGLXQUERYHYPERPIPECONFIGSGIXPROC)load("glXQueryHyperpipeConfigSGIX", userptr);
    glad_glXQueryHyperpipeBestAttribSGIX = (PFNGLXQUERYHYPERPIPEBESTATTRIBSGIXPROC)load("glXQueryHyperpipeBestAttribSGIX", userptr);
}
static void load_GLX_SGIX_pbuffer(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_SGIX_pbuffer) return;
    glad_glXGetSelectedEventSGIX = (PFNGLXGETSELECTEDEVENTSGIXPROC)load("glXGetSelectedEventSGIX", userptr);
    glad_glXDestroyGLXPbufferSGIX = (PFNGLXDESTROYGLXPBUFFERSGIXPROC)load("glXDestroyGLXPbufferSGIX", userptr);
    glad_glXCreateGLXPbufferSGIX = (PFNGLXCREATEGLXPBUFFERSGIXPROC)load("glXCreateGLXPbufferSGIX", userptr);
    glad_glXQueryGLXPbufferSGIX = (PFNGLXQUERYGLXPBUFFERSGIXPROC)load("glXQueryGLXPbufferSGIX", userptr);
    glad_glXSelectEventSGIX = (PFNGLXSELECTEVENTSGIXPROC)load("glXSelectEventSGIX", userptr);
}
static void load_GLX_SGIX_swap_barrier(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_SGIX_swap_barrier) return;
    glad_glXQueryMaxSwapBarriersSGIX = (PFNGLXQUERYMAXSWAPBARRIERSSGIXPROC)load("glXQueryMaxSwapBarriersSGIX", userptr);
    glad_glXBindSwapBarrierSGIX = (PFNGLXBINDSWAPBARRIERSGIXPROC)load("glXBindSwapBarrierSGIX", userptr);
}
static void load_GLX_SGIX_swap_group(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_SGIX_swap_group) return;
    glad_glXJoinSwapGroupSGIX = (PFNGLXJOINSWAPGROUPSGIXPROC)load("glXJoinSwapGroupSGIX", userptr);
}
static void load_GLX_SGIX_video_resize(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_SGIX_video_resize) return;
    glad_glXChannelRectSGIX = (PFNGLXCHANNELRECTSGIXPROC)load("glXChannelRectSGIX", userptr);
    glad_glXQueryChannelRectSGIX = (PFNGLXQUERYCHANNELRECTSGIXPROC)load("glXQueryChannelRectSGIX", userptr);
    glad_glXChannelRectSyncSGIX = (PFNGLXCHANNELRECTSYNCSGIXPROC)load("glXChannelRectSyncSGIX", userptr);
    glad_glXBindChannelToWindowSGIX = (PFNGLXBINDCHANNELTOWINDOWSGIXPROC)load("glXBindChannelToWindowSGIX", userptr);
    glad_glXQueryChannelDeltasSGIX = (PFNGLXQUERYCHANNELDELTASSGIXPROC)load("glXQueryChannelDeltasSGIX", userptr);
}
static void load_GLX_SGIX_visual_select_group(GLADloadproc load, void* userptr) {
}
static void load_GLX_SGI_cushion(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_SGI_cushion) return;
    glad_glXCushionSGI = (PFNGLXCUSHIONSGIPROC)load("glXCushionSGI", userptr);
}
static void load_GLX_SGI_make_current_read(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_SGI_make_current_read) return;
    glad_glXMakeCurrentReadSGI = (PFNGLXMAKECURRENTREADSGIPROC)load("glXMakeCurrentReadSGI", userptr);
    glad_glXGetCurrentReadDrawableSGI = (PFNGLXGETCURRENTREADDRAWABLESGIPROC)load("glXGetCurrentReadDrawableSGI", userptr);
}
static void load_GLX_SGI_swap_control(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_SGI_swap_control) return;
    glad_glXSwapIntervalSGI = (PFNGLXSWAPINTERVALSGIPROC)load("glXSwapIntervalSGI", userptr);
}
static void load_GLX_SGI_video_sync(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_SGI_video_sync) return;
    glad_glXGetVideoSyncSGI = (PFNGLXGETVIDEOSYNCSGIPROC)load("glXGetVideoSyncSGI", userptr);
    glad_glXWaitVideoSyncSGI = (PFNGLXWAITVIDEOSYNCSGIPROC)load("glXWaitVideoSyncSGI", userptr);
}
static void load_GLX_SUN_get_transparent_index(GLADloadproc load, void* userptr) {
    if(!GLAD_GLX_SUN_get_transparent_index) return;
    glad_glXGetTransparentIndexSUN = (PFNGLXGETTRANSPARENTINDEXSUNPROC)load("glXGetTransparentIndexSUN", userptr);
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

static int find_extensionsGLX(Display *display, int screen) {
    GLAD_GLX_3DFX_multisample = has_ext(display, screen, "GLX_3DFX_multisample");
    GLAD_GLX_AMD_gpu_association = has_ext(display, screen, "GLX_AMD_gpu_association");
    GLAD_GLX_ARB_context_flush_control = has_ext(display, screen, "GLX_ARB_context_flush_control");
    GLAD_GLX_ARB_create_context = has_ext(display, screen, "GLX_ARB_create_context");
    GLAD_GLX_ARB_create_context_no_error = has_ext(display, screen, "GLX_ARB_create_context_no_error");
    GLAD_GLX_ARB_create_context_profile = has_ext(display, screen, "GLX_ARB_create_context_profile");
    GLAD_GLX_ARB_create_context_robustness = has_ext(display, screen, "GLX_ARB_create_context_robustness");
    GLAD_GLX_ARB_fbconfig_float = has_ext(display, screen, "GLX_ARB_fbconfig_float");
    GLAD_GLX_ARB_framebuffer_sRGB = has_ext(display, screen, "GLX_ARB_framebuffer_sRGB");
    GLAD_GLX_ARB_get_proc_address = has_ext(display, screen, "GLX_ARB_get_proc_address");
    GLAD_GLX_ARB_multisample = has_ext(display, screen, "GLX_ARB_multisample");
    GLAD_GLX_ARB_robustness_application_isolation = has_ext(display, screen, "GLX_ARB_robustness_application_isolation");
    GLAD_GLX_ARB_robustness_share_group_isolation = has_ext(display, screen, "GLX_ARB_robustness_share_group_isolation");
    GLAD_GLX_ARB_vertex_buffer_object = has_ext(display, screen, "GLX_ARB_vertex_buffer_object");
    GLAD_GLX_EXT_buffer_age = has_ext(display, screen, "GLX_EXT_buffer_age");
    GLAD_GLX_EXT_create_context_es2_profile = has_ext(display, screen, "GLX_EXT_create_context_es2_profile");
    GLAD_GLX_EXT_create_context_es_profile = has_ext(display, screen, "GLX_EXT_create_context_es_profile");
    GLAD_GLX_EXT_fbconfig_packed_float = has_ext(display, screen, "GLX_EXT_fbconfig_packed_float");
    GLAD_GLX_EXT_framebuffer_sRGB = has_ext(display, screen, "GLX_EXT_framebuffer_sRGB");
    GLAD_GLX_EXT_import_context = has_ext(display, screen, "GLX_EXT_import_context");
    GLAD_GLX_EXT_libglvnd = has_ext(display, screen, "GLX_EXT_libglvnd");
    GLAD_GLX_EXT_no_config_context = has_ext(display, screen, "GLX_EXT_no_config_context");
    GLAD_GLX_EXT_stereo_tree = has_ext(display, screen, "GLX_EXT_stereo_tree");
    GLAD_GLX_EXT_swap_control = has_ext(display, screen, "GLX_EXT_swap_control");
    GLAD_GLX_EXT_swap_control_tear = has_ext(display, screen, "GLX_EXT_swap_control_tear");
    GLAD_GLX_EXT_texture_from_pixmap = has_ext(display, screen, "GLX_EXT_texture_from_pixmap");
    GLAD_GLX_EXT_visual_info = has_ext(display, screen, "GLX_EXT_visual_info");
    GLAD_GLX_EXT_visual_rating = has_ext(display, screen, "GLX_EXT_visual_rating");
    GLAD_GLX_INTEL_swap_event = has_ext(display, screen, "GLX_INTEL_swap_event");
    GLAD_GLX_MESA_agp_offset = has_ext(display, screen, "GLX_MESA_agp_offset");
    GLAD_GLX_MESA_copy_sub_buffer = has_ext(display, screen, "GLX_MESA_copy_sub_buffer");
    GLAD_GLX_MESA_pixmap_colormap = has_ext(display, screen, "GLX_MESA_pixmap_colormap");
    GLAD_GLX_MESA_query_renderer = has_ext(display, screen, "GLX_MESA_query_renderer");
    GLAD_GLX_MESA_release_buffers = has_ext(display, screen, "GLX_MESA_release_buffers");
    GLAD_GLX_MESA_set_3dfx_mode = has_ext(display, screen, "GLX_MESA_set_3dfx_mode");
    GLAD_GLX_MESA_swap_control = has_ext(display, screen, "GLX_MESA_swap_control");
    GLAD_GLX_NV_copy_buffer = has_ext(display, screen, "GLX_NV_copy_buffer");
    GLAD_GLX_NV_copy_image = has_ext(display, screen, "GLX_NV_copy_image");
    GLAD_GLX_NV_delay_before_swap = has_ext(display, screen, "GLX_NV_delay_before_swap");
    GLAD_GLX_NV_float_buffer = has_ext(display, screen, "GLX_NV_float_buffer");
    GLAD_GLX_NV_multisample_coverage = has_ext(display, screen, "GLX_NV_multisample_coverage");
    GLAD_GLX_NV_present_video = has_ext(display, screen, "GLX_NV_present_video");
    GLAD_GLX_NV_robustness_video_memory_purge = has_ext(display, screen, "GLX_NV_robustness_video_memory_purge");
    GLAD_GLX_NV_swap_group = has_ext(display, screen, "GLX_NV_swap_group");
    GLAD_GLX_NV_video_capture = has_ext(display, screen, "GLX_NV_video_capture");
    GLAD_GLX_NV_video_out = has_ext(display, screen, "GLX_NV_video_out");
    GLAD_GLX_OML_swap_method = has_ext(display, screen, "GLX_OML_swap_method");
    GLAD_GLX_OML_sync_control = has_ext(display, screen, "GLX_OML_sync_control");
    GLAD_GLX_SGIS_blended_overlay = has_ext(display, screen, "GLX_SGIS_blended_overlay");
    GLAD_GLX_SGIS_multisample = has_ext(display, screen, "GLX_SGIS_multisample");
    GLAD_GLX_SGIS_shared_multisample = has_ext(display, screen, "GLX_SGIS_shared_multisample");
    GLAD_GLX_SGIX_fbconfig = has_ext(display, screen, "GLX_SGIX_fbconfig");
    GLAD_GLX_SGIX_hyperpipe = has_ext(display, screen, "GLX_SGIX_hyperpipe");
    GLAD_GLX_SGIX_pbuffer = has_ext(display, screen, "GLX_SGIX_pbuffer");
    GLAD_GLX_SGIX_swap_barrier = has_ext(display, screen, "GLX_SGIX_swap_barrier");
    GLAD_GLX_SGIX_swap_group = has_ext(display, screen, "GLX_SGIX_swap_group");
    GLAD_GLX_SGIX_video_resize = has_ext(display, screen, "GLX_SGIX_video_resize");
    GLAD_GLX_SGIX_visual_select_group = has_ext(display, screen, "GLX_SGIX_visual_select_group");
    GLAD_GLX_SGI_cushion = has_ext(display, screen, "GLX_SGI_cushion");
    GLAD_GLX_SGI_make_current_read = has_ext(display, screen, "GLX_SGI_make_current_read");
    GLAD_GLX_SGI_swap_control = has_ext(display, screen, "GLX_SGI_swap_control");
    GLAD_GLX_SGI_video_sync = has_ext(display, screen, "GLX_SGI_video_sync");
    GLAD_GLX_SUN_get_transparent_index = has_ext(display, screen, "GLX_SUN_get_transparent_index");
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
    return major * 10 + minor;
}

int gladLoadGLX(Display *display, int screen, GLADloadproc load, void* userptr) {
    int version;
    glXQueryVersion = (PFNGLXQUERYVERSIONPROC)load("glXQueryVersion", userptr);
    if(glXQueryVersion == NULL) return 0;
    version = find_coreGLX(&display, &screen);

    load_GLX_VERSION_1_0(load, userptr);
    load_GLX_VERSION_1_1(load, userptr);
    load_GLX_VERSION_1_2(load, userptr);
    load_GLX_VERSION_1_3(load, userptr);
    load_GLX_VERSION_1_4(load, userptr);

    if (!find_extensionsGLX(display, screen)) return 0;
    load_GLX_3DFX_multisample(load, userptr);
    load_GLX_AMD_gpu_association(load, userptr);
    load_GLX_ARB_context_flush_control(load, userptr);
    load_GLX_ARB_create_context(load, userptr);
    load_GLX_ARB_create_context_no_error(load, userptr);
    load_GLX_ARB_create_context_profile(load, userptr);
    load_GLX_ARB_create_context_robustness(load, userptr);
    load_GLX_ARB_fbconfig_float(load, userptr);
    load_GLX_ARB_framebuffer_sRGB(load, userptr);
    load_GLX_ARB_get_proc_address(load, userptr);
    load_GLX_ARB_multisample(load, userptr);
    load_GLX_ARB_robustness_application_isolation(load, userptr);
    load_GLX_ARB_robustness_share_group_isolation(load, userptr);
    load_GLX_ARB_vertex_buffer_object(load, userptr);
    load_GLX_EXT_buffer_age(load, userptr);
    load_GLX_EXT_create_context_es2_profile(load, userptr);
    load_GLX_EXT_create_context_es_profile(load, userptr);
    load_GLX_EXT_fbconfig_packed_float(load, userptr);
    load_GLX_EXT_framebuffer_sRGB(load, userptr);
    load_GLX_EXT_import_context(load, userptr);
    load_GLX_EXT_libglvnd(load, userptr);
    load_GLX_EXT_no_config_context(load, userptr);
    load_GLX_EXT_stereo_tree(load, userptr);
    load_GLX_EXT_swap_control(load, userptr);
    load_GLX_EXT_swap_control_tear(load, userptr);
    load_GLX_EXT_texture_from_pixmap(load, userptr);
    load_GLX_EXT_visual_info(load, userptr);
    load_GLX_EXT_visual_rating(load, userptr);
    load_GLX_INTEL_swap_event(load, userptr);
    load_GLX_MESA_agp_offset(load, userptr);
    load_GLX_MESA_copy_sub_buffer(load, userptr);
    load_GLX_MESA_pixmap_colormap(load, userptr);
    load_GLX_MESA_query_renderer(load, userptr);
    load_GLX_MESA_release_buffers(load, userptr);
    load_GLX_MESA_set_3dfx_mode(load, userptr);
    load_GLX_MESA_swap_control(load, userptr);
    load_GLX_NV_copy_buffer(load, userptr);
    load_GLX_NV_copy_image(load, userptr);
    load_GLX_NV_delay_before_swap(load, userptr);
    load_GLX_NV_float_buffer(load, userptr);
    load_GLX_NV_multisample_coverage(load, userptr);
    load_GLX_NV_present_video(load, userptr);
    load_GLX_NV_robustness_video_memory_purge(load, userptr);
    load_GLX_NV_swap_group(load, userptr);
    load_GLX_NV_video_capture(load, userptr);
    load_GLX_NV_video_out(load, userptr);
    load_GLX_OML_swap_method(load, userptr);
    load_GLX_OML_sync_control(load, userptr);
    load_GLX_SGIS_blended_overlay(load, userptr);
    load_GLX_SGIS_multisample(load, userptr);
    load_GLX_SGIS_shared_multisample(load, userptr);
    load_GLX_SGIX_fbconfig(load, userptr);
    load_GLX_SGIX_hyperpipe(load, userptr);
    load_GLX_SGIX_pbuffer(load, userptr);
    load_GLX_SGIX_swap_barrier(load, userptr);
    load_GLX_SGIX_swap_group(load, userptr);
    load_GLX_SGIX_video_resize(load, userptr);
    load_GLX_SGIX_visual_select_group(load, userptr);
    load_GLX_SGI_cushion(load, userptr);
    load_GLX_SGI_make_current_read(load, userptr);
    load_GLX_SGI_swap_control(load, userptr);
    load_GLX_SGI_video_sync(load, userptr);
    load_GLX_SUN_get_transparent_index(load, userptr);

    return version;
}

static void* glad_glx_get_proc_from_userptr(const char* name, void *userptr) {
    return ((void* (*)(const char *name))userptr)(name);
}

int gladLoadGLXSimple(Display *display, int screen, GLADsimpleloadproc load) {
    return gladLoadGLX(display, screen, glad_glx_get_proc_from_userptr, (void*) load);
}


#ifdef GLAD_GLX

#ifndef __glad_loader_library_c_
#define __glad_loader_library_c_

#include <stddef.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif


static void* glad_get_dlopen_handle(const char *lib_names[], int length) {
    int i;
    void *handle;

    for (i = 0; i < length; ++i) {
#ifdef _WIN32
        handle = LoadLibraryA(lib_names[i]);
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
#ifdef _WIN32
        FreeLibrary((HMODULE) handle);
#else
        dlclose(handle);
#endif
    }
}

static void* glad_dlsym_handle(void* handle, const char *name) {
    if (handle == NULL) {
        return NULL;
    }

#ifdef _WIN32
    return (void*) GetProcAddress((HMODULE) handle, name);
#else
    return dlsym(handle, name);
#endif
}

#endif /* __glad_loader_library_c_ */
static void* glad_glx_get_proc(const char *name, void *userptr) {
    return ((void* (*)(const char *name))userptr)(name);
}

static void* _glx_handle;

int gladLoadGLXInternalLoader(Display *display, int screen) {
    static const char *NAMES[] = {
#if defined __CYGWIN__
        "libGL-1.so",
#endif
        "libGL.so.1",
        "libGL.so"
    };

    int version = 0;
    int did_load = 0;
    void *userptrLoader;

    if (_glx_handle == NULL) {
        _glx_handle = glad_get_dlopen_handle(NAMES, sizeof(NAMES) / sizeof(NAMES[0]));
        did_load = _glx_handle != NULL;
    }

    if (_glx_handle != NULL) {
        userptrLoader = glad_dlsym_handle(_glx_handle, "glXGetProcAddressARB");
        if (userptrLoader != NULL) {
            version = gladLoadGLX(display, screen, (GLADloadproc) glad_glx_get_proc, userptrLoader);
        }

        if (!version && did_load) {
            glad_close_dlopen_handle(_glx_handle);
            _glx_handle = NULL;
        }
    }

    return version;
}

void gladUnloadGLXInternalLoader() {
    if (_glx_handle != NULL) {
        glad_close_dlopen_handle(_glx_handle);
        _glx_handle = NULL;
    }
}

#endif /* GLAD_GLX */
