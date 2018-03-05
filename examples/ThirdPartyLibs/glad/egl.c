#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <glad/egl.h>


int GLAD_EGL_VERSION_1_0;
int GLAD_EGL_VERSION_1_1;
int GLAD_EGL_VERSION_1_2;
int GLAD_EGL_VERSION_1_3;
int GLAD_EGL_VERSION_1_4;
int GLAD_EGL_VERSION_1_5;
int GLAD_EGL_EXT_device_base;
int GLAD_EGL_EXT_device_enumeration;
int GLAD_EGL_EXT_device_query;
int GLAD_EGL_EXT_platform_base;
int GLAD_EGL_EXT_platform_device;
int GLAD_EGL_NV_cuda_event;
int GLAD_EGL_NV_device_cuda;


PFNEGLWAITNATIVEPROC glad_eglWaitNative;
PFNEGLCREATEWINDOWSURFACEPROC glad_eglCreateWindowSurface;
PFNEGLQUERYSTRINGPROC glad_eglQueryString;
PFNEGLINITIALIZEPROC glad_eglInitialize;
PFNEGLQUERYCONTEXTPROC glad_eglQueryContext;
PFNEGLGETCURRENTSURFACEPROC glad_eglGetCurrentSurface;
PFNEGLSWAPINTERVALPROC glad_eglSwapInterval;
PFNEGLCREATESYNCPROC glad_eglCreateSync;
PFNEGLCHOOSECONFIGPROC glad_eglChooseConfig;
PFNEGLCREATEPIXMAPSURFACEPROC glad_eglCreatePixmapSurface;
PFNEGLGETCONFIGATTRIBPROC glad_eglGetConfigAttrib;
PFNEGLQUERYDEVICEATTRIBEXTPROC glad_eglQueryDeviceAttribEXT;
PFNEGLCREATEPBUFFERSURFACEPROC glad_eglCreatePbufferSurface;
PFNEGLCREATEPBUFFERFROMCLIENTBUFFERPROC glad_eglCreatePbufferFromClientBuffer;
PFNEGLMAKECURRENTPROC glad_eglMakeCurrent;
PFNEGLGETPLATFORMDISPLAYPROC glad_eglGetPlatformDisplay;
PFNEGLRELEASETEXIMAGEPROC glad_eglReleaseTexImage;
PFNEGLGETERRORPROC glad_eglGetError;
PFNEGLWAITSYNCPROC glad_eglWaitSync;
PFNEGLWAITGLPROC glad_eglWaitGL;
PFNEGLGETPROCADDRESSPROC glad_eglGetProcAddress;
PFNEGLCREATECONTEXTPROC glad_eglCreateContext;
PFNEGLDESTROYCONTEXTPROC glad_eglDestroyContext;
PFNEGLBINDAPIPROC glad_eglBindAPI;
PFNEGLGETCURRENTCONTEXTPROC glad_eglGetCurrentContext;
PFNEGLGETCONFIGSPROC glad_eglGetConfigs;
PFNEGLQUERYDEVICESTRINGEXTPROC glad_eglQueryDeviceStringEXT;
PFNEGLCREATEPLATFORMPIXMAPSURFACEPROC glad_eglCreatePlatformPixmapSurface;
PFNEGLGETSYNCATTRIBPROC glad_eglGetSyncAttrib;
PFNEGLGETPLATFORMDISPLAYEXTPROC glad_eglGetPlatformDisplayEXT;
PFNEGLCREATEPLATFORMWINDOWSURFACEEXTPROC glad_eglCreatePlatformWindowSurfaceEXT;
PFNEGLBINDTEXIMAGEPROC glad_eglBindTexImage;
PFNEGLCREATEPLATFORMWINDOWSURFACEPROC glad_eglCreatePlatformWindowSurface;
PFNEGLCOPYBUFFERSPROC glad_eglCopyBuffers;
PFNEGLCREATEPLATFORMPIXMAPSURFACEEXTPROC glad_eglCreatePlatformPixmapSurfaceEXT;
PFNEGLGETDISPLAYPROC glad_eglGetDisplay;
PFNEGLCREATEIMAGEPROC glad_eglCreateImage;
PFNEGLWAITCLIENTPROC glad_eglWaitClient;
PFNEGLGETCURRENTDISPLAYPROC glad_eglGetCurrentDisplay;
PFNEGLTERMINATEPROC glad_eglTerminate;
PFNEGLDESTROYIMAGEPROC glad_eglDestroyImage;
PFNEGLQUERYDISPLAYATTRIBEXTPROC glad_eglQueryDisplayAttribEXT;
PFNEGLQUERYSURFACEPROC glad_eglQuerySurface;
PFNEGLSWAPBUFFERSPROC glad_eglSwapBuffers;
PFNEGLCLIENTWAITSYNCPROC glad_eglClientWaitSync;
PFNEGLRELEASETHREADPROC glad_eglReleaseThread;
PFNEGLDESTROYSURFACEPROC glad_eglDestroySurface;
PFNEGLQUERYDEVICESEXTPROC glad_eglQueryDevicesEXT;
PFNEGLSURFACEATTRIBPROC glad_eglSurfaceAttrib;
PFNEGLQUERYAPIPROC glad_eglQueryAPI;
PFNEGLDESTROYSYNCPROC glad_eglDestroySync;

static void load_EGL_VERSION_1_0(GLADloadproc load, void* userptr) {
    if(!GLAD_EGL_VERSION_1_0) return;
    glad_eglWaitNative = (PFNEGLWAITNATIVEPROC)load("eglWaitNative", userptr);
    glad_eglCreateWindowSurface = (PFNEGLCREATEWINDOWSURFACEPROC)load("eglCreateWindowSurface", userptr);
    glad_eglQueryString = (PFNEGLQUERYSTRINGPROC)load("eglQueryString", userptr);
    glad_eglInitialize = (PFNEGLINITIALIZEPROC)load("eglInitialize", userptr);
    glad_eglQueryContext = (PFNEGLQUERYCONTEXTPROC)load("eglQueryContext", userptr);
    glad_eglGetCurrentSurface = (PFNEGLGETCURRENTSURFACEPROC)load("eglGetCurrentSurface", userptr);
    glad_eglChooseConfig = (PFNEGLCHOOSECONFIGPROC)load("eglChooseConfig", userptr);
    glad_eglCreatePixmapSurface = (PFNEGLCREATEPIXMAPSURFACEPROC)load("eglCreatePixmapSurface", userptr);
    glad_eglGetConfigAttrib = (PFNEGLGETCONFIGATTRIBPROC)load("eglGetConfigAttrib", userptr);
    glad_eglCreatePbufferSurface = (PFNEGLCREATEPBUFFERSURFACEPROC)load("eglCreatePbufferSurface", userptr);
    glad_eglMakeCurrent = (PFNEGLMAKECURRENTPROC)load("eglMakeCurrent", userptr);
    glad_eglGetError = (PFNEGLGETERRORPROC)load("eglGetError", userptr);
    glad_eglWaitGL = (PFNEGLWAITGLPROC)load("eglWaitGL", userptr);
    glad_eglGetProcAddress = (PFNEGLGETPROCADDRESSPROC)load("eglGetProcAddress", userptr);
    glad_eglCreateContext = (PFNEGLCREATECONTEXTPROC)load("eglCreateContext", userptr);
    glad_eglDestroyContext = (PFNEGLDESTROYCONTEXTPROC)load("eglDestroyContext", userptr);
    glad_eglGetConfigs = (PFNEGLGETCONFIGSPROC)load("eglGetConfigs", userptr);
    glad_eglCopyBuffers = (PFNEGLCOPYBUFFERSPROC)load("eglCopyBuffers", userptr);
    glad_eglGetDisplay = (PFNEGLGETDISPLAYPROC)load("eglGetDisplay", userptr);
    glad_eglGetCurrentDisplay = (PFNEGLGETCURRENTDISPLAYPROC)load("eglGetCurrentDisplay", userptr);
    glad_eglQuerySurface = (PFNEGLQUERYSURFACEPROC)load("eglQuerySurface", userptr);
    glad_eglSwapBuffers = (PFNEGLSWAPBUFFERSPROC)load("eglSwapBuffers", userptr);
    glad_eglTerminate = (PFNEGLTERMINATEPROC)load("eglTerminate", userptr);
    glad_eglDestroySurface = (PFNEGLDESTROYSURFACEPROC)load("eglDestroySurface", userptr);
}
static void load_EGL_VERSION_1_1(GLADloadproc load, void* userptr) {
    if(!GLAD_EGL_VERSION_1_1) return;
    glad_eglSwapInterval = (PFNEGLSWAPINTERVALPROC)load("eglSwapInterval", userptr);
    glad_eglSurfaceAttrib = (PFNEGLSURFACEATTRIBPROC)load("eglSurfaceAttrib", userptr);
    glad_eglReleaseTexImage = (PFNEGLRELEASETEXIMAGEPROC)load("eglReleaseTexImage", userptr);
    glad_eglBindTexImage = (PFNEGLBINDTEXIMAGEPROC)load("eglBindTexImage", userptr);
}
static void load_EGL_VERSION_1_2(GLADloadproc load, void* userptr) {
    if(!GLAD_EGL_VERSION_1_2) return;
    glad_eglBindAPI = (PFNEGLBINDAPIPROC)load("eglBindAPI", userptr);
    glad_eglWaitClient = (PFNEGLWAITCLIENTPROC)load("eglWaitClient", userptr);
    glad_eglReleaseThread = (PFNEGLRELEASETHREADPROC)load("eglReleaseThread", userptr);
    glad_eglQueryAPI = (PFNEGLQUERYAPIPROC)load("eglQueryAPI", userptr);
    glad_eglCreatePbufferFromClientBuffer = (PFNEGLCREATEPBUFFERFROMCLIENTBUFFERPROC)load("eglCreatePbufferFromClientBuffer", userptr);
}
static void load_EGL_VERSION_1_3(GLADloadproc load, void* userptr) {
}
static void load_EGL_VERSION_1_4(GLADloadproc load, void* userptr) {
    if(!GLAD_EGL_VERSION_1_4) return;
    glad_eglGetCurrentContext = (PFNEGLGETCURRENTCONTEXTPROC)load("eglGetCurrentContext", userptr);
}
static void load_EGL_VERSION_1_5(GLADloadproc load, void* userptr) {
    if(!GLAD_EGL_VERSION_1_5) return;
    glad_eglCreatePlatformWindowSurface = (PFNEGLCREATEPLATFORMWINDOWSURFACEPROC)load("eglCreatePlatformWindowSurface", userptr);
    glad_eglWaitSync = (PFNEGLWAITSYNCPROC)load("eglWaitSync", userptr);
    glad_eglCreateImage = (PFNEGLCREATEIMAGEPROC)load("eglCreateImage", userptr);
    glad_eglDestroyImage = (PFNEGLDESTROYIMAGEPROC)load("eglDestroyImage", userptr);
    glad_eglGetPlatformDisplay = (PFNEGLGETPLATFORMDISPLAYPROC)load("eglGetPlatformDisplay", userptr);
    glad_eglClientWaitSync = (PFNEGLCLIENTWAITSYNCPROC)load("eglClientWaitSync", userptr);
    glad_eglCreatePlatformPixmapSurface = (PFNEGLCREATEPLATFORMPIXMAPSURFACEPROC)load("eglCreatePlatformPixmapSurface", userptr);
    glad_eglGetSyncAttrib = (PFNEGLGETSYNCATTRIBPROC)load("eglGetSyncAttrib", userptr);
    glad_eglDestroySync = (PFNEGLDESTROYSYNCPROC)load("eglDestroySync", userptr);
    glad_eglCreateSync = (PFNEGLCREATESYNCPROC)load("eglCreateSync", userptr);
}
static void load_EGL_EXT_device_base(GLADloadproc load, void* userptr) {
    if(!GLAD_EGL_EXT_device_base) return;
    glad_eglQueryDevicesEXT = (PFNEGLQUERYDEVICESEXTPROC)load("eglQueryDevicesEXT", userptr);
    glad_eglQueryDisplayAttribEXT = (PFNEGLQUERYDISPLAYATTRIBEXTPROC)load("eglQueryDisplayAttribEXT", userptr);
    glad_eglQueryDeviceAttribEXT = (PFNEGLQUERYDEVICEATTRIBEXTPROC)load("eglQueryDeviceAttribEXT", userptr);
    glad_eglQueryDeviceStringEXT = (PFNEGLQUERYDEVICESTRINGEXTPROC)load("eglQueryDeviceStringEXT", userptr);
}
static void load_EGL_EXT_device_enumeration(GLADloadproc load, void* userptr) {
    if(!GLAD_EGL_EXT_device_enumeration) return;
    glad_eglQueryDevicesEXT = (PFNEGLQUERYDEVICESEXTPROC)load("eglQueryDevicesEXT", userptr);
}
static void load_EGL_EXT_device_query(GLADloadproc load, void* userptr) {
    if(!GLAD_EGL_EXT_device_query) return;
    glad_eglQueryDeviceAttribEXT = (PFNEGLQUERYDEVICEATTRIBEXTPROC)load("eglQueryDeviceAttribEXT", userptr);
    glad_eglQueryDisplayAttribEXT = (PFNEGLQUERYDISPLAYATTRIBEXTPROC)load("eglQueryDisplayAttribEXT", userptr);
    glad_eglQueryDeviceStringEXT = (PFNEGLQUERYDEVICESTRINGEXTPROC)load("eglQueryDeviceStringEXT", userptr);
}
static void load_EGL_EXT_platform_base(GLADloadproc load, void* userptr) {
    if(!GLAD_EGL_EXT_platform_base) return;
    glad_eglCreatePlatformPixmapSurfaceEXT = (PFNEGLCREATEPLATFORMPIXMAPSURFACEEXTPROC)load("eglCreatePlatformPixmapSurfaceEXT", userptr);
    glad_eglGetPlatformDisplayEXT = (PFNEGLGETPLATFORMDISPLAYEXTPROC)load("eglGetPlatformDisplayEXT", userptr);
    glad_eglCreatePlatformWindowSurfaceEXT = (PFNEGLCREATEPLATFORMWINDOWSURFACEEXTPROC)load("eglCreatePlatformWindowSurfaceEXT", userptr);
}
static void load_EGL_EXT_platform_device(GLADloadproc load, void* userptr) {
}
static void load_EGL_NV_cuda_event(GLADloadproc load, void* userptr) {
}
static void load_EGL_NV_device_cuda(GLADloadproc load, void* userptr) {
}


static int get_exts(EGLDisplay display, const char **extensions) {
    *extensions = eglQueryString(display, EGL_EXTENSIONS);

    return extensions != NULL;
}

static int has_ext(const char *extensions, const char *name) {
    const char *loc;
    const char *terminator;
    if(extensions == NULL) {
        return 0;
    }
    while(1) {
        loc = strstr(extensions, extensions);
        if(loc == NULL) {
            return 0;
        }
        terminator = loc + strlen(extensions);
        if((loc == extensions || *(loc - 1) == ' ') &&
            (*terminator == ' ' || *terminator == '\0')) {
            return 1;
        }
        extensions = terminator;
    }
}

static int find_extensionsEGL(EGLDisplay display) {
    const char *extensions;
    if (!get_exts(display, &extensions)) return 0;

    GLAD_EGL_EXT_device_base = has_ext(extensions, "EGL_EXT_device_base");
    GLAD_EGL_EXT_device_enumeration = has_ext(extensions, "EGL_EXT_device_enumeration");
    GLAD_EGL_EXT_device_query = has_ext(extensions, "EGL_EXT_device_query");
    GLAD_EGL_EXT_platform_base = has_ext(extensions, "EGL_EXT_platform_base");
    GLAD_EGL_EXT_platform_device = has_ext(extensions, "EGL_EXT_platform_device");
    GLAD_EGL_NV_cuda_event = has_ext(extensions, "EGL_NV_cuda_event");
    GLAD_EGL_NV_device_cuda = has_ext(extensions, "EGL_NV_device_cuda");

    return 1;
}

static int find_coreEGL(EGLDisplay display) {
    int major, minor;
    const char *version;

    if (display == NULL) {
        display = EGL_NO_DISPLAY; /* this is usually NULL, better safe than sorry */
    }
    if (display == EGL_NO_DISPLAY) {
        display = eglGetCurrentDisplay();
    }
    if (display == EGL_NO_DISPLAY) {
        display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    }
    if (display == EGL_NO_DISPLAY) {
        return 0;
    }

    version = eglQueryString(display, EGL_VERSION);
    (void) eglGetError();

    if (version == NULL) {
        major = 1;
        minor = 0;
    } else {
#ifdef _MSC_VER
        sscanf_s(version, "%d.%d", &major, &minor);
#else
        sscanf(version, "%d.%d", &major, &minor);
#endif
    }

    GLAD_EGL_VERSION_1_0 = (major == 1 && minor >= 0) || major > 1;
    GLAD_EGL_VERSION_1_1 = (major == 1 && minor >= 1) || major > 1;
    GLAD_EGL_VERSION_1_2 = (major == 1 && minor >= 2) || major > 1;
    GLAD_EGL_VERSION_1_3 = (major == 1 && minor >= 3) || major > 1;
    GLAD_EGL_VERSION_1_4 = (major == 1 && minor >= 4) || major > 1;
    GLAD_EGL_VERSION_1_5 = (major == 1 && minor >= 5) || major > 1;

    return major * 10 + minor;
}

int gladLoadEGL(EGLDisplay display, GLADloadproc load, void* userptr) {
    int version;
    eglGetDisplay = (PFNEGLGETDISPLAYPROC)load("eglGetDisplay", userptr);
    eglGetCurrentDisplay = (PFNEGLGETCURRENTDISPLAYPROC)load("eglGetCurrentDisplay", userptr);
    eglQueryString = (PFNEGLQUERYSTRINGPROC)load("eglQueryString", userptr);
    eglGetError = (PFNEGLGETERRORPROC)load("eglGetError", userptr);
    if (eglGetDisplay == NULL || eglGetCurrentDisplay == NULL || eglQueryString == NULL || eglGetError == NULL) return 0;

    version = find_coreEGL(display);
    if (!version) return 0;
    load_EGL_VERSION_1_0(load, userptr);
    load_EGL_VERSION_1_1(load, userptr);
    load_EGL_VERSION_1_2(load, userptr);
    load_EGL_VERSION_1_3(load, userptr);
    load_EGL_VERSION_1_4(load, userptr);
    load_EGL_VERSION_1_5(load, userptr);

    if (!find_extensionsEGL(display)) return 0;
    load_EGL_EXT_device_base(load, userptr);
    load_EGL_EXT_device_enumeration(load, userptr);
    load_EGL_EXT_device_query(load, userptr);
    load_EGL_EXT_platform_base(load, userptr);
    load_EGL_EXT_platform_device(load, userptr);
    load_EGL_NV_cuda_event(load, userptr);
    load_EGL_NV_device_cuda(load, userptr);

    return version;
}

static void* glad_egl_get_proc_from_userptr(const char* name, void *userptr) {
    return ((void* (*)(const char *name))userptr)(name);
}

int gladLoadEGLSimple(EGLDisplay display, GLADsimpleloadproc load) {
    return gladLoadEGL(display, glad_egl_get_proc_from_userptr, (void*) load);
}


#ifdef GLAD_EGL

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
typedef __eglMustCastToProperFunctionPointerType (APIENTRYP GLAD_EGL_PFNGETPROCADDRESSPROC_PRIVATE)(const char*);
struct _glad_egl_userptr {
    void *handle;
    GLAD_EGL_PFNGETPROCADDRESSPROC_PRIVATE get_proc_address_ptr;
};

static void* glad_egl_get_proc(const char* name, void *vuserptr) {
    struct _glad_egl_userptr userptr = *(struct _glad_egl_userptr*) vuserptr;
    void* result = NULL;

    result = (void*) glad_dlsym_handle(userptr.handle, name);
    if (result == NULL) {
        result = (void*) userptr.get_proc_address_ptr(name);
    }

    return result;
}

static void* _egl_handle = NULL;

int gladLoadEGLInternalLoader(EGLDisplay display) {
#ifdef __APPLE__
    static const char *NAMES[] = {"libEGL.dylib"};
#elif defined _WIN32
    static const char *NAMES[] = {"libEGL.dll", "EGL.dll"};
#else
    static const char *NAMES[] = {"libEGL.so.1", "libEGL.so"};
#endif

    int version = 0;
    int did_load = 0;
    struct _glad_egl_userptr userptr;

    if (_egl_handle == NULL) {
        _egl_handle = glad_get_dlopen_handle(NAMES, sizeof(NAMES) / sizeof(NAMES[0]));
        did_load = _egl_handle != NULL;
    }

    if (_egl_handle != NULL) {
        userptr.handle = _egl_handle;
        userptr.get_proc_address_ptr = (GLAD_EGL_PFNGETPROCADDRESSPROC_PRIVATE) glad_dlsym_handle(_egl_handle, "eglGetProcAddress");
        if (userptr.get_proc_address_ptr != NULL) {
            version = gladLoadEGL(display, (GLADloadproc) glad_egl_get_proc, &userptr);
        }

        if (!version && did_load) {
            glad_close_dlopen_handle(_egl_handle);
            _egl_handle = NULL;
        }
    }

    return version;
}

void gladUnloadEGLInternalLoader() {
    if (_egl_handle != NULL) {
        glad_close_dlopen_handle(_egl_handle);
        _egl_handle = NULL;
    }
}

#endif /* GLAD_EGL */