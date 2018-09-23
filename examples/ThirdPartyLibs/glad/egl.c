#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <glad/egl.h>

#ifndef GLAD_IMPL_UTIL_C_
#define GLAD_IMPL_UTIL_C_

#if _MSC_VER >= 1400
#define GLAD_IMPL_UTIL_STRNCPY(dest, source, len) strncpy_s(dest, len, source, len - 1);
#else
#define GLAD_IMPL_UTIL_STRNCPY(dest, source, len) strncpy(dest, source, len);
#endif

#ifdef _MSC_VER
#define GLAD_IMPL_UTIL_SSCANF sscanf_s
#else
#define GLAD_IMPL_UTIL_SSCANF sscanf
#endif

#endif /* GLAD_IMPL_UTIL_C_ */

int GLAD_EGL_VERSION_1_0;
int GLAD_EGL_VERSION_1_1;
int GLAD_EGL_VERSION_1_2;
int GLAD_EGL_VERSION_1_3;
int GLAD_EGL_VERSION_1_4;
int GLAD_EGL_VERSION_1_5;
int GLAD_EGL_EXT_platform_device;
int GLAD_EGL_EXT_platform_base;
int GLAD_EGL_NV_device_cuda;
int GLAD_EGL_EXT_device_enumeration;
int GLAD_EGL_EXT_device_query;
int GLAD_EGL_EXT_device_base;
int GLAD_EGL_NV_cuda_event;

PFNEGLQUERYDEVICESEXTPROC glad_eglQueryDevicesEXT;
PFNEGLCREATEPBUFFERSURFACEPROC glad_eglCreatePbufferSurface;
PFNEGLGETERRORPROC glad_eglGetError;
PFNEGLGETPROCADDRESSPROC glad_eglGetProcAddress;
PFNEGLGETCURRENTDISPLAYPROC glad_eglGetCurrentDisplay;
PFNEGLQUERYAPIPROC glad_eglQueryAPI;
PFNEGLCREATEPLATFORMPIXMAPSURFACEPROC glad_eglCreatePlatformPixmapSurface;
PFNEGLQUERYDISPLAYATTRIBKHRPROC glad_eglQueryDisplayAttribKHR;
PFNEGLDESTROYSYNCPROC glad_eglDestroySync;
PFNEGLCREATEIMAGEPROC glad_eglCreateImage;
PFNEGLQUERYCONTEXTPROC glad_eglQueryContext;
PFNEGLSWAPBUFFERSPROC glad_eglSwapBuffers;
PFNEGLCREATECONTEXTPROC glad_eglCreateContext;
PFNEGLCHOOSECONFIGPROC glad_eglChooseConfig;
PFNEGLBINDAPIPROC glad_eglBindAPI;
PFNEGLWAITNATIVEPROC glad_eglWaitNative;
PFNEGLTERMINATEPROC glad_eglTerminate;
PFNEGLCREATEPLATFORMPIXMAPSURFACEEXTPROC glad_eglCreatePlatformPixmapSurfaceEXT;
PFNEGLRELEASETHREADPROC glad_eglReleaseThread;
PFNEGLCREATEPLATFORMWINDOWSURFACEPROC glad_eglCreatePlatformWindowSurface;
PFNEGLGETCONFIGATTRIBPROC glad_eglGetConfigAttrib;
PFNEGLGETCURRENTSURFACEPROC glad_eglGetCurrentSurface;
PFNEGLCOPYBUFFERSPROC glad_eglCopyBuffers;
PFNEGLGETPLATFORMDISPLAYEXTPROC glad_eglGetPlatformDisplayEXT;
PFNEGLQUERYDEVICEATTRIBEXTPROC glad_eglQueryDeviceAttribEXT;
PFNEGLCREATEPIXMAPSURFACEPROC glad_eglCreatePixmapSurface;
PFNEGLBINDTEXIMAGEPROC glad_eglBindTexImage;
PFNEGLGETPLATFORMDISPLAYPROC glad_eglGetPlatformDisplay;
PFNEGLGETDISPLAYPROC glad_eglGetDisplay;
PFNEGLQUERYSTRINGPROC glad_eglQueryString;
PFNEGLCLIENTWAITSYNCPROC glad_eglClientWaitSync;
PFNEGLWAITSYNCPROC glad_eglWaitSync;
PFNEGLDESTROYIMAGEPROC glad_eglDestroyImage;
PFNEGLRELEASETEXIMAGEPROC glad_eglReleaseTexImage;
PFNEGLQUERYDISPLAYATTRIBEXTPROC glad_eglQueryDisplayAttribEXT;
PFNEGLDESTROYCONTEXTPROC glad_eglDestroyContext;
PFNEGLQUERYDEVICESTRINGEXTPROC glad_eglQueryDeviceStringEXT;
PFNEGLCREATEWINDOWSURFACEPROC glad_eglCreateWindowSurface;
PFNEGLGETCURRENTCONTEXTPROC glad_eglGetCurrentContext;
PFNEGLINITIALIZEPROC glad_eglInitialize;
PFNEGLDESTROYSURFACEPROC glad_eglDestroySurface;
PFNEGLMAKECURRENTPROC glad_eglMakeCurrent;
PFNEGLCREATEPLATFORMWINDOWSURFACEEXTPROC glad_eglCreatePlatformWindowSurfaceEXT;
PFNEGLSWAPINTERVALPROC glad_eglSwapInterval;
PFNEGLCREATESYNCPROC glad_eglCreateSync;
PFNEGLGETSYNCATTRIBPROC glad_eglGetSyncAttrib;
PFNEGLSURFACEATTRIBPROC glad_eglSurfaceAttrib;
PFNEGLWAITGLPROC glad_eglWaitGL;
PFNEGLQUERYSURFACEPROC glad_eglQuerySurface;
PFNEGLGETCONFIGSPROC glad_eglGetConfigs;
PFNEGLWAITCLIENTPROC glad_eglWaitClient;
PFNEGLCREATEPBUFFERFROMCLIENTBUFFERPROC glad_eglCreatePbufferFromClientBuffer;

static void load_EGL_VERSION_1_0(GLADuserptrloadfunc load, void *userptr)
{
	if (!GLAD_EGL_VERSION_1_0) return;
	eglCreatePbufferSurface = (PFNEGLCREATEPBUFFERSURFACEPROC)load("eglCreatePbufferSurface", userptr);
	eglGetError = (PFNEGLGETERRORPROC)load("eglGetError", userptr);
	eglGetProcAddress = (PFNEGLGETPROCADDRESSPROC)load("eglGetProcAddress", userptr);
	eglGetCurrentDisplay = (PFNEGLGETCURRENTDISPLAYPROC)load("eglGetCurrentDisplay", userptr);
	eglQueryContext = (PFNEGLQUERYCONTEXTPROC)load("eglQueryContext", userptr);
	eglSwapBuffers = (PFNEGLSWAPBUFFERSPROC)load("eglSwapBuffers", userptr);
	eglCreateContext = (PFNEGLCREATECONTEXTPROC)load("eglCreateContext", userptr);
	eglChooseConfig = (PFNEGLCHOOSECONFIGPROC)load("eglChooseConfig", userptr);
	eglWaitNative = (PFNEGLWAITNATIVEPROC)load("eglWaitNative", userptr);
	eglTerminate = (PFNEGLTERMINATEPROC)load("eglTerminate", userptr);
	eglCopyBuffers = (PFNEGLCOPYBUFFERSPROC)load("eglCopyBuffers", userptr);
	eglGetConfigAttrib = (PFNEGLGETCONFIGATTRIBPROC)load("eglGetConfigAttrib", userptr);
	eglGetCurrentSurface = (PFNEGLGETCURRENTSURFACEPROC)load("eglGetCurrentSurface", userptr);
	eglCreatePixmapSurface = (PFNEGLCREATEPIXMAPSURFACEPROC)load("eglCreatePixmapSurface", userptr);
	eglGetDisplay = (PFNEGLGETDISPLAYPROC)load("eglGetDisplay", userptr);
	eglQueryString = (PFNEGLQUERYSTRINGPROC)load("eglQueryString", userptr);
	eglDestroyContext = (PFNEGLDESTROYCONTEXTPROC)load("eglDestroyContext", userptr);
	eglCreateWindowSurface = (PFNEGLCREATEWINDOWSURFACEPROC)load("eglCreateWindowSurface", userptr);
	eglInitialize = (PFNEGLINITIALIZEPROC)load("eglInitialize", userptr);
	eglDestroySurface = (PFNEGLDESTROYSURFACEPROC)load("eglDestroySurface", userptr);
	eglMakeCurrent = (PFNEGLMAKECURRENTPROC)load("eglMakeCurrent", userptr);
	eglWaitGL = (PFNEGLWAITGLPROC)load("eglWaitGL", userptr);
	eglQuerySurface = (PFNEGLQUERYSURFACEPROC)load("eglQuerySurface", userptr);
	eglGetConfigs = (PFNEGLGETCONFIGSPROC)load("eglGetConfigs", userptr);
}
static void load_EGL_VERSION_1_1(GLADuserptrloadfunc load, void *userptr)
{
	if (!GLAD_EGL_VERSION_1_1) return;
	eglReleaseTexImage = (PFNEGLRELEASETEXIMAGEPROC)load("eglReleaseTexImage", userptr);
	eglSurfaceAttrib = (PFNEGLSURFACEATTRIBPROC)load("eglSurfaceAttrib", userptr);
	eglBindTexImage = (PFNEGLBINDTEXIMAGEPROC)load("eglBindTexImage", userptr);
	eglSwapInterval = (PFNEGLSWAPINTERVALPROC)load("eglSwapInterval", userptr);
}
static void load_EGL_VERSION_1_2(GLADuserptrloadfunc load, void *userptr)
{
	if (!GLAD_EGL_VERSION_1_2) return;
	eglBindAPI = (PFNEGLBINDAPIPROC)load("eglBindAPI", userptr);
	eglQueryAPI = (PFNEGLQUERYAPIPROC)load("eglQueryAPI", userptr);
	eglWaitClient = (PFNEGLWAITCLIENTPROC)load("eglWaitClient", userptr);
	eglCreatePbufferFromClientBuffer = (PFNEGLCREATEPBUFFERFROMCLIENTBUFFERPROC)load("eglCreatePbufferFromClientBuffer", userptr);
	eglReleaseThread = (PFNEGLRELEASETHREADPROC)load("eglReleaseThread", userptr);
}
static void load_EGL_VERSION_1_4(GLADuserptrloadfunc load, void *userptr)
{
	if (!GLAD_EGL_VERSION_1_4) return;
	eglGetCurrentContext = (PFNEGLGETCURRENTCONTEXTPROC)load("eglGetCurrentContext", userptr);
}
static void load_EGL_VERSION_1_5(GLADuserptrloadfunc load, void *userptr)
{
	if (!GLAD_EGL_VERSION_1_5) return;
	eglDestroySync = (PFNEGLDESTROYSYNCPROC)load("eglDestroySync", userptr);
	eglCreateImage = (PFNEGLCREATEIMAGEPROC)load("eglCreateImage", userptr);
	eglGetPlatformDisplay = (PFNEGLGETPLATFORMDISPLAYPROC)load("eglGetPlatformDisplay", userptr);
	eglDestroyImage = (PFNEGLDESTROYIMAGEPROC)load("eglDestroyImage", userptr);
	eglClientWaitSync = (PFNEGLCLIENTWAITSYNCPROC)load("eglClientWaitSync", userptr);
	eglWaitSync = (PFNEGLWAITSYNCPROC)load("eglWaitSync", userptr);
	eglCreateSync = (PFNEGLCREATESYNCPROC)load("eglCreateSync", userptr);
	eglGetSyncAttrib = (PFNEGLGETSYNCATTRIBPROC)load("eglGetSyncAttrib", userptr);
	eglCreatePlatformWindowSurface = (PFNEGLCREATEPLATFORMWINDOWSURFACEPROC)load("eglCreatePlatformWindowSurface", userptr);
	eglCreatePlatformPixmapSurface = (PFNEGLCREATEPLATFORMPIXMAPSURFACEPROC)load("eglCreatePlatformPixmapSurface", userptr);
}
static void load_EGL_EXT_platform_base(GLADuserptrloadfunc load, void *userptr)
{
	if (!GLAD_EGL_EXT_platform_base) return;
	eglCreatePlatformWindowSurfaceEXT = (PFNEGLCREATEPLATFORMWINDOWSURFACEEXTPROC)load("eglCreatePlatformWindowSurfaceEXT", userptr);
	eglGetPlatformDisplayEXT = (PFNEGLGETPLATFORMDISPLAYEXTPROC)load("eglGetPlatformDisplayEXT", userptr);
	eglCreatePlatformPixmapSurfaceEXT = (PFNEGLCREATEPLATFORMPIXMAPSURFACEEXTPROC)load("eglCreatePlatformPixmapSurfaceEXT", userptr);
}
static void load_EGL_EXT_device_enumeration(GLADuserptrloadfunc load, void *userptr)
{
	if (!GLAD_EGL_EXT_device_enumeration) return;
	eglQueryDevicesEXT = (PFNEGLQUERYDEVICESEXTPROC)load("eglQueryDevicesEXT", userptr);
}
static void load_EGL_EXT_device_query(GLADuserptrloadfunc load, void *userptr)
{
	if (!GLAD_EGL_EXT_device_query) return;
	eglQueryDisplayAttribEXT = (PFNEGLQUERYDISPLAYATTRIBEXTPROC)load("eglQueryDisplayAttribEXT", userptr);
	eglQueryDeviceStringEXT = (PFNEGLQUERYDEVICESTRINGEXTPROC)load("eglQueryDeviceStringEXT", userptr);
	eglQueryDeviceAttribEXT = (PFNEGLQUERYDEVICEATTRIBEXTPROC)load("eglQueryDeviceAttribEXT", userptr);
	eglQueryDisplayAttribKHR = (PFNEGLQUERYDISPLAYATTRIBKHRPROC)load("eglQueryDisplayAttribKHR", userptr);
}
static void load_EGL_EXT_device_base(GLADuserptrloadfunc load, void *userptr)
{
	if (!GLAD_EGL_EXT_device_base) return;
	eglQueryDisplayAttribEXT = (PFNEGLQUERYDISPLAYATTRIBEXTPROC)load("eglQueryDisplayAttribEXT", userptr);
	eglQueryDevicesEXT = (PFNEGLQUERYDEVICESEXTPROC)load("eglQueryDevicesEXT", userptr);
	eglQueryDeviceStringEXT = (PFNEGLQUERYDEVICESTRINGEXTPROC)load("eglQueryDeviceStringEXT", userptr);
	eglQueryDeviceAttribEXT = (PFNEGLQUERYDEVICEATTRIBEXTPROC)load("eglQueryDeviceAttribEXT", userptr);
	eglQueryDisplayAttribKHR = (PFNEGLQUERYDISPLAYATTRIBKHRPROC)load("eglQueryDisplayAttribKHR", userptr);
}

static int get_exts(EGLDisplay display, const char **extensions)
{
	*extensions = eglQueryString(display, EGL_EXTENSIONS);

	return extensions != NULL;
}

static int has_ext(const char *extensions, const char *ext)
{
	const char *loc;
	const char *terminator;
	if (extensions == NULL)
	{
		return 0;
	}
	while (1)
	{
		loc = strstr(extensions, ext);
		if (loc == NULL)
		{
			return 0;
		}
		terminator = loc + strlen(ext);
		if ((loc == extensions || *(loc - 1) == ' ') &&
			(*terminator == ' ' || *terminator == '\0'))
		{
			return 1;
		}
		extensions = terminator;
	}
}

static GLADapiproc glad_egl_get_proc_from_userptr(const char *name, void *userptr)
{
	return (GLAD_GNUC_EXTENSION(GLADapiproc(*)(const char *name)) userptr)(name);
}

static int find_extensionsEGL(EGLDisplay display)
{
	const char *extensions;
	if (!get_exts(display, &extensions)) return 0;

	GLAD_EGL_EXT_platform_device = has_ext(extensions, "EGL_EXT_platform_device");
	GLAD_EGL_EXT_platform_base = has_ext(extensions, "EGL_EXT_platform_base");
	GLAD_EGL_NV_device_cuda = has_ext(extensions, "EGL_NV_device_cuda");
	GLAD_EGL_EXT_device_enumeration = has_ext(extensions, "EGL_EXT_device_enumeration");
	GLAD_EGL_EXT_device_query = has_ext(extensions, "EGL_EXT_device_query");
	GLAD_EGL_EXT_device_base = has_ext(extensions, "EGL_EXT_device_base");
	GLAD_EGL_NV_cuda_event = has_ext(extensions, "EGL_NV_cuda_event");

	return 1;
}

static int find_coreEGL(EGLDisplay display)
{
	int major, minor;
	const char *version;

	if (display == NULL)
	{
		display = EGL_NO_DISPLAY; /* this is usually NULL, better safe than sorry */
	}
	if (display == EGL_NO_DISPLAY)
	{
		display = eglGetCurrentDisplay();
	}
	if (display == EGL_NO_DISPLAY)
	{
		display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
	}
	if (display == EGL_NO_DISPLAY)
	{
		return 0;
	}

	version = eglQueryString(display, EGL_VERSION);
	(void)eglGetError();

	if (version == NULL)
	{
		major = 1;
		minor = 0;
	}
	else
	{
		GLAD_IMPL_UTIL_SSCANF(version, "%d.%d", &major, &minor);
	}

	GLAD_EGL_VERSION_1_0 = (major == 1 && minor >= 0) || major > 1;
	GLAD_EGL_VERSION_1_1 = (major == 1 && minor >= 1) || major > 1;
	GLAD_EGL_VERSION_1_2 = (major == 1 && minor >= 2) || major > 1;
	GLAD_EGL_VERSION_1_3 = (major == 1 && minor >= 3) || major > 1;
	GLAD_EGL_VERSION_1_4 = (major == 1 && minor >= 4) || major > 1;
	GLAD_EGL_VERSION_1_5 = (major == 1 && minor >= 5) || major > 1;

	return GLAD_MAKE_VERSION(major, minor);
}

int gladLoadEGLUserPtr(EGLDisplay display, GLADuserptrloadfunc load, void *userptr)
{
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
	load_EGL_VERSION_1_4(load, userptr);
	load_EGL_VERSION_1_5(load, userptr);

	if (!find_extensionsEGL(display)) return 0;
	load_EGL_EXT_platform_base(load, userptr);
	load_EGL_EXT_device_enumeration(load, userptr);
	load_EGL_EXT_device_query(load, userptr);
	load_EGL_EXT_device_base(load, userptr);

	return version;
}

int gladLoadEGL(EGLDisplay display, GLADloadfunc load)
{
	return gladLoadEGLUserPtr(display, glad_egl_get_proc_from_userptr, GLAD_GNUC_EXTENSION(void *) load);
}

#ifdef GLAD_EGL

#ifndef GLAD_LOADER_LIBRARY_C_
#define GLAD_LOADER_LIBRARY_C_

#include <stddef.h>
#include <stdlib.h>

#if GLAD_PLATFORM_WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

static void *glad_get_dlopen_handle(const char *lib_names[], int length)
{
	void *handle = NULL;
	int i;

	for (i = 0; i < length; ++i)
	{
#if GLAD_PLATFORM_WIN32
#if GLAD_PLATFORM_UWP
		size_t buffer_size = (strlen(lib_names[i]) + 1) * sizeof(WCHAR);
		LPWSTR buffer = (LPWSTR)malloc(buffer_size);
		if (buffer != NULL)
		{
			int ret = MultiByteToWideChar(CP_ACP, 0, lib_names[i], -1, buffer, buffer_size);
			if (ret != 0)
			{
				handle = (void *)LoadPackagedLibrary(buffer, 0);
			}
			free((void *)buffer);
		}
#else
		handle = (void *)LoadLibraryA(lib_names[i]);
#endif
#else
		handle = dlopen(lib_names[i], RTLD_LAZY | RTLD_LOCAL);
#endif
		if (handle != NULL)
		{
			return handle;
		}
	}

	return NULL;
}

static void glad_close_dlopen_handle(void *handle)
{
	if (handle != NULL)
	{
#if GLAD_PLATFORM_WIN32
		FreeLibrary((HMODULE)handle);
#else
		dlclose(handle);
#endif
	}
}

static GLADapiproc glad_dlsym_handle(void *handle, const char *name)
{
	if (handle == NULL)
	{
		return NULL;
	}

#if GLAD_PLATFORM_WIN32
	return (GLADapiproc)GetProcAddress((HMODULE)handle, name);
#else
	return GLAD_GNUC_EXTENSION(GLADapiproc) dlsym(handle, name);
#endif
}

#endif /* GLAD_LOADER_LIBRARY_C_ */

struct _glad_egl_userptr
{
	void *handle;
	PFNEGLGETPROCADDRESSPROC get_proc_address_ptr;
};

static GLADapiproc glad_egl_get_proc(const char *name, void *vuserptr)
{
	struct _glad_egl_userptr userptr = *(struct _glad_egl_userptr *)vuserptr;
	GLADapiproc result = NULL;

	result = glad_dlsym_handle(userptr.handle, name);
	if (result == NULL)
	{
		result = GLAD_GNUC_EXTENSION(GLADapiproc) userptr.get_proc_address_ptr(name);
	}

	return result;
}

static void *_egl_handle = NULL;

int gladLoaderLoadEGL(EGLDisplay display)
{
#ifdef __APPLE__
	static const char *NAMES[] = {"libEGL.dylib"};
#elif GLAD_PLATFORM_WIN32
	static const char *NAMES[] = {"libEGL.dll", "EGL.dll"};
#else
	static const char *NAMES[] = {"libEGL.so.1", "libEGL.so"};
#endif

	int version = 0;
	int did_load = 0;
	struct _glad_egl_userptr userptr;

	if (_egl_handle == NULL)
	{
		_egl_handle = glad_get_dlopen_handle(NAMES, sizeof(NAMES) / sizeof(NAMES[0]));
		did_load = _egl_handle != NULL;
	}

	if (_egl_handle != NULL)
	{
		userptr.handle = _egl_handle;
		userptr.get_proc_address_ptr = (PFNEGLGETPROCADDRESSPROC)glad_dlsym_handle(_egl_handle, "eglGetProcAddress");
		if (userptr.get_proc_address_ptr != NULL)
		{
			version = gladLoadEGLUserPtr(display, glad_egl_get_proc, &userptr);
		}

		if (!version && did_load)
		{
			glad_close_dlopen_handle(_egl_handle);
			_egl_handle = NULL;
		}
	}

	return version;
}

void gladLoaderUnloadEGL()
{
	if (_egl_handle != NULL)
	{
		glad_close_dlopen_handle(_egl_handle);
		_egl_handle = NULL;
	}
}

#endif /* GLAD_EGL */
