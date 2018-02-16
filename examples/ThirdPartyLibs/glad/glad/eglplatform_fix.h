#ifndef __eglplatform_fix_h_
#define __eglplatform_fix_h_

/* C++ / C typecast macros for special EGL handle values */
#if defined(__cplusplus)
#define EGL_CAST(type, value) (static_cast<type>(value))
#else
#define EGL_CAST(type, value) ((type) (value))
#endif

#endif /* __eglplatform_fix_h */
