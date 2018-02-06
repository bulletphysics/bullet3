#ifndef OPENVR_SAMPLES_SHARED_COMPAT_H_
#define OPENVR_SAMPLES_SHARED_COMPAT_H_

#include <cassert>
#include <cstdio>
#include <cstring>

// Handle non standard code.
#ifndef _WIN32

#include <unistd.h>
#include <cstdbool>

#define sprintf_s snprintf
#define vsprintf_s sprintf
#define _stricmp strcmp
#define stricmp strcmp
#define strnicmp strncasecmp
#define strcpy_s(dst, n, src) int(strncpy(dst, src, n) != nullptr)
#define fopen_s(fd, path, mode) int((*fd = fopen(path, mode)) != nullptr)
#define _vsnprintf_s(buffer, size, fmt, ap) vsnprintf(buffer, size, fmt, ap)
#define OutputDebugStringA(x) fprintf(stderr, "%s\n", x)

typedef int errno_t;

#endif  // _WIN32

#endif  // OPENVR_SAMPLES_SHARED_COMPAT_H_
