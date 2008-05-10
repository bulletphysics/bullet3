/*
 * libxml.h: internal header only used during the compilation of libxml
 *
 * See COPYRIGHT for the status of this software
 *
 * Author: breese@users.sourceforge.net
 */

#ifndef __XML_LIBXML_H__
#define __XML_LIBXML_H__

#ifndef NO_LARGEFILE_SOURCE
#ifndef _LARGEFILE_SOURCE
#define _LARGEFILE_SOURCE
#endif
#ifndef _FILE_OFFSET_BITS
#define _FILE_OFFSET_BITS 64
#endif
#endif

#if defined(macintosh)
#include "config-mac.h"
#else
#ifdef _MSC_VER
	#pragma warning(disable : 4324) // disable padding warning
	#pragma warning(disable:4530) // Disable the exception disable but used in MSCV Stl warning.
	#pragma warning(disable:4996) //Turn off warnings about deprecated C routines
	#pragma warning(disable:4786) // Disable the "debug name too long" warning
	#pragma warning (disable:4244) // possible loss of data
	#pragma warning (disable:4267) // possible loss of data
	#pragma warning (disable:4311) // type cast' : pointer truncation
	#pragma warning (disable:4312) // type cast' : pointer truncation
	#pragma warning (disable:4005) // macro redefinition
	#pragma warning (disable:4101) // unreferenced local variable








#include "config-win32.h"
#else
#include "config.h"
#include <libxml/xmlversion.h>
#endif //_MSVC
#endif

#if defined(__Lynx__)
#include <stdio.h> /* pull definition of size_t */
#include <varargs.h>
int snprintf(char *, size_t, const char *, ...);
int vfprintf(FILE *, const char *, va_list);
#endif

#ifndef WITH_TRIO
#include <stdio.h>
#else
/**
 * TRIO_REPLACE_STDIO:
 *
 * This macro is defined if teh trio string formatting functions are to
 * be used instead of the default stdio ones.
 */
#define TRIO_REPLACE_STDIO
#include "trio.h"
#endif

/*
 * Internal variable indicating if a callback has been registered for
 * node creation/destruction. It avoids spending a lot of time in locking
 * function while checking if the callback exists.
 */
extern int __xmlRegisterCallbacks;
/*
 * internal error reporting routines, shared but not partof the API.
 */
void __xmlIOErr(int domain, int code, const char *extra);
void __xmlLoaderErr(void *ctx, const char *msg, const char *filename);
#ifdef LIBXML_HTML_ENABLED
/*
 * internal function of HTML parser needed for xmlParseInNodeContext
 * but not part of the API
 */
void __htmlParseContent(void *ctx);
#endif


#ifdef IN_LIBXML
#ifdef __GNUC__
#ifdef PIC
#ifdef linux
#if (__GNUC__ == 3 && __GNUC_MINOR__ >= 3) || (__GNUC__ > 3)
#include "elfgcchack.h"
#endif
#endif
#endif
#endif
#endif
#endif /* ! __XML_LIBXML_H__ */
