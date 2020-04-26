/* ioapi.h -- IO base function header for compress/uncompress .zip
   part of the MiniZip project - ( http://www.winimage.com/zLibDll/minizip.html )

         Copyright (C) 1998-2010 Gilles Vollant (minizip) ( http://www.winimage.com/zLibDll/minizip.html )

         Modifications for Zip64 support
         Copyright (C) 2009-2010 Mathias Svensson ( http://result42.com )

         For more info read MiniZip_info.txt

*/

#if (defined(_WIN32))
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "ioapi.h"

voidpf call_zopen64(const zlib_filefunc64_32_def* pfilefunc, const void* filename, int mode)
{
	if (pfilefunc->zfile_func64.zopen64_file != NULL)
		return (*(pfilefunc->zfile_func64.zopen64_file))(pfilefunc->zfile_func64.opaque, filename, mode);
	else
	{
		return (*(pfilefunc->zopen32_file))(pfilefunc->zfile_func64.opaque, (const char*)filename, mode);
	}
}

long call_zseek64(const zlib_filefunc64_32_def* pfilefunc, voidpf filestream, ZPOS64_T offset, int origin)
{
	if (pfilefunc->zfile_func64.zseek64_file != NULL)
		return (*(pfilefunc->zfile_func64.zseek64_file))(pfilefunc->zfile_func64.opaque, filestream, offset, origin);
	else
	{
		uLong offsetTruncated = (uLong)offset;
		if (offsetTruncated != offset)
			return -1;
		else
			return (*(pfilefunc->zseek32_file))(pfilefunc->zfile_func64.opaque, filestream, offsetTruncated, origin);
	}
}

ZPOS64_T call_ztell64(const zlib_filefunc64_32_def* pfilefunc, voidpf filestream)
{
	if (pfilefunc->zfile_func64.zseek64_file != NULL)
		return (*(pfilefunc->zfile_func64.ztell64_file))(pfilefunc->zfile_func64.opaque, filestream);
	else
	{
		uLong tell_uLong = (*(pfilefunc->ztell32_file))(pfilefunc->zfile_func64.opaque, filestream);
		if ((tell_uLong) == ((uLong)-1))
			return (ZPOS64_T)-1;
		else
			return tell_uLong;
	}
}

void fill_zlib_filefunc64_32_def_from_filefunc32(zlib_filefunc64_32_def* p_filefunc64_32, const zlib_filefunc_def* p_filefunc32)
{
	p_filefunc64_32->zfile_func64.zopen64_file = NULL;
	p_filefunc64_32->zopen32_file = p_filefunc32->zopen_file;
	p_filefunc64_32->zfile_func64.zerror_file = p_filefunc32->zerror_file;
	p_filefunc64_32->zfile_func64.zread_file = p_filefunc32->zread_file;
	p_filefunc64_32->zfile_func64.zwrite_file = p_filefunc32->zwrite_file;
	p_filefunc64_32->zfile_func64.ztell64_file = NULL;
	p_filefunc64_32->zfile_func64.zseek64_file = NULL;
	p_filefunc64_32->zfile_func64.zclose_file = p_filefunc32->zclose_file;
	p_filefunc64_32->zfile_func64.zerror_file = p_filefunc32->zerror_file;
	p_filefunc64_32->zfile_func64.opaque = p_filefunc32->opaque;
	p_filefunc64_32->zseek32_file = p_filefunc32->zseek_file;
	p_filefunc64_32->ztell32_file = p_filefunc32->ztell_file;
}

static voidpf ZCALLBACK fopen_file_func OF((voidpf opaque, const char* filename, int mode));
static uLong ZCALLBACK fread_file_func OF((voidpf opaque, voidpf stream, void* buf, uLong size));
static uLong ZCALLBACK fwrite_file_func OF((voidpf opaque, voidpf stream, const void* buf, uLong size));
static ZPOS64_T ZCALLBACK ftell64_file_func OF((voidpf opaque, voidpf stream));
static long ZCALLBACK fseek64_file_func OF((voidpf opaque, voidpf stream, ZPOS64_T offset, int origin));
static int ZCALLBACK fclose_file_func OF((voidpf opaque, voidpf stream));
static int ZCALLBACK ferror_file_func OF((voidpf opaque, voidpf stream));

static voidpf ZCALLBACK fopen_file_func(voidpf opaque, const char* filename, int mode)
{
	FILE* file = NULL;
	const char* mode_fopen = NULL;
	if ((mode & ZLIB_FILEFUNC_MODE_READWRITEFILTER) == ZLIB_FILEFUNC_MODE_READ)
		mode_fopen = "rb";
	else if (mode & ZLIB_FILEFUNC_MODE_EXISTING)
		mode_fopen = "r+b";
	else if (mode & ZLIB_FILEFUNC_MODE_CREATE)
		mode_fopen = "wb";

	if ((filename != NULL) && (mode_fopen != NULL))
		file = fopen(filename, mode_fopen);
	return file;
}

static voidpf ZCALLBACK fopen64_file_func(voidpf opaque, const void* filename, int mode)
{
	FILE* file = NULL;
	const char* mode_fopen = NULL;
	if ((mode & ZLIB_FILEFUNC_MODE_READWRITEFILTER) == ZLIB_FILEFUNC_MODE_READ)
		mode_fopen = "rb";
	else if (mode & ZLIB_FILEFUNC_MODE_EXISTING)
		mode_fopen = "r+b";
	else if (mode & ZLIB_FILEFUNC_MODE_CREATE)
		mode_fopen = "wb";

	if ((filename != NULL) && (mode_fopen != NULL))
		//       file = fopen64((const char*)filename, mode_fopen);
		file = fopen((const char*)filename, mode_fopen);

	return file;
}

static uLong ZCALLBACK fread_file_func(voidpf opaque, voidpf stream, void* buf, uLong size)
{
	uLong ret;
	ret = (uLong)fread(buf, 1, (size_t)size, (FILE*)stream);
	return ret;
}

static uLong ZCALLBACK fwrite_file_func(voidpf opaque, voidpf stream, const void* buf, uLong size)
{
	uLong ret;
	ret = (uLong)fwrite(buf, 1, (size_t)size, (FILE*)stream);
	return ret;
}

static long ZCALLBACK ftell_file_func(voidpf opaque, voidpf stream)
{
	long ret;
	ret = ftell((FILE*)stream);
	return ret;
}

static ZPOS64_T ZCALLBACK ftell64_file_func(voidpf opaque, voidpf stream)
{
	ZPOS64_T ret;
	ret = ftell((FILE*)stream);

	//    ret = ftello64((FILE *)stream);
	return ret;
}

static long ZCALLBACK fseek_file_func(voidpf opaque, voidpf stream, uLong offset, int origin)
{
	int fseek_origin = 0;
	long ret;
	switch (origin)
	{
		case ZLIB_FILEFUNC_SEEK_CUR:
			fseek_origin = SEEK_CUR;
			break;
		case ZLIB_FILEFUNC_SEEK_END:
			fseek_origin = SEEK_END;
			break;
		case ZLIB_FILEFUNC_SEEK_SET:
			fseek_origin = SEEK_SET;
			break;
		default:
			return -1;
	}
	ret = 0;
	if (fseek((FILE*)stream, offset, fseek_origin) != 0)
		ret = -1;
	return ret;
}

static long ZCALLBACK fseek64_file_func(voidpf opaque, voidpf stream, ZPOS64_T offset, int origin)
{
	int fseek_origin = 0;
	long ret;
	switch (origin)
	{
		case ZLIB_FILEFUNC_SEEK_CUR:
			fseek_origin = SEEK_CUR;
			break;
		case ZLIB_FILEFUNC_SEEK_END:
			fseek_origin = SEEK_END;
			break;
		case ZLIB_FILEFUNC_SEEK_SET:
			fseek_origin = SEEK_SET;
			break;
		default:
			return -1;
	}
	ret = 0;

	if (fseek((FILE*)stream, offset, fseek_origin) != 0)

		//    if(fseeko64((FILE *)stream, offset, fseek_origin) != 0)
		ret = -1;

	return ret;
}

static int ZCALLBACK fclose_file_func(voidpf opaque, voidpf stream)
{
	int ret;
	ret = fclose((FILE*)stream);
	return ret;
}

static int ZCALLBACK ferror_file_func(voidpf opaque, voidpf stream)
{
	int ret;
	ret = ferror((FILE*)stream);
	return ret;
}

void fill_fopen_filefunc(pzlib_filefunc_def)
	zlib_filefunc_def* pzlib_filefunc_def;
{
	pzlib_filefunc_def->zopen_file = fopen_file_func;
	pzlib_filefunc_def->zread_file = fread_file_func;
	pzlib_filefunc_def->zwrite_file = fwrite_file_func;
	pzlib_filefunc_def->ztell_file = ftell_file_func;
	pzlib_filefunc_def->zseek_file = fseek_file_func;
	pzlib_filefunc_def->zclose_file = fclose_file_func;
	pzlib_filefunc_def->zerror_file = ferror_file_func;
	pzlib_filefunc_def->opaque = NULL;
}

void fill_fopen64_filefunc(zlib_filefunc64_def* pzlib_filefunc_def)
{
	pzlib_filefunc_def->zopen64_file = fopen64_file_func;
	pzlib_filefunc_def->zread_file = fread_file_func;
	pzlib_filefunc_def->zwrite_file = fwrite_file_func;
	pzlib_filefunc_def->ztell64_file = ftell64_file_func;
	pzlib_filefunc_def->zseek64_file = fseek64_file_func;
	pzlib_filefunc_def->zclose_file = fclose_file_func;
	pzlib_filefunc_def->zerror_file = ferror_file_func;
	pzlib_filefunc_def->opaque = NULL;
}

/* This file is from the proposed iomem_simple package found at
   http://code.trak.dk/
   Local modifications exist to fix various security bugs. See this
   file's revision history.
*/

/* ioapi_mem2.c -- IO base function header for compress/uncompress .zip
   files using zlib + zip or unzip API

   This version of ioapi is designed to access memory rather than files.
   We do use a region of memory to put data in to and take it out of. We do
   not have auto-extending buffers and do not inform anyone else that the
   data has been written. It is really intended for accessing a zip archive
   embedded in an application such that I can write an installer with no
   external files. Creation of archives has not been attempted, although
   parts of the framework are present.

   Based on Unzip ioapi.c version 0.22, May 19th, 2003

   Copyright (C) 1998-2003 Gilles Vollant
             (C) 2003 Justin Fletcher

   Dynamically allocated memory version. Troels K 2004
      mem_close deletes the data: file is single-session. No filenames.

   This file is under the same license as the Unzip tool it is distributed
   with.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "unzip.h"


#ifndef ZOFF_T
#define ZOFF_T uLong /* bw compability is default */
#endif
#ifndef ZPOS_T
#define ZPOS_T long /* bw compability is default */
#endif

#if defined(_INC_WINDOWS) || defined(_WINDOWS_H)
#define _BOOL_DEFINED
#endif

#ifndef _BOOL_DEFINED
#define _BOOL_DEFINED
typedef signed  int BOOL;
#endif
#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE  1
#endif

static int fseek_calc(ZPOS_T offset, int origin, ZPOS_T* position, ZPOS_T size)
{
    BOOL bOK = TRUE;
    switch (origin)
    {
    case SEEK_SET:
        //bOK = (offset >= 0) && (offset <= size);
        if (bOK) *position = offset;
        break;
    case SEEK_CUR:
        bOK = ((offset + *position) >= 0) && (((offset + *position) <= size));
        if (bOK) *position = offset + *position;
        break;
    case SEEK_END:
        bOK = ((size - offset) >= 0) && (((size - offset) <= size));
        if (bOK) *position = offset + size - 0;
        break;
    default:
        bOK = FALSE;
        break;
    }
    return bOK ? 0 : -1;
}

static voidpf ZCALLBACK mem_open OF((
    voidpf opaque,
    const char* filename,
    int mode));

static uLong ZCALLBACK mem_read OF((
    voidpf opaque,
    voidpf stream,
    void* buf,
    uLong size));

static uLong ZCALLBACK mem_write OF((
    voidpf opaque,
    voidpf stream,
    const void* buf,
    uLong size));

static ZPOS_T ZCALLBACK mem_tell OF((
    voidpf opaque,
    voidpf stream));

static long ZCALLBACK mem_seek OF((
    voidpf opaque,
    voidpf stream,
    ZOFF_T offset,
    int origin));

static int ZCALLBACK mem_close OF((
    voidpf opaque,
    voidpf stream));

static int ZCALLBACK mem_error OF((
    voidpf opaque,
    voidpf stream));

typedef struct _MEMFILE
{
    void* buffer;    /* Base of the region of memory we're using */
    ZPOS_T length;   /* Size of the region of memory we're using */
    ZPOS_T position; /* Current offset in the area */
} MEMFILE;

static uLong ZCALLBACK mem_read (opaque, stream, buf, size)
voidpf opaque;
voidpf stream;
void* buf;
uLong size;
{
    MEMFILE* handle = (MEMFILE*)stream;
    /* It's possible for this function to be called with an invalid position.
     * Additionally, unzip.h minizip uses an unsigned long for the
     * uncompressed size field, but everwhere else uses a signed long. For
     * safety, we check here that the handle position is not more than the max
     * size of a 32-bit signed int.
     */
    if (handle->position < 0 || handle->position >= 2147483647)
    {
        return 0;
    }

    if ((handle->position + size) > handle->length)
    {
        /* There is a bug in this original code. It's possible for the position
         * to exceed the size, which results in memcpy being handed a negative
         * size. See libkml's src/kml/base/zip_file_test.cc for some overflow
         * tests that exercise this.
         * size = handle->length - handle->position;
        */
        int size_ = handle->length - handle->position;
        size = (size_ < 0) ? 0 : (uLong)size_;
    }

    memcpy(buf, ((char*)handle->buffer) + handle->position, size);
    handle->position += size;

    return size;
}

static uLong ZCALLBACK mem_write (opaque, stream, buf, size)
voidpf opaque;
voidpf stream;
const void* buf;
uLong size;
{
    MEMFILE* handle = (MEMFILE*)stream;

    if ((handle->position + size) > handle->length)
    {
        handle->length = handle->position + size;
        handle->buffer = realloc(handle->buffer, handle->length);
    }

    memcpy(((char*)handle->buffer) + handle->position, buf, size);
    handle->position += size;

    return size;
}

static ZPOS_T ZCALLBACK mem_tell (opaque, stream)
voidpf opaque;
voidpf stream;
{
    MEMFILE* handle = (MEMFILE*)stream;
    return handle->position;
}

static long ZCALLBACK mem_seek (opaque, stream, offset, origin)
voidpf opaque;
voidpf stream;
ZOFF_T offset;
int origin;
{
    MEMFILE* handle = (MEMFILE*)stream;
    return fseek_calc(offset, origin, &handle->position, handle->length);
}

int ZCALLBACK mem_close (opaque, stream)
voidpf opaque;
voidpf stream;
{
    MEMFILE* handle = (MEMFILE*)stream;

    /* Note that once we've written to the buffer we don't tell anyone
       about it here. Probably the opaque handle could be used to inform
       some other component of how much data was written.

       This, and other aspects of writing through this interface, has
       not been tested.
     */

    free (handle);
    return 0;
}

int ZCALLBACK mem_error (opaque, stream)
voidpf opaque;
voidpf stream;
{
    /* MEMFILE *handle = (MEMFILE *)stream; */
    /* We never return errors */
    return 0;
}

ZEXTERN void* ZEXPORT mem_simple_create_file(zlib_filefunc_def* api, void* buffer, size_t buf_len)
{
    MEMFILE* handle = malloc(sizeof(*handle));
    api->zopen_file = NULL;
    api->zread_file = mem_read;
    api->zwrite_file = mem_write;
    api->ztell_file = mem_tell;
    api->zseek_file = mem_seek;
    api->zclose_file = mem_close;
    api->zerror_file = mem_error;
    api->opaque = handle;
    handle->position = 0;
    handle->buffer = buffer;
    handle->length = buf_len;
    return handle;
}
ZEXTERN void ZEXPORT mem_simple_destroy_file(void* handle_ptr)
{
    MEMFILE* handle = (MEMFILE*)handle_ptr;
    free (handle);
}