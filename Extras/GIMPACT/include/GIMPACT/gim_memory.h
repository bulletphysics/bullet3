#ifndef GIM_MEMORY_H_INCLUDED
#define GIM_MEMORY_H_INCLUDED
/*! \file gim_memory.h
\author Francisco León Nájera
*/
/*
-----------------------------------------------------------------------------
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2006 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com

 This library is free software; you can redistribute it and/or
 modify it under the terms of EITHER:
   (1) The GNU Lesser General Public License as published by the Free
       Software Foundation; either version 2.1 of the License, or (at
       your option) any later version. The text of the GNU Lesser
       General Public License is included with this library in the
       file GIMPACT-LICENSE-LGPL.TXT.
   (2) The BSD-style license that is included with this library in
       the file GIMPACT-LICENSE-BSD.TXT.
   (3) The zlib/libpng license that is included with this library in
       the file GIMPACT-LICENSE-ZLIB.TXT.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
 GIMPACT-LICENSE-LGPL.TXT, GIMPACT-LICENSE-ZLIB.TXT and GIMPACT-LICENSE-BSD.TXT for more details.

-----------------------------------------------------------------------------
*/


#include "GIMPACT/gim_math.h"
#include <memory.h>

//#define PREFETCH 1
//! \defgroup PREFETCH
//! @{
#ifdef PREFETCH
#include <xmmintrin.h>	// for prefetch
#define pfval	64
#define pfval2	128
//! Prefetch 64
#define pf(_x,_i)	_mm_prefetch((void *)(_x + _i + pfval), 0)
//! Prefetch 128
#define pf2(_x,_i)	_mm_prefetch((void *)(_x + _i + pfval2), 0)
#else
//! Prefetch 64
#define pf(_x,_i)
//! Prefetch 128
#define pf2(_x,_i)
#endif
//! @}

/*! \defgroup ARRAY_UTILITIES
\brief
Functions for manip packed arrays of numbers
*/
//! @{
#define GIM_COPY_ARRAYS(dest_array,source_array,element_count)\
{\
    GUINT _i_;\
    for (_i_=0;_i_<element_count ;_i_++)\
    {\
    	dest_array[_i_] = source_array[_i_];\
    }\
}\

#define GIM_COPY_ARRAYS_1(dest_array,source_array,element_count,copy_macro)\
{\
    GUINT _i_;\
    for (_i_=0;_i_<element_count ;_i_++)\
    {\
    	copy_macro(dest_array[_i_],source_array[_i_]);\
    }\
}\


#define GIM_ZERO_ARRAY(array,element_count)\
{\
    GUINT _i_;\
    for (_i_=0;_i_<element_count ;_i_++)\
    {\
    	array[_i_] = 0;\
    }\
}\

#define GIM_CONSTANT_ARRAY(array,element_count,constant)\
{\
    GUINT _i_;\
    for (_i_=0;_i_<element_count ;_i_++)\
    {\
    	array[_i_] = constant;\
    }\
}\
//! @}

/*! \defgroup MEMORY_FUNCTION_PROTOTYPES
Function prototypes to allocate and free memory.
*/
//! @{
typedef void * gim_alloc_function (size_t size);
typedef void * gim_alloca_function (size_t size);//Allocs on the heap
typedef void * gim_realloc_function (void *ptr, size_t oldsize, size_t newsize);
typedef void gim_free_function (void *ptr, size_t size);
//! @}

/*! \defgroup MEMORY_FUNCTION_HANDLERS
\brief
Memory Function Handlers
 set new memory management functions. if fn is 0, the default handlers are
  used. */
//! @{
void gim_set_alloc_handler (gim_alloc_function *fn);
void gim_set_alloca_handler (gim_alloca_function *fn);
void gim_set_realloc_handler (gim_realloc_function *fn);
void gim_set_free_handler (gim_free_function *fn);
//! @}

/*! \defgroup MEMORY_FUNCTION_GET_HANDLERS
\brief
get current memory management functions.
*/
//! @{
gim_alloc_function *gim_get_alloc_handler (void);
#ifdef ALLOCA_GIMPACT
gim_alloca_function *gim_get_alloca_handler(void);
#endif //
gim_realloc_function *gim_get_realloc_handler (void);
gim_free_function  *gim_get_free_handler (void);
//! @}

/*! \defgroup MEMORY_FUNCTIONS
Standar Memory functions
*/
//! @{
void * gim_alloc(size_t size);
void * gim_alloca(size_t size);
void * gim_realloc(void *ptr, size_t oldsize, size_t newsize);
void gim_free(void *ptr, size_t size);
//! @}

/*! \defgroup DYNAMIC_ARRAYS
\brief
Dynamic Arrays. Allocated from system memory.
<ul>
<li> For initializes a dynamic array, use GIM_DYNARRAY_CREATE or GIM_DYNARRAY_CREATE_SIZED.
<li> When an array is no longer used, must be terminated with the macro GIM_DYNARRAY_DESTROY.
</ul>
*/
//! @{
#define G_ARRAY_GROW_SIZE 100

//! Dynamic array handle.
struct GDYNAMIC_ARRAY
{
    char * m_pdata;
    GUINT m_size;
    GUINT m_reserve_size;
};
//typedef  struct _GDYNAMIC_ARRAY GDYNAMIC_ARRAY;

#define GIM_DYNARRAY_IS_VALID(array_data) (array_data.m_pdata != 0)

#define GIM_DYNARRAY_INVALIDATE(array_data) array_data.m_pdata = 0;

//! Creates a dynamic array zero sized
#define GIM_DYNARRAY_CREATE(type,array_data,reserve_size) \
{ \
    array_data.m_pdata = (char *)gim_alloc(reserve_size*sizeof(type)); \
    array_data.m_size = 0; \
    array_data.m_reserve_size = reserve_size; \
}\

//! Creates a dynamic array with n = size elements
#define GIM_DYNARRAY_CREATE_SIZED(type,array_data,size) \
{ \
    array_data.m_pdata = (char *)gim_alloc(size*sizeof(type)); \
    array_data.m_size = size; \
    array_data.m_reserve_size = size; \
}\

//! Reserves memory for a dynamic array.
#define GIM_DYNARRAY_RESERVE_SIZE(type,array_data,reserve_size) \
{ \
    if(reserve_size>array_data.m_reserve_size )\
    { \
        array_data.m_pdata = (char *) gim_realloc(array_data.m_pdata,array_data.m_size*sizeof(type),reserve_size*sizeof(type));\
        array_data.m_reserve_size = reserve_size; \
    }\
}\

//! Set the size of the array
#define GIM_DYNARRAY_SET_SIZE(type,array_data,size) \
{ \
    GIM_DYNARRAY_RESERVE_SIZE(type,array_data,size);\
    array_data.m_size = size;\
}\

//! Gets a pointer from the beginning of the array
#define GIM_DYNARRAY_POINTER(type,array_data) (type *)(array_data.m_pdata)

//! Gets a pointer from the last elemento of the array
#define GIM_DYNARRAY_POINTER_LAST(type,array_data) (((type *)array_data.m_pdata)+array_data.m_size-1)

//! Inserts an element at the last position
#define GIM_DYNARRAY_PUSH_ITEM(type,array_data,item)\
{\
    if(array_data.m_reserve_size<=array_data.m_size)\
    {\
        GIM_DYNARRAY_RESERVE_SIZE(type,array_data,(array_data.m_size+G_ARRAY_GROW_SIZE));\
    }\
    type * _pt = GIM_DYNARRAY_POINTER(type,array_data);\
    memcpy(&_pt[array_data.m_size],&item,sizeof(type));\
    array_data.m_size++;    \
}\

//! Inserts an element at the last position
#define GIM_DYNARRAY_PUSH_EMPTY(type,array_data)\
{\
    if(array_data.m_reserve_size<=array_data.m_size)\
    {\
        GIM_DYNARRAY_RESERVE_SIZE(type,array_data,(array_data.m_size+G_ARRAY_GROW_SIZE));\
    }\
    array_data.m_size++;    \
}\

//! Inserts an element
#define GIM_DYNARRAY_INSERT_ITEM(type,array_data,item,index) \
{ \
    if(array_data.m_reserve_size<=array_data.m_size)\
    {\
        GIM_DYNARRAY_RESERVE_SIZE(type,array_data,(array_data.m_size+G_ARRAY_GROW_SIZE));\
    }\
    type * _pt = GIM_DYNARRAY_POINTER(type,array_data);\
    if(index<array_data.m_size-1) \
    { \
        memcpy(&_pt[index+1],&_pt[index],(array_data.m_size-index)*sizeof(type));\
    } \
    memcpy(&_pt[index],&item,sizeof(type));\
    array_data.m_size++;    \
}\

//! Removes an element
#define GIM_DYNARRAY_DELETE_ITEM(type,array_data,index) \
{ \
    if(index<array_data.m_size-1) \
    { \
        type * _pt = GIM_DYNARRAY_POINTER(type,array_data);\
        memcpy(&_pt[index],&_pt[index+1],(array_data.m_size-index-1)*sizeof(type));\
    } \
    array_data.m_size--;    \
}\

//! Removes an element at the last position
#define GIM_DYNARRAY_POP_ITEM(array_data) \
{ \
    if(array_data.m_size>0) \
    { \
        array_data.m_size--;    \
    } \
}\

//! Destroys the array
void GIM_DYNARRAY_DESTROY(GDYNAMIC_ARRAY & array_data);
//! @}

/*! \defgroup BITSET
\brief
Bitsets , based on \ref DYNAMIC_ARRAYS .
<ul>
<li> For initializes a bitset array, use \ref GIM_BITSET_CREATE or \ref GIM_BITSET_CREATE_SIZED.
<li> When the bitset is no longer used, must be terminated with the macro \ref GIM_DYNARRAY_DESTROY.
<li> For putting a mark on the bitset, call \ref GIM_BITSET_SET
<li> For clearing a mark on the bitset, call \ref GIM_BITSET_CLEAR
<li> For retrieving a bit value from a bitset, call \ref GIM_BITSET_GET-
</ul>
*/
//! @{

//! Creates a bitset
#define GIM_BITSET_CREATE(array_data) GIM_DYNARRAY_CREATE(GUINT,array_data,G_ARRAY_GROW_SIZE)

//! Creates a bitset, with their bits set to 0.
#define GIM_BITSET_CREATE_SIZED(array_data,bits_count)\
{\
    array_data.m_size = bits_count/GUINT_BIT_COUNT + 1;\
    GIM_DYNARRAY_CREATE(GUINT,array_data,array_data.m_size);\
    GUINT * _pt = GIM_DYNARRAY_POINTER(GUINT,array_data);\
    memset(_pt,0,sizeof(GUINT)*(array_data.m_size));\
}\

//! Gets the bitset bit count.
#define GIM_BITSET_SIZE(array_data) (array_data.m_size*GUINT_BIT_COUNT)

//! Resizes a bitset, with their bits set to 0.
#define GIM_BITSET_RESIZE(array_data,new_bits_count)\
{    \
    GUINT _oldsize = array_data.m_size;\
    array_data.m_size = new_bits_count/GUINT_BIT_COUNT + 1;    \
    if(_oldsize<array_data.m_size)\
    {\
        if(array_data.m_size > array_data.m_reserve_size)\
        {\
            GIM_DYNARRAY_RESERVE_SIZE(GUINT,array_data,array_data.m_size+G_ARRAY_GROW_SIZE);\
        }\
        GUINT * _pt = GIM_DYNARRAY_POINTER(GUINT,array_data);\
        memset(&_pt[_oldsize],0,sizeof(GUINT)*(array_data.m_size-_oldsize));\
    }\
}\

//! Sets all bitset bit to 0.
#define GIM_BITSET_CLEAR_ALL(array_data)\
{\
    memset(array_data.m_pdata,0,sizeof(GUINT)*array_data.m_size);\
}\

//! Sets all bitset bit to 1.
#define GIM_BITSET_SET_ALL(array_data)\
{\
    memset(array_data.m_pdata,0xFF,sizeof(GUINT)*array_data.m_size);\
}\

///Sets the desired bit to 1
#define GIM_BITSET_SET(array_data,bit_index)\
{\
    if(bit_index>=GIM_BITSET_SIZE(array_data))\
	{\
	    GIM_BITSET_RESIZE(array_data,bit_index);\
	}\
	GUINT * _pt = GIM_DYNARRAY_POINTER(GUINT,array_data);\
	_pt[bit_index >> GUINT_EXPONENT] |= (1 << (bit_index & (GUINT_BIT_COUNT-1)));\
}\

///Return 0 or 1
#define GIM_BITSET_GET(array_data,bit_index,get_value) \
{\
    if(bit_index>=GIM_BITSET_SIZE(array_data))\
	{\
	    get_value = 0;\
	}\
	else\
	{\
        GUINT * _pt = GIM_DYNARRAY_POINTER(GUINT,array_data);\
        get_value = _pt[bit_index >> GUINT_EXPONENT] & (1 << (bit_index & (GUINT_BIT_COUNT-1)));\
	}\
}\

///Sets the desired bit to 0
#define GIM_BITSET_CLEAR(array_data,bit_index) \
{\
    if(bit_index<GIM_BITSET_SIZE(array_data))\
	{\
        GUINT * _pt = GIM_DYNARRAY_POINTER(GUINT,array_data);\
        _pt[bit_index >> GUINT_EXPONENT] &= ~(1 << (bit_index & (GUINT_BIT_COUNT-1)));\
	}\
}\
//! @}

/*! \defgroup MEMORY_ACCESS_CONSTANTS
\brief
Memory Access constants.
\sa BUFFERS
*/
//! @{
#define G_MA_READ_ONLY 1
#define G_MA_WRITE_ONLY 2
#define G_MA_READ_WRITE 3
//! @}

/*! \defgroup MEMORY_USAGE_CONSTANTS
\brief
Memory usage constants.
\sa BUFFERS
*/
//! @{
/// Don't care how memory is used
#define G_MU_EITHER 0
/// specified once, doesn't allow read information
#define G_MU_STATIC_WRITE 1
/// specified once, allows to read information from a shadow buffer
#define G_MU_STATIC_READ 2
/// write directly on buffer, allows to read information from a shadow buffer
#define G_MU_STATIC_READ_DYNAMIC_WRITE 3
/// upload data to buffer from the shadow buffer, allows to read information from a shadow buffer
#define G_MU_STATIC_READ_DYNAMIC_WRITE_COPY 4
/// specified once, allows to read information directly from memory
#define G_MU_STATIC_WRITE_DYNAMIC_READ 5
/// write directly on buffer, allows to read information directly from memory
#define G_MU_DYNAMIC_READ_WRITE 6
//! @}

/*! \defgroup BUFFER_ERRORS
\brief
Buffer operation errors
\sa BUFFERS
*/
//! @{
#define G_BUFFER_OP_SUCCESS 0
#define G_BUFFER_OP_INVALID 1
#define G_BUFFER_OP_STILLREFCOUNTED 2
//! @}

/*! \defgroup BUFFER_MANAGER_IDS
\brief
Buffer manager identifiers
\sa BUFFERS, BUFFER_MANAGERS
*/
//! @{
#define G_BUFFER_MANAGER_SYSTEM 0
#define G_BUFFER_MANAGER_SHARED 1
#define G_BUFFER_MANAGER_USER 2
//! @}

/*! \defgroup BUFFERS
\brief
Buffer operations and structs.
<ul>
<li> Before using buffers you must initializes GIMPACT buffer managers by calling \ref gimpact_init.
<li> For initializes a buffer, use  \ref gim_create_buffer, \ref gim_create_buffer_from_data , \ref gim_create_common_buffer, \ref gim_create_common_buffer_from_data or \ref gim_create_shared_buffer_from_data.
<li> For accessing to the buffer memory, you must call \ref gim_lock_buffer, and then \ref gim_unlock_buffer for finish the access.
<li> When a buffer is no longer needed, you must free it by calling \ref gim_buffer_free.
<li> You must call \ref gimpact_terminate when finish your application.
<li> For a safe manipulation of buffers, use \ref BUFFER_ARRAYS
</ul>
\sa BUFFER_MANAGERS, BUFFER_ARRAYS
*/
//! @{


#define GIM_IS_VALID_BUFFER_ID(buffer_handle) (buffer_handle.m_buffer_id!=G_UINT_INFINITY)

//! Buffer handle.
struct GBUFFER_ID
{
    GUINT m_buffer_id;
    GUINT m_buffer_manager_id;
};
//typedef  struct _GBUFFER_ID GBUFFER_ID;

//! Buffer internal data
struct GBUFFER_DATA
{
    GUINT m_buffer_handle;//!< if 0, buffer doesn't exists
    GUINT m_size;
    GUINT m_usage;
    GINT m_access;
    GUINT m_lock_count;
    char * m_mapped_pointer;
    GBUFFER_ID m_shadow_buffer;
    GUINT m_refcount;//! Reference counting for safe garbage collection
};
//typedef  struct _GBUFFER_DATA GBUFFER_DATA;
//! @}

/*! \defgroup BUFFERS_MANAGER_PROTOTYPES
\brief
Function prototypes to allocate and free memory for buffers
\sa BUFFER_MANAGERS, BUFFERS
*/
//! @{

//! Returns a Buffer handle
typedef GUINT gim_buffer_alloc_function(GUINT size,int usage);

//! Returns a Buffer handle, and copies the pdata to the buffer
typedef GUINT gim_buffer_alloc_data_function(const void * pdata,GUINT size,int usage);

//! Changes the size of the buffer preserving the content, and returns the new buffer id
typedef GUINT gim_buffer_realloc_function(GUINT buffer_handle,GUINT oldsize,int old_usage,GUINT newsize,int new_usage);

//! It changes the m_buffer_handle member to 0/0
typedef void gim_buffer_free_function(GUINT buffer_handle,GUINT size);

//! It maps the m_mapped_pointer. Returns a pointer
typedef char * gim_lock_buffer_function(GUINT buffer_handle,int access);

//! It sets the m_mapped_pointer to 0
typedef void gim_unlock_buffer_function(GUINT buffer_handle);

typedef void gim_download_from_buffer_function(
        GUINT source_buffer_handle,
		GUINT source_pos,
		void * destdata,
		GUINT copysize);

typedef void  gim_upload_to_buffer_function(
        GUINT dest_buffer_handle,
		GUINT dest_pos,
		void * sourcedata,
		GUINT copysize);

typedef void gim_copy_buffers_function(
		GUINT source_buffer_handle,
		GUINT source_pos,
		GUINT dest_buffer_handle,
		GUINT dest_pos,
		GUINT copysize);
//! @}


/*! \defgroup BUFFER_MANAGERS
\brief
Buffer Manager operations
*/
//! @{
//! Buffer manager prototype
struct GBUFFER_MANAGER_PROTOTYPE
{
    gim_buffer_alloc_function * alloc_fn;
    gim_buffer_alloc_data_function *alloc_data_fn;
    gim_buffer_realloc_function * realloc_fn;
    gim_buffer_free_function * free_fn;
    gim_lock_buffer_function * lock_buffer_fn;
    gim_unlock_buffer_function * unlock_buffer_fn;
    gim_download_from_buffer_function * download_from_buffer_fn;
    gim_upload_to_buffer_function * upload_to_buffer_fn;
    gim_copy_buffers_function * copy_buffers_fn;
};
//typedef  struct _GBUFFER_MANAGER_PROTOTYPE GBUFFER_MANAGER_PROTOTYPE;

//! Buffer manager
struct GBUFFER_MANAGER_DATA
{
    GDYNAMIC_ARRAY m_buffer_array;//!< Array of GBUFFER_DATA objects
    GDYNAMIC_ARRAY m_free_positions;//!< Array of GUINT elements. Free positions
    GBUFFER_MANAGER_PROTOTYPE m_prototype;//! Prototype of functions
    GUINT m_active; //!< 0 or 1
};
//typedef  struct _GBUFFER_MANAGER_DATA GBUFFER_MANAGER_DATA;

//! Adds a buffer Manager to the Memory Singleton
void gim_create_buffer_manager(GBUFFER_MANAGER_PROTOTYPE * prototype,GUINT buffer_manager_id);
//! Gets buffer manager
GUINT gim_get_buffer_manager_count();
//! Destroys a buffer manager
void gim_destroy_buffer_manager(GUINT buffer_manager_id);
void gim_get_buffer_manager_data(GUINT buffer_manager_id,GBUFFER_MANAGER_DATA ** pbm_data);
void gim_init_buffer_managers();
void gim_terminate_buffer_managers();

//! @}


/*! \addtogroup BUFFERS
*/
//! @{

//!Creates a buffer on the buffer manager specified by buffer_manager_id
/*!
\param buffer_manager_id
\param buffer_size
\param usage An usage constant. Use G_MU_DYNAMIC_READ_WRITE as default.
\param buffer_id a pointer for receive the new buffer id
\return An error code. 0 if success.
\post m_refcount = 0
*/
GUINT gim_create_buffer(
    GUINT buffer_manager_id,
    GUINT buffer_size,
    int usage,
    GBUFFER_ID * buffer_id);

//!Creates a buffer on the buffer manager specified by buffer_manager_id
/*!
\param buffer_manager_id
\param pdata Data for allocating
\param buffer_size Size of the data buffer
\param usage An usage constant. Use G_MU_DYNAMIC_READ_WRITE as default.
\param buffer_id a pointer for receive the new buffer id
\return An error code. 0 if success.
\post m_refcount = 0
*/
GUINT gim_create_buffer_from_data(
    GUINT buffer_manager_id,
    const void * pdata,
    GUINT buffer_size,
    int usage,
    GBUFFER_ID * buffer_id);

//!Allocates on the G_BUFFER_MANAGER_SYSTEM
GUINT gim_create_common_buffer(GUINT buffer_size, GBUFFER_ID * buffer_id);
//!Allocates on the G_BUFFER_MANAGER_SYSTEM, and copies the data
GUINT gim_create_common_buffer_from_data(
    const void * pdata, GUINT buffer_size, GBUFFER_ID * buffer_id);
//!Creates a buffer with shared data
GUINT gim_create_shared_buffer_from_data(
    const void * pdata, GUINT buffer_size, GBUFFER_ID * buffer_id);


//! Add reference counting to buffer.
GINT gim_buffer_add_ref(GBUFFER_ID * buffer_id);

//! Function for resize buffer, preserving the content
/*!
\param buffer_id
\param newsize
\return An error code. 0 if success.
\post If m_refcount>0 then it decrements it.
*/
GINT gim_buffer_realloc(GBUFFER_ID * buffer_id,GUINT newsize);

//! Eliminates the buffer.
/*!
If the buffer reference counting is <= 1 and is unlocked, then it eliminates the buffer.
*/
GINT gim_buffer_free(GBUFFER_ID * buffer_id);

//! Locks the buffer for memory access.
/*!
\param buffer_id Id from buffer.
\param access Must have the following values: G_MA_READ_ONLY,G_MA_WRITE_ONLY or G_MA_READ_WRITE.
\param map_pointer Dest Pointer of the memory address from buffer.
\post m_lock_count increases.
*/
GINT gim_lock_buffer(GBUFFER_ID * buffer_id,int access,char ** map_pointer);

//! Unlocks the buffer for memory access.
GINT gim_unlock_buffer(GBUFFER_ID * buffer_id);

//! Gets the buffer size in bytes
GINT gim_get_buffer_size(GBUFFER_ID * buffer_id,GUINT * buffer_size);

//! Determines if the buffer is locked
GINT gim_get_buffer_is_locked(GBUFFER_ID * buffer_id,GUINT * lock_count);

//! Copies the content of the buffer to a dest pointer
GINT gim_download_from_buffer(
        GBUFFER_ID * buffer_id,
		GUINT source_pos,
		void * destdata,
		GUINT copysize);

//! Copies the content of a memory pointer to the buffer
GINT  gim_upload_to_buffer(
		GBUFFER_ID * buffer_id,
		GUINT dest_pos,
		void * sourcedata,
		GUINT copysize);

//! Copies two buffers.
GINT  gim_copy_buffers(
		GBUFFER_ID * source_buffer_id,
		GUINT source_pos,
		GBUFFER_ID * dest_buffer_id,
		GUINT dest_pos,
		GUINT copysize);
//! @}


/*! \defgroup BUFFER_ARRAYS

\brief
Buffered Arrays, for manip elements on a buffer and treat it as an array.
<ul>
<li> Before using buffer arrays you must initializes GIMPACT buffer managers by calling gimpact_init.
<li> Before creating buffer arrays, you must create a buffer. see \ref BUFFERS.
<li> Create a buffer narray by calling \ref GIM_BUFFER_ARRAY_INIT_TYPE, \ref GIM_BUFFER_ARRAY_INIT_TYPE_OFFSET or \ref GIM_BUFFER_ARRAY_INIT_OFFSET_STRIDE.
<li> For accessing to the array elements, you must call \ref gim_buffer_array_lock, and then \ref gim_buffer_array_unlock for finish the access.
<li> When a buffer array is no longer needed, you must free it by calling \ref GIM_BUFFER_ARRAY_DESTROY.
</ul>
The following example shows how Buffer arrays can be used:

\code
int main()
{
    //init gimpact
    gimpact_init();

    //Buffer handle to use
    GBUFFER_ID bufferhandle;

    //Create a memory buffer of 100 float numbers
    gim_create_common_buffer(100*sizeof(float), &bufferhandle);

    //Create a buffer array from the bufferhandle
    GBUFFER_ARRAY buffer_float_array;
    GIM_BUFFER_ARRAY_INIT_TYPE(float,buffer_float_array,bufferhandle,100);

    ////Access to the buffer data, set all elements of the array

    int i, count;
    count = buffer_float_array.m_element_count;
    //Locks the array
    gim_buffer_array_lock(&buffer_float_array,G_MA_READ_WRITE);
    float  * pelements = GIM_BUFFER_ARRAY_POINTER(float, buffer_float_array, 0); // A pointer to the buffer memory

    //fill the array with random numbers
    for (i = 0;i < count;i++ )
    {
        pelements[i] = gim_unit_random();
    }
    //unlock buffer
    gim_buffer_array_unlock(&buffer_float_array);

    //Program code
        ....
        ....

    //Destroy array
    GIM_BUFFER_ARRAY_DESTROY(buffer_float_array);

    //terminate gimpact
    gimpact_terminate();
}
\endcode

\sa BUFFERS
*/
//! @{

//! Buffer managed array struct.
struct GBUFFER_ARRAY
{
  GBUFFER_ID m_buffer_id;
  char * m_buffer_data;
  char m_byte_stride;
  GUINT m_byte_offset;
  GUINT m_element_count;
};
//typedef  struct _GBUFFER_ARRAY GBUFFER_ARRAY;
//! Tells if this is a valid buffer array
#define GIM_BUFFER_ARRAY_IS_VALID(buffer_array) (buffer_array.m_buffer_id.m_buffer_id!=G_UINT_INFINITY)

//! Invalidates a buffer array
#define GIM_BUFFER_ARRAY_INVALIDATE(buffer_array) buffer_array.m_buffer_id.m_buffer_id = G_UINT_INFINITY;

//! Sets offset for a buffered array.
#define GIM_BUFFER_ARRAY_SET_OFFSET(_array_data,_offset) (_array_data).m_byte_offset = _offset*(_array_data).m_byte_stride;

//! Sets offset for a buffered array.
#define GIM_BUFFER_ARRAY_GET_OFFSET(_array_data,_offset) _offset = (_array_data).m_byte_offset/(_array_data).m_byte_stride;

//!Return a pointer of the element at the _index
#define GIM_BUFFER_ARRAY_POINTER(_type,_array_data,_index) (_type *)((_array_data).m_buffer_data + _index*(_array_data).m_byte_stride)

//! Sets stride for a buffered array.
#define GIM_BUFFER_ARRAY_SET_STRIDE(_type,_array_data) (_array_data).m_byte_stride = sizeof(_type);

//! Is array stride equal to the size of the type ?
#define GIM_BUFFER_ARRAY_IS_ALIGNED(_type,_array_data) ((_array_data).m_byte_stride == sizeof(_type))

///Verify if two arrays have the same data
#define GIM_BUFFER_ARRAY_ARE_SAME(_array_data1,_array_data2,aresame)\
{\
    aresame = 1;\
    if((_array_data1).m_buffer_id.m_buffer_id != (_array_data2).m_buffer_id.m_buffer_id || (_array_data1).m_buffer_id.m_buffer_manager_id != (_array_data2).m_buffer_id.m_buffer_manager_id || (_array_data1).m_byte_offset != (_array_data2).m_byte_offset)\
    {\
        aresame = 0;\
    }\
}\

//! Reserve size for a buffered array.
/*!
\pre array_data must be unlocked, and must be the aligned (GIM_BUFFER_ARRAY_IS_ALIGNED )
*/
#define GIM_BUFFER_ARRAY_RESERVE_SIZE(type,array_data,reserve_size)\
{ \
    if(reserve_size>(array_data).m_element_count)\
    {\
        GUINT _buffer_size,_newarray_size;\
        gim_get_buffer_size(&(array_data).m_buffer_id,_buffer_size);\
        _newarray_size = reserve_size*(array_data).m_byte_stride;\
        if(_newarray_size>_buffer_size)\
        { \
            _newarray_size += G_ARRAY_GROW_SIZE*(array_data).m_byte_stride;\
            gim_buffer_realloc(&(array_data).m_buffer_id,_newarray_size);\
        }\
    }\
}\

//! Pushes an element at last position
/*!
\pre array_data must be unlocked, and must be the aligned (GIM_BUFFER_ARRAY_IS_ALIGNED )
*/
#define GIM_BUFFER_ARRAY_PUSH_ITEM(type,array_data,item)\
{\
    GIM_BUFFER_ARRAY_RESERVE_SIZE(type,array_data,(array_data).m_element_count+1);\
    gim_buffer_array_lock(&array_data,G_MA_WRITE_ONLY);\
    type * _pt = GIM_BUFFER_ARRAY_POINTER(type,array_data,(array_data).m_element_count);\
    memcpy(_pt,&item,sizeof(type));\
    gim_buffer_array_unlock(&array_data);\
    (array_data)->m_element_count++; \
}\

//! Pushes a new element at last position
/*!
\pre array_data must be unlocked, and must be the aligned (GIM_BUFFER_ARRAY_IS_ALIGNED )
*/
#define GIM_BUFFER_ARRAY_PUSH_EMPTY(type,array_data)\
{\
    GIM_BUFFER_ARRAY_RESERVE_SIZE(type,array_data,(array_data).m_element_count+1);\
    array_data->m_element_count++; \
}\

//! Inserts an element
/*!
\pre array_data must be unlocked, and must be the aligned (GIM_BUFFER_ARRAY_IS_ALIGNED )
*/
#define GIM_BUFFER_ARRAY_INSERT_ITEM(type,array_data,item,index) \
{ \
    GIM_BUFFER_ARRAY_RESERVE_SIZE(type,array_data,(array_data).m_element_count+1);\
    gim_buffer_array_lock(&array_data,G_MA_WRITE_ONLY);\
    type * _pt = GIM_BUFFER_ARRAY_POINTER(type,array_data,0);\
    if(index<(array_data)->m_element_count-1) \
    { \
        memcpy(&_pt[index+1],&_pt[index],((array_data).m_element_count-index)*sizeof(type));\
    } \
    memcpy(&_pt[index],&item,sizeof(type));\
    gim_buffer_array_unlock(&array_data);\
    (array_data).m_element_count++; \
}\

//! Deletes an element
/*!
\pre array_data must be unlocked, and must be the aligned (GIM_BUFFER_ARRAY_IS_ALIGNED )
*/
#define GIM_BUFFER_ARRAY_DELETE_ITEM(type,array_data,index) \
{ \
    if(index<(array_data).m_element_count-1) \
    { \
        gim_buffer_array_lock(&array_data,G_MA_WRITE_ONLY);\
        type * _pt = GIM_BUFFER_ARRAY_POINTER(type,array_data,0);\
        memcpy(&_pt[index],&_pt[index+1],((array_data).m_element_count-index-1)*sizeof(type));\
        gim_buffer_array_unlock(&array_data);\
    } \
    (array_data).m_element_count--;    \
}\

//! Deletes an element at last position
/*!
\pre array_data must be unlocked, and must be the aligned (GIM_BUFFER_ARRAY_IS_ALIGNED )
*/
#define GIM_BUFFER_ARRAY_POP_ITEM(array_data) \
{ \
    if((array_data).m_element_count>0) \
    { \
        (array_data).m_element_count--;    \
    } \
}\


//! Initializes an GBUFFER_ARRAY object from a buffer ID
/*!
m_buffer_data will be 0, for acces to the elements, you'd need to call lock_array
\param array_data Array structure to be filled
\param buffer_id A GBUFFER_ID structure which this array_daya will refer to
\param element_count Number of elements
\param offset element offset, it isn't byte offset. 0 is recomended
\param byte_stride size of each element.  0 is recomended.
\post Adds reference to the buffer
\sa gim_buffer_add_ref
*/
#define GIM_BUFFER_ARRAY_INIT_OFFSET_STRIDE(array_data,buffer_id,element_count,offset,byte_stride)\
{\
    (array_data).m_buffer_id.m_buffer_id = (buffer_id).m_buffer_id;\
    (array_data).m_buffer_id.m_buffer_manager_id = (buffer_id).m_buffer_manager_id;\
    (array_data).m_buffer_data = 0;\
    (array_data).m_element_count = element_count;\
    (array_data).m_byte_stride = byte_stride;\
    GIM_BUFFER_ARRAY_SET_OFFSET(array_data,offset);\
    gim_buffer_add_ref(&(buffer_id));\
}\

//! Initializes an GBUFFER_ARRAY object from a buffer ID and a Given type
/*!
m_buffer_data will be 0, for acces to the elements, you'd need to call lock_array
\param type Type of the Array. It determines the stride.
\param array_data Array structure to be filled
\param buffer_id A GBUFFER_ID structure which this array_daya will refer to
\param element_count Number of elements
\param offset element offset, it isn't byte offset. 0 is recomended
\post Adds reference to the buffer
\sa gim_buffer_add_ref
*/
#define GIM_BUFFER_ARRAY_INIT_TYPE_OFFSET(type,array_data,buffer_id,element_count,offset)\
{\
    (array_data).m_buffer_id.m_buffer_id = (buffer_id).m_buffer_id;\
    (array_data).m_buffer_id.m_buffer_manager_id = (buffer_id).m_buffer_manager_id;\
    (array_data).m_buffer_data = 0;\
    (array_data).m_element_count = element_count;\
    GIM_BUFFER_ARRAY_SET_STRIDE(type,array_data);\
    GIM_BUFFER_ARRAY_SET_OFFSET(array_data,offset);\
    gim_buffer_add_ref(&(buffer_id));\
}\

//! Initializes a buffer array giving a data type and a buffer id
/*!
m_buffer_data will be 0, for acces to the elements, you'd need to call lock_array.
\param type Type of the Array. It determines the stride.
\param array_data Array structure to be filled
\param buffer_id A GBUFFER_ID structure which this array_daya will refer to
\param element_count Number of elements
\post Adds reference to the buffer
\sa gim_buffer_add_ref
*/
#define GIM_BUFFER_ARRAY_INIT_TYPE(type,array_data,buffer_id,element_count) GIM_BUFFER_ARRAY_INIT_TYPE_OFFSET(type,array_data,buffer_id,element_count,0)

//! Gain access to the array buffer through the m_buffer_data element
/*!
m_buffer_data pointer will be located at the m_byte_offset position of the buffer m_buffer
Then, You'd need to call unlock_array when finish to using the array access.

\pre if m_buffer_data != 0, the function returns
\param array_data Array structure to be locked
\param access A constant for access to the buffer. can be G_MA_READ_ONLY,G_MA_WRITE_ONLY or G_MA_READ_WRITE
\return an Buffer error code
*/
GINT gim_buffer_array_lock(GBUFFER_ARRAY * array_data, int access);

//! close the access to the array buffer through the m_buffer_data element
/*!
\param array_data Array structure to be locked
\return an Buffer error code
*/
GINT gim_buffer_array_unlock(GBUFFER_ARRAY * array_data);

//! Copy an array by reference
/*!
\post A reference to the m_buffer_id is increased.
*/
void gim_buffer_array_copy_ref(GBUFFER_ARRAY * source_data,GBUFFER_ARRAY  * dest_data);


//! Copy an array by value
/*!
\post A new buffer is created
*/
void gim_buffer_array_copy_value(GBUFFER_ARRAY * source_data,GBUFFER_ARRAY  * dest_data, GUINT buffer_manager_id,int usage);

//! Destroys an GBUFFER_ARRAY object
/*!
\post Attemps to destroy the buffer, decreases reference counting
*/
void GIM_BUFFER_ARRAY_DESTROY(GBUFFER_ARRAY & array_data);

//! Copy the content of the array to a pointer
/*!
\pre dest_data must have the same size as the array_data
\param type
\param array_data A GBUFFERED_ARRAY structure
\param dest_data A type pointer
*/
#define GIM_BUFFER_ARRAY_DOWNLOAD(type,array_data,dest_data)\
{\
    if(GIM_BUFFER_ARRAY_IS_ALIGNED(type,array_data))\
    {\
        gim_download_from_buffer(&(array_data).m_buffer_id, (array_data).m_byte_offset,(void *) dest_data, (array_data).m_element_count*(array_data).m_byte_stride);\
    }\
    else\
    {\
        GUINT _k_, _ecount_= (array_data).m_element_count;\
        type * _source_vert_;\
        type * _dest_vert_ = dest_data;\
        gim_buffer_array_lock(&(array_data),G_MA_READ_ONLY);\
        for (_k_ = 0;_k_< _ecount_; _k_++)\
        {\
            _source_vert_ = GIM_BUFFER_ARRAY_POINTER(type,array_data,_k_);\
            memcpy(_dest_vert_,_source_vert_,sizeof(type));\
            _dest_vert_++;\
        }\
        gim_buffer_array_unlock(&(array_data));\
    }\
}\

//! Upload the content of a a pointer to a buffered array
/*!
\pre source_data must have the same size as the array_data
\param type
\param array_data A GBUFFERED_ARRAY structure
\param source_data A void pointer
*/
#define GIM_BUFFER_ARRAY_UPLOAD(type,array_data,source_data)\
{\
    if(GIM_BUFFER_ARRAY_IS_ALIGNED(type,array_data))\
    {\
        gim_upload_to_buffer(&(array_data).m_buffer_id, (array_data).m_byte_offset,(void *) source_data, (array_data).m_element_count*(array_data).m_byte_stride);\
    }\
    else\
    {\
        GUINT _k_, _ecount_= (array_data).m_element_count;\
        type * _source_vert_ = source_data;\
        type * _dest_vert_;\
        gim_buffer_array_lock(&(array_data),G_MA_WRITE_ONLY);\
        for (_k_ = 0;_k_< _ecount_; _k_++)\
        {\
            _dest_vert_ = GIM_BUFFER_ARRAY_POINTER(type,array_data,_k_);\
            memcpy(_dest_vert_,_source_vert_,sizeof(type));\
            _source_vert_++;\
        }\
        gim_buffer_array_unlock(&(array_data));\
    }\
}\


//!Kernel function prototype for process streams, given a buffered array as source and
/*!
\param 1 the uniform arguments
\param 2 the source stream element
\param 3 the destination stream element
*/
typedef void (* gim_kernel_func)(void *,void *,void *);

//! Generic Stream Processingp loop
/*!

This macro executes a kernel macro or function for each element of the streams
\pre _src_array->m_count <= _dst_array->m_count

\param _uniform_data An argument to be passed to the Kernel function
\param _src_array An GBUFFER_ARRAY structure passed as the source stream
\param _dst_array An GBUFFER_ARRAY  structure passed as the source stream
\param _kernel Macro or function of the kernel
\param _src_type Required. Type of all elements of the source stream
\param _dst_type Required. Type of all elements of the dest stream
*/
#define GIM_PROCESS_BUFFER_ARRAY(_uniform_data,_src_array,_dst_array,_kernel,_src_type,_dst_type) {\
\
    gim_buffer_array_lock(&_src_array,G_MA_READ_ONLY);\
    gim_buffer_array_lock(&_dst_array,G_MA_WRITE_ONLY);\
\
    GUINT _i_, _count_=(_src_array).m_element_count;\
\
    _src_type * _source_vert_;\
    _dst_type * _dest_vert_;\
    if(GIM_BUFFER_ARRAY_IS_ALIGNED(_src_type,_src_array) && GIM_BUFFER_ARRAY_IS_ALIGNED(_dst_type,_dst_array))\
    {\
\
        _source_vert_ = GIM_BUFFER_ARRAY_POINTER(_src_type,_src_array,0);\
        _dest_vert_ = GIM_BUFFER_ARRAY_POINTER(_dst_type,_dst_array,0);\
        for (_i_ = 0;_i_< _count_; _i_++)\
        {\
            _kernel(_uniform_data,(*_source_vert_),(*_dest_vert_));\
            _source_vert_++;\
            _dest_vert_++;\
        }\
    }\
    else\
    {\
        for (_i_ = 0;_i_< _count_; _i_++)\
        {\
            _source_vert_ = GIM_BUFFER_ARRAY_POINTER(_src_type,_src_array,_i_);\
            _dest_vert_ = GIM_BUFFER_ARRAY_POINTER(_dst_type,_dst_array,_i_);\
            _kernel(_uniform_data,(*_source_vert_),(*_dest_vert_));\
        }\
    }\
    gim_buffer_array_unlock(&_src_array);\
    gim_buffer_array_unlock(&_dst_array);\
}\

//! @}

#endif // GIM_MEMORY_H_INCLUDED
