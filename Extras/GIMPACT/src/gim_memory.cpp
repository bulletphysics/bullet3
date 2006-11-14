
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


#include "GIMPACT/gim_memory.h"
#include "stdlib.h"
//#include "malloc.h"
//#include "mm_malloc.h"

static gim_alloc_function *g_allocfn = 0;
static gim_alloca_function *g_allocafn = 0;
static gim_realloc_function *g_reallocfn = 0;
static gim_free_function *g_freefn = 0;

// buffer managers
#define MAX_BUFFER_MANAGERS 16
static GBUFFER_MANAGER_DATA g_buffer_managers[MAX_BUFFER_MANAGERS];
static GUINT g_buffer_managers_count = 0;

#define VALIDATE_BUFFER_MANAGER(buffer_manager_id)\
    if(buffer_manager_id>=MAX_BUFFER_MANAGERS) return G_BUFFER_OP_INVALID;\
    GBUFFER_MANAGER_DATA * bm_data;\
    gim_get_buffer_manager_data(buffer_manager_id,&bm_data);\
    if(bm_data == 0) return G_BUFFER_OP_INVALID;\

#define VALIDATE_BUFFER_ID_PT(buffer_id)\
    VALIDATE_BUFFER_MANAGER(buffer_id->m_buffer_manager_id)\
    if(buffer_id->m_buffer_id>=bm_data->m_buffer_array.m_size) return G_BUFFER_OP_INVALID;\
    GBUFFER_DATA * pbuffer = GIM_DYNARRAY_POINTER(GBUFFER_DATA,bm_data->m_buffer_array);\
    pbuffer += buffer_id->m_buffer_id;\
    if(pbuffer->m_buffer_handle==0) return G_BUFFER_OP_INVALID;\


void GIM_BUFFER_ARRAY_DESTROY(GBUFFER_ARRAY & array_data)
{
    gim_buffer_array_unlock(&array_data);
    gim_buffer_free(&(array_data).m_buffer_id);
    array_data.m_buffer_id.m_buffer_id = G_UINT_INFINITY;
    array_data.m_element_count = 0;
}

void GIM_DYNARRAY_DESTROY(GDYNAMIC_ARRAY & array_data)
{
    if(array_data.m_pdata != 0)
    {
        gim_free(array_data.m_pdata,0);
        array_data.m_reserve_size = 0;
        array_data.m_size = 0;
        array_data.m_pdata = 0;
    }
}

void gim_set_alloc_handler (gim_alloc_function *fn)
{
  g_allocfn = fn;
}

void gim_set_alloca_handler (gim_alloca_function *fn)
{
  g_allocafn = fn;
}

void gim_set_realloc_handler (gim_realloc_function *fn)
{
  g_reallocfn = fn;
}

void gim_set_free_handler (gim_free_function *fn)
{
  g_freefn = fn;
}

gim_alloc_function *gim_get_alloc_handler()
{
  return g_allocfn;
}

gim_alloca_function *gim_get_alloca_handler()
{
  return g_allocafn;
}


gim_realloc_function *gim_get_realloc_handler ()
{
  return g_reallocfn;
}


gim_free_function  *gim_get_free_handler ()
{
  return g_freefn;
}


void * gim_alloc(size_t size)
{
   char * ptr = 0;
   ptr = new char[size]; //malloc(size);
  /*if (g_allocfn) ptr = g_allocfn(size); else ptr = malloc(size);//_mm_malloc(size,0);*/
  if(ptr==0)
  {
      float * fp = 0;
      *fp = 0.0f;
  }
  return ptr;
}

#ifdef ALLOCA_GIMPACT
void * gim_alloca(size_t size)
{
  if (g_allocafn) return g_allocafn(size); else 
	  return alloca(size);
}
#endif


void * gim_realloc(void *ptr, size_t oldsize, size_t newsize)
{
  /*if (g_reallocfn) return g_reallocfn(ptr,oldsize,newsize);
  else return realloc(ptr,newsize);*/
  //return realloc(ptr,newsize);
	void * newptr = gim_alloc(newsize);
	size_t copysize = newsize> oldsize? oldsize: newsize;
	memcpy(newptr,ptr,copysize);
	gim_free(ptr,oldsize);
	return newptr;
}

void gim_free(void *ptr, size_t size)
{
  if (!ptr) return;
  if (g_freefn)
  {
       g_freefn(ptr,size);
  }
  else
  {
	  char * cptr = (char *)ptr;
	  delete [] cptr;
      //free(ptr);//_mm_free(ptr);
  }
}

///******************************* BUFFER MANAGERS ******************************///

//!** Basic buffer prototyoe functions

GUINT _system_buffer_alloc_function(GUINT size,int usage)
{
    void * newdata = gim_alloc(size);
    memset(newdata,0,size);
    return (GUINT)(newdata);
}

GUINT _system_buffer_alloc_data_function(const void * pdata,GUINT size,int usage)
{
    void * newdata = gim_alloc(size);
    memcpy(newdata,pdata,size);
    return (GUINT)(newdata);
}

GUINT _system_buffer_realloc_function(GUINT buffer_handle,GUINT oldsize,int old_usage,GUINT newsize,int new_usage)
{
    void * newdata = gim_realloc((void *)buffer_handle,oldsize,newsize);
    return (GUINT)(newdata);
}

void _system_buffer_free_function(GUINT buffer_handle,GUINT size)
{
    gim_free((void*)buffer_handle,size);
}

char * _system_lock_buffer_function(GUINT buffer_handle,int access)
{
    return (char * )(buffer_handle);
}


void _system_unlock_buffer_function(GUINT buffer_handle)
{
}

void _system_download_from_buffer_function(
        GUINT source_buffer_handle,
		GUINT source_pos,
		void * destdata,
		GUINT copysize)
{
    char * pdata;
	pdata = (char *)source_buffer_handle;
	memcpy(destdata,pdata+source_pos,copysize);
}

void  _system_upload_to_buffer_function(
        GUINT dest_buffer_handle,
		GUINT dest_pos,
		void * sourcedata,
		GUINT copysize)
{
    char * pdata;
	pdata = (char * )dest_buffer_handle;
	memcpy(pdata+dest_pos,sourcedata,copysize);
}

void  _system_copy_buffers_function(
		GUINT source_buffer_handle,
		GUINT source_pos,
		GUINT dest_buffer_handle,
		GUINT dest_pos,
		GUINT copysize)
{
    char * pdata1,*pdata2;
	pdata1 = (char *)source_buffer_handle;
	pdata2 = (char *)dest_buffer_handle;
	memcpy(pdata2+dest_pos,pdata1+source_pos,copysize);
}

GUINT _shared_buffer_alloc_function(GUINT size,int usage)
{
    return 0;
}

GUINT _shared_buffer_alloc_data_function(const void * pdata,GUINT size,int usage)
{
    return (GUINT)pdata;
}

GUINT _shared_buffer_realloc_function(GUINT buffer_handle,GUINT oldsize,int old_usage,GUINT newsize,int new_usage)
{
    return 0;
}

void _shared_buffer_free_function(GUINT buffer_handle,GUINT size)
{
}

//!** Buffer manager operations
void gim_create_buffer_manager(GBUFFER_MANAGER_PROTOTYPE * prototype,GUINT buffer_manager_id)
{
    GBUFFER_MANAGER_DATA * bm_data;
    bm_data = &g_buffer_managers[buffer_manager_id];

    if(bm_data->m_active==0)
    {
        if(g_buffer_managers_count<=buffer_manager_id)
        {
            g_buffer_managers_count = buffer_manager_id+1;
        }
    }
    else
    {
        gim_destroy_buffer_manager(buffer_manager_id);
    }
    bm_data->m_active = 1;
    //CREATE ARRAYS
    GIM_DYNARRAY_CREATE(GBUFFER_DATA,bm_data->m_buffer_array,G_ARRAY_GROW_SIZE);
    GIM_DYNARRAY_CREATE(GUINT,bm_data->m_free_positions,G_ARRAY_GROW_SIZE);
    //INIT PROTOTYPE
    bm_data->m_prototype.alloc_data_fn = prototype->alloc_data_fn;
    bm_data->m_prototype.alloc_fn = prototype->alloc_fn;
    bm_data->m_prototype.copy_buffers_fn = prototype->copy_buffers_fn;
    bm_data->m_prototype.download_from_buffer_fn = prototype->download_from_buffer_fn;
    bm_data->m_prototype.free_fn = prototype->free_fn;
    bm_data->m_prototype.lock_buffer_fn = prototype->lock_buffer_fn;
    bm_data->m_prototype.realloc_fn = prototype->realloc_fn;
    bm_data->m_prototype.unlock_buffer_fn = prototype->unlock_buffer_fn;
    bm_data->m_prototype.upload_to_buffer_fn = prototype->upload_to_buffer_fn;
}

GUINT gim_get_buffer_manager_count()
{
    return g_buffer_managers_count;
}
void gim_destroy_buffer_manager(GUINT buffer_manager_id)
{
    GBUFFER_MANAGER_DATA * bm_data;
    gim_get_buffer_manager_data(buffer_manager_id,&bm_data);
    if(bm_data == 0) return;
    //Destroy all buffers

    GBUFFER_DATA * buffers = GIM_DYNARRAY_POINTER(GBUFFER_DATA,bm_data->m_buffer_array);
    GUINT i, buffer_count = bm_data->m_buffer_array.m_size;
    for (i=0;i<buffer_count ;i++ )
    {
    	if(buffers[i].m_buffer_handle!=0) //Is active
    	{
    	    // free handle
    	    bm_data->m_prototype.free_fn(buffers[i].m_buffer_handle,buffers[i].m_size);
    	}
    }

    //destroy buffer array
    GIM_DYNARRAY_DESTROY(bm_data->m_buffer_array);
    //destroy free positions
    GIM_DYNARRAY_DESTROY(bm_data->m_free_positions);
    //Mark as innactive
    bm_data->m_active = 0;
}
void gim_get_buffer_manager_data(GUINT buffer_manager_id,GBUFFER_MANAGER_DATA ** pbm_data)
{
    GBUFFER_MANAGER_DATA * bm_data;
    bm_data = &g_buffer_managers[buffer_manager_id];

    if(bm_data->m_active==0)
    {
        *pbm_data = 0;
    }
    else
    {
        *pbm_data = bm_data;
    }
}

void gim_init_buffer_managers()
{
    GUINT i;
    for (i=0;i<MAX_BUFFER_MANAGERS;i++)
    {
        g_buffer_managers[i].m_active = 0;
        g_buffer_managers[i].m_buffer_array.m_pdata = 0;
        g_buffer_managers[i].m_buffer_array.m_reserve_size = 0;
        g_buffer_managers[i].m_buffer_array.m_size = 0;
        g_buffer_managers[i].m_free_positions.m_pdata = 0;
        g_buffer_managers[i].m_free_positions.m_reserve_size = 0;
        g_buffer_managers[i].m_free_positions.m_size = 0;
    }
    g_buffer_managers_count = 0;
    // Add the two most important buffer managers
    GBUFFER_MANAGER_PROTOTYPE prototype;

    //add system buffer manager
    prototype.alloc_data_fn = _system_buffer_alloc_data_function;
    prototype.alloc_fn = _system_buffer_alloc_function;
    prototype.copy_buffers_fn = _system_copy_buffers_function;
    prototype.download_from_buffer_fn = _system_download_from_buffer_function;
    prototype.free_fn = _system_buffer_free_function;
    prototype.lock_buffer_fn = _system_lock_buffer_function;
    prototype.realloc_fn = _system_buffer_realloc_function;
    prototype.unlock_buffer_fn = _system_unlock_buffer_function;
    prototype.upload_to_buffer_fn = _system_upload_to_buffer_function;

    gim_create_buffer_manager(&prototype,G_BUFFER_MANAGER_SYSTEM );

    //add zhared buffer manager
    prototype.alloc_data_fn = _shared_buffer_alloc_data_function;
    prototype.alloc_fn = _shared_buffer_alloc_function;
    prototype.free_fn = _shared_buffer_free_function;
    gim_create_buffer_manager(&prototype,G_BUFFER_MANAGER_SHARED);
}

void gim_terminate_buffer_managers()
{
    GUINT i;
    for (i=0;i<g_buffer_managers_count;i++)
    {
        gim_destroy_buffer_manager(i);
    }
    g_buffer_managers_count = 0;
}

//!** Nuffer operations

void GET_AVALIABLE_BUFFER_ID(GBUFFER_MANAGER_DATA * buffer_manager, GUINT & buffer_id)
{
    if(buffer_manager->m_free_positions.m_size>0)\
    {
        GUINT * _pointer = GIM_DYNARRAY_POINTER(GUINT,buffer_manager->m_free_positions);
        buffer_id = _pointer[buffer_manager->m_free_positions.m_size-1];
        GIM_DYNARRAY_POP_ITEM(buffer_manager->m_free_positions);
    }
    else
    {
        buffer_id = buffer_manager->m_buffer_array.m_size;
        GIM_DYNARRAY_PUSH_EMPTY(GBUFFER_DATA,buffer_manager->m_buffer_array);
    }
}

GINT _validate_buffer_id(GBUFFER_ID * buffer_id,GBUFFER_DATA ** ppbuffer,GBUFFER_MANAGER_DATA ** pbm_data)
{
    VALIDATE_BUFFER_ID_PT(buffer_id)
    *ppbuffer = pbuffer;
    *pbm_data = bm_data;
    return G_BUFFER_OP_SUCCESS;
}

GUINT gim_create_buffer(
    GUINT buffer_manager_id,
    GUINT buffer_size,
    int usage,
    GBUFFER_ID * buffer_id)
{
    VALIDATE_BUFFER_MANAGER(buffer_manager_id)

    GUINT newbufferhandle = bm_data->m_prototype.alloc_fn(buffer_size,usage);
    if(newbufferhandle==0) return G_BUFFER_OP_INVALID;

    GET_AVALIABLE_BUFFER_ID(bm_data,buffer_id->m_buffer_id);
    buffer_id->m_buffer_manager_id = buffer_manager_id;

    GBUFFER_DATA * pbuffer = GIM_DYNARRAY_POINTER(GBUFFER_DATA,bm_data->m_buffer_array);
    pbuffer += buffer_id->m_buffer_id ;
    pbuffer->m_buffer_handle = newbufferhandle;
    pbuffer->m_size = buffer_size;
    pbuffer->m_usage = usage;
    pbuffer->m_lock_count = 0;
    pbuffer->m_refcount = 0;
    pbuffer->m_mapped_pointer = 0;

    //set shadow buffer if needed

    if(usage == G_MU_STATIC_READ ||
			usage == G_MU_STATIC_READ_DYNAMIC_WRITE||
			usage == G_MU_STATIC_READ_DYNAMIC_WRITE_COPY)
    {
        gim_create_common_buffer(buffer_size,&pbuffer->m_shadow_buffer);
    }
    else
    {
        pbuffer->m_shadow_buffer.m_buffer_id = G_UINT_INFINITY;
        pbuffer->m_shadow_buffer.m_buffer_manager_id = G_UINT_INFINITY;
    }
    return G_BUFFER_OP_SUCCESS;
}


GUINT gim_create_buffer_from_data(
    GUINT buffer_manager_id,
    const void * pdata,
    GUINT buffer_size,
    int usage,
    GBUFFER_ID * buffer_id)
{
    VALIDATE_BUFFER_MANAGER(buffer_manager_id)

    GUINT newbufferhandle = bm_data->m_prototype.alloc_data_fn(pdata,buffer_size,usage);
    if(newbufferhandle==0) return G_BUFFER_OP_INVALID;

    GET_AVALIABLE_BUFFER_ID(bm_data,buffer_id->m_buffer_id);
    buffer_id->m_buffer_manager_id = buffer_manager_id;

    GBUFFER_DATA * pbuffer = GIM_DYNARRAY_POINTER(GBUFFER_DATA,bm_data->m_buffer_array);
    pbuffer += buffer_id->m_buffer_id ;
    pbuffer->m_buffer_handle = newbufferhandle;
    pbuffer->m_size = buffer_size;
    pbuffer->m_usage = usage;
    pbuffer->m_lock_count = 0;
    pbuffer->m_mapped_pointer = 0;
    pbuffer->m_refcount = 0;

    //set shadow buffer if needed

    if(usage == G_MU_STATIC_READ ||
			usage == G_MU_STATIC_READ_DYNAMIC_WRITE||
			usage == G_MU_STATIC_READ_DYNAMIC_WRITE_COPY)
    {
        gim_create_common_buffer_from_data(pdata,buffer_size,&pbuffer->m_shadow_buffer);
    }
    else
    {
        pbuffer->m_shadow_buffer.m_buffer_id = G_UINT_INFINITY;
        pbuffer->m_shadow_buffer.m_buffer_manager_id = G_UINT_INFINITY;
    }
    return G_BUFFER_OP_SUCCESS;
}

GUINT gim_create_common_buffer(GUINT buffer_size, GBUFFER_ID * buffer_id)
{
    return gim_create_buffer(G_BUFFER_MANAGER_SYSTEM,buffer_size,G_MU_DYNAMIC_READ_WRITE,buffer_id);
}

GUINT gim_create_common_buffer_from_data(
    const void * pdata, GUINT buffer_size, GBUFFER_ID * buffer_id)
{
    return gim_create_buffer_from_data(G_BUFFER_MANAGER_SYSTEM,pdata,buffer_size,G_MU_DYNAMIC_READ_WRITE,buffer_id);
}

GUINT gim_create_shared_buffer_from_data(
    const void * pdata, GUINT buffer_size, GBUFFER_ID * buffer_id)
{
    return gim_create_buffer_from_data(G_BUFFER_MANAGER_SHARED,pdata,buffer_size,G_MU_DYNAMIC_READ_WRITE,buffer_id);
}

GINT gim_buffer_realloc(GBUFFER_ID * buffer_id,GUINT newsize)
{
    VALIDATE_BUFFER_ID_PT(buffer_id)
    if(pbuffer->m_lock_count>0) return G_BUFFER_OP_INVALID;
    GUINT newhandle = bm_data->m_prototype.realloc_fn(pbuffer->m_buffer_handle,pbuffer->m_size,pbuffer->m_usage,newsize,pbuffer->m_usage);
    if(newhandle==0) return G_BUFFER_OP_INVALID;
    pbuffer->m_buffer_handle = newhandle;
    //realloc shadow buffer if any
    gim_buffer_realloc(&pbuffer->m_shadow_buffer,newsize);
    return G_BUFFER_OP_SUCCESS;
}

GINT gim_buffer_add_ref(GBUFFER_ID * buffer_id)
{
    VALIDATE_BUFFER_ID_PT(buffer_id)
    pbuffer->m_refcount++;
    return G_BUFFER_OP_SUCCESS;
}

GINT gim_buffer_free(GBUFFER_ID * buffer_id)
{
    VALIDATE_BUFFER_ID_PT(buffer_id)
    if(pbuffer->m_lock_count>0) return G_BUFFER_OP_INVALID;
    if(pbuffer->m_refcount>0) pbuffer->m_refcount--;
    if(pbuffer->m_refcount>0) return G_BUFFER_OP_STILLREFCOUNTED;

    bm_data->m_prototype.free_fn(pbuffer->m_buffer_handle,pbuffer->m_size);
    //destroy shadow buffer if needed
    gim_buffer_free(&pbuffer->m_shadow_buffer);
    // Obtain a free slot index for a new buffer
    GIM_DYNARRAY_PUSH_ITEM(GUINT,bm_data->m_free_positions,buffer_id->m_buffer_id);
	pbuffer->m_buffer_handle = 0;
	pbuffer->m_size = 0;
	pbuffer->m_shadow_buffer.m_buffer_id = G_UINT_INFINITY;
    pbuffer->m_shadow_buffer.m_buffer_manager_id = G_UINT_INFINITY;
    return G_BUFFER_OP_SUCCESS;
}

GINT gim_lock_buffer(GBUFFER_ID * buffer_id,int access,char ** map_pointer)
{
    VALIDATE_BUFFER_ID_PT(buffer_id)
    if(pbuffer->m_lock_count>0)
    {
        if(pbuffer->m_access!=access) return G_BUFFER_OP_INVALID;
        pbuffer->m_lock_count++;
        *map_pointer = pbuffer->m_mapped_pointer;
        return G_BUFFER_OP_SUCCESS;
    }

    pbuffer->m_access = access;

    GUINT result;
    if(pbuffer->m_usage==G_MU_STATIC_WRITE)
	{
		*map_pointer = 0;///no access
		return G_BUFFER_OP_INVALID;
	}
	else if(pbuffer->m_usage==G_MU_STATIC_READ)
	{
		if(pbuffer->m_access == G_MA_READ_ONLY)
		{
			result = gim_lock_buffer(&pbuffer->m_shadow_buffer,access,map_pointer);
			if(result!= G_BUFFER_OP_SUCCESS) return G_BUFFER_OP_INVALID;
			pbuffer->m_mapped_pointer = *map_pointer;
			pbuffer->m_lock_count++;
		}
		else
		{
			*map_pointer = 0;
			return G_BUFFER_OP_INVALID;
		}
	}
	else if(pbuffer->m_usage==G_MU_STATIC_READ_DYNAMIC_WRITE)
	{
		if(pbuffer->m_access == G_MA_READ_ONLY)
		{
			result = gim_lock_buffer(&pbuffer->m_shadow_buffer,access,map_pointer);
			if(result!= G_BUFFER_OP_SUCCESS) return G_BUFFER_OP_INVALID;
			pbuffer->m_mapped_pointer = *map_pointer;
			pbuffer->m_lock_count++;
		}
		else if(pbuffer->m_access == G_MA_WRITE_ONLY)
		{
		    pbuffer->m_mapped_pointer = bm_data->m_prototype.lock_buffer_fn(pbuffer->m_buffer_handle,access);
            *map_pointer = pbuffer->m_mapped_pointer;
			pbuffer->m_lock_count++;
		}
		else if(pbuffer->m_access == G_MA_READ_WRITE)
		{
			*map_pointer = 0;
			return G_BUFFER_OP_INVALID;
		}
	}
	else if(pbuffer->m_usage==G_MU_STATIC_READ_DYNAMIC_WRITE_COPY)
	{
		result = gim_lock_buffer(&pbuffer->m_shadow_buffer,access,map_pointer);
        if(result!= G_BUFFER_OP_SUCCESS) return G_BUFFER_OP_INVALID;
        pbuffer->m_mapped_pointer = *map_pointer;
        pbuffer->m_lock_count++;
	}
	else if(pbuffer->m_usage==G_MU_STATIC_WRITE_DYNAMIC_READ)
	{
		if(pbuffer->m_access == G_MA_READ_ONLY)
		{
			pbuffer->m_mapped_pointer = bm_data->m_prototype.lock_buffer_fn(pbuffer->m_buffer_handle,access);
            *map_pointer = pbuffer->m_mapped_pointer;
			pbuffer->m_lock_count++;
		}
		else
		{
			*map_pointer = 0;
			return G_BUFFER_OP_INVALID;
		}
	}
	else if(pbuffer->m_usage==G_MU_DYNAMIC_READ_WRITE)
	{
		pbuffer->m_mapped_pointer = bm_data->m_prototype.lock_buffer_fn(pbuffer->m_buffer_handle,access);
        *map_pointer = pbuffer->m_mapped_pointer;
        pbuffer->m_lock_count++;
	}
    return G_BUFFER_OP_SUCCESS;
}

GINT gim_unlock_buffer(GBUFFER_ID * buffer_id)
{
    VALIDATE_BUFFER_ID_PT(buffer_id)
    if(pbuffer->m_lock_count==0) return G_BUFFER_OP_INVALID;

    if(pbuffer->m_lock_count>1)
    {
        pbuffer->m_lock_count--;
        return G_BUFFER_OP_SUCCESS;
    }


    GUINT result;
    if(pbuffer->m_usage==G_MU_STATIC_WRITE)
	{
		pbuffer->m_mapped_pointer = 0;
        pbuffer->m_lock_count=0;
        return G_BUFFER_OP_INVALID;
	}
	else if(pbuffer->m_usage==G_MU_STATIC_READ)
	{
		if(pbuffer->m_access == G_MA_READ_ONLY)
		{
			result = gim_unlock_buffer(&pbuffer->m_shadow_buffer);
			if(result!= G_BUFFER_OP_SUCCESS) return G_BUFFER_OP_INVALID;
			pbuffer->m_mapped_pointer = 0;
			pbuffer->m_lock_count=0;
		}
		else
		{
			pbuffer->m_mapped_pointer = 0;
			pbuffer->m_lock_count=0;
			return G_BUFFER_OP_INVALID;
		}
	}
	else if(pbuffer->m_usage==G_MU_STATIC_READ_DYNAMIC_WRITE)
	{
		if(pbuffer->m_access == G_MA_READ_ONLY)
		{
			result = gim_unlock_buffer(&pbuffer->m_shadow_buffer);
			if(result!= G_BUFFER_OP_SUCCESS) return G_BUFFER_OP_INVALID;
			pbuffer->m_mapped_pointer = 0;
			pbuffer->m_lock_count=0;
		}
		else if(pbuffer->m_access == G_MA_WRITE_ONLY)
		{
		    bm_data->m_prototype.unlock_buffer_fn(pbuffer->m_buffer_handle);
            pbuffer->m_mapped_pointer = 0;
			pbuffer->m_lock_count=0;
		}
		else if(pbuffer->m_access == G_MA_READ_WRITE)
		{
			pbuffer->m_mapped_pointer = 0;
			pbuffer->m_lock_count=0;
			return G_BUFFER_OP_INVALID;
		}
	}
	else if(pbuffer->m_usage==G_MU_STATIC_READ_DYNAMIC_WRITE_COPY)
	{
	    result = gim_unlock_buffer(&pbuffer->m_shadow_buffer);
        if(result!= G_BUFFER_OP_SUCCESS) return G_BUFFER_OP_INVALID;
        pbuffer->m_mapped_pointer = 0;
        pbuffer->m_lock_count=0;
	    if(pbuffer->m_access == G_MA_WRITE_ONLY||pbuffer->m_access == G_MA_READ_WRITE)
		{
		    gim_copy_buffers(&pbuffer->m_shadow_buffer,0,buffer_id,0,pbuffer->m_size);
		}
	}
	else if(pbuffer->m_usage==G_MU_STATIC_WRITE_DYNAMIC_READ)
	{
		if(pbuffer->m_access == G_MA_READ_ONLY)
		{
			bm_data->m_prototype.unlock_buffer_fn(pbuffer->m_buffer_handle);
            pbuffer->m_mapped_pointer = 0;
			pbuffer->m_lock_count=0;
		}
		else
		{
			pbuffer->m_mapped_pointer = 0;
			pbuffer->m_lock_count=0;
			return G_BUFFER_OP_INVALID;
		}
	}
	else if(pbuffer->m_usage==G_MU_DYNAMIC_READ_WRITE)
	{
		bm_data->m_prototype.unlock_buffer_fn(pbuffer->m_buffer_handle);
        pbuffer->m_mapped_pointer = 0;
        pbuffer->m_lock_count=0;
	}
	return G_BUFFER_OP_SUCCESS;
}

GINT gim_get_buffer_size(GBUFFER_ID * buffer_id,GUINT * buffer_size)
{
    VALIDATE_BUFFER_ID_PT(buffer_id)
    *buffer_size =  pbuffer->m_size;
    return G_BUFFER_OP_SUCCESS;
}

GINT gim_get_buffer_is_locked(GBUFFER_ID * buffer_id,GUINT * lock_count)
{
    VALIDATE_BUFFER_ID_PT(buffer_id)
    *lock_count =  pbuffer->m_lock_count;
    return G_BUFFER_OP_SUCCESS;
}


GINT gim_download_from_buffer(
        GBUFFER_ID * buffer_id,
		GUINT source_pos,
		void * destdata,
		GUINT copysize)
{
    VALIDATE_BUFFER_ID_PT(buffer_id)
    bm_data->m_prototype.download_from_buffer_fn(
        pbuffer->m_buffer_handle,source_pos,destdata,copysize);
    return G_BUFFER_OP_SUCCESS;
}

GINT  gim_upload_to_buffer(
		GBUFFER_ID * buffer_id,
		GUINT dest_pos,
		void * sourcedata,
		GUINT copysize)
{
    VALIDATE_BUFFER_ID_PT(buffer_id)
    bm_data->m_prototype.upload_to_buffer_fn(
        pbuffer->m_buffer_handle,dest_pos,sourcedata,copysize);
    return G_BUFFER_OP_SUCCESS;
}

GINT  gim_copy_buffers(
		GBUFFER_ID * source_buffer_id,
		GUINT source_pos,
		GBUFFER_ID * dest_buffer_id,
		GUINT dest_pos,
		GUINT copysize)
{
    GBUFFER_MANAGER_DATA * bm_data1,* bm_data2;
    GBUFFER_DATA * pbuffer1, * pbuffer2;
    void * tempdata;
    if(_validate_buffer_id(source_buffer_id,&pbuffer1,&bm_data1)!= G_BUFFER_OP_SUCCESS) return G_BUFFER_OP_INVALID;

    if(_validate_buffer_id(dest_buffer_id,&pbuffer2,&bm_data2)!= G_BUFFER_OP_SUCCESS) return G_BUFFER_OP_INVALID;

    if((source_buffer_id->m_buffer_manager_id == dest_buffer_id->m_buffer_manager_id)||
        (source_buffer_id->m_buffer_manager_id == G_BUFFER_MANAGER_SYSTEM && dest_buffer_id->m_buffer_manager_id == G_BUFFER_MANAGER_SHARED)||
        (source_buffer_id->m_buffer_manager_id == G_BUFFER_MANAGER_SHARED && dest_buffer_id->m_buffer_manager_id == G_BUFFER_MANAGER_SYSTEM)
    )
    {//smooth copy
        bm_data1->m_prototype.copy_buffers_fn(pbuffer1->m_buffer_handle,source_pos,pbuffer2->m_buffer_handle,dest_pos,copysize);
    }
    else if(source_buffer_id->m_buffer_manager_id == G_BUFFER_MANAGER_SYSTEM || source_buffer_id->m_buffer_manager_id == G_BUFFER_MANAGER_SHARED)
    {
        //hard copy
        tempdata = (void *)pbuffer1->m_buffer_handle;
        //upload data
        bm_data2->m_prototype.upload_to_buffer_fn(pbuffer2->m_buffer_handle,dest_pos,
            tempdata,
            copysize);
    }
    else
    {
        //very hard copy
        void * tempdata = gim_alloc(copysize);
        //download data
        bm_data1->m_prototype.download_from_buffer_fn(pbuffer1->m_buffer_handle,source_pos,
            tempdata,
            copysize);

        //upload data
        bm_data2->m_prototype.upload_to_buffer_fn(pbuffer2->m_buffer_handle,dest_pos,
            tempdata,
            copysize);
        //delete temp buffer
        gim_free(tempdata,copysize);
    }
    return G_BUFFER_OP_SUCCESS;
}

GINT gim_buffer_array_lock(GBUFFER_ARRAY * array_data, int access)
{
    if(array_data->m_buffer_data != 0) return G_BUFFER_OP_SUCCESS;
    GINT result = gim_lock_buffer(&array_data->m_buffer_id,access,&array_data->m_buffer_data);
    if(result!= G_BUFFER_OP_SUCCESS) return result;
    array_data->m_buffer_data += array_data->m_byte_offset;
    return result;
}

GINT gim_buffer_array_unlock(GBUFFER_ARRAY * array_data)
{
    if(array_data->m_buffer_data == 0) return G_BUFFER_OP_SUCCESS;
    GINT result = gim_unlock_buffer(&array_data->m_buffer_id);
    if(result!= G_BUFFER_OP_SUCCESS) return result;
    array_data->m_buffer_data = 0;
    return result;
}

void gim_buffer_array_copy_ref(GBUFFER_ARRAY * source_data,GBUFFER_ARRAY  * dest_data)
{
    dest_data->m_buffer_id.m_buffer_id = source_data->m_buffer_id.m_buffer_id;
    dest_data->m_buffer_id.m_buffer_manager_id = source_data->m_buffer_id.m_buffer_manager_id;
    dest_data->m_buffer_data = 0;
    dest_data->m_byte_stride = source_data->m_byte_stride;
    dest_data->m_byte_offset = source_data->m_byte_offset;
    dest_data->m_element_count = source_data->m_element_count;
    gim_buffer_add_ref(&dest_data->m_buffer_id);
}

void gim_buffer_array_copy_value(GBUFFER_ARRAY * source_data,GBUFFER_ARRAY  * dest_data, GUINT buffer_manager_id,int usage)
{
    //Create new buffer
    GUINT buffsize = source_data->m_element_count*source_data->m_byte_stride;
    gim_create_buffer(buffer_manager_id,buffsize,usage,&dest_data->m_buffer_id);

    //copy ref data
    dest_data->m_buffer_data = 0;
    dest_data->m_byte_stride = source_data->m_byte_stride;
    dest_data->m_byte_offset = 0;
    dest_data->m_element_count = source_data->m_element_count;
    gim_buffer_add_ref(&dest_data->m_buffer_id);
    //copy buffers
    gim_copy_buffers(&source_data->m_buffer_id,source_data->m_byte_offset,&dest_data->m_buffer_id,0,buffsize);
}
