
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

#include "GIMPACT/gim_vertex_buffer_util.h"



void gim_create_common_vertex_buffer(GBUFFER_ARRAY * vertex_buffer,GUINT vertex_count)
{
    gim_create_common_buffer(vertex_count*sizeof(vec3f), &vertex_buffer->m_buffer_id);
    GIM_BUFFER_ARRAY_INIT_TYPE(vec3f,(*vertex_buffer),vertex_buffer->m_buffer_id,vertex_count);
}


void gim_copy_vertex_buffers(GBUFFER_ARRAY * source_vertex_buffer,GUINT source_scalar_type,GBUFFER_ARRAY * dest_vertex_buffer,GUINT dest_scalar_type)
{
    GIM_PROCESS_VERTEX_BUFFER_ARRAY(0,(*source_vertex_buffer),(*dest_vertex_buffer),COPY_VEC3_KERNEL,source_scalar_type,dest_scalar_type);
}


void gim_transform_vertex_buffers(GBUFFER_ARRAY * source_vertex_buffer,GUINT source_scalar_type,GBUFFER_ARRAY * dest_vertex_buffer,GUINT dest_scalar_type, mat4f transform)
{
    GIM_PROCESS_VERTEX_BUFFER_ARRAY(transform,(*source_vertex_buffer),(*dest_vertex_buffer),MULT_MAT_VEC4_KERNEL,source_scalar_type,dest_scalar_type);
}

void gim_process_vertex_buffers(GBUFFER_ARRAY * source_vertex_buffer,GUINT source_scalar_type,GBUFFER_ARRAY * dest_vertex_buffer,GUINT dest_scalar_type, void *uniform_data,gim_kernel_func kernel_func)
{
    GIM_PROCESS_VERTEX_BUFFER_ARRAY(uniform_data,(*source_vertex_buffer),(*dest_vertex_buffer),kernel_func,source_scalar_type,dest_scalar_type);
}
