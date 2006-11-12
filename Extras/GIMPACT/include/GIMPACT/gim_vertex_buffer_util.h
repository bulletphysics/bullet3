#ifndef GIM_VERTEX_BUFFER_UTIL_H_INCLUDED
#define GIM_VERTEX_BUFFER_UTIL_H_INCLUDED
/*! \file gim_trimesh_data.h
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

#include "GIMPACT/gim_geometry.h"
#include "GIMPACT/gim_memory.h"

/*! \addtogroup VERTEX_BUFFER_UTIL
\brief
Functions for processing vertex buffers

*/
//! @{

//Macro for processing vertex buffers
#define GIM_PROCESS_VERTEX_BUFFER_ARRAY(_uniform_data,_src_array,_dst_array,_kernel,_src_scalar_type,_dst_scalar_type)\
{\
    if(_src_scalar_type == G_STYPE_REAL)\
    {\
        if(_dst_scalar_type == G_STYPE_REAL)\
        {\
            GIM_PROCESS_BUFFER_ARRAY(_uniform_data,_src_array,_dst_array,_kernel,vec3f,vec3f);\
        }\
        else if(_dst_scalar_type == G_STYPE_REAL2)\
        {\
            GIM_PROCESS_BUFFER_ARRAY(_uniform_data,_src_array,_dst_array,_kernel,vec3f,vec3d);\
        }\
    }\
    else if(_src_scalar_type == G_STYPE_REAL2)\
    {\
        if(_dst_scalar_type == G_STYPE_REAL)\
        {\
            GIM_PROCESS_BUFFER_ARRAY(_uniform_data,_src_array,_dst_array,_kernel,vec3d,vec3f);\
        }\
        else if(_dst_scalar_type == G_STYPE_REAL2)\
        {\
            GIM_PROCESS_BUFFER_ARRAY(_uniform_data,_src_array,_dst_array,_kernel,vec3d,vec3f);\
        }\
    }\
}\


//******Common kernels*******//
#define MULT_MAT_VEC4_KERNEL(_mat,_src,_dst) MAT_DOT_VEC_3X4((_dst),(_mat),(_src))

#define COPY_VEC3_KERNEL(_uni,_src,_dst) VEC_COPY((_dst),(_src))
//******+++++++++++*******//

//! Creates a packed vertex buffer with vec3f
void gim_create_common_vertex_buffer(GBUFFER_ARRAY * vertex_buffer,GUINT vertex_count);

//! Copies vertex buffer arrays
/*!
They must have the same size
*/
void gim_copy_vertex_buffers(GBUFFER_ARRAY * source_vertex_buffer,GUINT source_scalar_type,GBUFFER_ARRAY * dest_vertex_buffer,GUINT dest_scalar_type);


//! Applies a linear transformation to vertices
/*!
They must have the same size
*/
void gim_transform_vertex_buffers(GBUFFER_ARRAY * source_vertex_buffer,GUINT source_scalar_type,GBUFFER_ARRAY * dest_vertex_buffer,GUINT dest_scalar_type, mat4f transform);


//! General processing for vertex arrays
/*!
It's an alternative to GIM_PROCESS_VERTEX_BUFFER_ARRAY
*/
void gim_process_vertex_buffers(GBUFFER_ARRAY * source_vertex_buffer,GUINT source_scalar_type,GBUFFER_ARRAY * dest_vertex_buffer,GUINT dest_scalar_type, void *uniform_data,gim_kernel_func kernel_func);

//! @}



#endif // GIM_VERTEX_BUFFER_UTIL_H_INCLUDED
