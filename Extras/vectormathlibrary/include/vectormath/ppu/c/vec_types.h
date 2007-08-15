/* (C) Copyright
   Sony Computer Entertainment, Inc.,
   2001,2002,2003,2004,2005,2006,2007.

   This file is free software; you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation; either version 2 of the License, or (at your option) 
   any later version.

   This file is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
   for more details.

   You should have received a copy of the GNU General Public License
   along with this file; see the file COPYING.  If not, write to the Free
   Software Foundation, 51 Franklin Street, Fifth Floor, Boston, MA
   02110-1301, USA.  */

/* As a special exception, if you include this header file into source files 
   compiled by GCC, this header file does not by itself cause  the resulting 
   executable to be covered by the GNU General Public License.  This exception 
   does not however invalidate any other reasons why the executable file might be 
   covered by the GNU General Public License.  */ 

/* Single token vector data types for the PowerPC SIMD/Vector Multi-media 
   eXtension */

#ifndef _VECTORMATH_VEC_TYPES_H_
#define _VECTORMATH_VEC_TYPES_H_	1

#define qword		__vector unsigned char

#define vec_uchar16	__vector unsigned char
#define vec_char16	__vector signed char
#define vec_bchar16	__vector bool char

#define vec_ushort8	__vector unsigned short
#define vec_short8	__vector signed short
#define vec_bshort8	__vector bool short

#define vec_pixel8	__vector pixel

#define vec_uint4	__vector unsigned int
#define vec_int4	__vector signed int
#define vec_bint4	__vector bool int

#define vec_float4	__vector float

#define vec_ullong2	__vector bool char
#define vec_llong2	__vector bool short

#define vec_double2	__vector bool int

#endif /* _VECTORMATH_VEC_TYPES_H_ */
