#ifndef GIM_CONTACT_H_INCLUDED
#define GIM_CONTACT_H_INCLUDED

/*! \file gim_contact.h
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
#include "GIMPACT/gim_radixsort.h"

/*! \defgroup CONTACTS
\brief
Functions for managing and sorting contacts resulting from a collision query.
<ul>
<li> Contact lists must be create by calling \ref GIM_CREATE_CONTACT_LIST
<li> After querys, contact lists must be destroy by calling \ref GIM_DYNARRAY_DESTROY
<li> Contacts can be merge for avoid duplicate results by calling \ref gim_merge_contacts
</ul>

*/
//! @{

/**
Configuration var for applying interpolation of  contact normals
*/
#define NORMAL_CONTACT_AVERAGE 1

/// Structure for collision results
struct GIM_CONTACT
{
    vec3f m_point;
    vec3f m_normal;
    GREAL m_depth;//Positive value indicates interpenetration
    void * m_handle1;
    void * m_handle2;
    GUINT m_feature1;//Face number
    GUINT m_feature2;//Face number
};
//typedef struct _GIM_CONTACT GIM_CONTACT;

#define CONTACT_DIFF_EPSILON 0.00001f

#define GIM_CALC_KEY_CONTACT(pos,hash)\
{\
	GINT _coords[] = {(GINT)(pos[0]*1000.0f+1.0f),(GINT)(pos[1]*1333.0f),(GINT)(pos[2]*2133.0f+3.0f)};\
	GUINT _hash=0;\
	GUINT *_uitmp = (GUINT *)(&_coords[0]);\
	_hash = *_uitmp;\
	_uitmp++;\
	_hash += (*_uitmp)<<4;\
	_uitmp++;\
	_hash += (*_uitmp)<<8;\
	hash = _hash;\
}\

///Creates a contact list for queries
#define GIM_CREATE_CONTACT_LIST(contact_array) GIM_DYNARRAY_CREATE(GIM_CONTACT,contact_array,64)

#define GIM_PUSH_CONTACT(contact_array, point, normal, deep,handle1, handle2, feat1, feat2)\
{\
    GIM_DYNARRAY_PUSH_EMPTY(GIM_CONTACT,contact_array);\
    GIM_CONTACT * _last = GIM_DYNARRAY_POINTER_LAST(GIM_CONTACT,contact_array);\
    VEC_COPY(_last->m_point,point);\
    VEC_COPY(_last->m_normal,normal);\
    _last->m_depth = deep;\
    _last->m_handle1 = handle1;\
    _last->m_handle2 = handle2;\
    _last->m_feature1 = feat1;\
    _last->m_feature2 = feat2;\
}\

///Receive pointer to contacts
#define GIM_COPY_CONTACTS(dest_contact, source_contact)\
{\
    VEC_COPY(dest_contact->m_point,source_contact->m_point);\
    VEC_COPY(dest_contact->m_normal,source_contact->m_normal);\
    dest_contact->m_depth = source_contact->m_depth;\
    dest_contact->m_handle1 = source_contact->m_handle1;\
    dest_contact->m_handle2 = source_contact->m_handle2;\
    dest_contact->m_feature1 = source_contact->m_feature1;\
    dest_contact->m_feature2 = source_contact->m_feature2;\
}\

//! Merges duplicate contacts with minimum depth criterion
void gim_merge_contacts(GDYNAMIC_ARRAY * source_contacts,
					GDYNAMIC_ARRAY * dest_contacts);


//! Merges to an unique contact
void gim_merge_contacts_unique(GDYNAMIC_ARRAY * source_contacts,
					GDYNAMIC_ARRAY * dest_contacts);

//! @}
#endif // GIM_CONTACT_H_INCLUDED
