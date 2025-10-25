#ifndef GIM_CONTACT_H_INCLUDED
#define GIM_CONTACT_H_INCLUDED

/*! \file gim_contact.h
\author Francisco Leon Najera
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
#include "gim_geometry.h"
#include "gim_radixsort.h"
#include "gim_array.h"

#include "btContactProcessingStructs.h"

class gim_contact_array : public gim_array<GIM_CONTACT>
{
public:
	gim_contact_array() : gim_array<GIM_CONTACT>(64)
	{
	}

	SIMD_FORCE_INLINE void push_contact(const btVector3 &point, const btVector3 &normal,
										GREAL depth, GUINT feature1, GUINT feature2)
	{
		push_back_mem();
		GIM_CONTACT &newele = back();
		newele.m_point = point;
		newele.m_normal = normal;
		newele.m_depth = depth;
		newele.m_feature1 = feature1;
		newele.m_feature2 = feature2;
	}

	SIMD_FORCE_INLINE void push_triangle_contacts(
		const GIM_TRIANGLE_CONTACT_DATA &tricontact,
		GUINT feature1, GUINT feature2)
	{
		for (GUINT i = 0; i < tricontact.m_point_count; i++)
		{
			push_back_mem();
			GIM_CONTACT &newele = back();
			newele.m_point = tricontact.m_points[i];
			newele.m_normal = tricontact.m_separating_normal;
			newele.m_depth = tricontact.m_penetration_depth;
			newele.m_feature1 = feature1;
			newele.m_feature2 = feature2;
		}
	}

	void merge_contacts(const gim_contact_array &contacts, bool normal_contact_average = true);
	void merge_contacts_unique(const gim_contact_array &contacts);
};

#endif  // GIM_CONTACT_H_INCLUDED
