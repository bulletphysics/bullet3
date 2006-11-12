
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

#include "GIMPACT/gim_contact.h"

#define MAX_COINCIDENT 8

void gim_interpolate_contact_normals(GIM_CONTACT * destcontact,vec3f * normals,GUINT normal_count)
{
    GREAL vec_sum[]={0.0f,0.0f,0.0f};
    vec3f test_vec;
    for(GUINT i=0;i<normal_count;i++)
    {
        VEC_SCALE(test_vec,destcontact->m_depth,normals[i]);
        VEC_SUM(vec_sum,vec_sum,test_vec);
    }

    if(VEC_DOT(vec_sum,vec_sum)<CONTACT_DIFF_EPSILON) return;

    test_vec[0] = 1.0f/((GREAL)normal_count);
    VEC_SCALE(vec_sum,test_vec[0],vec_sum);

    VEC_LENGTH(vec_sum,destcontact->m_depth);
    test_vec[0] = 1.0f/destcontact->m_depth;

    VEC_SCALE(destcontact->m_normal,test_vec[0],vec_sum);
}



void gim_merge_contacts(GDYNAMIC_ARRAY * source_contacts,
					GDYNAMIC_ARRAY * dest_contacts)
{
    dest_contacts->m_size = 0;

	GUINT source_count = source_contacts->m_size;
	GIM_CONTACT * psource_contacts	= GIM_DYNARRAY_POINTER(GIM_CONTACT,(*source_contacts));
	//create keys
	GIM_RSORT_TOKEN * keycontacts = (GIM_RSORT_TOKEN * )gim_alloc(sizeof(GIM_RSORT_TOKEN)*source_count);

    GUINT i;
	for(i=0;i<source_count;i++)
	{
		keycontacts[i].m_value = i;
		GIM_CALC_KEY_CONTACT(psource_contacts[i].m_point,keycontacts[i].m_key);
	}

	//sort keys
	GIM_QUICK_SORT_ARRAY(GIM_RSORT_TOKEN , keycontacts, source_count, RSORT_TOKEN_COMPARATOR,GIM_DEF_EXCHANGE_MACRO);

	// Merge contacts
	GIM_CONTACT * pcontact = 0;
	GIM_CONTACT * scontact = 0;
	GUINT key,last_key=0;
	GUINT coincident_count=0;
	vec3f coincident_normals[MAX_COINCIDENT];

	for(i=0;i<source_contacts->m_size;i++)
	{
	    key = keycontacts[i].m_key;
		scontact = &psource_contacts[keycontacts[i].m_value];

		if(i>0 && last_key ==  key)
		{
			//merge contact
			if(pcontact->m_depth - CONTACT_DIFF_EPSILON > scontact->m_depth)//)
			{
                GIM_COPY_CONTACTS(pcontact, scontact);
                coincident_count = 0;
			}
			else
			{
			#if (NORMAL_CONTACT_AVERAGE == 1)

                if(fabsf(pcontact->m_depth - scontact->m_depth)<CONTACT_DIFF_EPSILON)
                {
                    if(coincident_count<MAX_COINCIDENT)
                    {
                        VEC_COPY(coincident_normals[coincident_count],scontact->m_normal);
                        coincident_count++;
                    }
                }
            #endif
			}
		}
		else
		{//add new contact
        #if (NORMAL_CONTACT_AVERAGE == 1)
		    if(pcontact&&coincident_count>0)
		    {
		        gim_interpolate_contact_normals(pcontact,coincident_normals,coincident_count);
		        coincident_count = 0;
		    }
        #endif
		    GIM_DYNARRAY_PUSH_EMPTY(GIM_CONTACT,(*dest_contacts));
            pcontact = GIM_DYNARRAY_POINTER_LAST(GIM_CONTACT,(*dest_contacts));
		    GIM_COPY_CONTACTS(pcontact, scontact);
		    coincident_count=0;

        }
		last_key = key;
	}
	gim_free(keycontacts,0);
}

void gim_merge_contacts_unique(GDYNAMIC_ARRAY * source_contacts,
					GDYNAMIC_ARRAY * dest_contacts)
{
    dest_contacts->m_size = 0;
    //Traverse the source contacts
	GUINT source_count = source_contacts->m_size;
	if(source_count==0) return;

	GIM_CONTACT * psource_contacts	= GIM_DYNARRAY_POINTER(GIM_CONTACT,(*source_contacts));

	//add the unique contact
	GIM_CONTACT * pcontact = 0;
    GIM_DYNARRAY_PUSH_EMPTY(GIM_CONTACT,(*dest_contacts));
    pcontact = GIM_DYNARRAY_POINTER_LAST(GIM_CONTACT,(*dest_contacts));
    //set the first contact
    GIM_COPY_CONTACTS(pcontact, psource_contacts);

    if(source_count==1) return;
    //scale the first contact
    VEC_SCALE(pcontact->m_normal,pcontact->m_depth,pcontact->m_normal);

    psource_contacts++;

	//Average the contacts
    GUINT i;
	for(i=1;i<source_count;i++)
	{
	    VEC_SUM(pcontact->m_point,pcontact->m_point,psource_contacts->m_point);
	    VEC_ACCUM(pcontact->m_normal,psource_contacts->m_depth,psource_contacts->m_normal);
	    psource_contacts++;
	}

	GREAL divide_average = 1.0f/((GREAL)source_count);

	VEC_SCALE(pcontact->m_point,divide_average,pcontact->m_point);

	pcontact->m_depth = VEC_DOT(pcontact->m_normal,pcontact->m_normal)*divide_average;
	GIM_SQRT(pcontact->m_depth,pcontact->m_depth);

	VEC_NORMALIZE(pcontact->m_normal);

	/*GREAL normal_len;
    VEC_INV_LENGTH(pcontact->m_normal,normal_len);
	VEC_SCALE(pcontact->m_normal,normal_len,pcontact->m_normal);

    //Deep = LEN(normal)/SQRT(source_count)
    GIM_SQRT(divide_average,divide_average);
	pcontact->m_depth = divide_average/normal_len;
	*/
}



