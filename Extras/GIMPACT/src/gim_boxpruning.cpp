
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


#include "GIMPACT/gim_boxpruning.h"
#include <assert.h>



//! Allocate memory for all aabb set.
void gim_aabbset_alloc(GIM_AABB_SET * aabbset, GUINT count)
{
    aabbset->m_count = count;
    aabbset->m_boxes = (aabb3f *)gim_alloc(sizeof(aabb3f)*count);

    if(count<GIM_MIN_SORTED_BIPARTITE_PRUNING_BOXES)
    {
        aabbset->m_maxcoords = 0;
        aabbset->m_sorted_mincoords = 0;
    }
    else
    {
        aabbset->m_maxcoords = (GUINT *)gim_alloc(sizeof(GUINT)*aabbset->m_count );
        aabbset->m_sorted_mincoords = (GIM_RSORT_TOKEN *)gim_alloc(sizeof(GIM_RSORT_TOKEN)*aabbset->m_count);
    }
    aabbset->m_shared = 0;
    INVALIDATE_AABB(aabbset->m_global_bound);
}

//! Destroys the aabb set.
void gim_aabbset_destroy(GIM_AABB_SET * aabbset)
{
    aabbset->m_count = 0;
    if(aabbset->m_shared==0)
    {
        gim_free(aabbset->m_boxes,0);
        gim_free(aabbset->m_maxcoords,0);
        gim_free(aabbset->m_sorted_mincoords,0);
    }
    aabbset->m_boxes = 0;
    aabbset->m_sorted_mincoords = 0;
    aabbset->m_maxcoords = 0;
}

void gim_aabbset_calc_global_bound(GIM_AABB_SET * aabbset)
{
    aabb3f * paabb = aabbset->m_boxes;
    aabb3f * globalbox = &aabbset->m_global_bound;
    AABB_COPY((*globalbox),(*paabb));

    GUINT count = aabbset->m_count-1;
    paabb++;
    while(count)
    {
        MERGEBOXES(*globalbox,*paabb)
        paabb++;
        count--;
    }
}


//! Sorts the boxes for box prunning.
/*!
1) find the integer representation of the aabb coords
2) Sorts the min coords
3) Calcs the global bound
\pre aabbset must be allocated. And the boxes must be already set.
\param aabbset
\param calc_global_bound If 1 , calcs the global bound
\post If aabbset->m_sorted_mincoords == 0, then it allocs the sorted coordinates
*/
void gim_aabbset_sort(GIM_AABB_SET * aabbset, char calc_global_bound)
{
    if(aabbset->m_sorted_mincoords == 0)
    {//allocate
        aabbset->m_maxcoords = (GUINT *)gim_alloc(sizeof(GUINT)*aabbset->m_count );
        aabbset->m_sorted_mincoords = (GIM_RSORT_TOKEN *)gim_alloc(sizeof(GIM_RSORT_TOKEN)*aabbset->m_count);
    }

    GUINT i, count = aabbset->m_count;
    aabb3f * paabb = aabbset->m_boxes;
    GUINT * maxcoords = aabbset->m_maxcoords;
    GIM_RSORT_TOKEN * sorted_tokens = aabbset->m_sorted_mincoords;

    if(count<860)//Calibrated on a Pentium IV
    {
        //Sort by quick sort
            //Calculate keys
        for(i=0;i<count;i++)
        {
            GIM_CONVERT_VEC3F_GUINT_XZ_UPPER(paabb[i].maxX,paabb[i].maxZ,maxcoords[i]);
            GIM_CONVERT_VEC3F_GUINT_XZ(paabb[i].minX,paabb[i].minZ,sorted_tokens[i].m_key);
            sorted_tokens[i].m_value = i;
        }
        GIM_QUICK_SORT_ARRAY(GIM_RSORT_TOKEN , sorted_tokens, count, RSORT_TOKEN_COMPARATOR,GIM_DEF_EXCHANGE_MACRO);
    }
    else
    {
        //Sort by radix sort
        GIM_RSORT_TOKEN * unsorted = (GIM_RSORT_TOKEN *)gim_alloc(sizeof(GIM_RSORT_TOKEN )*count);
        //Calculate keys
        for(i=0;i<count;i++)
        {
            GIM_CONVERT_VEC3F_GUINT_XZ_UPPER(paabb[i].maxX,paabb[i].maxZ,maxcoords[i]);
            GIM_CONVERT_VEC3F_GUINT_XZ(paabb[i].minX,paabb[i].minZ,unsorted[i].m_key);
            unsorted[i].m_value = i;
        }
        GIM_RADIX_SORT_RTOKENS(unsorted,sorted_tokens,count);
        gim_free(unsorted,0);
    }

    if(calc_global_bound) gim_aabbset_calc_global_bound(aabbset);
}

//****** Basic Collision Detection******//
//!Traverse set for collision
void _gim_aabbset_traverse_box_collision_brute_force(aabb3f & test_aabb, GIM_AABB_SET * aabbset, GDYNAMIC_ARRAY * collided)
{
    collided->m_size = 0;
    char intersected;
    GUINT i;
    GUINT count = aabbset->m_count;
    aabb3f * paabb = aabbset->m_boxes;
    for (i=0;i< count;i++ )
    {
        AABBCOLLISION(intersected,paabb[i],test_aabb);
        if(intersected)
        {
            GIM_DYNARRAY_PUSH_ITEM(GUINT,(*collided),i);
        }
    }
}

#define COMP_MACRO_TOKEN(x, key) ((GINT)((x.m_key) - (key)))
//!Traverse set for collision sorted
void _gim_aabbset_traverse_box_collision_sorted(aabb3f & test_aabb, GIM_AABB_SET * aabbset, GDYNAMIC_ARRAY * collided)
{
    collided->m_size = 0;
    GUINT count = aabbset->m_count;
    aabb3f * paabb = aabbset->m_boxes;
    GIM_RSORT_TOKEN * ptokens = aabbset->m_sorted_mincoords;

    //Convert the upper corner
    GUINT uppercorner;
    GIM_CONVERT_VEC3F_GUINT_XZ_UPPER(test_aabb.maxX,test_aabb.maxZ,uppercorner);

    //Search the token
    GUINT i,starttoken=0;
    char intersected;
    GIM_BINARY_SEARCH(ptokens, 0,(count-1), uppercorner, COMP_MACRO_TOKEN, intersected, starttoken);

    if(starttoken>=count) starttoken = count-1;
    if(ptokens[starttoken].m_key>uppercorner)
    {
        count = starttoken;
    }

    for (starttoken = 0;starttoken<count;starttoken++)
    {
        i = ptokens[starttoken].m_value;
        AABBCOLLISION(intersected,paabb[i],test_aabb);
        if(intersected)
        {
            GIM_DYNARRAY_PUSH_ITEM(GUINT,(*collided),i);
        }
    }
}

//! Search Tokens those are overlapping with test_box
/*!
\param collided An empty array of GIM_RSORT_TOKEN objects
*/
void _gim_aabbset_traverse_box_collision_sorted_pushtoken(aabb3f & test_aabb, GIM_AABB_SET * aabbset, GDYNAMIC_ARRAY * collided)
{
    collided->m_size = 0;
    GUINT count = aabbset->m_count;
    aabb3f * paabb = aabbset->m_boxes;
    GIM_RSORT_TOKEN * ptokens = aabbset->m_sorted_mincoords;

    //Convert the upper corner
    GUINT uppercorner;
    GIM_CONVERT_VEC3F_GUINT_XZ_UPPER(test_aabb.maxX,test_aabb.maxZ,uppercorner);

    //Search the token
    GUINT i,starttoken=0;
    char intersected;
    GIM_BINARY_SEARCH(ptokens, 0,
     count-1, uppercorner, COMP_MACRO_TOKEN, intersected, starttoken);

    if(starttoken>=count) starttoken = count-1;
    if(ptokens[starttoken].m_key>uppercorner)
    {
        count = starttoken;
    }
    else
    {
        count = starttoken+1;
    }

    for (starttoken = 0;starttoken<count;starttoken++)
    {
        i = ptokens[starttoken].m_value;
        AABBCOLLISION(intersected,paabb[i],test_aabb);
        if(intersected)
        {
            GIM_DYNARRAY_PUSH_ITEM(GIM_RSORT_TOKEN,(*collided),ptokens[i]);
        }
    }
}


//!Traverse set for collision
void _gim_aabbset_traverse_box_collision(aabb3f & test_aabb, GIM_AABB_SET * aabbset, GDYNAMIC_ARRAY * collided)
{
    if(aabbset->m_sorted_mincoords == 0)
    {
        _gim_aabbset_traverse_box_collision_brute_force(test_aabb, aabbset, collided);
    }
    else
    {
        _gim_aabbset_traverse_box_collision_sorted(test_aabb, aabbset, collided);
    }
}


void gim_aabbset_box_collision(aabb3f *test_aabb, GIM_AABB_SET * aabbset, GDYNAMIC_ARRAY * collided)
{
    collided->m_size = 0;
    char intersected;
    AABBCOLLISION(intersected,aabbset->m_global_bound,(*test_aabb));
    if(intersected == 0) return;

    aabb3f _testaabb;
    AABB_COPY(_testaabb,*test_aabb);

    _gim_aabbset_traverse_box_collision(_testaabb,aabbset, collided);
}


//utility macros

/*#define PUSH_PAIR(i,j,pairset)\
{\
    GIM_PAIR _pair={i,j};\
    GIM_DYNARRAY_PUSH_ITEM(GIM_PAIR,pairset,_pair);\
}*/

#define PUSH_PAIR(i,j,pairset)\
{\
    GIM_DYNARRAY_PUSH_EMPTY(GIM_PAIR,pairset);\
    GIM_PAIR * _pair = GIM_DYNARRAY_POINTER(GIM_PAIR,pairset) + (pairset).m_size - 1;\
    _pair->m_index1 = i;\
    _pair->m_index2 = j;\
}

#define PUSH_PAIR_INV(i,j,pairset)\
{\
    GIM_DYNARRAY_PUSH_EMPTY(GIM_PAIR,pairset);\
    GIM_PAIR * _pair = GIM_DYNARRAY_POINTER(GIM_PAIR,pairset) + (pairset).m_size - 1;\
    _pair->m_index1 = j;\
    _pair->m_index2 = i;\
}

#define FIND_OVERLAPPING_FOWARD(\
 curr_index,\
 test_count,\
 test_aabb,\
 max_coord_uint,\
 sorted_tokens,\
 aabbarray,\
 pairset,\
 push_pair_macro)\
{\
    GUINT _i = test_count;\
    char _intersected;\
    GIM_RSORT_TOKEN * _psorted_tokens = sorted_tokens;\
    while(max_coord_uint >= _psorted_tokens->m_key && _i>0)\
    {\
        AABBCOLLISION(_intersected,test_aabb,aabbarray[_psorted_tokens->m_value]);\
    	if(_intersected)\
    	{\
    	    push_pair_macro(curr_index, _psorted_tokens->m_value,pairset);\
    	}\
    	_psorted_tokens++;\
    	_i--;\
    }\
}

//! log(N) Complete box pruning. Returns a list of overlapping pairs of boxes, each box of the pair belongs to the same set.
/*!
\pre aabbset must be allocated and sorted, the boxes must be already set.
\param aabbset Must be sorted. Global bound isn't required
\param collision_pairs Array of GIM_PAIR elements. Must be initialized before (Reserve size ~ 100)
*/
void gim_aabbset_self_intersections_sorted(GIM_AABB_SET * aabbset, GDYNAMIC_ARRAY * collision_pairs)
{
    collision_pairs->m_size = 0;
    GUINT count = aabbset->m_count;
    aabb3f * paabb = aabbset->m_boxes;
    GUINT * maxcoords = aabbset->m_maxcoords;
    GIM_RSORT_TOKEN * sorted_tokens = aabbset->m_sorted_mincoords;
    aabb3f test_aabb;
    while(count>1)
    {
        ///current cache variables
        GUINT  curr_index = sorted_tokens->m_value;
        GUINT max_coord_uint = maxcoords[curr_index];
        AABB_COPY(test_aabb,paabb[curr_index]);

        ///next pairs
        sorted_tokens++;
        count--;
    	FIND_OVERLAPPING_FOWARD( curr_index, count, test_aabb, max_coord_uint, sorted_tokens , paabb, (*collision_pairs),PUSH_PAIR);
    }
}

//! NxN Complete box pruning. Returns a list of overlapping pairs of boxes, each box of the pair belongs to the same set.
/*!
\pre aabbset must be allocated, the boxes must be already set.
\param aabbset Global bound isn't required. Doen't need to be sorted.
\param collision_pairs Array of GIM_PAIR elements. Must be initialized before (Reserve size ~ 100)
*/
void gim_aabbset_self_intersections_brute_force(GIM_AABB_SET * aabbset, GDYNAMIC_ARRAY * collision_pairs)
{
    collision_pairs->m_size = 0;
    GUINT i,j;
    GUINT count = aabbset->m_count;
    aabb3f * paabb = aabbset->m_boxes;
    char intersected;
    for (i=0;i< count-1 ;i++ )
    {
        for (j=i+1;j<count ;j++ )
        {
            AABBCOLLISION(intersected,paabb[i],paabb[j]);
            if(intersected)
            {
                PUSH_PAIR(i,j,(*collision_pairs));
            }
        }
    }
}

//! log(N) Bipartite box pruning. Returns a list of overlapping pairs of boxes, each box of the pair belongs to a different set.
/*!
\pre aabbset1 and aabbset2 must be allocated and sorted, the boxes must be already set.
\param aabbset1 Must be sorted, Global bound is required.
\param aabbset2 Must be sorted, Global bound is required.
\param collision_pairs Array of GIM_PAIR elements. Must be initialized before (Reserve size ~ 100)
*/
void gim_aabbset_bipartite_intersections_sorted(GIM_AABB_SET * aabbset1, GIM_AABB_SET * aabbset2, GDYNAMIC_ARRAY * collision_pairs)
{
    char intersected;
    collision_pairs->m_size = 0;

    AABBCOLLISION(intersected,aabbset1->m_global_bound,aabbset2->m_global_bound);
    if(intersected == 0) return;

    //Classify boxes
    //Find  Set intersection
    aabb3f int_abbb;
    BOXINTERSECTION(aabbset1->m_global_bound,aabbset2->m_global_bound, int_abbb);

    //Clasify set 1
    GDYNAMIC_ARRAY classified_results1;
    GIM_DYNARRAY_CREATE(GIM_RSORT_TOKEN,classified_results1,aabbset1->m_count);

    _gim_aabbset_traverse_box_collision_sorted_pushtoken(int_abbb,aabbset1, &classified_results1);

    if(classified_results1.m_size==0)
    {
        GIM_DYNARRAY_DESTROY(classified_results1);
        return;
    }

    //Clasify set 2
    GDYNAMIC_ARRAY classified_results2;
    GIM_DYNARRAY_CREATE(GIM_RSORT_TOKEN,classified_results2,aabbset2->m_count);

    _gim_aabbset_traverse_box_collision_sorted_pushtoken(int_abbb,aabbset2, &classified_results2);

    if(classified_results2.m_size==0)
    {
        GIM_DYNARRAY_DESTROY(classified_results1);
        GIM_DYNARRAY_DESTROY(classified_results2);
        return;
    }


    aabb3f * paabb1 = aabbset1->m_boxes;
    GUINT * maxcoords1 = aabbset1->m_maxcoords;

    aabb3f * paabb2 = aabbset2->m_boxes;
    GUINT * maxcoords2 = aabbset2->m_maxcoords;

    GUINT  curr_index;

    GUINT max_coord_uint;
    aabb3f test_aabb;

    GUINT  classified_count1 = classified_results1.m_size;
    GUINT  classified_count2 = classified_results2.m_size;

    GIM_RSORT_TOKEN * sorted_tokens1 = GIM_DYNARRAY_POINTER(GIM_RSORT_TOKEN,classified_results1);
    GIM_RSORT_TOKEN * sorted_tokens2 = GIM_DYNARRAY_POINTER(GIM_RSORT_TOKEN,classified_results2);

    while(classified_count1>0&&classified_count2>0)
    {
        if(sorted_tokens1->m_key <= sorted_tokens2->m_key)
        {
            ///current cache variables
            curr_index = sorted_tokens1->m_value;
            max_coord_uint = maxcoords1[curr_index];
            AABB_COPY(test_aabb,paabb1[curr_index]);
            ///next pairs
            sorted_tokens1++;
            classified_count1--;
            FIND_OVERLAPPING_FOWARD( curr_index, classified_count2, test_aabb, max_coord_uint, sorted_tokens2 , paabb2, (*collision_pairs), PUSH_PAIR);
        }
        else ///Switch test
        {
            ///current cache variables
            curr_index = sorted_tokens2->m_value;
            max_coord_uint = maxcoords2[curr_index];
            AABB_COPY(test_aabb,paabb2[curr_index]);
            ///next pairs
            sorted_tokens2++;
            classified_count2--;
            FIND_OVERLAPPING_FOWARD( curr_index, classified_count1, test_aabb, max_coord_uint, sorted_tokens1 , paabb1, (*collision_pairs), PUSH_PAIR_INV );
        }
    }
    GIM_DYNARRAY_DESTROY(classified_results1);
    GIM_DYNARRAY_DESTROY(classified_results2);
}

//! NxM Bipartite box pruning. Returns a list of overlapping pairs of boxes, each box of the pair belongs to a different set.
/*!
\pre aabbset1 and aabbset2 must be allocated and sorted, the boxes must be already set.
\param aabbset1 Must be sorted, Global bound is required.
\param aabbset2 Must be sorted, Global bound is required.
\param collision_pairs Array of GIM_PAIR elements. Must be initialized before (Reserve size ~ 100)
*/
void gim_aabbset_bipartite_intersections_brute_force(GIM_AABB_SET * aabbset1,GIM_AABB_SET * aabbset2, GDYNAMIC_ARRAY * collision_pairs)
{
    char intersected;
    collision_pairs->m_size = 0;
    AABBCOLLISION(intersected,aabbset1->m_global_bound,aabbset2->m_global_bound);
    if(intersected == 0) return;

    aabb3f int_abbb;
    //Find  Set intersection
    BOXINTERSECTION(aabbset1->m_global_bound,aabbset2->m_global_bound, int_abbb);

    //Clasify set 1
    GDYNAMIC_ARRAY classified_results1;
    GIM_DYNARRAY_CREATE(GUINT,classified_results1,(aabbset1->m_count/4));

    _gim_aabbset_traverse_box_collision(int_abbb,aabbset1, &classified_results1);

    if(classified_results1.m_size==0)
    {
        GIM_DYNARRAY_DESTROY(classified_results1);
        return;
    }

    aabb3f * paabb1 = aabbset1->m_boxes;
    aabb3f * paabb2 = aabbset2->m_boxes;

    GUINT i,j,count1,count2;

    //intesect set2
    GUINT * classified1 = GIM_DYNARRAY_POINTER(GUINT,classified_results1);
    count1 = classified_results1.m_size;
    count2 = aabbset2->m_count;
    for (i=0;i<count2;i++)
    {
        AABBCOLLISION(intersected,paabb2[i],int_abbb);
        if(intersected)
        {
            for (j=0;j<count1;j++)
            {
                AABBCOLLISION(intersected,paabb2[i],paabb1[classified1[j]]);
                if(intersected)
                {
                    PUSH_PAIR(classified1[j],i,(*collision_pairs));
                }
            }
        }
    }
    GIM_DYNARRAY_DESTROY(classified_results1);
}


//! Initalizes the set. Sort Boxes if needed.
/*!
\pre aabbset must be allocated. And the boxes must be already set.
\post If the set has less of GIM_MIN_SORTED_BIPARTITE_PRUNING_BOXES boxes, only calcs the global box,
 else it Sorts the entire set( Only applicable for large sets)
*/
void gim_aabbset_update(GIM_AABB_SET * aabbset)
{
    if(aabbset->m_count < GIM_MIN_SORTED_BIPARTITE_PRUNING_BOXES)
    {//Brute force approach
        gim_aabbset_calc_global_bound(aabbset);
    }
    else
    {//Sorted force approach
        gim_aabbset_sort(aabbset,1);
    }
}

//! Complete box pruning. Returns a list of overlapping pairs of boxes, each box of the pair belongs to the same set.
/*!
This function sorts the set and then it calls to gim_aabbset_self_intersections_brute_force or gim_aabbset_self_intersections_sorted.

\param aabbset Set of boxes. Sorting isn't required.
\param collision_pairs Array of GIM_PAIR elements. Must be initialized before (Reserve size ~ 100)
\pre aabbset must be allocated and initialized.
\post If aabbset->m_count >= GIM_MIN_SORTED_PRUNING_BOXES, then it calls to gim_aabbset_sort and then to gim_aabbset_self_intersections_sorted.
*/
void gim_aabbset_self_intersections(GIM_AABB_SET * aabbset, GDYNAMIC_ARRAY * collision_pairs)
{
    if(aabbset->m_count < GIM_MIN_SORTED_PRUNING_BOXES)
    {//Brute force approach
        gim_aabbset_self_intersections_brute_force(aabbset,collision_pairs);
    }
    else
    {//Sorted force approach
        gim_aabbset_sort(aabbset,0);
        gim_aabbset_self_intersections_sorted(aabbset,collision_pairs);
    }
}

//! Collides two sets. Returns a list of overlapping pairs of boxes, each box of the pair belongs to a different set.
/*!
\pre aabbset1 and aabbset2 must be allocated and updated. See .
\param aabbset1 Must be sorted, Global bound is required.
\param aabbset2 Must be sorted, Global bound is required.
\param collision_pairs Array of GIM_PAIR elements. Must be initialized before (Reserve size ~ 100)
*/
void gim_aabbset_bipartite_intersections(GIM_AABB_SET * aabbset1, GIM_AABB_SET * aabbset2, GDYNAMIC_ARRAY * collision_pairs)
{
    if(aabbset1->m_sorted_mincoords == 0||aabbset2->m_sorted_mincoords == 0)
    {//Brute force approach
        gim_aabbset_bipartite_intersections_brute_force(aabbset1,aabbset2,collision_pairs);
    }
    else
    {//Sorted force approach
        gim_aabbset_bipartite_intersections_sorted(aabbset1,aabbset2,collision_pairs);
    }
}


void gim_aabbset_ray_collision(vec3f vorigin,vec3f vdir, GREAL tmax, GIM_AABB_SET * aabbset, GDYNAMIC_ARRAY * collided)
{
    collided->m_size = 0;
    char intersected;

    intersected = BOX_INTERSECTS_RAY_FAST(aabbset->m_global_bound, vorigin,vdir);

    if(intersected==0) return;

    GUINT i;
    GUINT count = aabbset->m_count;
    aabb3f * paabb = aabbset->m_boxes;

    for (i=0;i< count;i++ )
    {
        intersected = BOX_INTERSECTS_RAY_FAST(paabb[i], vorigin,vdir);
        if(intersected)
        {
            GIM_DYNARRAY_PUSH_ITEM(GUINT,(*collided),i);
        }
    }
}

//***********************************AABB Trees ***************************************************//

GUINT _gim_sort_and_calc_splitting_index(GIM_AABB_DATA * boxarray, GUINT startIndex,  GUINT endIndex, GUINT splitAxis)
{
	GUINT i;
	GUINT splitIndex =startIndex;
	GUINT numIndices = endIndex - startIndex;
	GREAL splitValue = 0.f;
    GREAL * boxvalues;
	for (i=startIndex;i<endIndex;i++)
	{
        boxvalues = &boxarray[i].m_aabb.minX;
	    splitValue += 0.5f*(boxvalues[2*splitAxis+1] + boxvalues[2*splitAxis]);
	}

	splitValue *= 1.f/(GREAL)numIndices;

	//sort leafNodes so all values larger then splitValue comes first, and smaller values start from 'splitIndex'.
	GREAL center;
	GIM_AABB_DATA tmp;
	for (i=startIndex;i<endIndex;i++)
	{
	    boxvalues = &boxarray[i].m_aabb.minX;
	    center = 0.5f*(boxvalues[2*splitAxis+1] + boxvalues[2*splitAxis]);

		if (center > splitValue)
		{
			//swap
			AABB_DATA_COPY(tmp,boxarray[i]);
			AABB_DATA_COPY(boxarray[i],boxarray[splitIndex]);
			AABB_DATA_COPY(boxarray[splitIndex],tmp);
			splitIndex++;
		}
	}

	if ((splitIndex==startIndex) || (splitIndex == (endIndex-1)))
	{
		splitIndex = startIndex+ (numIndices>>1);
	}
	return splitIndex;
}


GUINT _gim_calc_splitting_axis(GIM_AABB_DATA * boxarray, GUINT startIndex,  GUINT endIndex)
{
	GUINT i;
	GREAL means[]= {0.f,0.f,0.f};
	GUINT numIndices = endIndex-startIndex;

	for (i=startIndex;i<endIndex;i++)
	{
	    //means += Center
	    means[0] += 0.5f*(boxarray[i].m_aabb.maxX+boxarray[i].m_aabb.minX);
	    means[1] += 0.5f*(boxarray[i].m_aabb.maxY+boxarray[i].m_aabb.minY);
	    means[2] += 0.5f*(boxarray[i].m_aabb.maxZ+boxarray[i].m_aabb.minZ);
	}

	GREAL sval = (1.f/(GREAL)numIndices);
	VEC_SCALE(means,sval,means);

	vec3f diff2;
	GREAL variance[] = {0.f,0.f,0.f};

	for (i=startIndex;i<endIndex;i++)
	{
	    //diff2 = (Center - means)^2
	    diff2[0] = 0.5f*(boxarray[i].m_aabb.maxX+boxarray[i].m_aabb.minX)-means[0];
	    diff2[0] *= diff2[0];
	    diff2[1] = 0.5f*(boxarray[i].m_aabb.maxY+boxarray[i].m_aabb.minY)-means[1];
	    diff2[1] *= diff2[1];
	    diff2[2] = 0.5f*(boxarray[i].m_aabb.maxZ+boxarray[i].m_aabb.minZ)-means[2];
	    diff2[2] *= diff2[2];
		VEC_SUM(variance,variance,diff2);
	}
	sval = 1.f/((GREAL)numIndices-1);
	VEC_SCALE(variance,sval,variance);

    VEC_MAYOR_COORD(variance,i);
	return i;
}


GIM_AABB_TREE_NODE * _gim_build_subtree(
GIM_AABB_TREE_NODE * node_array,
 GUINT & node_index,GIM_AABB_DATA * boxarray, GUINT startIndex,  GUINT endIndex)
{
    GIM_AABB_TREE_NODE * internalNode;

	GUINT splitAxis, splitIndex, i;
	GUINT numIndices = endIndex-startIndex;
	GUINT curIndex = node_index;

	assert(numIndices>0);

    internalNode = &node_array[node_index];
    node_index++;

	if (numIndices==1)
	{
	    //We have a leaf node
        internalNode->m_left = 0;
	    internalNode->m_right = 0;
	    internalNode->m_escapeIndex = 0;
	    AABB_DATA_COPY(internalNode->m_data,boxarray[startIndex]);

		return internalNode;
	}
	//calculate Best Splitting Axis and where to split it. Sort the incoming 'leafNodes' array within range 'startIndex/endIndex'.

	splitAxis = _gim_calc_splitting_axis(boxarray,startIndex,endIndex);

	splitIndex = _gim_sort_and_calc_splitting_index(boxarray,startIndex,endIndex,splitAxis);

    INVALIDATE_AABB(internalNode->m_data.m_aabb);

	for (i=startIndex;i<endIndex;i++)
	{
	    MERGEBOXES(internalNode->m_data.m_aabb,boxarray[i].m_aabb);
	}

	//internalNode->m_escapeIndex;
	internalNode->m_left = _gim_build_subtree(node_array,node_index,boxarray,startIndex,splitIndex);
	internalNode->m_right = _gim_build_subtree(node_array,node_index,boxarray,splitIndex,endIndex);
	internalNode->m_escapeIndex  = node_index - curIndex;
	return internalNode;
}



void gim_aabbtree_create(GIM_AABB_TREE * newtree, GIM_AABB_DATA * boxarray, GUINT boxcount)
{
    newtree->clear_nodes();//For security
    newtree->m_nodes_array_size = boxcount*2;
    newtree->m_node_array = (GIM_AABB_TREE_NODE * )gim_alloc(newtree->m_nodes_array_size*sizeof(GIM_AABB_TREE_NODE));
    newtree->m_num_nodes = 0;
    newtree->m_root_node = _gim_build_subtree(newtree->m_node_array,newtree->m_num_nodes,boxarray,0,boxcount);
}

void gim_aabbtree_destroy(GIM_AABB_TREE * tree)
{
    tree->clear_nodes();
}

//******boxtree collision


//Tree traversing macros
#define NEXT_SUBNODE(rootNode,curIndex)\
{\
    rootNode++;\
    curIndex++;\
}\

#define SKIP_NODE(rootNode,curIndex)\
{\
    curIndex+= rootNode->m_escapeIndex;\
    rootNode+= rootNode->m_escapeIndex;\
}\


void gim_aabbtree_box_collision_get_nodes(aabb3f *test_aabb, GIM_AABB_TREE * aabbtree, GDYNAMIC_ARRAY * collided)
{
    GUINT curIndex = 0;

	GUINT aabbOverlap;
	bool isLeafNode;
	GIM_AABB_TREE_NODE * pnode = aabbtree->m_node_array;

	while (curIndex < aabbtree->m_num_nodes)
	{
		//catch bugs in tree data

		AABBCOLLISION(aabbOverlap,pnode->m_data.m_aabb,(*test_aabb));
		isLeafNode = pnode->is_leaf_node();

		if (isLeafNode && aabbOverlap)
		{
		    GIM_DYNARRAY_PUSH_ITEM(GUINT,(*collided),curIndex);//Push leaf index
		}

		if (aabbOverlap || isLeafNode)
		{
			NEXT_SUBNODE(pnode,curIndex);
		}
		else
		{
		    SKIP_NODE(pnode,curIndex);
		}
	}
}


void gim_aabbtree_box_collision(aabb3f *test_aabb, GIM_AABB_TREE * aabbtree, GDYNAMIC_ARRAY * collided)
{
    GUINT curIndex = 0;

	GUINT aabbOverlap;
	bool isLeafNode;
	GIM_AABB_TREE_NODE * pnode = aabbtree->m_node_array;

	while (curIndex < aabbtree->m_num_nodes)
	{
		//catch bugs in tree data

		AABBCOLLISION(aabbOverlap,pnode->m_data.m_aabb,(*test_aabb));
		isLeafNode = pnode->is_leaf_node();

		if (isLeafNode && aabbOverlap)
		{
		    GIM_DYNARRAY_PUSH_ITEM(GUINT,(*collided),pnode->m_data.m_data);//Push leaf dataindex
		}

		if (aabbOverlap || isLeafNode)
		{
			NEXT_SUBNODE(pnode,curIndex);
		}
		else
		{
		    SKIP_NODE(pnode,curIndex);
		}
	}
}

void gim_aabbtree_ray_collision(vec3f vorigin,vec3f vdir, GREAL tmax, GIM_AABB_TREE * aabbtree, GDYNAMIC_ARRAY * collided)
{
	GUINT curIndex = 0;

	GUINT rayOverlap;
	bool isLeafNode;
	GIM_AABB_TREE_NODE * pnode = aabbtree->m_node_array;

	while (curIndex < aabbtree->m_num_nodes)
	{
		//catch bugs in tree data

		rayOverlap = BOX_INTERSECTS_RAY_FAST(pnode->m_data.m_aabb, vorigin,vdir);

		isLeafNode = pnode->is_leaf_node();

		if (isLeafNode && rayOverlap)
		{
		    GIM_DYNARRAY_PUSH_ITEM(GUINT,(*collided),pnode->m_data.m_data);//Push leaf dataindex
		}

		if (rayOverlap || isLeafNode)
		{
			NEXT_SUBNODE(pnode,curIndex);
		}
		else
		{
		    SKIP_NODE(pnode,curIndex);
		}
	}
}


static void _gim_walk_tree_duo_cache(
GIM_AABB_TREE_NODE * node0,
GIM_AABB_TREE_NODE * node1,
GIM_BOX_BOX_TRANSFORM_CACHE * trans_cache,
GDYNAMIC_ARRAY * collision_pairs, bool  full_test=true)
{
    if(gim_box_box_overlap_cache(&node0->m_data.m_aabb,
    &node1->m_data.m_aabb, trans_cache,full_test)==0) return;

    //Test children
    if(node0->is_leaf_node())
    {
        if(node1->is_leaf_node())
        {
            PUSH_PAIR(node0->m_data.m_data,node1->m_data.m_data,(*collision_pairs));
            return;
        }
        else
        {
            if(node1->m_left)
            {
				_gim_walk_tree_duo_cache(node0,node1->m_left,
					trans_cache,collision_pairs,false);
            }
            if(node1->m_right)
            {
                _gim_walk_tree_duo_cache(node0,node1->m_right,
					trans_cache,collision_pairs,false);
            }
        }
    }
    else
    {
        if(node1->is_leaf_node())
        {
            if(node0->m_left)
            {
                _gim_walk_tree_duo_cache(node0->m_left,node1,
					trans_cache,collision_pairs,false);
            }
            if(node0->m_right)
            {
                _gim_walk_tree_duo_cache(node0->m_right,node1,
					trans_cache,collision_pairs,false);
            }
        }
        else
        {
            if(node0->m_left)
            {
                if(node1->m_left)
                {
                    _gim_walk_tree_duo_cache(node0->m_left,node1->m_left,
						trans_cache,collision_pairs,false);
                }
                if(node1->m_right)
                {
                    _gim_walk_tree_duo_cache(node0->m_left,node1->m_right,
						trans_cache,collision_pairs,false);
                }
            }
            if(node0->m_right)
            {
                if(node1->m_left)
                {
                    _gim_walk_tree_duo_cache(node0->m_right,node1->m_left,
						trans_cache,collision_pairs,false);
                }
                if(node1->m_right)
                {
                    _gim_walk_tree_duo_cache(node0->m_right,node1->m_right,
						trans_cache,collision_pairs,false);
                }
            }
        }
    }// if(node0->is_leaf_node()) else
}


void gim_aabbtree_bipartite_intersections_trans(GIM_AABB_TREE * aabbtree1, GIM_AABB_TREE * aabbtree2,
mat4f trans1,mat4f trans1inverse,
mat4f trans2,mat4f trans2inverse,
 GDYNAMIC_ARRAY * collision_pairs)
{
	collision_pairs->m_size = 0;

	//Find relative transformation
    mat4f trans2to1;
    MATRIX_PRODUCT_4X4(trans2to1,trans1inverse,trans2);
	//Find Box filter
	/*aabb3f transbox1,transbox2;
	AABB_TRANSFORM(transbox1,aabbtree1->m_root_node->m_data.m_aabb,trans1);
	AABB_TRANSFORM(transbox2,aabbtree2->m_root_node->m_data.m_aabb,trans2);

	char intersected;
    AABBCOLLISION(intersected,transbox1,transbox2);
    if(intersected == 0) return;
    aabb3f filter_aabb;

	BOXINTERSECTION(transbox1,transbox2, filter_aabb);

	AABB_TRANSFORM(transbox1,filter_aabb,trans1inverse);
	AABB_TRANSFORM(transbox2,filter_aabb,trans2inverse);*/


	GIM_BOX_BOX_TRANSFORM_CACHE boxcache(trans2to1);
    //_gim_walk_tree_duo_cache(aabbtree1->m_root_node,aabbtree2->m_root_node,&boxcache,&transbox1,&transbox2,collision_pairs);
	_gim_walk_tree_duo_cache(aabbtree1->m_root_node,aabbtree2->m_root_node,
		&boxcache,collision_pairs);

}

void gim_aabbset_aabbtree_intersections_trans(GIM_AABB_SET * aabbset, GIM_AABB_TREE * aabbtree, mat4f trans_aabbtree, mat4f inv_trans_aabbtree, GDYNAMIC_ARRAY * collision_pairs)
{
    aabb3f global_box_aabbtree;

    //Calc the box aabbtree
    AABB_TRANSFORM(global_box_aabbtree,aabbtree->m_root_node->m_data.m_aabb,trans_aabbtree);

    char intersected;
    collision_pairs->m_size = 0;
    AABBCOLLISION(intersected,aabbset->m_global_bound,global_box_aabbtree);
    if(intersected == 0) return;


    aabb3f int_aabb;
    //Find  Set intersection
    BOXINTERSECTION(aabbset->m_global_bound,global_box_aabbtree, int_aabb);

    //Classify set
    GDYNAMIC_ARRAY classified_results1;
    GIM_DYNARRAY_CREATE(GUINT,classified_results1,(aabbset->m_count/4));

    _gim_aabbset_traverse_box_collision(int_aabb,aabbset, &classified_results1);

    if(classified_results1.m_size==0)
    {
        GIM_DYNARRAY_DESTROY(classified_results1);
        return;
    }

    //Test against tree
    //Collide classified
    aabb3f * paabb = aabbset->m_boxes;
    GDYNAMIC_ARRAY classified_results2;
    GIM_DYNARRAY_CREATE(GUINT,classified_results2,G_ARRAY_GROW_SIZE);

    GUINT * classified1 = GIM_DYNARRAY_POINTER(GUINT,classified_results1);
    GUINT * classified2;
    GUINT i,j;

    for (i=0;i<classified_results1.m_size ;i++)
    {
        //Test against tree
        classified_results2.m_size = 0;
        AABB_TRANSFORM(global_box_aabbtree,paabb[classified1[i]],inv_trans_aabbtree);
        gim_aabbtree_box_collision(&global_box_aabbtree, aabbtree,&classified_results2);

        classified2 = GIM_DYNARRAY_POINTER(GUINT,classified_results2);

    	for (j=0;j<classified_results2.m_size ;j++)
    	{
            PUSH_PAIR(classified1[i],classified2[j],(*collision_pairs));
    	}
    }

    GIM_DYNARRAY_DESTROY(classified_results1);
    GIM_DYNARRAY_DESTROY(classified_results2);
}

