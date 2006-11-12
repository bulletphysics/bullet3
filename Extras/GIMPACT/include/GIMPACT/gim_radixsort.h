#ifndef GIM_RADIXSORT_H_INCLUDED
#define GIM_RADIXSORT_H_INCLUDED
/*! \file gim_radixsort.h
\author Francisco León Nájera.
Based on the work of Michael Herf : "fast floating-point radix sort"
Avaliable on http://www.stereopsis.com/radix.html
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

#include "GIMPACT/gim_memory.h"

/*! \defgroup SORTING
\brief
Macros for sorting.
*/
//! @{
struct GIM_RSORT_TOKEN
{
    GUINT m_key;
    GUINT m_value;
};
//typedef struct _GIM_RSORT_TOKEN GIM_RSORT_TOKEN;

//comparator for sorting
#define RSORT_TOKEN_COMPARATOR(x, y) ((int)((x.m_key) - (y.m_key)))

// ---- utils for accessing 11-bit quantities
#define D11_0(x)	(x & 0x7FF)
#define D11_1(x)	(x >> 11 & 0x7FF)
#define D11_2(x)	(x >> 22 )


//COMMON FUNCTIONS FOR ACCESSING THE KEY OF AN ELEMENT


//For the type of your array, you need to declare a macro for obtaining the key, like these:
#define SIMPLE_GET_FLOAT32KEY(e,key) {key =(GREAL)(e);}

#define SIMPLE_GET_INTKEY(e,key) {key =(GINT)(e);}

#define SIMPLE_GET_UINTKEY(e,key) {key =(GUINT)(e);}

//For the type of your array, you need to declare a macro for copy elements, like this:

#define SIMPLE_COPY_ELEMENTS(dest,src) {dest = src;}

#define kHist 2048

///Radix sort for unsigned integer keys

#define GIM_RADIX_SORT_RTOKENS(array,sorted,element_count)\
{\
	GUINT i;\
	GUINT b0[kHist * 3];\
	GUINT *b1 = b0 + kHist;\
	GUINT *b2 = b1 + kHist;\
	for (i = 0; i < kHist * 3; i++)\
	{\
		b0[i] = 0;\
	}\
	GUINT fi;\
	GUINT pos;\
	for (i = 0; i < element_count; i++)\
	{\
	    fi = array[i].m_key;\
		b0[D11_0(fi)] ++;\
		b1[D11_1(fi)] ++;\
		b2[D11_2(fi)] ++;\
	}\
	{\
		GUINT sum0 = 0, sum1 = 0, sum2 = 0;\
		GUINT tsum;\
		for (i = 0; i < kHist; i++)\
		{\
			tsum = b0[i] + sum0;\
			b0[i] = sum0 - 1;\
			sum0 = tsum;\
			tsum = b1[i] + sum1;\
			b1[i] = sum1 - 1;\
			sum1 = tsum;\
			tsum = b2[i] + sum2;\
			b2[i] = sum2 - 1;\
			sum2 = tsum;\
		}\
	}\
	for (i = 0; i < element_count; i++)\
	{\
        fi = array[i].m_key;\
		pos = D11_0(fi);\
		pos = ++b0[pos];\
		sorted[pos].m_key = array[i].m_key;\
		sorted[pos].m_value = array[i].m_value;\
	}\
	for (i = 0; i < element_count; i++)\
	{\
        fi = sorted[i].m_key;\
		pos = D11_1(fi);\
		pos = ++b1[pos];\
		array[pos].m_key = sorted[i].m_key;\
		array[pos].m_value = sorted[i].m_value;\
	}\
	for (i = 0; i < element_count; i++)\
	{\
        fi = array[i].m_key;\
		pos = D11_2(fi);\
		pos = ++b2[pos];\
		sorted[pos].m_key = array[i].m_key;\
		sorted[pos].m_value = array[i].m_value;\
	}\
}\

/// Get the sorted tokens from an array. For generic use. Tokens are GIM_RSORT_TOKEN
#define GIM_RADIX_SORT_ARRAY_TOKENS(array, sorted_tokens, element_count, get_uintkey_macro)\
{\
    GIM_RSORT_TOKEN * _unsorted = (GIM_RSORT_TOKEN *) gim_alloc(sizeof(GIM_RSORT_TOKEN )*element_count);\
    GUINT _i;\
    for (_i=0;_i<element_count;_i++)\
    {\
        get_uintkey_macro(array[_i],_unsorted[_i].m_key);\
        _unsorted[_i].m_value = _i;\
    }\
    GIM_RADIX_SORT_RTOKENS(_unsorted,sorted_tokens,element_count);\
    gim_free(_unsorted,sizeof(GIM_RSORT_TOKEN )*element_count);\
}\

/// Sorts array in place. For generic use
#define GIM_RADIX_SORT(type,array,element_count,get_uintkey_macro,copy_elements_macro)\
{\
    GIM_RSORT_TOKEN * _sorted = (GIM_RSORT_TOKEN *) gim_alloc(sizeof(GIM_RSORT_TOKEN )*element_count);\
    GIM_RADIX_SORT_ARRAY_TOKENS(array,_sorted,element_count,get_uintkey_macro);\
    type * _original_array = (type *) gim_alloc(sizeof(type)*element_count); \
    memcpy(_original_array,array,sizeof(type)*element_count);\
    GUINT _i;\
    for (_i=0;_i<element_count;_i++)\
    {\
        copy_elements_macro(array[_i],_original_array[_sorted[_i].m_value]);\
    }\
    gim_free(_original_array,sizeof(type)*element_count);\
    gim_free(_sorted,sizeof(GIM_RSORT_TOKEN )*element_count);\
}\

/// Sorts array in place using quick sort
#define GIM_QUICK_SORT_ARRAY(type, array, array_count, comp_macro, exchange_macro) \
{\
  GINT   _i_, _j_, _p_, _stack_index_, _start_, _end_;\
  GINT   _start_stack_[64]; \
  GINT   _end_stack_[64];\
  _start_stack_[0] = 0;\
  _end_stack_[0] = (array_count);\
  _stack_index_ = 1;\
  while (_stack_index_ > 0)\
  {\
    _stack_index_ --;\
    _start_ = _start_stack_[_stack_index_];\
    _end_ = _end_stack_[_stack_index_];\
    while (_end_ - _start_ > 2)\
    {\
      _p_ = _start_;\
      _i_ = _start_ + 1;\
      _j_ = _end_ - 1;\
      while (_i_<_j_) \
      {\
        for(; _i_<=_j_ && comp_macro(((array)[_i_]),((array)[_p_]))<=0; _i_++) ;\
        if (_i_ > _j_) \
        {\
          exchange_macro(type, array, _j_, _p_);\
          _i_ = _j_;\
        }\
        else\
        {\
          for(; _i_<=_j_ && comp_macro(((array)[_j_]),((array)[_p_]))>=0; _j_--) ;\
          if (_i_ > _j_) \
          {\
            exchange_macro(type, array, _j_, _p_);\
            _i_ = _j_;\
          }\
          else if (_i_ < _j_)\
          {\
            exchange_macro(type, array, _i_, _j_);\
            if (_i_+2 < _j_) {_i_++; _j_--;}\
            else if (_i_+1 < _j_) _i_++;\
          }\
        }\
      }\
      if (_i_-_start_ > 1 && _end_-_j_ > 1) \
      {\
        if (_i_-_start_ < _end_-_j_-1) \
        {\
          _start_stack_[_stack_index_] = _j_+1;\
          _end_stack_[_stack_index_] = _end_;\
          _stack_index_ ++;\
          _end_ = _i_;\
        }\
        else\
        {\
          _start_stack_[_stack_index_] = _start_;\
          _end_stack_[_stack_index_] = _i_;\
          _stack_index_ ++;\
          _start_ = _j_+1;\
        }\
      }\
      else\
      {\
        if (_i_-_start_ > 1)\
        {\
          _end_ = _i_;\
        }\
        else \
        {\
          _start_ = _j_+1;\
        }\
      }\
    }\
    if (_end_ - _start_ == 2) \
    {\
      if (comp_macro(((array)[_start_]),((array)[_end_-1])) > 0) \
      {\
        exchange_macro(type, array, _start_, _end_-1);\
      }\
    }\
  }\
}\

#define GIM_DEF_EXCHANGE_MACRO(type, _array, _i, _j)\
{\
    type _e_tmp_ =(_array)[(_i)];\
    (_array)[(_i)]=(_array)[(_j)];\
    (_array)[(_j)]= _e_tmp_;\
}\

#define GIM_COMP_MACRO(x, y) ((GINT)((x) - (y)))

//! Iterative binary search
/*!
\param _array
\param _start_i the beginning of the array
\param _end_i the ending  index of the array
\param _search_key Value to find
\param _comp_macro macro for comparing elements
\param _found If 1 the value has found
\param _result_index the index of the found element, or if not found then it will get the index of the  closest bigger value
*/
#define GIM_BINARY_SEARCH(_array, _start_i, _end_i, _search_key, _comp_macro, _found, _result_index)\
{\
  GUINT _k, _comp_result, _i, _j;\
  _i = _start_i; \
  _j = _end_i+1;\
  _found = 0;\
  while ((_i < _j) &&( _found==0)) \
  {\
    _k = (_j+_i-1)/2;\
    _comp_result = _comp_macro(_array[_k], _search_key);\
    if (_comp_result == 0) \
    {\
      _result_index = _k;    \
      _found = 1;\
    }\
    else if (_comp_result < 0) \
    {\
      _i = _k+1;\
    } \
    else\
    {\
       _j = _k;\
    }\
  }\
  if (_found == 0) \
  {\
    _result_index = _j;\
  }\
}\


//! @}
#endif // GIM_RADIXSORT_H_INCLUDED
