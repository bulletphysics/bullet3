#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_TRANSLATE_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_TRANSLATE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <boost/iterator/reverse_iterator.hpp>

#include <cstdlib> // for abs()

namespace OpenTissue
{
  namespace grid
  {
    /**
    * Translate a grid by an integer vector and wrap around at border.
    *
    * Behavior is un-expected if input and output grid is the same....
    * TODO: make it work for zero-trans and negative trans.
    *
    * @param src   Source grid to be translated.
    * @param v     translation vector.
    * @param dst   Upon return, contains the translated grid.
    */
    template <typename grid_type, typename vector3_int_type>
    inline void translate(grid_type const& src, vector3_int_type const& v, grid_type & dst)
    {
      assert( v(0)>0 && !"translate(): Only works for positive translations for now");
      assert( v(1)>0 && !"translate(): Only works for positive translations for now");
      assert( v(2)>0 && !"translate(): Only works for positive translations for now");

      typedef typename grid_type::const_index_iterator  const_index_iterator;
      typedef typename grid_type::math_types            math_types;
      typedef typename math_types::vector3_type         vector3_type;

      // make sure translation is >0
      vector3_int_type t=v;
      //if( t(0)<0 )
      //  t(0) += src.I()*(1+abs(t(0)/src.I()));
      //if( t(1)<0 )
      //  t(1) += src.J()*(1+abs(t(1)/src.J()));
      //if( t(2)<0 )
      //  t(2) += src.K()*(1+abs(t(2)/src.K()));

      // Save last block of data
      grid_type tmp;
      tmp.create(vector3_type(0,0,0), vector3_type(1,1,1), t(0), t(1), t(2));
      size_t ti = src.I()-t(0);
      size_t tj = src.J()-t(1);
      size_t tk = src.K()-t(2);
      size_t lk=0;
      size_t lj=0;
      size_t li=0;
      for (size_t gk=tk; gk<src.K(); ++gk, ++lk)
        for (size_t gj=tj; gj<src.J(); ++gj, ++lj)
          for (size_t gi=ti; gi<src.I(); ++gi, ++li)
            tmp(li,lj,lk) = src(gi,gj,gk);

      // do translation (copy backwards)
      // TODO: implement proper reverse_iterator_stuff
      //boost::reverse_iterator<grid_type::const_index_iterator> p( src.end() ), end( src.begin() );
      //for(p; p!=end; ++p)
      //{
      //  dst( (p.base().i()+t(0))%src.I(), (p.base().j()+t(1))%src.J(), (p.base().k()+t(2))%src.K() )  = *p;
      //}
      for(const_index_iterator p=src.end()-1; p!=src.begin()-1; --p)
      {
        dst( (p.i()+t(0))%src.I(), (p.j()+t(1))%src.J(), (p.k()+t(2))%src.K() )  = *p;
      }
      // copy over the stored values
      for (size_t k=0; k<tmp.K(); ++k)
        for (size_t j=0; j<tmp.J(); ++j)
          for (size_t i=0; i<tmp.I(); ++i)
            dst(i,j,k) = tmp(i,j,k);
    }

  } // namespace grid

} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_TRANSLATE_H
#endif
