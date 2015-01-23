#ifndef OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_APPROXIMATE_GAUSSIAN_FILTER_H
#define OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_APPROXIMATE_GAUSSIAN_FILTER_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/containers/grid/util/grid_box_filter.h>
#include <OpenTissue/core/containers/grid/util/grid_translate.h>

namespace OpenTissue
{
  namespace grid
  {
    /**
    * Fast convolution by an approximate "integer" guassian filter.
    * This just applies several box_filter's (boxsize=size and order times),
    * and is only efficient for low orders.
    * If size is even, the order must also be even for the resulting image to lay on the same grid
    * (else translated 0.5 voxel towards (0,0,0) ).
    * @param src   Source grid to be convolved.
    * @param order Order of filter. Corresponds to the number of times the boxfilter is applied.
    * @param size  Size of filter.
    * @param dst   Upon return, contains the filtered grid.
    */
    template <class grid_type>
    inline void approximate_gaussian_filter(grid_type const& src, size_t order, size_t size, grid_type& dst)
    {
      typedef typename grid_type::index_vector index_vector;
      dst=src;
      for(size_t i=0; i<order; ++i)
      {
        box_filter(dst,size,dst);
        // recenter image by 1 voxel for every second application of the box_filter when the size is odd
        if (i>0 && (i%2) && !(size%2))
        {
          std::cout << "--calling translate()" << std::endl;
          translate(dst, index_vector(1,1,1), dst);
        }
      }
    }

    /**
    * Approximate Gaussian filter with compensation on image borders.
    * Attenuation on border regions is compensated.
    * This is equivalent to Gaussian filtering using only filter coefficients
    * that are inside the "shape" being considered.
    * @param src   Source grid to be convolved.
    * @param order Order of filter. Corresponds to the number of times the boxfilter is applied.
    * @param size  Size of filter.
    * @param dst   Upon return, contains the filtered grid.
    */
    template <class grid_type>
    inline void approximate_gaussian_filter_border_correct(grid_type const& src, size_t order, size_t size, grid_type &dst)
    {
      grid_type lowFreq=src;

      // apply filter in normal fashion
      approximate_gaussian_filter(lowFreq, order, size, lowFreq);

      // compensate for border regions in filtering
      // FIXME: refactor this!
      /*    {
      Image3Df smask(src.Size());
      smask.Fill(1);
      approximate_gaussian_filter(smask,order,size,smask);
      for(typename grid_type::index_iterator p=lowFreq.begin(); p!=lowFreq.end(); ++p)
      {
      if(smask(p.Pos()))
      {
      *p/=smask(p.Pos());
      }
      }
      }*/
      dst=lowFreq;
    }

  } // namespace grid
} // namespace OpenTissue

// OPENTISSUE_CORE_CONTAINERS_GRID_UTIL_GRID_APPROXIMATE_GAUSSIAN_FILTER_H
#endif
