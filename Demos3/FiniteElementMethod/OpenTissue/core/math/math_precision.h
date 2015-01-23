#ifndef OPENTISSUE_CORE_MATH_MATH_PRECISION_H
#define OPENTISSUE_CORE_MATH_MATH_PRECISION_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <limits>  // for std::numeric_limits<T>::epsilon()


namespace OpenTissue
{

  namespace math
  {
    template <typename T>
    inline T machine_precision()
    {
      return std::numeric_limits<T>::epsilon();
    }

    template <typename T>
    inline T working_precision()
    {
      return std::numeric_limits<T>::epsilon()*10;
    }

    template <typename T>
    inline T working_precision(unsigned int scale_factor)
    {
      return std::numeric_limits<T>::epsilon()*scale_factor;
    }

  }
}

//OPENTISSUE_CORE_MATH_MATH_PRECISION_H
#endif
