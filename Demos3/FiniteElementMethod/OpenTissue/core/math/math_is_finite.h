#ifndef OPENTISSUE_CORE_MATH_IS_FINITE_H
#define OPENTISSUE_CORE_MATH_IS_FINITE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#ifdef WIN32
#include <float.h>
#endif

namespace OpenTissue
{
  namespace math
  {

#ifdef WIN32
#define is_finite(val) (_finite(val)!=0)  ///< Is finite number test
#else
#define is_finite(val) (finite(val)!=0)  ///< Is finite number test
#endif

  } // namespace math

} // namespace OpenTissue

//OPENTISSUE_CORE_MATH_IS_FINITE_H
#endif
