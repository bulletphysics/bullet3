#ifndef OPENTISSUE_CORE_MATH_IS_NUMBER_H
#define OPENTISSUE_CORE_MATH_IS_NUMBER_H
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
#else
#include <cmath>
#endif

namespace OpenTissue
{
  namespace math
  {

#ifdef WIN32
#define is_number(val) (_isnan(val)==0)   ///< Is a number test
#else 
#if (__APPLE__)
#define is_number(val) (std::isnan(val)==0)   ///< Is a number test
#else
#define is_number(val) (isnan(val)==0)   ///< Is a number test
#endif
#endif

  } // namespace math

} // namespace OpenTissue

//OPENTISSUE_CORE_MATH_IS_NUMBER_H
#endif
