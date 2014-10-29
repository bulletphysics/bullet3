#ifndef OPENTISSUE_CORE_MATH_MATH_BASIC_TYPES_H
#define OPENTISSUE_CORE_MATH_MATH_BASIC_TYPES_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_vector3.h>
#include <OpenTissue/core/math/math_matrix3x3.h>
#include <OpenTissue/core/math/math_quaternion.h>
#include <OpenTissue/core/math/math_dual_quaternion.h>
#include <OpenTissue/core/math/math_coordsys.h>
#include <OpenTissue/core/math/math_value_traits.h>

namespace OpenTissue
{
  namespace math
  {

    // TODO add doxygen documentation explaining rationale behind this typebinder class. Why have we made it, and for what purpose? Also give example usages
    template< typename real_type_
            , typename index_type_
            >
    class BasicMathTypes
    {
    public:

      typedef real_type_                  real_type;
      typedef index_type_                 index_type;
      typedef Vector3<real_type>          vector3_type;
      typedef Quaternion<real_type>       quaternion_type;
      typedef DualQuaternion<real_type>   dual_quaternion_type;
      typedef Matrix3x3<real_type>        matrix3x3_type;
      typedef CoordSys<real_type>         coordsys_type;

      typedef Vector3<index_type>         index_vector3_type;
      typedef ValueTraits<real_type>      value_traits;
    };


    // TODO add doxygen comments explaining what this type is useful for
    typedef BasicMathTypes<double,unsigned int> default_math_types;

  } // namespace math
} // namespace OpenTissue

//OPENTISSUE_CORE_MATH_MATH_BASIC_TYPES_H
#endif
