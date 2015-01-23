#ifndef OPENTISSUE_CORE_MATH_MATH_VALUE_TRAITS_H
#define OPENTISSUE_CORE_MATH_MATH_VALUE_TRAITS_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/core/math/math_constants.h>


namespace OpenTissue
{

  namespace math
  {

    template <typename T>
    class ValueTraits
    {
    public:

      static T zero()      { return detail::zero<T>();      }
      static T one()       { return detail::one<T>();       }
      static T two()       { return detail::two<T>();       }
      static T three()     { return detail::three<T>();     }
      static T four()      { return detail::four<T>();      }
      static T eight()     { return detail::eight<T>();     }
      static T half()      { return detail::half<T>();      }
      static T pi()        { return detail::pi<T>();        }
      static T pi_2()      { return detail::pi_half<T>();   }
      static T pi_half()   { return detail::pi_half<T>();   }
      static T pi_quarter(){ return detail::pi_quarter<T>();}
      static T pi_4()      { return detail::pi_quarter<T>();}
      static T degree()    { return detail::degree<T>();    }
      static T radian()    { return detail::radian<T>();    }

    };

  } // namespace math
 
}  // namespace OpenTissue

//OPENTISSUE_CORE_MATH_MATH_VALUE_TRAITS_H
#endif
