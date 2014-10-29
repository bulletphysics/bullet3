#ifndef OPENTISSUE_UTILITY_UTILITY_RUNTIME_TYPE_H
#define OPENTISSUE_UTILITY_UTILITY_RUNTIME_TYPE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

namespace OpenTissue
{
  namespace utility
  {
    /**
    * Runtime type.
    * great for passing non-integral values as template arguments.
    */
    template < typename Type >
    struct RuntimeType
    {
      typedef Type  type;
      RuntimeType(){}
      RuntimeType(const RuntimeType& rhs):m_value(rhs.m_value){}
      RuntimeType& operator = (const type& rhs) {m_value=rhs; return *this;}
      operator const type& () const {return m_value;}
    protected:
      type  m_value;
    };

  } // namespace utility

} // namespace OpenTissue

// OPENTISSUE_UTILITY_UTILITY_RUNTIME_TYPE_H
#endif
