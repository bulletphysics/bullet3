#ifndef OPENTISSUE_UTILITY_UTILITY_GET_ENVIRONMENT_VARIABLE_H
#define OPENTISSUE_UTILITY_UTILITY_GET_ENVIRONMENT_VARIABLE_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <stdexcept>

#include <cstdlib>  //--- For getenv
#include <string>
#include <cassert>

namespace OpenTissue
{
  namespace utility
  {

    /**
    * Retrieve Value of Environment Variable.
    * If environment variable can not be found then a std::logic_error warning is thrown.
    *
    * @param    A string containing the name of the environment
    *           variable. Example: "OPENTISSUE".
    * @return   A string containing the value.
    */
    inline std::string get_environment_variable(std::string const & name)
    {
      char * value = getenv( name.c_str() );
      if(value)
        return std::string( value );
      throw std::logic_error("get_environment_variable(): the specified environment variable was not set");
    }

  } // namespace utility

} // namespace OpenTissue

//OPENTISSUE_UTILITY_UTILITY_GET_ENVIRONMENT_VARIABLE_H
#endif
