#ifndef	OPENTISSUE_UTILITY_UTILITY_LESS_PTR_H
#define	OPENTISSUE_UTILITY_UTILITY_LESS_PTR_H
//
// OpenTissue, A toolbox for physical based	simulation and animation.
// Copyright (C) 2007 Department of	Computer Science, University of	Copenhagen
//
#include <OpenTissue/configuration.h>

#include <functional>

namespace OpenTissue
{
  namespace utility
  {

    /**
    * Less Than Test.
    * This is used for doing a less than test on data
    * structures containing pointers to data and not instances.
    */
    template<class T>
    struct less_ptr 
      : public std::binary_function<T, T, bool>
    {
      bool operator()(const T& x, const T& y) const  {  return (*x)<(*y);  }
    };

  } //End of namespace utility

} //End of namespace OpenTissue

// OPENTISSUE_UTILITY_UTILITY_LESS_PTR_H
#endif
