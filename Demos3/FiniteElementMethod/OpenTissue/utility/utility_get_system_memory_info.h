#ifndef OPENTISSUE_UTILITY_UTILITY_GET_SYSTEM_MEMORY_INFO_H
#define OPENTISSUE_UTILITY_UTILITY_GET_SYSTEM_MEMORY_INFO_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#ifdef WIN32
# define NOMINMAX
# define WIN32_LEAN_AND_MEAN
# include <windows.h>
# undef WIN32_LEAN_AND_MEAN
# undef NOMINMAX
#endif

namespace OpenTissue
{
  namespace utility
  {

#ifdef WIN32

    double get_memory_usage()
    {
      typedef double real_type;
      MEMORYSTATUSEX statex;
      statex.dwLength = sizeof (statex);
      GlobalMemoryStatusEx (&statex);
      real_type memory_usage = statex.dwMemoryLoad;
      return memory_usage;
    }

    unsigned int get_total_physical_memory_in_bytes()
    {
      MEMORYSTATUSEX statex;
      statex.dwLength = sizeof (statex);
      GlobalMemoryStatusEx (&statex);
      unsigned int total_physical_memory_in_bytes = statex.ullTotalPhys;
      return total_physical_memory_in_bytes;
    }

    unsigned int get_free_physical_memory_in_bytes()
    {
      MEMORYSTATUSEX statex;
      statex.dwLength = sizeof (statex);
      GlobalMemoryStatusEx (&statex);
      unsigned int free_physical_memory_in_bytes = statex.ullAvailPhys;
      return free_physical_memory_in_bytes;
    }

    unsigned int get_total_page_file_in_bytes()
    {
      MEMORYSTATUSEX statex;
      statex.dwLength = sizeof (statex);
      GlobalMemoryStatusEx (&statex);
      unsigned int total_page_file_in_bytes = statex.ullTotalPageFile;
      return total_page_file_in_bytes;
    }

    unsigned int get_free_page_file_in_bytes()
    {
      MEMORYSTATUSEX statex;
      statex.dwLength = sizeof (statex);
      GlobalMemoryStatusEx (&statex);
      unsigned int free_page_file_in_bytes = statex.ullAvailPageFile;
      return free_page_file_in_bytes;
    }

    unsigned int get_total_virtual_memory_in_bytes()
    {
      MEMORYSTATUSEX statex;
      statex.dwLength = sizeof (statex);
      GlobalMemoryStatusEx (&statex);
      unsigned int total_virtual_memory = statex.ullTotalVirtual;
      return total_virtual_memory;
    }

    unsigned int get_free_virtual_memory_in_bytes()
    {
      MEMORYSTATUSEX statex;
      statex.dwLength = sizeof (statex);
      GlobalMemoryStatusEx (&statex);
      unsigned int free_virtual_memory_in_bytes = statex.ullAvailVirtual;
      return free_virtual_memory;
    }

    unsigned int get_free_extended_memory_in_bytes()
    {
      MEMORYSTATUSEX statex;
      statex.dwLength = sizeof (statex);
      GlobalMemoryStatusEx (&statex);
      unsigned int free_extended_memory_in_bytes = statex.ullAvailExtendedVirtual;
      return free_extended_memory_in_bytes;
    }


#endif

  } //End of namespace utility

} //End of namespace OpenTissue

// OPENTISSUE_UTILITY_UTILITY_GET_SYSTEM_MEMORY_INFO_H
#endif
