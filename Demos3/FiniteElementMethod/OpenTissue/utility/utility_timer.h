#ifndef OPENTISSUE_UTILITY_UTILITY_TIMER_H
#define OPENTISSUE_UTILITY_UTILITY_TIMER_H
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
#else
# include<sys/time.h>
#endif

#include <cassert>

namespace OpenTissue
{
  namespace utility
  {

    /**
    * High Resoultion Timer.
    * Based on http://www-106.ibm.com/developerworks/library/l-rt1/
    *
    * RunTime: High-performance programming techniques on Linux and Windows 2000
    * Setting up timing routines
    *   by
    * Edward G. Bradford (egb@us.ibm.com)
    * Senior Programmer, IBM
    * 01 Apr 2001
    *
    * Example usage (We recommand doubles):
    *
    *  Timer<double> timer;
    *
    *  timer.start()
    *  ...
    *  timer.stop()
    *  std::cout << "It took " << timer() << " seconds to do it" << std::endl;
    */
    template<typename real_type>
    class Timer
    {
#ifdef WIN32
    private:
      LARGE_INTEGER m_start;   ///<
      LARGE_INTEGER m_end;     ///<
      LARGE_INTEGER m_freq;    ///<
      bool m_first;            ///<
    public:
      Timer():m_first(true){}
    public:
      void start()
      {
        if(m_first)
        {
          QueryPerformanceFrequency(&m_freq);
          m_first = false;
        }
        QueryPerformanceCounter(&m_start);
      }
      void stop()
      {
        QueryPerformanceCounter(&m_end);
      }
      real_type operator()()const
      {
        real_type end = static_cast<real_type>(m_end.QuadPart);
        real_type start = static_cast<real_type>(m_start.QuadPart);
        real_type freq = static_cast<real_type>(m_freq.QuadPart);
        return (end - start)/ freq;
      }
#else
    private:
      struct timeval m_start;   ///<
      struct timeval m_end;     ///<
      struct timezone m_tz;     ///<
    public:
      void start() { gettimeofday(&m_start, &m_tz); }
      void stop()  { gettimeofday(&m_end,&m_tz); }
      real_type operator()()const
      {
        real_type t1 =  static_cast<real_type>(m_start.tv_sec) + static_cast<real_type>(m_start.tv_usec)/(1000*1000);
        real_type t2 =  static_cast<real_type>(m_end.tv_sec) + static_cast<real_type>(m_end.tv_usec)/(1000*1000);
        return t2-t1;
      }
#endif
    };

  } //End of namespace utility
} //End of namespace OpenTissue

// OPENTISSUE_UTILITY_UTILITY_TIMER_H
#endif
