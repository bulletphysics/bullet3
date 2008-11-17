
#include <stdio.h>
#include <stdlib.h>
#include "GPU_physics.h"
#include "clock.h"

#ifdef GPUP_CYGWIN
  typedef long long _int64;
  #define LARGEINTEGER _int64
#endif

#ifndef GPUP_WIN32
#  include <sys/time.h>
#endif

#include <time.h>

#ifdef GPUP_WIN32

double Clock::res ;
int Clock::perf_timer = -1;

void Clock::initPerformanceTimer ()
{
  if ( perf_timer == -1 )
  {
    /* Use Performance Timer if it's available, mmtimer if not.  */

    __int64 frequency ;

    perf_timer = QueryPerformanceFrequency ( (LARGE_INTEGER *) & frequency ) ;

    if ( perf_timer )
    {
      res = 1.0 / (double) frequency ;
      perf_timer = 1 ;
    }
  }
}
#endif

double Clock::getRawTime () const
{
#ifdef GPUP_WIN32

  /* Use Performance Timer if it's available, mmtimer if not.  */

  if ( perf_timer )
  {
    __int64 t ;
 
    QueryPerformanceCounter ( (LARGE_INTEGER *) &t ) ;
 
    return res * (double) t ;
  }
 
  return (double) timeGetTime() * 0.001 ;

#else
  timeval tv ;

  gettimeofday ( & tv, NULL ) ;

  return (double) tv.tv_sec + (double) tv.tv_usec / 1000000.0 ;
#endif
}


void Clock::update ()
{
  now = getRawTime() - start ;

  delta = now - last_time ;

  last_time = now ;
}



