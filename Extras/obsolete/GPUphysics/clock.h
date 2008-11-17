
/*
  High precision clocks.
*/

class Clock
{
  double start ;
  double now   ;
  double delta ;
  double last_time ;
  double max_delta ;
  
#ifdef GPUP_WIN32
  static double res ;
  static int perf_timer ;
  void initPerformanceTimer () ;
#endif

  double getRawTime () const ;

public:

  Clock () { reset () ; }

  void reset ()
  {
#ifdef GPUP_WIN32
    initPerformanceTimer () ;
#endif
    start     = getRawTime () ;
    now       = 0.0 ;
    max_delta = 0.2 ; 
    delta     = 0.0000001 ;  /* Faked so stoopid programs won't div0 */
    last_time = 0.0 ;
  }

  void   setMaxDelta  ( double maxDelta ) { max_delta = maxDelta ; }
  double getMaxDelta  () const { return max_delta ; }
  void   update       () ;
  double getAbsTime   () const { return now   ; }
  double getDeltaTime () const { return delta ; }
  double getFrameRate () const { return 1.0 / delta ; }
} ;

