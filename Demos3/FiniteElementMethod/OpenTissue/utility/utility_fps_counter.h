#ifndef OPENTISSUE_UTILITY_UTILITY_FPS_COUNTER_H
#define OPENTISSUE_UTILITY_UTILITY_FPS_COUNTER_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

#include <OpenTissue/utility/utility_timer.h>

namespace OpenTissue
{
  namespace utility
  {
    /*
    * Frame Per Second (FPS) Counter
    */
    template<typename real_type>
    class FPSCounter
    {
    public:
      FPSCounter()
        : m_fps(0)
        , m_fpscnt(0)
        , m_time(0.)
      {
        m_hrc.start();
      }

      /*
      * /return  the last known fps
      */
      unsigned long operator()() const
      {
        return m_fps;

      }

      /*
      * /return  true if 1 sec has passed
      */
      bool frame()
      {
        m_hrc.stop();
        m_time += m_hrc();
        m_fpscnt++;
        if (m_time >= 1.) {
          //        m_time -= 1.;
          m_time = 0.;
          m_fps = m_fpscnt;
          m_fpscnt = 0;
        }
        m_hrc.start();
        return 0 == m_fpscnt;
      }

    private:

      Timer<real_type>  m_hrc;
      unsigned long  m_fps;
      unsigned long  m_fpscnt;
      double  m_time;
    };

  } //End of namespace utility

} //End of namespace OpenTissue

// OPENTISSUE_UTILITY_UTILITY_FPS_COUNTER_H
#endif
