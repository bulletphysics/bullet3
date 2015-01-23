#ifndef OPENTISSUE_UTILITY_UTILITY_QHULL_H
#define OPENTISSUE_UTILITY_UTILITY_QHULL_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#include <OpenTissue/configuration.h>

  //////////////////////////////////////////////////////////////////
  //
  // This is a bit tricky, I cant just include qhull_a.h as
  // QHull advices, because MVC complains about math.h which
  // is included "indirectly by qhull_a.h
  //
  // Hopefully furture releases of QHull will allow me to
  // just write:
  //
  //  extern "C"
  //  {
  //    #include <Qhull/qhull_a.h>
  //  }
  //
#if defined(__cplusplus)
  extern "C"
  {
#endif
#include <stdio.h>
#include <stdlib.h>
#include <qhull/qhull.h>
#include <qhull/mem.h>
#include <qhull/qset.h>
#include <qhull/geom.h>
#include <qhull/merge.h>
#include <qhull/poly.h>
#include <qhull/io.h>
#include <qhull/stat.h>
#if defined(__cplusplus)
  }
#endif

//OPENTISSUE_UTILITY_UTILITY_QHULL_H
#endif 
