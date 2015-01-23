#ifndef OPENTISSUE_CONFIGURATIOM_H
#define OPENTISSUE_CONFIGURATIOM_H
//
// OpenTissue Template Library
// - A generic toolbox for physics-based modeling and simulation.
// Copyright (C) 2008 Department of Computer Science, University of Copenhagen.
//
// OTTL is licensed under zlib: http://opensource.org/licenses/zlib-license.php
//
#if (_MSC_VER >= 1200)
# pragma once
# pragma warning(default: 56 61 62 191 263 264 265 287 289 296 347 529 686)
# pragma warning(disable: 503)
#endif

#ifdef WIN32
#  define WIN32_LEAN_AND_MEAN
#  define _USE_MATH_DEFINES
#  define NOMINMAX
#  include <windows.h>
#  undef WIN32_LEAN_AND_MEAN
#  undef NOMINMAX
#endif


/**
 * OpenTissue Version
 */
#define OPENTISSUE_VERSION        0.994
#define OPENTISSUE_VERSION_MAJOR  0
#define OPENTISSUE_VERSION_MINOR  994

#include <string>

/**
 * OpenTissue Path.
 * This is the path where OpenTissue was copied onto ones
 * system. It can be used to locate shader programs or data resources.
 */
std::string const opentissue_path = "F:/develop/opentissue/sandbox/";

/**
 * OpenTissue Version String.
 * This string value can be used by end users for compatibility testing.
 */
std::string const opentissue_version = "0.994";


//OPENTISSUE_CONFIGURATIOM_H
#endif
