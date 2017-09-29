# - Find python libraries
# This module finds if Python is installed and determines where the
# include files and libraries are. It also determines what the name of
# the library is. This code sets the following variables:
#
#  PYTHONLIBS_FOUND           - have the Python libs been found
#  PYTHON_LIBRARIES           - path to the python library
#  PYTHON_INCLUDE_PATH        - path to where Python.h is found (deprecated)
#  PYTHON_INCLUDE_DIRS        - path to where Python.h is found
#  PYTHON_DEBUG_LIBRARIES     - path to the debug library (deprecated)
#  PYTHONLIBS_VERSION_STRING  - version of the Python libs found (since CMake 2.8.8)
#
# The Python_ADDITIONAL_VERSIONS variable can be used to specify a list of
# version numbers that should be taken into account when searching for Python.
# You need to set this variable before calling find_package(PythonLibs).
#
# If you'd like to specify the installation of Python to use, you should modify
# the following cache variables:
#  PYTHON_LIBRARY             - path to the python library
#  PYTHON_INCLUDE_DIR         - path to where Python.h is found

#=============================================================================
# Copyright 2001-2009 Kitware, Inc.
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# (To distribute this file outside of CMake, substitute the full
#  License text for the above reference.)

# Note by Nikolaus Demmel 28.03.2014: My contributions are licensend under the
# same as CMake (BSD). My adaptations are in part based
# https://github.com/qgis/QGIS/tree/master/cmake which has the following
# copyright note:

# Copyright (c) 2007, Simon Edwards <simon@simonzone.com>
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.


if(NOT DEFINED PYTHON_INCLUDE_DIR)
  if(DEFINED PYTHON_INCLUDE_PATH)
    # For backward compatibility, repect PYTHON_INCLUDE_PATH.
    set(PYTHON_INCLUDE_DIR "${PYTHON_INCLUDE_PATH}" CACHE PATH
      "Path to where Python.h is found" FORCE)
  else()
    set(PYTHON_INCLUDE_DIR "" CACHE PATH
      "Path to where Python.h is found" FORCE)
  endif()
endif()

if(EXISTS "${PYTHON_INCLUDE_DIR}" AND EXISTS "${PYTHON_LIBRARY}")
  if(EXISTS "${PYTHON_INCLUDE_DIR}/patchlevel.h")
    file(STRINGS "${PYTHON_INCLUDE_DIR}/patchlevel.h" _PYTHON_VERSION_STR
      REGEX "^#define[ \t]+PY_VERSION[ \t]+\"[^\"]+\"")
    string(REGEX REPLACE "^#define[ \t]+PY_VERSION[ \t]+\"([^\"]+)\".*" "\\1"
      PYTHONLIBS_VERSION_STRING "${_PYTHON_VERSION_STR}")
    unset(_PYTHON_VERSION_STR)
  endif()
else()
  set(_PYTHON1_VERSIONS 1.6 1.5)
  set(_PYTHON2_VERSIONS 2.7 2.6 2.5 2.4 2.3 2.2 2.1 2.0)
  set(_PYTHON3_VERSIONS 3.6 3.5 3.4 3.3 3.2 3.1 3.0)

  unset(_PYTHON_FIND_OTHER_VERSIONS)
  if(PythonLibs_FIND_VERSION)
    if(PythonLibs_FIND_VERSION_COUNT GREATER 1)
      set(_PYTHON_FIND_MAJ_MIN "${PythonLibs_FIND_VERSION_MAJOR}.${PythonLibs_FIND_VERSION_MINOR}")
      if(NOT PythonLibs_FIND_VERSION_EXACT)
        foreach(_PYTHON_V ${_PYTHON${PythonLibs_FIND_VERSION_MAJOR}_VERSIONS})
          if(NOT _PYTHON_V VERSION_LESS _PYTHON_FIND_MAJ_MIN)
            if(NOT _PYTHON_V STREQUAL PythonLibs_FIND_VERSION)
              list(APPEND _PYTHON_FIND_OTHER_VERSIONS ${_PYTHON_V})
            endif()
          endif()
        endforeach()
      endif()
      unset(_PYTHON_FIND_MAJ_MIN)
    else()
      set(_PYTHON_FIND_OTHER_VERSIONS ${_PYTHON${PythonLibs_FIND_VERSION_MAJOR}_VERSIONS})
    endif()
  else()
    # add an empty version to check the `python` executable first in case no version is requested
    set(_PYTHON_FIND_OTHER_VERSIONS ${_PYTHON3_VERSIONS} ${_PYTHON2_VERSIONS} ${_PYTHON1_VERSIONS})
  endif()

  unset(_PYTHON1_VERSIONS)
  unset(_PYTHON2_VERSIONS)
  unset(_PYTHON3_VERSIONS)

  # Set up the versions we know about, in the order we will search. Always add
  # the user supplied additional versions to the front.
  # If FindPythonInterp has already found the major and minor version,
  # insert that version between the user supplied versions and the stock
  # version list.
  # If no specific version is requested or suggested by PythonInterp, always look
  # for "python" executable first
  set(_PYTHON_VERSIONS ${PythonLibs_FIND_VERSION} ${PythonLibs_ADDITIONAL_VERSIONS} )
  if(DEFINED PYTHON_VERSION_MAJOR AND DEFINED PYTHON_VERSION_MINOR)
    list(APPEND _PYTHON_VERSIONS ${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR})
  endif()
  if (NOT _PYTHON_VERSIONS)
    set(_PYTHON_VERSIONS ";") # empty entry at the front makeing sure we search for "python" first
  endif()
  list(APPEND _PYTHON_VERSIONS ${_PYTHON_FIND_OTHER_VERSIONS})

  unset(_PYTHON_FIND_OTHER_VERSIONS)

  message(STATUS "Looking for versions: ${_PYTHON_VERSIONS}")

  FIND_FILE(_FIND_LIB_PYTHON_PY FindLibPython.py PATHS ${CMAKE_MODULE_PATH} ${CMAKE_ROOT}/Modules)

  if(NOT _FIND_LIB_PYTHON_PY)
    message(FATAL_ERROR "Could not find required file 'FindLibPython.py'")
  endif()

  unset(PYTHONLIBS_VERSION_STRING)
  foreach(_CURRENT_VERSION IN LISTS _PYTHON_VERSIONS)

    STRING(REGEX REPLACE "^([0-9]+).*$"          "\\1" _VERSION_MAJOR "${_CURRENT_VERSION}")
    STRING(REGEX REPLACE "^[0-9]+\\.([0-9]+).*$" "\\1" _VERSION_MINOR "${_CURRENT_VERSION}")

    set(_PYTHON_NAMES ${PYTHON_EXECUTABLE} python)

    if (_CURRENT_VERSION MATCHES "^[0-9]+.*$")
      list(APPEND _PYTHON_NAMES "python${_VERSION_MAJOR}")
      if (_CURRENT_VERSION MATCHES "^[0-9]+\\.[0-9].*$")
        list(APPEND _PYTHON_NAMES "python${_VERSION_MAJOR}.${_VERSION_MINOR}")
      endif()
    endif()

    message(STATUS "Looking for python version '${_CURRENT_VERSION}' by checking executables: ${_PYTHON_NAMES}.")

    foreach(_CURRENT_PYTHON_NAME IN LISTS _PYTHON_NAMES)

      unset(_PYTHON_EXECUTABLE CACHE)
      find_program(_PYTHON_EXECUTABLE ${_CURRENT_PYTHON_NAME}
        PATHS [HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\${_CURRENT_VERSION}\\InstallPath])

      if(_PYTHON_EXECUTABLE)

        EXECUTE_PROCESS(
          COMMAND ${_PYTHON_EXECUTABLE} "${_FIND_LIB_PYTHON_PY}"
          OUTPUT_VARIABLE _PYTHON_CONFIG
          RESULT_VARIABLE _PYTHON_CONFIG_RESULT
          ERROR_QUIET)

        if(NOT ${_PYTHON_CONFIG_RESULT} AND (NOT ${_PYTHON_CONFIG} STREQUAL ""))
          STRING(REGEX REPLACE ".*\nmajor_version:([0-9]+).*$" "\\1" _PYTHON_MAJOR_VERSION ${_PYTHON_CONFIG})
          STRING(REGEX REPLACE ".*\nminor_version:([0-9]+).*$" "\\1" _PYTHON_MINOR_VERSION ${_PYTHON_CONFIG})
          STRING(REGEX REPLACE ".*\npatch_version:([0-9]+).*$" "\\1" _PYTHON_PATCH_VERSION ${_PYTHON_CONFIG})
          STRING(REGEX REPLACE ".*\nshort_version:([^\n]+).*$" "\\1" _PYTHON_SHORT_VERSION ${_PYTHON_CONFIG})
          STRING(REGEX REPLACE ".*\nlong_version:([^\n]+).*$"  "\\1" _PYTHON_LONG_VERSION  ${_PYTHON_CONFIG})
          STRING(REGEX REPLACE ".*\npy_inc_dir:([^\n]+).*$"    "\\1" _PYTHON_INCLUDE_DIR   ${_PYTHON_CONFIG})
          STRING(REGEX REPLACE ".*\npy_lib_dir:([^\n]+).*$"    "\\1" _PYTHON_LIBRARY_DIR   ${_PYTHON_CONFIG})
          STRING(REGEX REPLACE ".*\nexec_prefix:(^\n+).*$"     "\\1" _PYTHON_PREFIX        ${_PYTHON_CONFIG})

          if ("${_CURRENT_VERSION}" STREQUAL ""                         OR
              "${_CURRENT_VERSION}" STREQUAL "${_PYTHON_MAJOR_VERSION}" OR
              "${_CURRENT_VERSION}" STREQUAL "${_PYTHON_SHORT_VERSION}" OR
              "${_CURRENT_VERSION}" STREQUAL "${_PYTHON_LONG_VERSION}")

            message(STATUS "Found executable ${_PYTHON_EXECUTABLE} with suitable version ${_PYTHON_LONG_VERSION}")

            if(NOT EXISTS "${PYTHON_INCLUDE_DIR}")
              set(PYTHON_INCLUDE_DIR "${_PYTHON_INCLUDE_DIR}")
            endif()

            if(NOT EXISTS "${PYTHON_LIBRARY}")
              set(_PYTHON_SHORT_VERSION_NO_DOT "${_PYTHON_MAJOR_VERSION}${_PYTHON_MINOR_VERSION}")
              set(_PYTHON_LIBRARY_NAMES python${_PYTHON_SHORT_VERSION} python${_PYTHON_SHORT_VERSION_NO_DOT} python${_PYTHON_SHORT_VERSION}m python${_PYTHON_SHORT_VERSION_NO_DOT}m)
              FIND_LIBRARY(PYTHON_LIBRARY
                NAMES ${_PYTHON_LIBRARY_NAMES}
                PATH_SUFFIXES
                "python${_PYTHON_SHORT_VERSION}/config"
                "python${_PYTHON_SHORT_VERSION_NO_DOT}/config"
                PATHS
                ${_PYTHON_LIBRARY_DIR}
                ${_PYTHON_PREFIX}/lib
                ${_PYTHON_PREFIX}/libs
                ${_PYTHON_LIBRARY_DIR}/x86_64-linux-gnu/
                NO_DEFAULT_PATH)

              if(WIN32)
                find_library(PYTHON_DEBUG_LIBRARY
                  NAMES python${_PYTHON_SHORT_VERSION_NO_DOT}_d python
                  PATHS
                  [HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\${_CURRENT_VERSION}\\InstallPath]/libs/Debug
                  [HKEY_CURRENT_USER\\SOFTWARE\\Python\\PythonCore\\${_CURRENT_VERSION}\\InstallPath]/libs/Debug
                  [HKEY_LOCAL_MACHINE\\SOFTWARE\\Python\\PythonCore\\${_CURRENT_VERSION}\\InstallPath]/libs
                  [HKEY_CURRENT_USER\\SOFTWARE\\Python\\PythonCore\\${_CURRENT_VERSION}\\InstallPath]/libs
                  )
              endif()
            endif()

            set(PYTHONLIBS_VERSION_STRING ${_PYTHON_LONG_VERSION})
            if(_PYTHON_PATCH_VERSION STREQUAL "0")
              # it's called "Python 2.7", not "2.7.0"
              string(REGEX REPLACE "\\.0$" "" PYTHONLIBS_VERSION_STRING "${PYTHONLIBS_VERSION_STRING}")
            endif()

            break()
          else()
            message(STATUS "Found executable ${_PYTHON_EXECUTABLE} with UNsuitable version ${_PYTHON_LONG_VERSION}")
          endif() # version ok
        else()
          message(WARNING "Found executable ${_PYTHON_EXECUTABLE}, but could not extract version info.")
        endif() # could extract config
      endif() # found executable
    endforeach() # python names
    if (PYTHONLIBS_VERSION_STRING)
      break()
    endif()
  endforeach() # python versions
endif()

unset(_PYTHON_NAMES)
unset(_PYTHON_VERSIONS)
unset(_PYTHON_EXECUTABLE CACHE)
unset(_PYTHON_MAJOR_VERSION)
unset(_PYTHON_MINOR_VERSION)
unset(_PYTHON_PATCH_VERSION)
unset(_PYTHON_SHORT_VERSION)
unset(_PYTHON_LONG_VERSION)
unset(_PYTHON_LIBRARY_DIR)
unset(_PYTHON_INCLUDE_DIR)
unset(_PYTHON_PREFIX)
unset(_PYTHON_SHORT_VERSION_NO_DOT)
unset(_PYTHON_LIBRARY_NAMES)


# For backward compatibility, set PYTHON_INCLUDE_PATH.
set(PYTHON_INCLUDE_PATH "${PYTHON_INCLUDE_DIR}")

mark_as_advanced(
  PYTHON_DEBUG_LIBRARY
  PYTHON_LIBRARY
  PYTHON_INCLUDE_DIR
)

# We use PYTHON_INCLUDE_DIR, PYTHON_LIBRARY and PYTHON_DEBUG_LIBRARY for the
# cache entries because they are meant to specify the location of a single
# library. We now set the variables listed by the documentation for this
# module.
set(PYTHON_INCLUDE_DIRS "${PYTHON_INCLUDE_DIR}")
set(PYTHON_DEBUG_LIBRARIES "${PYTHON_DEBUG_LIBRARY}")

# These variables have been historically named in this module different from
# what SELECT_LIBRARY_CONFIGURATIONS() expects.
set(PYTHON_LIBRARY_DEBUG "${PYTHON_DEBUG_LIBRARY}")
set(PYTHON_LIBRARY_RELEASE "${PYTHON_LIBRARY}")
include(${CMAKE_CURRENT_LIST_DIR}/SelectLibraryConfigurations.cmake)
SELECT_LIBRARY_CONFIGURATIONS(PYTHON)

if(PYTHON_LIBRARY AND NOT PYTHON_LIBRARIES)
  set(PYTHON_LIBRARIES "${PYTHON_LIBRARY}")
endif()
# SELECT_LIBRARY_CONFIGURATIONS() sets ${PREFIX}_FOUND if it has a library.
# Unset this, this prefix doesn't match the module prefix, they are different
# for historical reasons.
unset(PYTHON_FOUND)

# include(${CMAKE_CURRENT_LIST_DIR}/FindPackageHandleStandardArgs.cmake)
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(PythonLibs
                                  REQUIRED_VARS PYTHON_LIBRARIES PYTHON_INCLUDE_DIRS
                                  VERSION_VAR PYTHONLIBS_VERSION_STRING)

# PYTHON_ADD_MODULE(<name> src1 src2 ... srcN) is used to build modules for python.
# PYTHON_WRITE_MODULES_HEADER(<filename>) writes a header file you can include
# in your sources to initialize the static python modules
function(PYTHON_ADD_MODULE _NAME )
  get_property(_TARGET_SUPPORTS_SHARED_LIBS
    GLOBAL PROPERTY TARGET_SUPPORTS_SHARED_LIBS)
  option(PYTHON_ENABLE_MODULE_${_NAME} "Add module ${_NAME}" TRUE)
  option(PYTHON_MODULE_${_NAME}_BUILD_SHARED
    "Add module ${_NAME} shared" ${_TARGET_SUPPORTS_SHARED_LIBS})

  # Mark these options as advanced
  mark_as_advanced(PYTHON_ENABLE_MODULE_${_NAME}
    PYTHON_MODULE_${_NAME}_BUILD_SHARED)

  if(PYTHON_ENABLE_MODULE_${_NAME})
    if(PYTHON_MODULE_${_NAME}_BUILD_SHARED)
      set(PY_MODULE_TYPE MODULE)
    else()
      set(PY_MODULE_TYPE STATIC)
      set_property(GLOBAL  APPEND  PROPERTY  PY_STATIC_MODULES_LIST ${_NAME})
    endif()

    set_property(GLOBAL  APPEND  PROPERTY  PY_MODULES_LIST ${_NAME})
    add_library(${_NAME} ${PY_MODULE_TYPE} ${ARGN})
#    target_link_libraries(${_NAME} ${PYTHON_LIBRARIES})

    if(PYTHON_MODULE_${_NAME}_BUILD_SHARED)
      set_target_properties(${_NAME} PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}")
      if(WIN32 AND NOT CYGWIN)
        set_target_properties(${_NAME} PROPERTIES SUFFIX ".pyd")
      endif()
    endif()

  endif()
endfunction()

function(PYTHON_WRITE_MODULES_HEADER _filename)

  get_property(PY_STATIC_MODULES_LIST  GLOBAL  PROPERTY PY_STATIC_MODULES_LIST)

  get_filename_component(_name "${_filename}" NAME)
  string(REPLACE "." "_" _name "${_name}")
  string(TOUPPER ${_name} _nameUpper)
  set(_filename ${CMAKE_CURRENT_BINARY_DIR}/${_filename})

  set(_filenameTmp "${_filename}.in")
  file(WRITE ${_filenameTmp} "/*Created by cmake, do not edit, changes will be lost*/\n")
  file(APPEND ${_filenameTmp}
"#ifndef ${_nameUpper}
#define ${_nameUpper}

#include <Python.h>

#ifdef __cplusplus
extern \"C\" {
#endif /* __cplusplus */

")

  foreach(_currentModule ${PY_STATIC_MODULES_LIST})
    file(APPEND ${_filenameTmp} "extern void init${PYTHON_MODULE_PREFIX}${_currentModule}(void);\n\n")
  endforeach()

  file(APPEND ${_filenameTmp}
"#ifdef __cplusplus
}
#endif /* __cplusplus */

")


  foreach(_currentModule ${PY_STATIC_MODULES_LIST})
    file(APPEND ${_filenameTmp} "int ${_name}_${_currentModule}(void) \n{\n  static char name[]=\"${PYTHON_MODULE_PREFIX}${_currentModule}\"; return PyImport_AppendInittab(name, init${PYTHON_MODULE_PREFIX}${_currentModule});\n}\n\n")
  endforeach()

  file(APPEND ${_filenameTmp} "void ${_name}_LoadAllPythonModules(void)\n{\n")
  foreach(_currentModule ${PY_STATIC_MODULES_LIST})
    file(APPEND ${_filenameTmp} "  ${_name}_${_currentModule}();\n")
  endforeach()
  file(APPEND ${_filenameTmp} "}\n\n")
  file(APPEND ${_filenameTmp} "#ifndef EXCLUDE_LOAD_ALL_FUNCTION\nvoid CMakeLoadAllPythonModules(void)\n{\n  ${_name}_LoadAllPythonModules();\n}\n#endif\n\n#endif\n")

# with configure_file() cmake complains that you may not use a file created using file(WRITE) as input file for configure_file()
  execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different "${_filenameTmp}" "${_filename}" OUTPUT_QUIET ERROR_QUIET)

endfunction()
