# Find the Python NumPy package
# PYTHON_NUMPY_INCLUDE_DIR
# PYTHON_NUMPY_FOUND
# will be set by this script

cmake_minimum_required(VERSION 2.6)

if(NOT PYTHON_EXECUTABLE)
  if(NumPy_FIND_QUIETLY)
    find_package(PythonInterp QUIET)
  else()
    find_package(PythonInterp)
    set(__numpy_out 1)
  endif()
endif()

if (PYTHON_EXECUTABLE)
  # Find out the include path
  execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -c
            "from __future__ import print_function\ntry: import numpy; print(numpy.get_include(), end='')\nexcept:pass\n"
            OUTPUT_VARIABLE __numpy_path)
  # And the version
  execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -c
            "from __future__ import print_function\ntry: import numpy; print(numpy.__version__, end='')\nexcept:pass\n"
    OUTPUT_VARIABLE __numpy_version)
elseif(__numpy_out)
  message(STATUS "Python executable not found.")
endif(PYTHON_EXECUTABLE)

find_path(PYTHON_NUMPY_INCLUDE_DIR numpy/arrayobject.h
  HINTS "${__numpy_path}" "${PYTHON_INCLUDE_PATH}" NO_DEFAULT_PATH)

if(PYTHON_NUMPY_INCLUDE_DIR)
  set(PYTHON_NUMPY_FOUND 1 CACHE INTERNAL "Python numpy found")
endif(PYTHON_NUMPY_INCLUDE_DIR)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(NumPy REQUIRED_VARS PYTHON_NUMPY_INCLUDE_DIR
                                        VERSION_VAR __numpy_version)
