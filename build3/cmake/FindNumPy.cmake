# - Find the NumPy libraries
# This module finds if NumPy is installed, and sets the following variables
# indicating where it is.
#
# TODO: Update to provide the libraries and paths for linking npymath lib.
#
#  PYTHON_NUMPY_FOUND               - was NumPy found
#  PYTHON_NUMPY_VERSION             - the version of NumPy found as a string
#  PYTHON_NUMPY_VERSION_MAJOR       - the major version number of NumPy
#  PYTHON_NUMPY_VERSION_MINOR       - the minor version number of NumPy
#  PYTHON_NUMPY_VERSION_PATCH       - the patch version number of NumPy
#  PYTHON_NUMPY_VERSION_DECIMAL     - e.g. version 1.6.1 is 10601
#  PYTHON_NUMPY_INCLUDE_DIR         - path to the NumPy include files

unset(PYTHON_NUMPY_VERSION)
unset(PYTHON_NUMPY_INCLUDE_DIR)

if(PYTHONINTERP_FOUND)
  execute_process(COMMAND "${PYTHON_EXECUTABLE}" "-c"
    "import numpy as n; print(n.__version__); print(n.get_include());"
    RESULT_VARIABLE __result
    OUTPUT_VARIABLE __output
    OUTPUT_STRIP_TRAILING_WHITESPACE)

  if(__result MATCHES 0)
    string(REGEX REPLACE ";" "\\\\;" __values ${__output})
    string(REGEX REPLACE "\r?\n" ";"    __values ${__values})
    list(GET __values 0 PYTHON_NUMPY_VERSION)
    list(GET __values 1 PYTHON_NUMPY_INCLUDE_DIR)

    string(REGEX MATCH "^([0-9])+\\.([0-9])+\\.([0-9])+" __ver_check "${PYTHON_NUMPY_VERSION}")
    if(NOT "${__ver_check}" STREQUAL "")
      set(PYTHON_NUMPY_VERSION_MAJOR ${CMAKE_MATCH_1})
      set(PYTHON_NUMPY_VERSION_MINOR ${CMAKE_MATCH_2})
      set(PYTHON_NUMPY_VERSION_PATCH ${CMAKE_MATCH_3})
      math(EXPR PYTHON_NUMPY_VERSION_DECIMAL
        "(${PYTHON_NUMPY_VERSION_MAJOR} * 10000) + (${PYTHON_NUMPY_VERSION_MINOR} * 100) + ${PYTHON_NUMPY_VERSION_PATCH}")
      string(REGEX REPLACE "\\\\" "/"  PYTHON_NUMPY_INCLUDE_DIR ${PYTHON_NUMPY_INCLUDE_DIR})
    else()
     unset(PYTHON_NUMPY_VERSION)
     unset(PYTHON_NUMPY_INCLUDE_DIR)
     message(STATUS "Requested NumPy version and include path, but got instead:\n${__output}\n")
    endif()
  endif()
else()
  message(STATUS "To find NumPy Python interpretor is required to be found.")
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(NumPy REQUIRED_VARS PYTHON_NUMPY_INCLUDE_DIR PYTHON_NUMPY_VERSION
                                        VERSION_VAR   PYTHON_NUMPY_VERSION)

if(NUMPY_FOUND)
  set(PYTHON_NUMPY_FOUND TRUE)
  message(STATUS "NumPy ver. ${PYTHON_NUMPY_VERSION} found (include: ${PYTHON_NUMPY_INCLUDE_DIR})")
endif()

# caffe_clear_vars(__result __output __error_value __values __ver_check __error_value)

