# FindBulletExtras.cmake
# =============
# 
# Finds the BulletExtras library.
#
# Based on: https://cmake.org/cmake/help/latest/manual/cmake-developer.7.html
# 
# Imported Targets
# ----------------
# 
# This module provides the following imported targets, if found:
# 
# ``BulletExtras::BulletExtras``
# 
# Result Variables
# ----------------
# 
# This will define the following variables:
# 
# ``BulletExtras_FOUND``         True if the system has the BulletExtras library.
# ``BulletExtras_VERSION``       The version of the BulletExtras library which was found.
# ``BulletExtras_INCLUDE_DIRS``  Include directories needed to use BulletExtras.
# ``BulletExtras_LIBRARIES``     Libraries needed to link to BulletExtras.
# ``BulletExtras_DEFINITIONS``   Compile definitions
# 
# Cache Variables
# ---------------
# 
# The following cache variables may also be set:
# 
# ``BulletExtras_INCLUDE_DIR`` The directory containing ``BulletExtras.h``.
# ``BulletExtras_LIBRARY``     The path to the BulletExtras library.

# try to use pkg-config to find the library
find_package(PkgConfig)
pkg_check_modules(PC_Bullet QUIET Bullet)

# find the libraries and include files
find_path(BulletExtras_INCLUDE_DIR
  NAMES SharedMemory/b3RobotSimulatorClientAPI_NoGUI.h
  PATHS ${PC_Bullet_INCLUDE_DIRS}
  PATH_SUFFIXES bullet
)

set(BulletExtras_LIBRARIES_DESIRED
  BulletRobotics
  Bullet3Common
  BulletRoboticsGUI
  BulletInverseDynamicsUtils
  BulletInverseDynamics
)

set(BulletExtras_LIBRARIES_RESULT)
foreach(LIBRARY_NAME ${BulletExtras_LIBRARIES_DESIRED})
  find_library(${LIBRARY_NAME}_LIBRARY
    NAMES ${LIBRARY_NAME}
    REQUIRED
  )
  list(APPEND BulletExtras_LIBRARIES_RESULT ${${LIBRARY_NAME}_LIBRARY})
endforeach()

set(BulletExtras_VERSION ${PC_Bullet_VERSION})

# check that the REQUIRED_VARS contain values and set BulletExtras_FOUND
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(BulletExtras
  FOUND_VAR BulletExtras_FOUND
  REQUIRED_VARS
    BulletExtras_INCLUDE_DIR
  VERSION_VAR BulletExtras_VERSION
)

# traditional approach is to use variables for everything, 
# including libraries and executables
if(BulletExtras_FOUND)
  set(BulletExtras_LIBRARIES  ${BulletExtras_LIBRARIES_RESULT})
  set(BulletExtras_INCLUDE_DIRS ${BulletExtras_INCLUDE_DIR})

  # Some or all of these compile definitions are required for the simulator graphics to work correctly
  # These flags were found by:
  # 1. Add the following to the top of bullet3/examples/RobotSimulator/CMakeLists.txt:
  #        set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
  # 2. Inspect all compile defintions (-Dxxx) in bullet3/build/examples/RobotSimulator/compile_commands.json
  set(BulletExtras_DEFINITIONS ${PC_BulletExtras_CFLAGS_OTHER}
    -DB3_USE_ROBOTSIM_GUI
    -DBT_ENABLE_CLSOCKET
    -DBT_USE_DOUBLE_PRECISION
    -DBT_USE_EGL
    -DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1
    -DGLEW_INIT_OPENGL11_FUNCTIONS=1
    -DGLEW_STATIC
    -DUSE_GRAPHICAL_BENCHMARK
    -D_LINUX
    )
endif()

#
list(GET BulletExtras_LIBRARIES 0 BulletExtras_LIBRARY)
message(STATUS "BulletExtras_LIBRARIES: ${BulletExtras_LIBRARIES}")
message(STATUS "BulletExtras_LIBRARY: ${BulletExtras_LIBRARY}")


# modern approach is to behave as much like config file packages files as possible,
# by providing imported target.
if(BulletExtras_FOUND AND NOT TARGET BulletExtras::BulletExtras)
  add_library(BulletExtras::BulletExtras UNKNOWN IMPORTED)
  set_target_properties(BulletExtras::BulletExtras PROPERTIES
    IMPORTED_LOCATION "${BulletExtras_LIBRARY}"
    INTERFACE_LINK_LIBRARIES "${BulletExtras_LIBRARIES}"
    INTERFACE_COMPILE_OPTIONS "${BulletExtras_DEFINITIONS}"
    INTERFACE_INCLUDE_DIRECTORIES "${BulletExtras_INCLUDE_DIRS}"
  )
endif()

# Most of the cache variables should be hidden in the ccmake interface
mark_as_advanced(
  BulletExtras_INCLUDE_DIR
  BulletExtras_LIBRARY
)