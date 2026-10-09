# source: https://gist.github.com/peterspackman/8cf73f7f12ba270aa8192d6911972fe8

# Sample toolchain file for building for Windows from an Ubuntu Linux system.
#
# Typical usage:
#    *) install cross compiler: `sudo apt-get install mingw-w64`
#    *) cd build
#    *) cmake -DCMAKE_TOOLCHAIN_FILE=~/mingw-w64-x86_64.cmake ..
# This is free and unencumbered software released into the public domain.

if(CMAKE_HOST_SYSTEM_NAME STREQUAL "Windows")
  set(EXE_EXTENSION .exe)
else()
  set(EXE_EXTENSION)
endif()
set(CMAKE_SYSTEM_NAME Windows)
set(TOOLCHAIN_PREFIX ${TOOLCHAIN_ROOT}x86_64-w64-mingw32)

# cross compilers to use for C, C++ and Fortran
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}-gcc${EXE_EXTENSION})
set(CMAKE_C_COMPILER_AR ${TOOLCHAIN_PREFIX}-ar${EXE_EXTENSION})
set(CMAKE_C_COMPILER_RANLIB ${TOOLCHAIN_PREFIX}-ranlib${EXE_EXTENSION})
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}-gcc${EXE_EXTENSION})
set(CMAKE_CXX_COMPILER_AR ${TOOLCHAIN_PREFIX}-ar${EXE_EXTENSION})
set(CMAKE_CXX_COMPILER_RANLIB ${TOOLCHAIN_PREFIX}-ranlib${EXE_EXTENSION})
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}-g++${EXE_EXTENSION})
set(CMAKE_Fortran_COMPILER ${TOOLCHAIN_PREFIX}-gfortran${EXE_EXTENSION})
set(CMAKE_RC_COMPILER ${TOOLCHAIN_PREFIX}-windres${EXE_EXTENSION})
# target environment on the build host system
set(CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_PREFIX})

# modify default behavior of FIND_XXX() commands
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
