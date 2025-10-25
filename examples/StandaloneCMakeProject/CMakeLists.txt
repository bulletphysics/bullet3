cmake_minimum_required(VERSION 3.10)

# =========================
#      PROJECT SETUP
# =========================
# set the project name
project(RobotSimulator)

# add path to FindBulletExtras.cmake
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
set(CMAKE_BUILD_TYPE "Release" CACHE PATH "Debug Release RelWithDebInfo." FORCE)
set(CMAKE_VERBOSE_MAKEFILE ON)

# =========================
# Required Packages
# =========================

# Search CMAKE_MODULE_PATH, CMAKE_PREFIX_PATH, and CMAKE_INSTALL_PREFIX
find_package(Bullet REQUIRED)
find_package(BulletExtras REQUIRED)
find_package(OpenGL REQUIRED)

# =========================
# Build and link executable
# =========================

add_executable(${PROJECT_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/../RobotSimulator/RobotSimulatorMain.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../RobotSimulator/b3RobotSimulatorClientAPI.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../RobotSimulator/MinitaurSetup.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC 
    ${BULLET_INCLUDE_DIRS}
  )

target_link_libraries(${PROJECT_NAME}
  ${BULLET_LIBRARIES}
  BulletExtras::BulletExtras
  #OpenGL::GL
  #OpenGL::GLU
  OpenGL::OpenGL
)

# Ideally, we use include paths like:
# include <SharedMemory/b3RobotSimulatorClientAPI_NoGUI.h>
# so that this is not needed
target_include_directories(${PROJECT_NAME}
  PUBLIC 
    ${BULLET_INCLUDE_DIRS}/SharedMemory
  )