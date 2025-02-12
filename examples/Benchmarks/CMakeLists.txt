project(App_Benchmarks)
cmake_minimum_required(VERSION 3.5.1) # per google benchmark

if(NOT DEFINED BULLET_PHYSICS_SOURCE_DIR)
  option(BUILD_BULLET2_DEMOS OFF) # Prevent circular dependency
  option(BUILD_BULLET3 OFF)
  option(BUILD_EXTRAS OFF)
  option(BUILD_UNIT_TESTS OFF)
  option(INSTALL_LIBS OFF)
  option(USE_GLUT OFF)
  add_subdirectory(../../ bullet EXCLUDE_FROM_ALL)
endif()

option(BENCHMARK_ENABLE_INSTALL OFF)
option(BENCHMARK_ENABLE_TESTING OFF)
add_subdirectory(
  ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/benchmark
  ${BULLET_PHYSICS_BINARY_DIR}/examples/ThirdPartyLibs/benchmark
  EXCLUDE_FROM_ALL)

add_executable(App_Benchmarks
  ${BULLET_PHYSICS_SOURCE_DIR}/examples/Benchmarks/BenchmarkDemo.cpp
  ${BULLET_PHYSICS_SOURCE_DIR}/examples/Benchmarks/main.cpp
  ${BULLET_PHYSICS_SOURCE_DIR}/examples/Headless/HeadlessInterfaceHelper.cpp
  ${BULLET_PHYSICS_SOURCE_DIR}/examples/Headless/HeadlessParameters.cpp
  ${BULLET_PHYSICS_SOURCE_DIR}/examples/Headless/HeadlessRenderer.cpp
  ${BULLET_PHYSICS_SOURCE_DIR}/examples/MultiThreadedDemo/CommonRigidBodyMTBase.cpp)

target_compile_definitions(App_Benchmarks PUBLIC
  BT_ENABLE_PROFILE
  $<$<BOOL:${USE_DOUBLE_PRECISION}>:BT_USE_DOUBLE_PRECISION>)

target_compile_options(App_Benchmarks PUBLIC
  -std=c++11)

target_include_directories(App_Benchmarks PUBLIC
  ${BULLET_PHYSICS_SOURCE_DIR}/examples
  ${BULLET_PHYSICS_SOURCE_DIR}/src)

target_link_libraries(App_Benchmarks
  BulletCollision
  BulletDynamics
  LinearMath
  benchmark::benchmark)
