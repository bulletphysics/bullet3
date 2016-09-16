#!/bin/sh
rm CMakeCache.txt
mkdir build_cmake
cd build_cmake
cmake -DBUILD_PYBULLET=OFF -DCMAKE_BUILD_TYPE=Release ..
make -j12
examples/ExampleBrowser/App_ExampleBrowser
