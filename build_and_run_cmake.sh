#!/bin/sh
rm CMakeCache.txt
mkdir build_cmake
cd build_cmake
cmake ..
make -j12
examples/ExampleBrowser/App_ExampleBrowser
