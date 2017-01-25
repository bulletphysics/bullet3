#!/bin/sh
rm CMakeCache.txt
mkdir build_cmake_python3
cd build_cmake_python3
cmake -DBUILD_PYBULLET=ON -DBUILD_PYBULLET_NUMPY=OFF -DUSE_DOUBLE_PRECISION=ON -DCMAKE_BUILD_TYPE=Release -DPYTHON_INCLUDE_DIR=~/anaconda/envs/rllab3/include/python3.5m/ -DPYTHON_LIBRARY=~/anaconda/envs/rllab3/lib/libpython3.5m.dylib ..
make -j8