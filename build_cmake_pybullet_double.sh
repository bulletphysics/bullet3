#!/bin/sh
rm CMakeCache.txt
mkdir build_cmake
cd build_cmake
cmake -DBUILD_PYBULLET=ON -DBUILD_PYBULLET_NUMPY=OFF -DUSE_DOUBLE_PRECISION=ON -DCMAKE_BUILD_TYPE=Release ..
make -j12
cd examples
cd pybullet
ln -s pybullet.dylib pybullet.so

